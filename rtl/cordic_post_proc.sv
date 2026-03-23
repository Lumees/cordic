// =============================================================================
// Copyright (c) 2026 Lumees Lab / Hasan Kurşun
// SPDX-License-Identifier: Apache-2.0 WITH Commons-Clause
//
// Licensed under the Apache License 2.0 with Commons Clause restriction.
// You may use this file freely for non-commercial purposes (academic,
// research, hobby, education, personal projects).
//
// COMMERCIAL USE requires a separate license from Lumees Lab.
// Contact: info@lumeeslab.com · https://lumeeslab.com
// =============================================================================
// =============================================================================
// CORDIC IP - Post-Processing (Gain Compensation + Quadrant Offset + Sat)
// =============================================================================
// 1. Applies CORDIC gain correction (multiply-by-constant, DSP-friendly):
//      Circular  : multiply x,y by K_circ_inv ≈ 0.6073
//      Hyperbolic: multiply x,y by K_hypr_inv ≈ 1.2074
//      Linear    : no gain correction
//
// 2. Adds accumulated quadrant offset to z_out (circular vectoring only)
//
// 3. Saturation clamping (NEW):
//      If the gain-corrected product exceeds the DATA_WIDTH-bit signed range,
//      x/y are clamped to MAX_INT / MIN_INT and dout.sat is asserted.
//      dout.sat is OR'd with incoming din.sat (sticky accumulation).
//
// 4. Optional rounding (NEW, controlled by ROUND_MODE package parameter):
//      ROUND_MODE=0 → arithmetic-right-shift (truncate, default)
//      ROUND_MODE=1 → add 0.5 LSB before shift (round-to-nearest)
//
// Gain constants are computed at elaboration via circ_kinv_q()/hypr_kinv_q().
// =============================================================================

`timescale 1ns/1ps

import cordic_pkg::*;

module cordic_post_proc #(
  parameter bit APPLY_GAIN = 1  // Set 0 if user pre-scales inputs
) (
  input  logic         clk,
  input  logic         rst_n,
  // From CORDIC core output
  input  cordic_data_t din,
  // Quadrant offset (delayed to match CORDIC pipeline latency)
  input  logic signed [DATA_WIDTH-1:0] quad_offset,
  input  logic                          quad_offset_valid,
  // Final output
  output cordic_data_t dout
);

  // -------------------------------------------------------------------------
  // Elaboration-time constants (computed from FRAC_BITS and STAGES)
  // NOTE: Use (2*DATA_WIDTH)'(...) not 2*DATA_WIDTH'(...) — the latter parses
  //       as 2 * (DATA_WIDTH'(...)) due to SV operator precedence.
  // -------------------------------------------------------------------------
  localparam logic signed [2*DATA_WIDTH-1:0] CIRC_KINV = circ_kinv_q();
  localparam logic signed [2*DATA_WIDTH-1:0] HYPR_KINV = hypr_kinv_q();

  // Round-to-nearest addend: 0.5 in Q-format = 2^(FRAC_BITS-1)
  // 0 when ROUND_MODE=0 (truncate) → no bias
  localparam logic signed [2*DATA_WIDTH-1:0] ROUND_HALF =
      ROUND_MODE ? (2*DATA_WIDTH)'(longint'(1) << (FRAC_BITS - 1)) : '0;

  // Saturation limits
  localparam logic signed [DATA_WIDTH-1:0] SAT_MAX = {1'b0, {(DATA_WIDTH-1){1'b1}}};
  localparam logic signed [DATA_WIDTH-1:0] SAT_MIN = {1'b1, {(DATA_WIDTH-1){1'b0}}};

  // -------------------------------------------------------------------------
  // Combinational gain correction (Vivado/DC compatible)
  // -------------------------------------------------------------------------
  logic signed [2*DATA_WIDTH-1:0] pp_kinv, pp_xc, pp_yc;
  logic                            pp_sat_x, pp_sat_y, pp_sat_new;

  always_comb begin
    pp_kinv = (din.coord == CIRC) ? CIRC_KINV : HYPR_KINV;
    pp_xc   = ($signed(din.x) * $signed(pp_kinv) + ROUND_HALF) >>> FRAC_BITS;
    pp_yc   = ($signed(din.y) * $signed(pp_kinv) + ROUND_HALF) >>> FRAC_BITS;
    pp_sat_x   = (pp_xc[2*DATA_WIDTH-1:DATA_WIDTH] != {DATA_WIDTH{pp_xc[DATA_WIDTH-1]}});
    pp_sat_y   = (pp_yc[2*DATA_WIDTH-1:DATA_WIDTH] != {DATA_WIDTH{pp_yc[DATA_WIDTH-1]}});
    pp_sat_new = pp_sat_x | pp_sat_y;
  end

  // -------------------------------------------------------------------------
  // Registered output
  // -------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dout <= '0;
    end else begin
      dout.coord <= din.coord;
      dout.mode  <= din.mode;
      dout.tag   <= din.tag;
      dout.valid <= din.valid;

      if (!APPLY_GAIN || din.coord == LINR) begin
        dout.x   <= din.x;
        dout.y   <= din.y;
        dout.sat <= din.sat;
        if (din.coord == CIRC && din.mode == VECTORING && quad_offset_valid)
          dout.z <= din.z + quad_offset;
        else
          dout.z <= din.z;

      end else begin
        dout.x <= pp_sat_x ? (pp_xc[2*DATA_WIDTH-1] ? SAT_MIN : SAT_MAX)
                           : pp_xc[DATA_WIDTH-1:0];
        dout.y <= pp_sat_y ? (pp_yc[2*DATA_WIDTH-1] ? SAT_MIN : SAT_MAX)
                           : pp_yc[DATA_WIDTH-1:0];
        dout.sat <= din.sat | pp_sat_new;

        if (din.coord == CIRC && din.mode == VECTORING && quad_offset_valid)
          dout.z <= din.z + quad_offset;
        else
          dout.z <= din.z;
      end
    end
  end

endmodule : cordic_post_proc
