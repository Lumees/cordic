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
  // Gain-corrected pipeline stage
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
        // ── No gain: pass through x, y ──────────────────────────────────
        dout.x   <= din.x;
        dout.y   <= din.y;
        dout.sat <= din.sat;   // no new saturation possible here
        // Add quadrant offset for circular vectoring
        if (din.coord == CIRC && din.mode == VECTORING && quad_offset_valid)
          dout.z <= din.z + quad_offset;
        else
          dout.z <= din.z;

      end else begin
        // ── Gain correction: multiply-by-constant then arithmetic shift ──
        automatic logic signed [2*DATA_WIDTH-1:0] kinv;
        automatic logic signed [2*DATA_WIDTH-1:0] xc, yc;
        automatic logic                            sat_x, sat_y, sat_new;

        kinv = (din.coord == CIRC) ? CIRC_KINV : HYPR_KINV;

        // Multiply, add optional rounding addend, then shift
        xc = ($signed(din.x) * $signed(kinv) + ROUND_HALF) >>> FRAC_BITS;
        yc = ($signed(din.y) * $signed(kinv) + ROUND_HALF) >>> FRAC_BITS;

        // Saturation check: result must fit in DATA_WIDTH signed bits.
        // Valid if upper DATA_WIDTH bits are all copies of bit [DATA_WIDTH-1].
        sat_x = (xc[2*DATA_WIDTH-1:DATA_WIDTH] != {DATA_WIDTH{xc[DATA_WIDTH-1]}});
        sat_y = (yc[2*DATA_WIDTH-1:DATA_WIDTH] != {DATA_WIDTH{yc[DATA_WIDTH-1]}});
        sat_new = sat_x | sat_y;

        // Clamp to saturation limits; use full-product sign for direction
        dout.x <= sat_x ? (xc[2*DATA_WIDTH-1] ? SAT_MIN : SAT_MAX)
                        : xc[DATA_WIDTH-1:0];
        dout.y <= sat_y ? (yc[2*DATA_WIDTH-1] ? SAT_MIN : SAT_MAX)
                        : yc[DATA_WIDTH-1:0];

        // Sticky sat: OR with upstream sat
        dout.sat <= din.sat | sat_new;

        // z: no gain correction; add quadrant offset for circ vectoring
        if (din.coord == CIRC && din.mode == VECTORING && quad_offset_valid)
          dout.z <= din.z + quad_offset;
        else
          dout.z <= din.z;
      end
    end
  end

endmodule : cordic_post_proc
