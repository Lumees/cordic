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
// CORDIC IP - Pre-Processing (Quadrant Correction + Saturation Detection)
// =============================================================================
// Extends CORDIC convergence range to full [-π, +π] for circular mode.
//
// Circular ROTATION: CORDIC converges for |z| < ~1.74 rad ≈ 0.5547*π
//   Pre-process: if |z| > π/2, negate x and y, adjust z by ±π
//   z format: normalized so π ↔ 2^(DATA_WIDTH-2)
//
// Circular VECTORING: ensure x > 0 before entering core.
//   If x < 0: negate x, negate y, add π to accumulated angle offset
//   The angle offset is added back post-CORDIC to z_out.
//
// Linear/Hyperbolic: no pre-processing needed.
//
// Saturation (NEW): negation of the minimum signed integer overflows in
// two's complement. If din.x or din.y equals MIN_INT = -2^(DATA_WIDTH-1)
// when a negation is about to be applied, dout.sat is set.  The result
// is clamped to MAX_INT to avoid producing a wrong-sign value.
// =============================================================================

`timescale 1ns/1ps

import cordic_pkg::*;

module cordic_pre_proc (
  input  logic         clk,
  input  logic         rst_n,
  // Raw input
  input  cordic_data_t din,
  // Pre-processed output to CORDIC core
  output cordic_data_t dout,
  // Quadrant offset to be added back to z_out (vectoring only)
  output logic signed [DATA_WIDTH-1:0] quad_offset,
  output logic                          quad_offset_valid
);

  // π/2 in angle-normalized format: 2^(DATA_WIDTH-3)
  localparam logic signed [DATA_WIDTH-1:0] PI_HALF = DATA_WIDTH'(1 << (DATA_WIDTH-3));
  // π in angle-normalized format: 2^(DATA_WIDTH-2)
  localparam logic signed [DATA_WIDTH-1:0] PI      = DATA_WIDTH'(1 << (DATA_WIDTH-2));
  // Saturation limits
  localparam logic signed [DATA_WIDTH-1:0] MAX_INT = {1'b0, {(DATA_WIDTH-1){1'b1}}};
  localparam logic signed [DATA_WIDTH-1:0] MIN_INT = {1'b1, {(DATA_WIDTH-1){1'b0}}};

  // Negation with saturation: -MIN_INT overflows to MIN_INT; clamp to MAX_INT
  function automatic logic signed [DATA_WIDTH-1:0] safe_neg(
    input logic signed [DATA_WIDTH-1:0] v
  );
    return (v == MIN_INT) ? MAX_INT : -v;
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dout             <= '0;
      quad_offset       <= '0;
      quad_offset_valid <= 1'b0;
    end else begin
      // Default: pass through (sat propagates from input)
      dout             <= din;
      quad_offset       <= '0;
      quad_offset_valid <= din.valid;

      if (din.valid && din.coord == CIRC) begin

        unique case (din.mode)

          // ------------------------------------------------------------------
          // Circular ROTATION: extend range to ±π
          // ------------------------------------------------------------------
          ROTATION: begin
            if (din.z > PI_HALF) begin
              // z in (π/2, π]: negate x,y and subtract π from z
              dout.x   <= safe_neg(din.x);
              dout.y   <= safe_neg(din.y);
              dout.z   <= din.z - PI;
              dout.sat <= din.sat | (din.x == MIN_INT) | (din.y == MIN_INT);
            end else if (din.z < -PI_HALF) begin
              // z in [−π, −π/2): negate x,y and add π to z
              dout.x   <= safe_neg(din.x);
              dout.y   <= safe_neg(din.y);
              dout.z   <= din.z + PI;
              dout.sat <= din.sat | (din.x == MIN_INT) | (din.y == MIN_INT);
            end
            // else: no adjustment — sat already propagated via default
          end

          // ------------------------------------------------------------------
          // Circular VECTORING: ensure x > 0
          // ------------------------------------------------------------------
          VECTORING: begin
            if (din.x[DATA_WIDTH-1]) begin
              // x < 0: negate both x and y
              dout.x   <= safe_neg(din.x);
              dout.y   <= safe_neg(din.y);
              dout.sat <= din.sat | (din.x == MIN_INT) | (din.y == MIN_INT);
              // Record quadrant offset based on ORIGINAL y sign:
              // Q2 (x<0, y>=0): CORDIC gives [-π/2,0], add +π → [π/2, π]
              // Q3 (x<0, y< 0): CORDIC gives [0, π/2], add -π → [-π,-π/2]
              quad_offset <= din.y[DATA_WIDTH-1] ? -PI : PI;
            end
          end

          default: ;
        endcase

      end
    end
  end

endmodule : cordic_pre_proc
