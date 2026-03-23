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
// CORDIC IP - Sin/Cos Convenience Wrapper
// =============================================================================
// Computes cos(θ) and sin(θ) simultaneously using the CORDIC circular
// rotation mode.  This is the most common CORDIC use case; this wrapper
// hard-wires the algorithm parameters so the caller only sees:
//
//   Input:  s_theta  — angle in Q2.(DATA_WIDTH-2) normalized format
//                      π maps to 2^(DATA_WIDTH-2)   (e.g. 0x4000 for 16-bit)
//                     -π maps to -2^(DATA_WIDTH-2)  (e.g. 0xC000 for 16-bit)
//                      Full range [-π, +π] is supported.
//
//   Output: m_cos    — cos(θ) in the same Q format; 1.0 → 2^(DATA_WIDTH-2)
//           m_sin    — sin(θ) in the same Q format
//           m_sat    — asserted when saturation was detected in the pipeline
//
// With APPLY_GAIN=1 (default):
//   The x input to the CORDIC is set to 1.0 (= 2^FRAC_BITS in Q format).
//   After the CORDIC gain K is applied and corrected (× K_inv), the result
//   is exactly cos(θ) and sin(θ) within the fixed-point tolerance.
//
// With APPLY_GAIN=0 (advanced):
//   The x input is set to K_inv (≈ 0.6073 × 2^FRAC_BITS), which pre-
//   compensates for the CORDIC gain.  Post-processing gain correction is
//   skipped; slightly fewer operations, same output accuracy.
//
// Pipeline latency: STAGES + 2 cycles (same as cordic_top).
// Throughput: 1 result per clock (fully pipelined).
// =============================================================================

`timescale 1ns/1ps

import cordic_pkg::*;

module cordic_sincos #(
  parameter bit APPLY_GAIN = 1
) (
  input  logic        clk,
  input  logic        rst_n,

  // Input
  input  logic                          s_valid,
  output logic                          s_ready,
  input  logic signed [DATA_WIDTH-1:0]  s_theta,  // angle
  input  logic [7:0]                    s_tag,

  // Output
  output logic                          m_valid,
  input  logic                          m_ready,
  output logic signed [DATA_WIDTH-1:0]  m_cos,
  output logic signed [DATA_WIDTH-1:0]  m_sin,
  output logic signed [DATA_WIDTH-1:0]  m_theta_residual, // z_out (should be ~0)
  output logic [7:0]                    m_tag,
  output logic                          m_sat
);

  // -------------------------------------------------------------------------
  // Compute the appropriate x input at elaboration time
  //
  //   APPLY_GAIN=1: x_in = 1.0 (CORDIC gain is corrected in post-proc)
  //   APPLY_GAIN=0: x_in = K_inv (pre-compensates for CORDIC gain)
  //
  // Both give the same output; APPLY_GAIN=1 is simpler to use.
  // -------------------------------------------------------------------------
  localparam logic signed [DATA_WIDTH-1:0] UNITY =
      DATA_WIDTH'(longint'(1) << FRAC_BITS);           // 1.0 in Q format

  localparam logic signed [DATA_WIDTH-1:0] KINV_INPUT =
      DATA_WIDTH'(circ_kinv_q()[DATA_WIDTH-1:0]);       // K_inv in Q format

  localparam logic signed [DATA_WIDTH-1:0] X_IN =
      APPLY_GAIN ? UNITY : KINV_INPUT;

  // -------------------------------------------------------------------------
  // CORDIC core — hardwired for circular rotation
  // -------------------------------------------------------------------------
  cordic_top #(
    .APPLY_GAIN(APPLY_GAIN)
  ) u_cordic (
    .clk     (clk),
    .rst_n   (rst_n),
    .s_valid (s_valid),
    .s_ready (s_ready),
    .s_x     (X_IN),       // constant: 1.0 or K_inv
    .s_y     ('0),          // y = 0
    .s_z     (s_theta),     // angle input
    .s_coord (CIRC),        // circular coordinate system
    .s_mode  (ROTATION),    // rotation mode: z→0
    .s_tag   (s_tag),
    .m_valid (m_valid),
    .m_ready (m_ready),
    .m_x     (m_cos),       // cos(θ)
    .m_y     (m_sin),       // sin(θ)
    .m_z     (m_theta_residual),
    .m_tag   (m_tag),
    .m_sat   (m_sat)
  );

endmodule : cordic_sincos
