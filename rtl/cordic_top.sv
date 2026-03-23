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
// CORDIC IP - Top Level (Internal datapath, no bus interface)
// =============================================================================
// Combines pre-processing, pipelined CORDIC core, and post-processing.
// Uses a simple valid/ready handshake. Total latency = STAGES + 2 cycles
// (1 pre-proc + STAGES core + 1 post-proc).
//
// New output m_sat: asserted alongside m_valid when the pipeline detected
// a saturation/overflow event in either the pre-processing or post-processing
// stage.  The flag is sticky within the transaction (OR across pre and post).
//
// Bus interfaces (AXI4-Lite, AXI4-Stream, Wishbone) are separate wrappers
// that instantiate this module.
// =============================================================================

`timescale 1ns/1ps

import cordic_pkg::*;

module cordic_top #(
  parameter bit APPLY_GAIN = 1
) (
  input  logic         clk,
  input  logic         rst_n,

  // Input handshake
  input  logic                          s_valid,
  output logic                          s_ready,
  input  logic signed [DATA_WIDTH-1:0]  s_x,
  input  logic signed [DATA_WIDTH-1:0]  s_y,
  input  logic signed [DATA_WIDTH-1:0]  s_z,
  input  coord_t                        s_coord,
  input  mode_t                         s_mode,
  input  logic [7:0]                    s_tag,

  // Output handshake
  output logic                          m_valid,
  input  logic                          m_ready,
  output logic signed [DATA_WIDTH-1:0]  m_x,
  output logic signed [DATA_WIDTH-1:0]  m_y,
  output logic signed [DATA_WIDTH-1:0]  m_z,
  output logic [7:0]                    m_tag,
  output logic                          m_sat    // saturation/overflow flag
);

  // TOTAL pipeline latency in clock cycles (pre + core + post)
  localparam int TOTAL_LATENCY = STAGES + 2;

  // Pipeline is always running (no stall support - backpressure drops output)
  // For full backpressure, use AXI4-Stream wrapper which adds a FIFO.
  assign s_ready = 1'b1;

  // -------------------------------------------------------------------------
  // Pack input bundle
  // -------------------------------------------------------------------------
  cordic_data_t pre_in;
  assign pre_in.sat   = 1'b0;   // no saturation on input; detected downstream
  assign pre_in.x     = s_x;
  assign pre_in.y     = s_y;
  assign pre_in.z     = s_z;
  assign pre_in.coord = s_coord;
  assign pre_in.mode  = s_mode;
  assign pre_in.tag   = s_tag;
  assign pre_in.valid = s_valid;

  // -------------------------------------------------------------------------
  // Pre-processing
  // -------------------------------------------------------------------------
  cordic_data_t                 pre_out;
  logic signed [DATA_WIDTH-1:0] quad_off_raw;
  logic                          quad_off_valid_raw;

  cordic_pre_proc u_pre (
    .clk              (clk),
    .rst_n            (rst_n),
    .din              (pre_in),
    .dout             (pre_out),
    .quad_offset       (quad_off_raw),
    .quad_offset_valid (quad_off_valid_raw)
  );

  // -------------------------------------------------------------------------
  // Delay quadrant offset to match CORDIC core latency (STAGES cycles)
  // -------------------------------------------------------------------------
  logic signed [DATA_WIDTH-1:0] quad_off_dly [0:STAGES-1];
  logic                          quad_off_vld_dly [0:STAGES-1];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < STAGES; i++) begin
        quad_off_dly[i]     <= '0;
        quad_off_vld_dly[i] <= 1'b0;
      end
    end else begin
      quad_off_dly[0]     <= quad_off_raw;
      quad_off_vld_dly[0] <= quad_off_valid_raw;
      for (int i = 1; i < STAGES; i++) begin
        quad_off_dly[i]     <= quad_off_dly[i-1];
        quad_off_vld_dly[i] <= quad_off_vld_dly[i-1];
      end
    end
  end

  // -------------------------------------------------------------------------
  // CORDIC core pipeline
  // -------------------------------------------------------------------------
  cordic_data_t core_out;

  cordic_core u_core (
    .clk   (clk),
    .rst_n (rst_n),
    .din   (pre_out),
    .dout  (core_out)
  );

  // -------------------------------------------------------------------------
  // Post-processing
  // -------------------------------------------------------------------------
  cordic_data_t post_out;

  cordic_post_proc #(
    .APPLY_GAIN(APPLY_GAIN)
  ) u_post (
    .clk               (clk),
    .rst_n             (rst_n),
    .din               (core_out),
    .quad_offset        (quad_off_dly[STAGES-1]),
    .quad_offset_valid  (quad_off_vld_dly[STAGES-1]),
    .dout              (post_out)
  );

  // -------------------------------------------------------------------------
  // Output unpack
  // -------------------------------------------------------------------------
  assign m_valid = post_out.valid;
  assign m_x     = post_out.x;
  assign m_y     = post_out.y;
  assign m_z     = post_out.z;
  assign m_tag   = post_out.tag;
  assign m_sat   = post_out.sat;

endmodule : cordic_top
