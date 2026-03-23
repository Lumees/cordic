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
// CORDIC IP - AXI4-Stream Interface Wrapper
// =============================================================================
// TDATA packing (little-endian, DATA_WIDTH bits per field):
//
//   Slave  TDATA [4*DATA_WIDTH-1:0]:
//     [DATA_WIDTH-1:0]           = x
//     [2*DATA_WIDTH-1:DATA_WIDTH] = y
//     [3*DATA_WIDTH-1:2*DATA_WIDTH] = z
//     [3*DATA_WIDTH+7:3*DATA_WIDTH] = tag[7:0]
//     [3*DATA_WIDTH+8]              = mode  (0=ROT, 1=VEC)
//     [3*DATA_WIDTH+10:3*DATA_WIDTH+9] = coord (0=CIRC,1=LINR,2=HYPR)
//     [3*DATA_WIDTH+14:3*DATA_WIDTH+11] = reserved
//
//   Master TDATA [3*DATA_WIDTH-1:0]:
//     [DATA_WIDTH-1:0]           = x_out
//     [2*DATA_WIDTH-1:DATA_WIDTH] = y_out
//     [3*DATA_WIDTH-1:2*DATA_WIDTH] = z_out
//   Master TUSER [7:0]  = tag_out
//   Master m_axis_tsat  = saturation/overflow flag (NEW)
//
// For backward compatibility with DATA_WIDTH=16:
//   Slave  TDATA remains 64-bit (bit [63:48] = reserved)
//   Master TDATA remains 48-bit
//
// Output FIFO depth: configurable via FIFO_DEPTH parameter (default 16).
// =============================================================================

`timescale 1ns/1ps

import cordic_pkg::*;

module cordic_axis #(
  parameter bit APPLY_GAIN   = 1,
  parameter int FIFO_DEPTH   = 16
) (
  input  logic        clk,
  input  logic        rst_n,

  // AXI4-Stream Slave (input)
  input  logic        s_axis_tvalid,
  output logic        s_axis_tready,
  input  logic [63:0] s_axis_tdata,   // fixed 64-bit for WB compat; upper bits reserved
  input  logic        s_axis_tlast,

  // AXI4-Stream Master (output)
  output logic                    m_axis_tvalid,
  input  logic                    m_axis_tready,
  output logic [3*DATA_WIDTH-1:0] m_axis_tdata,   // {z_out, y_out, x_out}
  output logic [7:0]              m_axis_tuser,   // tag_out
  output logic                    m_axis_tlast,
  output logic                    m_axis_tsat     // saturation flag (NEW)
);

  // Unpack slave TDATA (using fixed bit positions compatible with legacy 64-bit layout)
  logic signed [DATA_WIDTH-1:0] s_x, s_y, s_z;
  logic [7:0]                    s_tag;
  mode_t                          s_mode;
  coord_t                         s_coord;

  assign s_x     = s_axis_tdata[DATA_WIDTH-1:0];
  assign s_y     = s_axis_tdata[2*DATA_WIDTH-1:DATA_WIDTH];
  assign s_z     = s_axis_tdata[3*DATA_WIDTH-1:2*DATA_WIDTH];
  assign s_tag   = s_axis_tdata[55:48];
  assign s_mode  = mode_t'(s_axis_tdata[56]);
  assign s_coord = coord_t'(s_axis_tdata[58:57]);

  // CORDIC core signals
  logic                          core_out_valid;
  logic signed [DATA_WIDTH-1:0] core_x, core_y, core_z;
  logic [7:0]                    core_tag;
  logic                          core_sat;

  // Propagate tlast through pipeline (TOTAL_LATENCY cycles)
  localparam int TOTAL_LATENCY = STAGES + 2;
  logic tlast_pipe [0:TOTAL_LATENCY];

  assign tlast_pipe[0] = s_axis_tlast;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      for (int i = 1; i <= TOTAL_LATENCY; i++) tlast_pipe[i] <= 1'b0;
    else
      for (int i = 1; i <= TOTAL_LATENCY; i++) tlast_pipe[i] <= tlast_pipe[i-1];
  end

  // CORDIC top instance
  cordic_top #(
    .APPLY_GAIN(APPLY_GAIN)
  ) u_cordic (
    .clk     (clk),
    .rst_n   (rst_n),
    .s_valid (s_axis_tvalid),
    .s_ready (s_axis_tready),
    .s_x     (s_x),
    .s_y     (s_y),
    .s_z     (s_z),
    .s_coord (s_coord),
    .s_mode  (s_mode),
    .s_tag   (s_tag),
    .m_valid (core_out_valid),
    .m_ready (1'b1),          // FIFO absorbs backpressure
    .m_x     (core_x),
    .m_y     (core_y),
    .m_z     (core_z),
    .m_tag   (core_tag),
    .m_sat   (core_sat)
  );

  // -------------------------------------------------------------------------
  // Output FIFO — synchronous, stores {sat, tlast, tag[7:0], z, y, x}
  //   Width: 3*DATA_WIDTH + 10 bits
  //     [3*DW-1:0]     = {z, y, x}
  //     [3*DW+7:3*DW]  = tag
  //     [3*DW+8]       = tlast
  //     [3*DW+9]       = sat
  // -------------------------------------------------------------------------
  localparam int FIFO_W  = 3*DATA_WIDTH + 10;
  localparam int FIFO_AW = $clog2(FIFO_DEPTH);

  logic [FIFO_W-1:0]    fifo_mem [0:FIFO_DEPTH-1];
  logic [FIFO_AW:0]      fifo_wptr, fifo_rptr;
  logic                   fifo_empty, fifo_full;
  logic [FIFO_W-1:0]    fifo_din, fifo_dout;

  assign fifo_din   = {core_sat,
                       tlast_pipe[TOTAL_LATENCY],
                       core_tag,
                       core_z, core_y, core_x};
  assign fifo_empty = (fifo_wptr == fifo_rptr);
  assign fifo_full  = (fifo_wptr[FIFO_AW-1:0] == fifo_rptr[FIFO_AW-1:0]) &&
                      (fifo_wptr[FIFO_AW]      != fifo_rptr[FIFO_AW]);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fifo_wptr <= '0;
      fifo_rptr <= '0;
    end else begin
      if (core_out_valid && !fifo_full) begin
        fifo_mem[fifo_wptr[FIFO_AW-1:0]] <= fifo_din;
        fifo_wptr <= fifo_wptr + 1;
      end
      if (m_axis_tvalid && m_axis_tready) begin
        fifo_rptr <= fifo_rptr + 1;
      end
    end
  end

  assign fifo_dout = fifo_mem[fifo_rptr[FIFO_AW-1:0]];

  assign m_axis_tvalid = !fifo_empty;
  assign m_axis_tdata  = fifo_dout[3*DATA_WIDTH-1:0];
  assign m_axis_tuser  = fifo_dout[3*DATA_WIDTH+7:3*DATA_WIDTH];
  assign m_axis_tlast  = fifo_dout[3*DATA_WIDTH+8];
  assign m_axis_tsat   = fifo_dout[3*DATA_WIDTH+9];

endmodule : cordic_axis
