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
// CORDIC IP - AXI4-Lite Interface Wrapper
// =============================================================================
// Register map (32-bit word address, 4-byte aligned):
//
//  Offset  Name        Access  Description
//  0x00    CTRL        R/W     [0]=start,  [1]=busy(RO),  [2]=done(RO)
//                              [3]=sat(RO) — sticky overflow flag, cleared on next start
//                              [5:4]=coord (0=CIRC,1=LINR,2=HYPR)
//                              [6]=mode    (0=ROT,1=VEC)
//  0x04    TAG         R/W     [7:0]=transaction tag
//  0x08    X_IN        R/W     Signed DATA_WIDTH-bit input x (lower DATA_WIDTH bits)
//  0x0C    Y_IN        R/W     Signed DATA_WIDTH-bit input y
//  0x10    Z_IN        R/W     Signed DATA_WIDTH-bit input z (angle)
//  0x14    X_OUT       RO      Signed DATA_WIDTH-bit output x
//  0x18    Y_OUT       RO      Signed DATA_WIDTH-bit output y
//  0x1C    Z_OUT       RO      Signed DATA_WIDTH-bit output z
//  0x20    LATENCY     RO      Pipeline latency in cycles (= STAGES+2)
//  0x24    VERSION     RO      IP version [31:16]=major [15:0]=minor
//
// Write CTRL[0]=1 to trigger a single computation.
// Poll CTRL[2] (done) to read result, or wait for irq pulse.
// Done auto-clears on next start.  CTRL[3] (sat) auto-clears on next start.
//
// irq (NEW): single-cycle output pulse when done transitions 0→1.
// =============================================================================

`timescale 1ns/1ps

import cordic_pkg::*;

module cordic_axil #(
  parameter bit APPLY_GAIN  = 1,
  parameter int IP_VERSION  = 32'h0001_0000  // v1.0
) (
  input  logic        clk,
  input  logic        rst_n,

  // AXI4-Lite Slave
  input  logic [31:0] s_axil_awaddr,
  input  logic        s_axil_awvalid,
  output logic        s_axil_awready,
  input  logic [31:0] s_axil_wdata,
  input  logic [3:0]  s_axil_wstrb,
  input  logic        s_axil_wvalid,
  output logic        s_axil_wready,
  output logic [1:0]  s_axil_bresp,
  output logic        s_axil_bvalid,
  input  logic        s_axil_bready,
  input  logic [31:0] s_axil_araddr,
  input  logic        s_axil_arvalid,
  output logic        s_axil_arready,
  output logic [31:0] s_axil_rdata,
  output logic [1:0]  s_axil_rresp,
  output logic        s_axil_rvalid,
  input  logic        s_axil_rready,

  // Interrupt — single-cycle pulse when done (CTRL[2]) rises
  output logic        irq
);

  localparam int TOTAL_LATENCY = STAGES + 2;

  // -------------------------------------------------------------------------
  // Internal registers
  // -------------------------------------------------------------------------
  logic [7:0]                    reg_ctrl;
  logic [7:0]                    reg_tag;
  logic signed [DATA_WIDTH-1:0]  reg_x_in, reg_y_in, reg_z_in;
  logic signed [DATA_WIDTH-1:0]  reg_x_out, reg_y_out, reg_z_out;

  coord_t  cordic_coord;
  mode_t   cordic_mode;
  assign cordic_coord = coord_t'(reg_ctrl[5:4]);
  assign cordic_mode  = mode_t'(reg_ctrl[6]);

  // -------------------------------------------------------------------------
  // CORDIC engine
  // -------------------------------------------------------------------------
  logic                          core_in_valid, core_out_valid;
  logic signed [DATA_WIDTH-1:0]  core_x_out, core_y_out, core_z_out;
  logic [7:0]                    core_tag_out;
  logic                          core_sat_out;

  cordic_top #(.APPLY_GAIN(APPLY_GAIN)) u_cordic (
    .clk     (clk),
    .rst_n   (rst_n),
    .s_valid (core_in_valid),
    .s_ready (),          // ignored: single-shot
    .s_x     (reg_x_in),
    .s_y     (reg_y_in),
    .s_z     (reg_z_in),
    .s_coord (cordic_coord),
    .s_mode  (cordic_mode),
    .s_tag   (reg_tag),
    .m_valid (core_out_valid),
    .m_ready (1'b1),
    .m_x     (core_x_out),
    .m_y     (core_y_out),
    .m_z     (core_z_out),
    .m_tag   (core_tag_out),
    .m_sat   (core_sat_out)
  );

  // -------------------------------------------------------------------------
  // Control state machine
  // -------------------------------------------------------------------------
  typedef enum logic {
    S_IDLE = 1'b0,
    S_RUN  = 1'b1
  } state_t;
  state_t state;

  // core_in_valid: one-cycle pulse on the cycle AFTER start is triggered
  logic core_started;
  assign core_in_valid = core_started;

  // -------------------------------------------------------------------------
  // AXI4-Lite write path + FSM — single always_ff block drives reg_ctrl
  // -------------------------------------------------------------------------
  logic [5:0]  wr_addr;
  logic [31:0] wdata_lat;
  logic        aw_active, w_active;

  assign s_axil_awready = !aw_active;
  assign s_axil_wready  = !w_active;
  assign s_axil_bresp   = 2'b00;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      aw_active     <= 1'b0;
      w_active      <= 1'b0;
      wr_addr       <= '0;
      wdata_lat     <= '0;
      s_axil_bvalid <= 1'b0;
      reg_ctrl      <= '0;
      reg_tag       <= '0;
      reg_x_in      <= '0;
      reg_y_in      <= '0;
      reg_z_in      <= '0;
      reg_x_out     <= '0;
      reg_y_out     <= '0;
      reg_z_out     <= '0;
      state         <= S_IDLE;
      core_started  <= 1'b0;
    end else begin
      // ── AXI4-Lite write handshake ──────────────────────────────────────
      if (s_axil_awvalid && s_axil_awready) begin
        wr_addr   <= s_axil_awaddr[7:2];
        aw_active <= 1'b1;
      end
      if (s_axil_wvalid && s_axil_wready) begin
        wdata_lat <= s_axil_wdata;
        w_active  <= 1'b1;
      end
      if (s_axil_bvalid && s_axil_bready)
        s_axil_bvalid <= 1'b0;

      // ── Non-CTRL register writes (any FSM state) ───────────────────────
      if (aw_active && w_active && (wr_addr != 6'h00)) begin
        aw_active     <= 1'b0;
        w_active      <= 1'b0;
        s_axil_bvalid <= 1'b1;
        unique case (wr_addr)
          6'h01: reg_tag  <= wdata_lat[7:0];
          6'h02: reg_x_in <= wdata_lat[DATA_WIDTH-1:0];
          6'h03: reg_y_in <= wdata_lat[DATA_WIDTH-1:0];
          6'h04: reg_z_in <= wdata_lat[DATA_WIDTH-1:0];
          default: ;
        endcase
      end

      // ── FSM + CTRL register (sole driver of reg_ctrl) ─────────────────
      core_started <= 1'b0;   // default: de-assert each cycle

      unique case (state)
        S_IDLE: begin
          reg_ctrl[1] <= 1'b0;
          if (aw_active && w_active && (wr_addr == 6'h00)) begin
            aw_active     <= 1'b0;
            w_active      <= 1'b0;
            s_axil_bvalid <= 1'b1;
            reg_ctrl      <= wdata_lat[7:0];
            if (wdata_lat[0]) begin
              reg_ctrl[0]  <= 1'b0;   // auto-clear start
              reg_ctrl[1]  <= 1'b1;   // set busy
              reg_ctrl[2]  <= 1'b0;   // clear done
              reg_ctrl[3]  <= 1'b0;   // clear sat on new start
              core_started <= 1'b1;   // pulse core_in_valid next cycle
              state        <= S_RUN;
            end
          end
        end

        S_RUN: begin
          if (core_out_valid) begin
            reg_x_out   <= core_x_out;
            reg_y_out   <= core_y_out;
            reg_z_out   <= core_z_out;
            reg_ctrl[1] <= 1'b0;              // clear busy
            reg_ctrl[2] <= 1'b1;              // set done
            reg_ctrl[3] <= core_sat_out;      // capture saturation flag
            state       <= S_IDLE;
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Interrupt: single-cycle pulse when done bit (CTRL[2]) rises 0→1
  // -------------------------------------------------------------------------
  logic done_prev;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done_prev <= 1'b0;
      irq       <= 1'b0;
    end else begin
      done_prev <= reg_ctrl[2];
      irq       <= reg_ctrl[2] & ~done_prev;
    end
  end

  // -------------------------------------------------------------------------
  // AXI4-Lite read logic
  // -------------------------------------------------------------------------
  assign s_axil_rresp = 2'b00;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_axil_arready <= 1'b1;
      s_axil_rvalid  <= 1'b0;
      s_axil_rdata   <= '0;
    end else begin
      if (s_axil_arvalid && s_axil_arready) begin
        s_axil_arready <= 1'b0;
        s_axil_rvalid  <= 1'b1;
        unique case (s_axil_araddr[7:2])
          6'h00: s_axil_rdata <= {24'h0, reg_ctrl};
          6'h01: s_axil_rdata <= {24'h0, reg_tag};
          6'h02: s_axil_rdata <= {{(32-DATA_WIDTH){reg_x_in[DATA_WIDTH-1]}}, reg_x_in};
          6'h03: s_axil_rdata <= {{(32-DATA_WIDTH){reg_y_in[DATA_WIDTH-1]}}, reg_y_in};
          6'h04: s_axil_rdata <= {{(32-DATA_WIDTH){reg_z_in[DATA_WIDTH-1]}}, reg_z_in};
          6'h05: s_axil_rdata <= {{(32-DATA_WIDTH){reg_x_out[DATA_WIDTH-1]}}, reg_x_out};
          6'h06: s_axil_rdata <= {{(32-DATA_WIDTH){reg_y_out[DATA_WIDTH-1]}}, reg_y_out};
          6'h07: s_axil_rdata <= {{(32-DATA_WIDTH){reg_z_out[DATA_WIDTH-1]}}, reg_z_out};
          6'h08: s_axil_rdata <= TOTAL_LATENCY;
          6'h09: s_axil_rdata <= IP_VERSION;
          default: s_axil_rdata <= 32'hDEAD_BEEF;
        endcase
      end
      if (s_axil_rvalid && s_axil_rready) begin
        s_axil_rvalid  <= 1'b0;
        s_axil_arready <= 1'b1;
      end
    end
  end

endmodule : cordic_axil
