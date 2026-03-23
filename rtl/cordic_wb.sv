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
// CORDIC IP - Wishbone B4 Classic Interface Wrapper
// =============================================================================
// Classic Wishbone (non-pipelined) single master.
// Same register map as AXI4-Lite wrapper (cordic_axil.sv).
//
//  Offset  Name        Access
//  0x00    CTRL        R/W   [0]=start [1]=busy(RO) [2]=done(RO)
//                            [3]=sat(RO) — sticky overflow, cleared on next start
//                            [5:4]=coord [6]=mode
//  0x04    TAG         R/W   [7:0]=tag
//  0x08    X_IN        R/W   DATA_WIDTH-bit signed
//  0x0C    Y_IN        R/W   DATA_WIDTH-bit signed
//  0x10    Z_IN        R/W   DATA_WIDTH-bit signed
//  0x14    X_OUT       RO    DATA_WIDTH-bit signed
//  0x18    Y_OUT       RO    DATA_WIDTH-bit signed
//  0x1C    Z_OUT       RO    DATA_WIDTH-bit signed
//  0x20    LATENCY     RO    pipeline latency
//  0x24    VERSION     RO    IP version
//
// irq (NEW): single-cycle output pulse when done (CTRL[2]) rises 0→1.
//
// Wishbone signals follow WB B4 spec:
//   CLK_I, RST_I, ADR_I, DAT_I, DAT_O, WE_I, SEL_I, STB_I, ACK_O, CYC_I
// =============================================================================

`timescale 1ns/1ps

import cordic_pkg::*;

module cordic_wb #(
  parameter bit APPLY_GAIN = 1,
  parameter int IP_VERSION = 32'h0001_0000
) (
  // Wishbone system
  input  logic        CLK_I,
  input  logic        RST_I,

  // Wishbone slave
  input  logic [31:0] ADR_I,
  input  logic [31:0] DAT_I,
  output logic [31:0] DAT_O,
  input  logic        WE_I,
  input  logic [3:0]  SEL_I,
  input  logic        STB_I,
  input  logic        CYC_I,
  output logic        ACK_O,
  output logic        ERR_O,
  output logic        RTY_O,

  // Interrupt — single-cycle pulse when done (CTRL[2]) rises
  output logic        irq
);

  localparam int TOTAL_LATENCY = STAGES + 2;

  assign ERR_O = 1'b0;
  assign RTY_O = 1'b0;

  // -------------------------------------------------------------------------
  // Registers
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
    .clk     (CLK_I),
    .rst_n   (~RST_I),
    .s_valid (core_in_valid),
    .s_ready (),
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
  typedef enum logic {S_IDLE=0, S_RUN=1} state_t;
  state_t state;

  // core_in_valid: one-cycle pulse on the cycle AFTER start is triggered
  logic core_started;
  assign core_in_valid = core_started;

  // -------------------------------------------------------------------------
  // Wishbone bus logic + FSM — single always_ff block drives reg_ctrl
  // -------------------------------------------------------------------------
  always_ff @(posedge CLK_I) begin
    if (RST_I) begin
      ACK_O    <= 1'b0;
      DAT_O    <= '0;
      reg_ctrl <= '0;
      reg_tag  <= '0;
      reg_x_in <= '0;
      reg_y_in <= '0;
      reg_z_in <= '0;
      reg_x_out    <= '0;
      reg_y_out    <= '0;
      reg_z_out    <= '0;
      state        <= S_IDLE;
      core_started <= 1'b0;
    end else begin
      ACK_O <= 1'b0;

      // ── Wishbone bus transaction ────────────────────────────────────────
      if (CYC_I && STB_I && !ACK_O) begin
        ACK_O <= 1'b1;
        if (WE_I) begin
          // Write: non-CTRL registers handled here
          unique case (ADR_I[5:2])
            4'h1: reg_tag  <= DAT_I[7:0];
            4'h2: reg_x_in <= DAT_I[DATA_WIDTH-1:0];
            4'h3: reg_y_in <= DAT_I[DATA_WIDTH-1:0];
            4'h4: reg_z_in <= DAT_I[DATA_WIDTH-1:0];
            default: ;
          endcase
        end else begin
          // Read
          unique case (ADR_I[5:2])
            4'h0: DAT_O <= {24'h0, reg_ctrl};
            4'h1: DAT_O <= {24'h0, reg_tag};
            4'h2: DAT_O <= {{(32-DATA_WIDTH){reg_x_in[DATA_WIDTH-1]}},  reg_x_in};
            4'h3: DAT_O <= {{(32-DATA_WIDTH){reg_y_in[DATA_WIDTH-1]}},  reg_y_in};
            4'h4: DAT_O <= {{(32-DATA_WIDTH){reg_z_in[DATA_WIDTH-1]}},  reg_z_in};
            4'h5: DAT_O <= {{(32-DATA_WIDTH){reg_x_out[DATA_WIDTH-1]}}, reg_x_out};
            4'h6: DAT_O <= {{(32-DATA_WIDTH){reg_y_out[DATA_WIDTH-1]}}, reg_y_out};
            4'h7: DAT_O <= {{(32-DATA_WIDTH){reg_z_out[DATA_WIDTH-1]}}, reg_z_out};
            4'h8: DAT_O <= TOTAL_LATENCY;
            4'h9: DAT_O <= IP_VERSION;
            default: DAT_O <= 32'hDEAD_BEEF;
          endcase
        end
      end

      // ── FSM + CTRL register (sole driver of reg_ctrl) ──────────────────
      core_started <= 1'b0;   // default: de-assert each cycle

      unique case (state)
        S_IDLE: begin
          reg_ctrl[1] <= 1'b0;
          // CTRL write with start bit: handled here to trigger FSM
          if (CYC_I && STB_I && !ACK_O && WE_I && (ADR_I[5:2] == 4'h0)) begin
            reg_ctrl <= DAT_I[7:0];
            if (DAT_I[0]) begin
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
          // Wait for pipeline output
          if (core_out_valid) begin
            reg_x_out   <= core_x_out;
            reg_y_out   <= core_y_out;
            reg_z_out   <= core_z_out;
            reg_ctrl[1] <= 1'b0;         // clear busy
            reg_ctrl[2] <= 1'b1;         // set done
            reg_ctrl[3] <= core_sat_out; // capture saturation flag
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
  always_ff @(posedge CLK_I) begin
    if (RST_I) begin
      done_prev <= 1'b0;
      irq       <= 1'b0;
    end else begin
      done_prev <= reg_ctrl[2];
      irq       <= reg_ctrl[2] & ~done_prev;
    end
  end

endmodule : cordic_wb
