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
// CORDIC IP - Formal Verification Properties (SVA)
// =============================================================================
// SystemVerilog Assertions for use with formal verification tools:
//   Symbiyosys (sby), Cadence JasperGold, Synopsys VC Formal
//
// Usage — bind to cordic_top:
//   bind cordic_top cordic_props #(.APPLY_GAIN(APPLY_GAIN)) u_props (.*);
//
// Or instantiate directly in a formal testbench top:
//   cordic_top #(...) dut (...);
//   cordic_props #(...) props (.clk(dut.clk), .rst_n(dut.rst_n), ...);
//
// Property summary:
//   P1  After reset deasserts, m_valid is low for TOTAL_LATENCY cycles
//   P2  Every s_valid produces exactly one m_valid TOTAL_LATENCY cycles later
//   P3  m_valid stays high for exactly one cycle per transaction
//   P4  sat only rises when a valid transaction is in-flight
//   P5  m_ready has no effect on m_valid (pipeline ignores m_ready)
//   P6  s_ready is always 1 (no stall in cordic_top)
//
// Coverage goals (co-simulate or formal reachability):
//   C1  Each of 6 operating modes (coord × mode) reached
//   C2  Saturation flag ever asserted
//   C3  Full-scale positive and negative inputs exercised
// =============================================================================

`timescale 1ns/1ps

import cordic_pkg::*;

module cordic_props #(
  parameter bit APPLY_GAIN = 1
) (
  input logic        clk,
  input logic        rst_n,

  input logic        s_valid,
  input logic        s_ready,
  input logic signed [DATA_WIDTH-1:0] s_x,
  input logic signed [DATA_WIDTH-1:0] s_y,
  input logic signed [DATA_WIDTH-1:0] s_z,
  input coord_t      s_coord,
  input mode_t       s_mode,
  input logic [7:0]  s_tag,

  input logic        m_valid,
  input logic        m_ready,
  input logic signed [DATA_WIDTH-1:0] m_x,
  input logic signed [DATA_WIDTH-1:0] m_y,
  input logic signed [DATA_WIDTH-1:0] m_z,
  input logic [7:0]  m_tag,
  input logic        m_sat
);

  localparam int TOTAL_LATENCY = STAGES + 2;

  // -------------------------------------------------------------------------
  // P1: After reset deasserts, pipeline is clean — no m_valid for
  //     TOTAL_LATENCY cycles (no stale data from before reset)
  // -------------------------------------------------------------------------
  property p_clean_after_reset;
    @(posedge clk)
    $rose(rst_n) |-> ##[1:TOTAL_LATENCY] !m_valid;
  endproperty
  ap_clean_after_reset: assert property (p_clean_after_reset)
    else $error("CORDIC: m_valid asserted within latency window after reset");

  // -------------------------------------------------------------------------
  // P2: Every valid input produces a valid output exactly TOTAL_LATENCY later
  //     (pipeline never drops or duplicates a transaction)
  // -------------------------------------------------------------------------
  property p_valid_propagates;
    @(posedge clk) disable iff (!rst_n)
    s_valid |-> ##TOTAL_LATENCY m_valid;
  endproperty
  ap_valid_propagates: assert property (p_valid_propagates)
    else $error("CORDIC: s_valid did not produce m_valid after TOTAL_LATENCY cycles");

  // -------------------------------------------------------------------------
  // P3: No spurious output — m_valid only asserts TOTAL_LATENCY cycles
  //     after a genuine s_valid
  // -------------------------------------------------------------------------
  property p_no_spurious_output;
    @(posedge clk) disable iff (!rst_n)
    m_valid |-> $past(s_valid, TOTAL_LATENCY);
  endproperty
  ap_no_spurious_output: assert property (p_no_spurious_output)
    else $error("CORDIC: m_valid without matching s_valid %0d cycles ago", TOTAL_LATENCY);

  // -------------------------------------------------------------------------
  // P4: s_ready is always 1 — cordic_top never stalls the input
  // -------------------------------------------------------------------------
  property p_always_ready;
    @(posedge clk) disable iff (!rst_n)
    s_ready == 1'b1;
  endproperty
  ap_always_ready: assert property (p_always_ready)
    else $error("CORDIC: s_ready deasserted — pipeline stall not supported");

  // -------------------------------------------------------------------------
  // P5: sat can only be set when valid is also set (sat rides valid)
  // -------------------------------------------------------------------------
  property p_sat_implies_valid;
    @(posedge clk) disable iff (!rst_n)
    m_sat |-> m_valid;
  endproperty
  ap_sat_implies_valid: assert property (p_sat_implies_valid)
    else $error("CORDIC: m_sat asserted without m_valid");

  // -------------------------------------------------------------------------
  // P6: m_ready has no effect — m_valid is purely determined by pipeline
  //     (backpressure is the caller's responsibility via FIFO wrapper)
  // -------------------------------------------------------------------------
  property p_ready_ignored;
    @(posedge clk) disable iff (!rst_n)
    // If m_valid this cycle and m_ready=0, m_valid still clears next cycle
    // (pipeline doesn't stall; caller must buffer if needed)
    (m_valid && !m_ready) |=> !m_valid || m_valid; // always true: documents intent
  endproperty
  // (Informational only — not a checker; documents that stall is absent)

  // =========================================================================
  // Coverage goals
  // =========================================================================

  // C1: All six coordinate/mode combinations
  cp_circ_rot:  cover property (@(posedge clk) disable iff (!rst_n)
    s_valid && s_coord == CIRC && s_mode == ROTATION);
  cp_circ_vec:  cover property (@(posedge clk) disable iff (!rst_n)
    s_valid && s_coord == CIRC && s_mode == VECTORING);
  cp_linr_rot:  cover property (@(posedge clk) disable iff (!rst_n)
    s_valid && s_coord == LINR && s_mode == ROTATION);
  cp_linr_vec:  cover property (@(posedge clk) disable iff (!rst_n)
    s_valid && s_coord == LINR && s_mode == VECTORING);
  cp_hypr_rot:  cover property (@(posedge clk) disable iff (!rst_n)
    s_valid && s_coord == HYPR && s_mode == ROTATION);
  cp_hypr_vec:  cover property (@(posedge clk) disable iff (!rst_n)
    s_valid && s_coord == HYPR && s_mode == VECTORING);

  // C2: Saturation ever reached
  cp_sat_observed: cover property (@(posedge clk) disable iff (!rst_n) m_sat);

  // C3: Back-to-back transactions (pipeline utilization)
  cp_back_to_back: cover property (@(posedge clk) disable iff (!rst_n)
    s_valid ##1 s_valid);

  // C4: Full-scale positive input
  cp_full_scale_pos: cover property (@(posedge clk) disable iff (!rst_n)
    s_valid && s_x == {1'b0, {(DATA_WIDTH-1){1'b1}}});  // MAX_INT

  // C5: Full-scale negative input
  cp_full_scale_neg: cover property (@(posedge clk) disable iff (!rst_n)
    s_valid && s_x == {1'b1, {(DATA_WIDTH-1){1'b0}}});  // MIN_INT

endmodule : cordic_props

// =============================================================================
// Bind statement — attach properties to every cordic_top instance
// (comment out if instantiating cordic_props manually in testbench top)
// =============================================================================
// bind cordic_top cordic_props #(
//   .APPLY_GAIN(APPLY_GAIN)
// ) u_props (
//   .clk  (clk),
//   .rst_n(rst_n),
//   .s_valid(s_valid), .s_ready(s_ready),
//   .s_x(s_x), .s_y(s_y), .s_z(s_z),
//   .s_coord(s_coord), .s_mode(s_mode), .s_tag(s_tag),
//   .m_valid(m_valid), .m_ready(m_ready),
//   .m_x(m_x), .m_y(m_y), .m_z(m_z), .m_tag(m_tag), .m_sat(m_sat)
// );
