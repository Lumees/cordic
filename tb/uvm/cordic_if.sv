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

`timescale 1ns/1ps

import cordic_pkg::*;

interface cordic_if (input logic clk, input logic rst_n);

  // DUT ports
  logic                          s_valid;
  logic                          s_ready;
  logic signed [DATA_WIDTH-1:0]  s_x;
  logic signed [DATA_WIDTH-1:0]  s_y;
  logic signed [DATA_WIDTH-1:0]  s_z;
  coord_t                        s_coord;
  mode_t                         s_mode;
  logic [7:0]                    s_tag;

  logic                          m_valid;
  logic                          m_ready;
  logic signed [DATA_WIDTH-1:0]  m_x;
  logic signed [DATA_WIDTH-1:0]  m_y;
  logic signed [DATA_WIDTH-1:0]  m_z;
  logic [7:0]                    m_tag;

  // Driver clocking block
  clocking driver_cb @(posedge clk);
    default input #1 output #1;
    output s_valid, s_x, s_y, s_z, s_coord, s_mode, s_tag;
    input  s_ready;
    output m_ready;
  endclocking

  // Monitor clocking block
  clocking monitor_cb @(posedge clk);
    default input #1;
    input  s_valid, s_x, s_y, s_z, s_coord, s_mode, s_tag;
    input  m_valid, m_x, m_y, m_z, m_tag;
  endclocking

  // Modports
  modport driver_mp  (clocking driver_cb,  input clk, input rst_n);
  modport monitor_mp (clocking monitor_cb, input clk, input rst_n);

  // -------------------------------------------------------------------------
  // Assertions
  // -------------------------------------------------------------------------
  // Output must be valid within STAGES+2+5 cycles of a valid input
  property p_output_eventually_valid;
    @(posedge clk) disable iff (!rst_n)
    $rose(s_valid) |-> ##[STAGES:STAGES+5] m_valid;
  endproperty
  assert property(p_output_eventually_valid)
    else $error("CORDIC output not valid within expected latency");

  // s_ready should not be de-asserted in the basic pipeline (no stall)
  property p_ready_stable;
    @(posedge clk) disable iff (!rst_n)
    s_ready;  // pipeline is always ready
  endproperty
  assert property(p_ready_stable)
    else $warning("s_ready deasserted - possible stall");

endinterface : cordic_if
