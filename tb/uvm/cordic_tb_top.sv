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

`include "uvm_macros.svh"
import uvm_pkg::*;
import cordic_pkg::*;

// Include all UVM components
`include "cordic_seq_item.sv"
`include "cordic_sequencer.sv"
`include "cordic_driver.sv"
`include "cordic_monitor.sv"
`include "cordic_scoreboard.sv"
`include "cordic_coverage.sv"
`include "cordic_agent.sv"
`include "cordic_env.sv"
`include "cordic_sequences.sv"
`include "cordic_tests.sv"

module cordic_tb_top;

  // -------------------------------------------------------------------------
  // Clock and reset
  // -------------------------------------------------------------------------
  logic clk, rst_n;

  initial clk = 0;
  always #5 clk = ~clk;   // 100 MHz

  initial begin
    rst_n = 0;
    repeat(10) @(posedge clk);
    rst_n = 1;
  end

  // -------------------------------------------------------------------------
  // Interface instantiation
  // -------------------------------------------------------------------------
  cordic_if dut_if (.clk(clk), .rst_n(rst_n));

  // -------------------------------------------------------------------------
  // DUT instantiation
  // -------------------------------------------------------------------------
  cordic_top #(
    .APPLY_GAIN(1)
  ) dut (
    .clk     (clk),
    .rst_n   (rst_n),
    .s_valid (dut_if.s_valid),
    .s_ready (dut_if.s_ready),
    .s_x     (dut_if.s_x),
    .s_y     (dut_if.s_y),
    .s_z     (dut_if.s_z),
    .s_coord (dut_if.s_coord),
    .s_mode  (dut_if.s_mode),
    .s_tag   (dut_if.s_tag),
    .m_valid (dut_if.m_valid),
    .m_ready (dut_if.m_ready),
    .m_x     (dut_if.m_x),
    .m_y     (dut_if.m_y),
    .m_z     (dut_if.m_z),
    .m_tag   (dut_if.m_tag)
  );

  // -------------------------------------------------------------------------
  // UVM start
  // -------------------------------------------------------------------------
  initial begin
    // Register interface with config DB
    uvm_config_db #(virtual cordic_if.driver_mp)::set(
      null, "uvm_test_top.env.agent.driver", "vif", dut_if);
    uvm_config_db #(virtual cordic_if.monitor_mp)::set(
      null, "uvm_test_top.env.agent.in_monitor",  "vif", dut_if);
    uvm_config_db #(virtual cordic_if.monitor_mp)::set(
      null, "uvm_test_top.env.agent.out_monitor", "vif", dut_if);

    // Start UVM test (name from +UVM_TESTNAME command line)
    run_test();
  end

  // -------------------------------------------------------------------------
  // Waveform dump
  // -------------------------------------------------------------------------
  initial begin
    $dumpfile("cordic_uvm.vcd");
    $dumpvars(0, cordic_tb_top);
  end

  // -------------------------------------------------------------------------
  // Timeout watchdog
  // -------------------------------------------------------------------------
  initial begin
    #10_000_000;
    `uvm_fatal("TIMEOUT", "Simulation timeout at 10ms")
  end

endmodule : cordic_tb_top
