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
// Receives input transactions from in_ap and output from out_ap.
// Calls DPI-C golden model (Python via shared lib, or native C model).
// Compares DUT output vs golden output within error tolerance.
// =============================================================================

`ifndef CORDIC_SCOREBOARD_SV
`define CORDIC_SCOREBOARD_SV

import cordic_pkg::*;

// DPI-C golden model function
// Implemented in model/cordic_dpi.c (compiled as shared library)
import "DPI-C" function void cordic_golden(
  input  int  x_in,
  input  int  y_in,
  input  int  z_in,
  input  int  coord,    // 0=CIRC 1=LINR 2=HYPR
  input  int  mode,     // 0=ROT  1=VEC
  output int  x_out,
  output int  y_out,
  output int  z_out
);

class cordic_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(cordic_scoreboard)

  // Input analysis FIFO (stores expected inputs ordered by tag)
  uvm_tlm_analysis_fifo #(cordic_seq_item) in_fifo;
  // Output analysis export
  uvm_analysis_export   #(cordic_seq_item) out_ap;

  // Statistics
  int unsigned pass_cnt, fail_cnt, total_cnt;

  // Error tolerance (LSBs) - CORDIC has quantization error
  localparam int TOL_LSB = 4;   // ±4 LSB tolerance

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    in_fifo = new("in_fifo", this);
    out_ap  = new("out_ap", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    // out_ap is connected from output monitor in env
  endfunction

  task run_phase(uvm_phase phase);
    cordic_seq_item in_item, out_item;
    int gx, gy, gz;

    forever begin
      // Get output from DUT
      out_ap.get(out_item);  // blocks until output arrives

      // Get corresponding input (in order)
      in_fifo.get(in_item);

      // Compute golden reference
      cordic_golden(
        int'(in_item.x_in),
        int'(in_item.y_in),
        int'(in_item.z_in),
        int'(in_item.coord),
        int'(in_item.mode),
        gx, gy, gz
      );

      // Compare
      total_cnt++;
      begin
        int ex = $abs($signed(out_item.x_out) - gx);
        int ey = $abs($signed(out_item.y_out) - gy);
        int ez = $abs($signed(out_item.z_out) - gz);

        if (ex > TOL_LSB || ey > TOL_LSB || ez > TOL_LSB) begin
          fail_cnt++;
          `uvm_error("SCOREBOARD",
            $sformatf("MISMATCH tag=%0d coord=%s mode=%s\n  IN:    x=%0d y=%0d z=%0d\n  DUT:   x=%0d y=%0d z=%0d\n  GOLD:  x=%0d y=%0d z=%0d\n  ERROR: ex=%0d ey=%0d ez=%0d (tol=%0d)",
              in_item.tag, in_item.coord.name(), in_item.mode.name(),
              $signed(in_item.x_in), $signed(in_item.y_in), $signed(in_item.z_in),
              $signed(out_item.x_out), $signed(out_item.y_out), $signed(out_item.z_out),
              gx, gy, gz, ex, ey, ez, TOL_LSB))
        end else begin
          pass_cnt++;
          `uvm_info("SCOREBOARD",
            $sformatf("PASS tag=%0d | DUT x=%0d y=%0d z=%0d | GOLD x=%0d y=%0d z=%0d",
              in_item.tag, $signed(out_item.x_out), $signed(out_item.y_out),
              $signed(out_item.z_out), gx, gy, gz), UVM_HIGH)
        end
      end
    end
  endtask

  function void report_phase(uvm_phase phase);
    `uvm_info("SCOREBOARD", $sformatf(
      "\n=== CORDIC SCOREBOARD SUMMARY ===\n  TOTAL : %0d\n  PASS  : %0d\n  FAIL  : %0d\n=================================",
      total_cnt, pass_cnt, fail_cnt), UVM_NONE)
    if (fail_cnt > 0)
      `uvm_error("SCOREBOARD", "TEST FAILED - mismatches detected")
  endfunction

endclass : cordic_scoreboard

`endif
