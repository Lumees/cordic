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

`ifndef CORDIC_COVERAGE_SV
`define CORDIC_COVERAGE_SV

import cordic_pkg::*;

class cordic_coverage extends uvm_subscriber #(cordic_seq_item);
  `uvm_component_utils(cordic_coverage)

  cordic_seq_item item;

  // -------------------------------------------------------------------------
  // Covergroup: Input space
  // -------------------------------------------------------------------------
  covergroup cg_input;
    cp_coord: coverpoint item.coord {
      bins circ = {CIRC};
      bins linr = {LINR};
      bins hypr = {HYPR};
    }
    cp_mode: coverpoint item.mode {
      bins rotation  = {ROTATION};
      bins vectoring = {VECTORING};
    }
    cp_coord_x_mode: cross cp_coord, cp_mode;

    // x_in sign coverage
    cp_x_sign: coverpoint item.x_in[DATA_WIDTH-1] {
      bins pos = {0};
      bins neg = {1};
    }
    cp_y_sign: coverpoint item.y_in[DATA_WIDTH-1] {
      bins pos = {0};
      bins neg = {1};
    }
    cp_z_sign: coverpoint item.z_in[DATA_WIDTH-1] {
      bins pos = {0};
      bins neg = {1};
    }

    // Quadrant coverage (angle z for circular rotation)
    cp_angle_quad: coverpoint item.z_in[DATA_WIDTH-1:DATA_WIDTH-3] {
      bins q1 = {3'b000};   // 0 to π/4
      bins q2 = {3'b001};   // π/4 to π/2
      bins q3 = {3'b010};   // π/2 to 3π/4
      bins q4 = {3'b011};   // 3π/4 to π
      bins q5 = {3'b111};   // -π/4 to 0
      bins q6 = {3'b110};   // -π/2 to -π/4
      bins q7 = {3'b101};   // -3π/4 to -π/2
      bins q8 = {3'b100};   // -π to -3π/4
    }

    // Magnitude coverage bins
    cp_x_mag: coverpoint $unsigned(item.x_in[DATA_WIDTH-2:DATA_WIDTH-5]) {
      bins small  = {[0:3]};    // x < 0.25
      bins medium = {[4:11]};   // 0.25 ≤ x < 0.75
      bins large  = {[12:15]};  // x ≥ 0.75
    }

    // Tag coverage
    cp_tag_range: coverpoint item.tag {
      bins low    = {[0:63]};
      bins mid    = {[64:191]};
      bins high   = {[192:255]};
    }
  endgroup

  // -------------------------------------------------------------------------
  // Covergroup: Corner cases
  // -------------------------------------------------------------------------
  covergroup cg_corners;
    // Zero inputs
    cp_x_zero: coverpoint (item.x_in == 0);
    cp_y_zero: coverpoint (item.y_in == 0);
    cp_z_zero: coverpoint (item.z_in == 0);

    // Max/min values
    cp_x_max: coverpoint (item.x_in == {1'b0, {(DATA_WIDTH-1){1'b1}}});
    cp_x_min: coverpoint (item.x_in == {1'b1, {(DATA_WIDTH-1){1'b0}}});

    // Pure sine/cosine inputs (y=0, rotation mode)
    cp_unit_circle: coverpoint (item.y_in == 0 && item.coord == CIRC && item.mode == ROTATION);

    // π/2 angle
    cp_pi_half: coverpoint (item.z_in == 16'sh2000 && item.coord == CIRC);

    // Negative x (triggers quadrant correction in vectoring)
    cp_neg_x_vec: coverpoint (item.x_in[DATA_WIDTH-1] && item.coord == CIRC && item.mode == VECTORING);
  endgroup

  // -------------------------------------------------------------------------
  // Constructor
  // -------------------------------------------------------------------------
  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg_input   = new();
    cg_corners = new();
  endfunction

  // -------------------------------------------------------------------------
  // Sample on each received transaction
  // -------------------------------------------------------------------------
  function void write(cordic_seq_item t);
    item = t;
    cg_input.sample();
    cg_corners.sample();
  endfunction

  function void report_phase(uvm_phase phase);
    `uvm_info("COVERAGE", $sformatf(
      "\n=== COVERAGE SUMMARY ===\n  Input Coverage   : %.1f%%\n  Corners Coverage : %.1f%%\n========================",
      cg_input.get_coverage(), cg_corners.get_coverage()), UVM_NONE)
  endfunction

endclass : cordic_coverage

`endif
