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

`ifndef CORDIC_SEQ_ITEM_SV
`define CORDIC_SEQ_ITEM_SV

import cordic_pkg::*;

class cordic_seq_item extends uvm_sequence_item;

  `uvm_object_utils_begin(cordic_seq_item)
    `uvm_field_int    (x_in,   UVM_ALL_ON)
    `uvm_field_int    (y_in,   UVM_ALL_ON)
    `uvm_field_int    (z_in,   UVM_ALL_ON)
    `uvm_field_enum   (coord_t, coord, UVM_ALL_ON)
    `uvm_field_enum   (mode_t,  mode,  UVM_ALL_ON)
    `uvm_field_int    (tag,    UVM_ALL_ON)
    `uvm_field_int    (x_out,  UVM_ALL_ON)
    `uvm_field_int    (y_out,  UVM_ALL_ON)
    `uvm_field_int    (z_out,  UVM_ALL_ON)
  `uvm_object_utils_end

  // Input fields (randomizable)
  rand logic signed [DATA_WIDTH-1:0]  x_in;
  rand logic signed [DATA_WIDTH-1:0]  y_in;
  rand logic signed [DATA_WIDTH-1:0]  z_in;
  rand coord_t                         coord;
  rand mode_t                          mode;
  rand logic [7:0]                     tag;

  // Output fields (captured by monitor)
  logic signed [DATA_WIDTH-1:0]  x_out;
  logic signed [DATA_WIDTH-1:0]  y_out;
  logic signed [DATA_WIDTH-1:0]  z_out;

  // -------------------------------------------------------------------------
  // Constraints
  // -------------------------------------------------------------------------

  // Circular rotation: angle in [-π, π] → normalized [-0x4000, 0x4000)
  constraint c_circ_rot_angle {
    if (coord == CIRC && mode == ROTATION) {
      z_in inside {[$signed(-16'h4000) : $signed(16'h3FFF)]};
      // Input magnitude < 1 (to stay in bounds after CORDIC gain)
      x_in inside {[$signed(-16'h2800) : $signed(16'h2800)]};
      y_in inside {[$signed(-16'h2800) : $signed(16'h2800)]};
    }
  }

  // Circular vectoring: x should be positive before pre-proc
  constraint c_circ_vec_input {
    if (coord == CIRC && mode == VECTORING) {
      // x ≠ 0 (avoid singularity)
      x_in != 0;
      // Magnitude constraint for meaningful result
      x_in inside {[$signed(-16'h4000) : $signed(16'h4000)]};
      y_in inside {[$signed(-16'h4000) : $signed(16'h4000)]};
      z_in == 0;  // z starts at 0 for vectoring (accumulates angle)
    }
  }

  // Hyperbolic: |x| > |y| for convergence (x²>y²)
  constraint c_hypr_convergence {
    if (coord == HYPR) {
      // |x| > |y|: safe convergence region
      (x_in >= 0 && y_in >= 0) -> (x_in > y_in);
      (x_in >= 0 && y_in <  0) -> (x_in > -y_in);
      (x_in <  0 && y_in >= 0) -> (-x_in > y_in);
      (x_in <  0 && y_in <  0) -> (-x_in > -y_in);
      // Limit magnitude
      x_in inside {[$signed(-16'h3000) : $signed(16'h3000)]};
    }
  }

  // Linear: z in [-2, 2) for rotation mode
  constraint c_linr_valid {
    if (coord == LINR && mode == ROTATION) {
      z_in inside {[$signed(-16'h4000) : $signed(16'h3FFF)]};
    }
  }

  // Tag always valid
  constraint c_tag { tag inside {[0:255]}; }

  // -------------------------------------------------------------------------
  // Constructor
  // -------------------------------------------------------------------------
  function new(string name = "cordic_seq_item");
    super.new(name);
  endfunction

  // -------------------------------------------------------------------------
  // do_compare (for scoreboard matching by tag)
  // -------------------------------------------------------------------------
  function bit do_compare(uvm_object rhs, uvm_comparer comparer);
    cordic_seq_item rhs_item;
    if (!$cast(rhs_item, rhs)) return 0;
    return (tag == rhs_item.tag);  // match on tag only (scoreboard does value check)
  endfunction

  function string convert2string();
    return $sformatf(
      "coord=%s mode=%s tag=%0d | IN: x=%0d y=%0d z=%0d | OUT: x=%0d y=%0d z=%0d",
      coord.name(), mode.name(), tag,
      $signed(x_in), $signed(y_in), $signed(z_in),
      $signed(x_out), $signed(y_out), $signed(z_out)
    );
  endfunction

endclass : cordic_seq_item

`endif
