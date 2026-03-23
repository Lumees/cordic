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
// Two monitors share the same class:
//   IS_INPUT=1 : captures input transactions → sends to scoreboard for golden
//   IS_INPUT=0 : captures output transactions → sends to scoreboard for check
// =============================================================================

`ifndef CORDIC_MONITOR_SV
`define CORDIC_MONITOR_SV

import cordic_pkg::*;

class cordic_monitor #(bit IS_INPUT = 0) extends uvm_monitor;
  `uvm_component_param_utils(cordic_monitor#(IS_INPUT))

  virtual cordic_if.monitor_mp vif;

  // Analysis port: sends items to scoreboard / coverage
  uvm_analysis_port #(cordic_seq_item) ap;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
    if (!uvm_config_db #(virtual cordic_if.monitor_mp)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "Monitor: no virtual interface found")
  endfunction

  task run_phase(uvm_phase phase);
    @(posedge vif.rst_n);
    forever begin
      @(vif.monitor_cb);
      if (IS_INPUT) begin
        if (vif.monitor_cb.s_valid) begin
          cordic_seq_item item = cordic_seq_item::type_id::create("item");
          item.x_in  = vif.monitor_cb.s_x;
          item.y_in  = vif.monitor_cb.s_y;
          item.z_in  = vif.monitor_cb.s_z;
          item.coord = vif.monitor_cb.s_coord;
          item.mode  = vif.monitor_cb.s_mode;
          item.tag   = vif.monitor_cb.s_tag;
          ap.write(item);
        end
      end else begin
        if (vif.monitor_cb.m_valid) begin
          cordic_seq_item item = cordic_seq_item::type_id::create("item");
          item.x_out = vif.monitor_cb.m_x;
          item.y_out = vif.monitor_cb.m_y;
          item.z_out = vif.monitor_cb.m_z;
          item.tag   = vif.monitor_cb.m_tag;
          ap.write(item);
        end
      end
    end
  endtask

endclass : cordic_monitor

`endif
