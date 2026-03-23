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

`ifndef CORDIC_DRIVER_SV
`define CORDIC_DRIVER_SV

import cordic_pkg::*;

class cordic_driver extends uvm_driver #(cordic_seq_item);
  `uvm_component_utils(cordic_driver)

  virtual cordic_if.driver_mp vif;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(virtual cordic_if.driver_mp)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "Driver: no virtual interface found")
  endfunction

  task run_phase(uvm_phase phase);
    cordic_seq_item item;

    // De-assert inputs on reset
    vif.driver_cb.s_valid <= 1'b0;
    vif.driver_cb.m_ready <= 1'b1;
    vif.driver_cb.s_x     <= '0;
    vif.driver_cb.s_y     <= '0;
    vif.driver_cb.s_z     <= '0;
    vif.driver_cb.s_coord <= CIRC;
    vif.driver_cb.s_mode  <= ROTATION;
    vif.driver_cb.s_tag   <= '0;

    @(posedge vif.rst_n);         // Wait for reset deassertion
    @(vif.driver_cb);

    forever begin
      seq_item_port.get_next_item(item);
      drive_item(item);
      seq_item_port.item_done();
    end
  endtask

  task drive_item(cordic_seq_item item);
    // Wait for ready
    while (!vif.driver_cb.s_ready) @(vif.driver_cb);

    vif.driver_cb.s_valid <= 1'b1;
    vif.driver_cb.s_x     <= item.x_in;
    vif.driver_cb.s_y     <= item.y_in;
    vif.driver_cb.s_z     <= item.z_in;
    vif.driver_cb.s_coord <= item.coord;
    vif.driver_cb.s_mode  <= item.mode;
    vif.driver_cb.s_tag   <= item.tag;

    @(vif.driver_cb);

    vif.driver_cb.s_valid <= 1'b0;
  endtask

endclass : cordic_driver

`endif
