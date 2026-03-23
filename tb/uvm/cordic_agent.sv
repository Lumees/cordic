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

`ifndef CORDIC_AGENT_SV
`define CORDIC_AGENT_SV

class cordic_agent extends uvm_agent;
  `uvm_component_utils(cordic_agent)

  cordic_driver         driver;
  cordic_sequencer      sequencer;
  cordic_monitor #(1)   in_monitor;   // IS_INPUT=1
  cordic_monitor #(0)   out_monitor;  // IS_INPUT=0

  // Analysis ports (forwarded to env)
  uvm_analysis_port #(cordic_seq_item) in_ap;
  uvm_analysis_port #(cordic_seq_item) out_ap;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    in_ap  = new("in_ap",  this);
    out_ap = new("out_ap", this);

    if (get_is_active() == UVM_ACTIVE) begin
      driver    = cordic_driver::type_id::create("driver", this);
      sequencer = cordic_sequencer::type_id::create("sequencer", this);
    end
    in_monitor  = cordic_monitor #(1)::type_id::create("in_monitor",  this);
    out_monitor = cordic_monitor #(0)::type_id::create("out_monitor", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    if (get_is_active() == UVM_ACTIVE)
      driver.seq_item_port.connect(sequencer.seq_item_export);
    in_monitor.ap.connect(in_ap);
    out_monitor.ap.connect(out_ap);
  endfunction

endclass : cordic_agent

`endif
