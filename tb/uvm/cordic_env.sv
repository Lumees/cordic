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

`ifndef CORDIC_ENV_SV
`define CORDIC_ENV_SV

class cordic_env extends uvm_env;
  `uvm_component_utils(cordic_env)

  cordic_agent       agent;
  cordic_scoreboard  scoreboard;
  cordic_coverage    coverage;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agent      = cordic_agent::type_id::create("agent", this);
    scoreboard = cordic_scoreboard::type_id::create("scoreboard", this);
    coverage   = cordic_coverage::type_id::create("coverage", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    // Input monitor → scoreboard FIFO
    agent.in_ap.connect(scoreboard.in_fifo.analysis_export);
    // Output monitor → scoreboard check
    agent.out_ap.connect(scoreboard.out_ap);
    // Input monitor → coverage
    agent.in_ap.connect(coverage.analysis_export);
  endfunction

endclass : cordic_env

`endif
