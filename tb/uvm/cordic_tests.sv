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

`ifndef CORDIC_TESTS_SV
`define CORDIC_TESTS_SV

// -----------------------------------------------------------------------------
// Base test
// -----------------------------------------------------------------------------
class cordic_base_test extends uvm_test;
  `uvm_component_utils(cordic_base_test)

  cordic_env env;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = cordic_env::type_id::create("env", this);
  endfunction

  task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    #100;   // reset wait
    run_test_body(phase);
    #(100 * (STAGES + 10));   // drain pipeline
    phase.drop_objection(this);
  endtask

  virtual task run_test_body(uvm_phase phase);
    // Override in subclasses
  endtask
endclass

// -----------------------------------------------------------------------------
// Smoke test: basic sin/cos
// -----------------------------------------------------------------------------
class cordic_smoke_test extends cordic_base_test;
  `uvm_component_utils(cordic_smoke_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_test_body(uvm_phase phase);
    cordic_sincos_seq seq = cordic_sincos_seq::type_id::create("seq");
    seq.start(env.agent.sequencer);
    `uvm_info("SMOKE_TEST", "Smoke test complete", UVM_LOW)
  endtask
endclass

// -----------------------------------------------------------------------------
// Sin/Cos accuracy test
// -----------------------------------------------------------------------------
class cordic_sincos_test extends cordic_base_test;
  `uvm_component_utils(cordic_sincos_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_test_body(uvm_phase phase);
    cordic_sincos_seq seq;
    cordic_atan2_seq  seq2;
    seq  = cordic_sincos_seq::type_id::create("seq");
    seq2 = cordic_atan2_seq::type_id::create("seq2");
    seq.start(env.agent.sequencer);
    seq2.start(env.agent.sequencer);
  endtask
endclass

// -----------------------------------------------------------------------------
// Random test: all modes
// -----------------------------------------------------------------------------
class cordic_rand_test extends cordic_base_test;
  `uvm_component_utils(cordic_rand_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_test_body(uvm_phase phase);
    cordic_rand_seq seq = cordic_rand_seq::type_id::create("seq");
    seq.num_transactions = 200;
    seq.start(env.agent.sequencer);
  endtask
endclass

// -----------------------------------------------------------------------------
// Linear test: multiply/divide
// -----------------------------------------------------------------------------
class cordic_linear_test extends cordic_base_test;
  `uvm_component_utils(cordic_linear_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_test_body(uvm_phase phase);
    cordic_linear_seq seq = cordic_linear_seq::type_id::create("seq");
    seq.start(env.agent.sequencer);
  endtask
endclass

// -----------------------------------------------------------------------------
// Hyperbolic test: sinh/cosh/atanh
// -----------------------------------------------------------------------------
class cordic_hypr_test extends cordic_base_test;
  `uvm_component_utils(cordic_hypr_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_test_body(uvm_phase phase);
    cordic_hypr_seq seq = cordic_hypr_seq::type_id::create("seq");
    seq.start(env.agent.sequencer);
  endtask
endclass

// -----------------------------------------------------------------------------
// Stress test: full pipeline utilization
// -----------------------------------------------------------------------------
class cordic_stress_test extends cordic_base_test;
  `uvm_component_utils(cordic_stress_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_test_body(uvm_phase phase);
    cordic_stress_seq seq = cordic_stress_seq::type_id::create("seq");
    seq.start(env.agent.sequencer);
  endtask
endclass

// -----------------------------------------------------------------------------
// Full regression test: all sequences
// -----------------------------------------------------------------------------
class cordic_full_test extends cordic_base_test;
  `uvm_component_utils(cordic_full_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_test_body(uvm_phase phase);
    cordic_sincos_seq seq1 = cordic_sincos_seq::type_id::create("seq1");
    cordic_atan2_seq  seq2 = cordic_atan2_seq::type_id::create("seq2");
    cordic_linear_seq seq3 = cordic_linear_seq::type_id::create("seq3");
    cordic_hypr_seq   seq4 = cordic_hypr_seq::type_id::create("seq4");
    cordic_rand_seq   seq5 = cordic_rand_seq::type_id::create("seq5");
    seq5.num_transactions = 500;

    seq1.start(env.agent.sequencer);
    seq2.start(env.agent.sequencer);
    seq3.start(env.agent.sequencer);
    seq4.start(env.agent.sequencer);
    seq5.start(env.agent.sequencer);
  endtask
endclass

`endif
