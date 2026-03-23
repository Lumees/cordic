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

`ifndef CORDIC_SEQUENCER_SV
`define CORDIC_SEQUENCER_SV

class cordic_sequencer extends uvm_sequencer #(cordic_seq_item);
  `uvm_component_utils(cordic_sequencer)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

endclass : cordic_sequencer

`endif
