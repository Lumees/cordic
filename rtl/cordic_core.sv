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
// =============================================================================
// CORDIC IP - Pipelined Core
// =============================================================================
// Instantiates STAGES pipeline stages. Fully pipelined: new input accepted
// every clock cycle. Latency = STAGES cycles.
// =============================================================================

`timescale 1ns/1ps

import cordic_pkg::*;

module cordic_core (
  input  logic         clk,
  input  logic         rst_n,
  input  cordic_data_t din,
  output cordic_data_t dout
);

  // Pipeline inter-stage signals
  cordic_data_t pipe [0:STAGES];

  assign pipe[0] = din;
  assign dout     = pipe[STAGES];

  // Generate STAGES pipeline stages
  generate
    for (genvar i = 0; i < STAGES; i++) begin : gen_stage
      cordic_stage #(
        .STAGE_IDX(i)
      ) u_stage (
        .clk   (clk),
        .rst_n (rst_n),
        .din   (pipe[i]),
        .dout  (pipe[i+1])
      );
    end
  endgenerate

endmodule : cordic_core
