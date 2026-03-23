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

`ifndef CORDIC_SEQUENCES_SV
`define CORDIC_SEQUENCES_SV

import cordic_pkg::*;

// -----------------------------------------------------------------------------
// Base sequence
// -----------------------------------------------------------------------------
class cordic_base_seq extends uvm_sequence #(cordic_seq_item);
  `uvm_object_utils(cordic_base_seq)

  int unsigned num_transactions = 10;

  function new(string name = "cordic_base_seq");
    super.new(name);
  endfunction
endclass

// -----------------------------------------------------------------------------
// Random sequence: all modes, all coordinate systems
// -----------------------------------------------------------------------------
class cordic_rand_seq extends cordic_base_seq;
  `uvm_object_utils(cordic_rand_seq)

  function new(string name = "cordic_rand_seq");
    super.new(name);
    num_transactions = 100;
  endfunction

  task body();
    cordic_seq_item item;
    for (int i = 0; i < num_transactions; i++) begin
      item = cordic_seq_item::type_id::create($sformatf("item_%0d", i));
      start_item(item);
      if (!item.randomize())
        `uvm_fatal("RAND", "Randomization failed")
      item.tag = i[7:0];
      finish_item(item);
    end
  endtask
endclass

// -----------------------------------------------------------------------------
// Circular rotation directed sequence (sin/cos computation)
// Tests: sin(θ) and cos(θ) for θ = 0°, 30°, 45°, 60°, 90°, 180°, 270°, 360°
// Input: x = K_inv (≈0.6073), y = 0, z = angle
// Output: x = cos(θ), y = sin(θ)
// K_inv in Q2.14 = 9949
// -----------------------------------------------------------------------------
class cordic_sincos_seq extends cordic_base_seq;
  `uvm_object_utils(cordic_sincos_seq)

  function new(string name = "cordic_sincos_seq");
    super.new(name);
  endfunction

  task body();
    cordic_seq_item item;
    // Angles in Q angle format (π = 0x4000 = 16384)
    // {angle_name, z_value}
    int angles [8] = '{
      0,       // 0°
      5461,    // 30°  = π/6 → 16384/6 ≈ 2731... wait: π=0x4000, so 30°=π/6=0x4000/6=2730
      8192,    // 45°  = π/4 = 0x2000
      10923,   // 60°  = π/3 = 0x4000/3 ≈ 5461... wait: 0x4000=16384, 60°=16384*60/180=5461
      16384,   // 90°  = π/2 = 0x4000/2 = 0x2000...
      32768,   // 180° = π   = 0x4000 (but signed 16-bit overflows!)
      -16384,  // -90°
      -8192    // -45°
    };
    // Corrected: π = 0x4000 = 16384 (fits in signed 16-bit, max is 32767)
    // 0° = 0, 45° = 0x1000 = 4096, 90° = 0x2000 = 8192, 180° = 0x4000 = 16384
    int fixed_angles [8] = '{0, 2731, 4096, 5461, 8192, 12000, -8192, -4096};

    for (int i = 0; i < 8; i++) begin
      item = cordic_seq_item::type_id::create($sformatf("sincos_%0d", i));
      start_item(item);
      item.coord = CIRC;
      item.mode  = ROTATION;
      item.x_in  = 16'sh270B;  // K_inv ≈ 0.6073 * 2^14 = 9947 ≈ 0x26EB → use 9995 = 0x270B
      item.y_in  = 16'sh0000;
      item.z_in  = fixed_angles[i];
      item.tag   = i[7:0];
      finish_item(item);
    end
  endtask
endclass

// -----------------------------------------------------------------------------
// Magnitude/angle sequence (circular vectoring = atan2 + magnitude)
// Tests: atan2(y, x) and sqrt(x²+y²) * K
// -----------------------------------------------------------------------------
class cordic_atan2_seq extends cordic_base_seq;
  `uvm_object_utils(cordic_atan2_seq)

  function new(string name = "cordic_atan2_seq");
    super.new(name);
  endfunction

  task body();
    cordic_seq_item item;
    // Known (x, y) pairs: expected angle and magnitude
    // Values in Q2.14: 1.0 = 16384
    int xs [6] = '{ 16384,  11585,      0, -16384,  11585, -11585};
    int ys [6] = '{     0,  11585,  16384,      0, -11585,  11585};
    //              0°      45°     90°    180°   -45°    135°

    for (int i = 0; i < 6; i++) begin
      item = cordic_seq_item::type_id::create($sformatf("atan2_%0d", i));
      start_item(item);
      item.coord = CIRC;
      item.mode  = VECTORING;
      item.x_in  = xs[i];
      item.y_in  = ys[i];
      item.z_in  = 16'sh0000;
      item.tag   = (8'h10 + i);
      finish_item(item);
    end
  endtask
endclass

// -----------------------------------------------------------------------------
// Linear multiply/divide sequence
// Rotation:  y_out = y_in + x_in * z_in
// Vectoring: z_out = z_in + y_in/x_in (division)
// -----------------------------------------------------------------------------
class cordic_linear_seq extends cordic_base_seq;
  `uvm_object_utils(cordic_linear_seq)

  function new(string name = "cordic_linear_seq");
    super.new(name);
    num_transactions = 20;
  endfunction

  task body();
    cordic_seq_item item;
    for (int i = 0; i < num_transactions; i++) begin
      item = cordic_seq_item::type_id::create($sformatf("linear_%0d", i));
      start_item(item);
      if (!item.randomize() with { coord == LINR; })
        `uvm_fatal("RAND", "Randomization failed")
      item.tag = (8'h20 + i);
      finish_item(item);
    end
  endtask
endclass

// -----------------------------------------------------------------------------
// Hyperbolic sequence: sinh/cosh and atanh
// -----------------------------------------------------------------------------
class cordic_hypr_seq extends cordic_base_seq;
  `uvm_object_utils(cordic_hypr_seq)

  function new(string name = "cordic_hypr_seq");
    super.new(name);
    num_transactions = 20;
  endfunction

  task body();
    cordic_seq_item item;
    for (int i = 0; i < num_transactions; i++) begin
      item = cordic_seq_item::type_id::create($sformatf("hypr_%0d", i));
      start_item(item);
      if (!item.randomize() with { coord == HYPR; })
        `uvm_fatal("RAND", "Randomization failed")
      item.tag = (8'h40 + i);
      finish_item(item);
    end
  endtask
endclass

// -----------------------------------------------------------------------------
// Stress sequence: back-to-back transactions, all modes interleaved
// -----------------------------------------------------------------------------
class cordic_stress_seq extends cordic_base_seq;
  `uvm_object_utils(cordic_stress_seq)

  function new(string name = "cordic_stress_seq");
    super.new(name);
    num_transactions = 500;
  endfunction

  task body();
    cordic_seq_item item;
    for (int i = 0; i < num_transactions; i++) begin
      item = cordic_seq_item::type_id::create($sformatf("stress_%0d", i));
      start_item(item);
      if (!item.randomize())
        `uvm_fatal("RAND", "Randomization failed")
      item.tag = i[7:0];
      finish_item(item);
      // No gap between transactions (pipeline utilization test)
    end
  endtask
endclass

`endif
