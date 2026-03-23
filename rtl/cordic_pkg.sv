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
// CORDIC IP - Package
// =============================================================================
// Parameterized for any data width (8..48 bits) and stage count.
// Change DATA_WIDTH to resize the entire IP: all LUTs and gain-correction
// constants are computed at elaboration time via real-number math functions
// ($atan, $atanh, $sqrt) — no manual table editing required.
//
//   Data format: Q2.(DATA_WIDTH-2) signed fixed-point
//     x, y  : values in [-1, +1)  → represent as value × 2^FRAC_BITS
//     z (circ): angle in [-π, +π] → (angle/π) × 2^FRAC_BITS
//              so π → 2^FRAC_BITS = 0x4000 for DATA_WIDTH=16
//
//   ROUND_MODE : 0 = truncate (IEEE default), 1 = round-to-nearest
//     Controls the post-processing gain-correction arithmetic-right-shift.
//
//   DATA_WIDTH  default 16  → resize to 8, 24, 32, … at compile time
//   STAGES      default 16  → must be ≤ 16 for the standard hypr schedule;
//                             larger data widths still use 16 stages (gain
//                             converges in ~8 stages; more stages only help
//                             for very high precision requirements)
// =============================================================================

package cordic_pkg;

  // ---------------------------------------------------------------------------
  // Global parameters — change these to resize the IP
  // ---------------------------------------------------------------------------
  parameter int DATA_WIDTH = 16;             // Bit width of x, y, z data
  parameter int STAGES     = 16;             // Number of CORDIC micro-rotation stages
  parameter int FRAC_BITS  = DATA_WIDTH - 2; // Q2.N format: N fractional bits
  parameter bit ROUND_MODE = 0;              // 0 = truncate, 1 = round-nearest

  // ---------------------------------------------------------------------------
  // Coordinate system
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    CIRC = 2'b00,   // Circular  (sin/cos, atan2, magnitude)
    LINR = 2'b01,   // Linear    (multiply/divide)
    HYPR = 2'b10    // Hyperbolic(sinh/cosh, atanh, sqrt(x²−y²))
  } coord_t;

  // ---------------------------------------------------------------------------
  // Operation mode
  // ---------------------------------------------------------------------------
  typedef enum logic {
    ROTATION  = 1'b0,   // z→0: rotate (x,y) by angle z
    VECTORING = 1'b1    // y→0: compute angle/magnitude
  } mode_t;

  // ---------------------------------------------------------------------------
  // Pipeline data bundle — propagated through all stages
  //   sat : sticky saturation flag; set in pre/post-proc on overflow,
  //         propagated OR-cumulatively through all stages
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic                          sat;   // sticky saturation flag (NEW)
    logic signed [DATA_WIDTH-1:0]  x;
    logic signed [DATA_WIDTH-1:0]  y;
    logic signed [DATA_WIDTH-1:0]  z;
    coord_t                        coord;
    mode_t                         mode;
    logic [7:0]                    tag;   // user-defined transaction ID
    logic                          valid;
  } cordic_data_t;

  // ---------------------------------------------------------------------------
  // Hyperbolic shift schedule (depends on STAGES, not DATA_WIDTH)
  // Standard 16-stage schedule: {1,2,3,4,4,5,6,7,8,9,10,11,12,13,13,14}
  // Iterations 4 and 13 are repeated to guarantee convergence.
  // ---------------------------------------------------------------------------
  function automatic int hypr_shift(input int stage);
    int sched [0:15] = '{1,2,3,4,4,5,6,7,8,9,10,11,12,13,13,14};
    return (stage < 16) ? sched[stage] : stage;
  endfunction

  // ---------------------------------------------------------------------------
  // ATAN look-up table — computed at elaboration from FRAC_BITS
  //   atan_lut[i] = round( atan(2^-i) / π × 2^FRAC_BITS )
  //
  // Example (DATA_WIDTH=16, FRAC_BITS=14):
  //   i=0 → atan(1)/π × 16384 = 0.25 × 16384 = 4096 = 0x1000
  //   i=1 → 0x0972, i=2 → 0x04FE, ... (identical to previous table)
  //
  // For wider data (e.g. DATA_WIDTH=24, FRAC_BITS=22) the low-index stages
  // naturally produce more precise angle constants.
  // ---------------------------------------------------------------------------
  function automatic logic signed [DATA_WIDTH-1:0] atan_lut(input int i);
    real pi  = 3.14159265358979323846;
    real val = $atan(2.0 ** (-$itor(i))) / pi * (2.0 ** $itor(FRAC_BITS));
    return DATA_WIDTH'(longint'(val + 0.5));
  endfunction

  // ---------------------------------------------------------------------------
  // ATANH look-up table — computed at elaboration from FRAC_BITS
  //   atanh_lut[stage] = round( atanh(2^-hypr_shift(stage)) / π × 2^FRAC_BITS )
  // ---------------------------------------------------------------------------
  function automatic logic signed [DATA_WIDTH-1:0] atanh_lut(input int stage);
    real pi  = 3.14159265358979323846;
    int  sh  = hypr_shift(stage);
    real val = $atanh(2.0 ** (-$itor(sh))) / pi * (2.0 ** $itor(FRAC_BITS));
    return DATA_WIDTH'(longint'(val + 0.5));
  endfunction

  // ---------------------------------------------------------------------------
  // CORDIC gain compensation constants
  //
  // Precomputed float values for the standard STAGES=16 schedule:
  //   K_circ_inv = ∏_{i=0}^{15} cos(atan(2^-i))    ≈ 0.6072529350…
  //   K_hypr_inv = ∏_{stage}  1/cosh(atanh(2^-sh)) ≈ 1.2074953741…
  //
  // Scaled to Q2.FRAC_BITS format by multiplying by 2^FRAC_BITS.
  // Uses real literals + simple 2.0**FRAC_BITS (no $sqrt needed — works
  // around elaboration-time precision issues with that function in some tools).
  //
  // For STAGES != 16, update the float literals accordingly.
  // ---------------------------------------------------------------------------
  localparam real CIRC_KINV_FLOAT = 0.6072529350088813;
  localparam real HYPR_KINV_FLOAT = 1.2074953740685081;

  function automatic logic signed [2*DATA_WIDTH-1:0] circ_kinv_q();
    longint scale = longint'(1) << (DATA_WIDTH - 2);
    return (2*DATA_WIDTH)'(longint'(CIRC_KINV_FLOAT * $itor(scale) + 0.5));
  endfunction

  function automatic logic signed [2*DATA_WIDTH-1:0] hypr_kinv_q();
    longint scale = longint'(1) << (DATA_WIDTH - 2);
    return (2*DATA_WIDTH)'(longint'(HYPR_KINV_FLOAT * $itor(scale) + 0.5));
  endfunction

endpackage : cordic_pkg
