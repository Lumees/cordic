# =============================================================================
# Copyright (c) 2026 Lumees Lab / Hasan Kurşun
# SPDX-License-Identifier: Apache-2.0 WITH Commons-Clause
#
# Free for non-commercial use (academic, research, hobby, education).
# Commercial use requires a Lumees Lab license: info@lumeeslab.com
# =============================================================================
"""
CORDIC Golden Model
===================
Pure Python implementation of the pipelined CORDIC algorithm.
Supports Circular, Linear, and Hyperbolic coordinate systems
in both Rotation and Vectoring modes.

Parameterized (NEW): DATA_WIDTH, STAGES, ROUND_MODE are CORDICModel
constructor arguments.  Module-level constants default to 16-bit for
backward compatibility with existing testbenches.

  Data format: Q2.(DATA_WIDTH-2) signed fixed-point
    x, y  : values in [-1, +1)  → represent as value × 2^FRAC_BITS
    z (circ): angle in [-π, +π] → (angle/π) × 2^FRAC_BITS

Usage:
  # Default 16-bit (backward compatible)
  model = CORDICModel()
  x_out, y_out, z_out = model.compute(x_in, y_in, z_in, Coord.CIRC, Mode.ROTATION)

  # 24-bit version
  model24 = CORDICModel(data_width=24)
  x_out, y_out, z_out = model24.compute(x_in, y_in, z_in, Coord.CIRC, Mode.ROTATION)

  # With round-to-nearest (matches RTL ROUND_MODE=1)
  model_rnd = CORDICModel(round_mode=True)
"""

import math
from enum import IntEnum
from typing import Tuple


# ─────────────────────────────────────────────
# Module-level defaults (backward compat)
# ─────────────────────────────────────────────
DATA_WIDTH = 16
FRAC_BITS  = DATA_WIDTH - 2       # 14
STAGES     = DATA_WIDTH           # 16
SCALE      = 1 << FRAC_BITS       # 16384  → 1.0 in Q2.14
PI_SCALE   = SCALE                # π ↔ SCALE


class Coord(IntEnum):
    CIRC = 0
    LINR = 1
    HYPR = 2


class Mode(IntEnum):
    ROTATION  = 0
    VECTORING = 1


# Standard 16-stage hyperbolic schedule (invariant to data width)
_HYPR_SCHEDULE_16 = [1, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14]


# ─────────────────────────────────────────────
# Module-level gain constants for 16-bit default
# ─────────────────────────────────────────────
def _circ_kinv_float(stages: int) -> float:
    k_inv = 1.0
    for i in range(stages):
        k_inv /= math.sqrt(1.0 + 2.0 ** (-2.0 * i))
    return k_inv


def _hypr_kinv_float(stages: int, sched: list) -> float:
    k_inv = 1.0
    for sh in sched:
        k_inv /= math.sqrt(1.0 - 2.0 ** (-2.0 * sh))
    return k_inv


CIRC_KINV = _circ_kinv_float(STAGES)            # ≈ 0.6073 for 16 stages
HYPR_KINV = _hypr_kinv_float(STAGES, _HYPR_SCHEDULE_16)  # ≈ 1.2074


# ─────────────────────────────────────────────
# Main CORDIC fixed-point model
# ─────────────────────────────────────────────
class CORDICModel:
    """
    Bit-accurate fixed-point CORDIC model matching the RTL implementation.

    Parameters
    ----------
    stages     : number of CORDIC iterations (default 16, matches RTL STAGES)
    data_width : bit width of x/y/z (default 16, matches RTL DATA_WIDTH)
    apply_gain : apply K_inv gain correction in post-processing (default True)
    round_mode : False = truncate (RTL default), True = round-to-nearest
                 (matches RTL ROUND_MODE=1 for post-proc gain correction)
    """

    def __init__(
        self,
        stages:     int  = 16,
        data_width: int  = 16,
        apply_gain: bool = True,
        round_mode: bool = False,
    ):
        self.stages     = stages
        self.data_width = data_width
        self.apply_gain = apply_gain
        self.round_mode = round_mode

        # Derived constants
        self.frac_bits = data_width - 2
        self.scale     = 1 << self.frac_bits
        self.int_max   = (1 << (data_width - 1)) - 1
        self.int_min   = -(1 << (data_width - 1))
        self.mask      = (1 << data_width) - 1

        # Hyperbolic schedule (use standard 16-entry table, truncate to stages)
        self.hypr_schedule = _HYPR_SCHEDULE_16[:stages]

        # LUT tables (computed from frac_bits — auto-scales for any data_width)
        self.atan_lut  = [
            round(math.atan(2.0 ** (-i)) / math.pi * self.scale)
            for i in range(stages)
        ]
        self.atanh_lut = [
            round(math.atanh(2.0 ** (-sh)) / math.pi * self.scale)
            for sh in self.hypr_schedule
        ]
        self.linr_lut  = [
            round((2.0 ** (-i)) * self.scale)
            for i in range(stages)
        ]

        # Gain correction constants in Q format
        self.circ_kinv_q = round(_circ_kinv_float(stages) * self.scale)
        self.hypr_kinv_q = round(_hypr_kinv_float(stages, self.hypr_schedule) * self.scale)

        # Round-to-nearest addend for post-proc gain shift (0.5 in Q format)
        self.round_half = (1 << (self.frac_bits - 1)) if round_mode else 0

    # ─────────────────────────────────────────
    # Bit-level helpers (instance-specific)
    # ─────────────────────────────────────────
    def _to_signed(self, v: int) -> int:
        """Wrap to data_width two's complement signed."""
        v = v & self.mask
        if v >= (1 << (self.data_width - 1)):
            v -= (1 << self.data_width)
        return v

    def _clamp(self, v: int) -> int:
        return max(self.int_min, min(self.int_max, v))

    def _arith_shift_right(self, v: int, n: int) -> int:
        if n <= 0:
            return v
        return v >> n  # Python >> is arithmetic for negative numbers

    def _safe_neg(self, v: int) -> int:
        """Negate with saturation (matches RTL safe_neg function)."""
        return self.int_max if v == self.int_min else -v

    # ─────────────────────────────────────────
    # Pre-processing
    # ─────────────────────────────────────────
    def _pre_proc(
        self, x: int, y: int, z: int, coord: Coord, mode: Mode
    ) -> Tuple[int, int, int, int, bool]:
        """
        Returns (x', y', z', quad_offset, sat).
        sat is True if negation overflow was detected.
        """
        quad_offset = 0
        sat = False

        if coord == Coord.CIRC:
            PI_HALF = self.scale >> 1    # π/2
            PI_VAL  = self.scale          # π

            if mode == Mode.ROTATION:
                if z > PI_HALF:
                    sat = (x == self.int_min) or (y == self.int_min)
                    x = self._safe_neg(x)
                    y = self._safe_neg(y)
                    z = self._to_signed(z - PI_VAL)
                elif z < -PI_HALF:
                    sat = (x == self.int_min) or (y == self.int_min)
                    x = self._safe_neg(x)
                    y = self._safe_neg(y)
                    z = self._to_signed(z + PI_VAL)

            elif mode == Mode.VECTORING:
                if x < 0:
                    orig_y = y
                    sat = (x == self.int_min) or (y == self.int_min)
                    x = self._safe_neg(x)
                    y = self._safe_neg(y)
                    quad_offset = -PI_VAL if orig_y < 0 else PI_VAL

        return x, y, z, quad_offset, sat

    # ─────────────────────────────────────────
    # Fixed-point CORDIC iterations
    # ─────────────────────────────────────────
    def _iterate(
        self, x: int, y: int, z: int, coord: Coord, mode: Mode
    ) -> Tuple[int, int, int]:
        for stage in range(self.stages):
            if coord == Coord.CIRC:
                shift = stage
                alpha = self.atan_lut[stage]
            elif coord == Coord.LINR:
                shift = stage
                alpha = self.linr_lut[stage]
            else:  # HYPR
                shift = self.hypr_schedule[stage]
                alpha = self.atanh_lut[stage]

            # Direction
            if mode == Mode.ROTATION:
                d = 1 if z >= 0 else -1
            else:  # VECTORING
                if coord == Coord.HYPR:
                    d = 1 if y >= 0 else -1
                else:
                    d = -1 if y >= 0 else 1

            x_s = self._arith_shift_right(x, shift)
            y_s = self._arith_shift_right(y, shift)

            z_new = self._to_signed(z - d * alpha)

            if coord == Coord.CIRC:
                x_new = self._to_signed(x - d * y_s)
                y_new = self._to_signed(y + d * x_s)
            elif coord == Coord.LINR:
                x_new = x  # x unchanged
                y_new = self._to_signed(y + d * x_s)
            else:  # HYPR
                x_new = self._to_signed(x + d * y_s)
                y_new = self._to_signed(y + d * x_s)

            x, y, z = x_new, y_new, z_new

        return x, y, z

    # ─────────────────────────────────────────
    # Public interface
    # ─────────────────────────────────────────
    def compute(
        self,
        x_in: int, y_in: int, z_in: int,
        coord: Coord = Coord.CIRC,
        mode:  Mode  = Mode.ROTATION,
    ) -> Tuple[int, int, int]:
        """
        Compute CORDIC result in fixed-point, matching RTL behavior.

        Returns (x_out, y_out, z_out) in Q2.(data_width-2) format.
        """
        # Pre-processing
        x, y, z, quad_offset, pre_sat = self._pre_proc(x_in, y_in, z_in, coord, mode)

        # CORDIC iterations
        x, y, z = self._iterate(x, y, z, coord, mode)

        # Post-processing: gain correction
        post_sat = False
        if self.apply_gain and coord != Coord.LINR:
            kinv_q = self.circ_kinv_q if coord == Coord.CIRC else self.hypr_kinv_q
            xc = (x * kinv_q + self.round_half) >> self.frac_bits
            yc = (y * kinv_q + self.round_half) >> self.frac_bits
            # Saturation check
            if xc > self.int_max or xc < self.int_min:
                post_sat = True
                xc = self._clamp(xc)
            if yc > self.int_max or yc < self.int_min:
                post_sat = True
                yc = self._clamp(yc)
            x = self._to_signed(xc)
            y = self._to_signed(yc)

        # Add quadrant offset for circular vectoring
        if coord == Coord.CIRC and mode == Mode.VECTORING:
            z = self._to_signed(z + quad_offset)

        return x, y, z

    def compute_float(
        self,
        x_in: int, y_in: int, z_in: int,
        coord: Coord = Coord.CIRC,
        mode:  Mode  = Mode.ROTATION,
    ) -> Tuple[float, float, float]:
        """
        Ideal floating-point CORDIC result (values in Q-format units).
        Useful for measuring fixed-point error.
        """
        scale = self.scale
        xf = x_in / scale
        yf = y_in / scale

        if coord == Coord.LINR:
            zf = z_in / scale
            if mode == Mode.ROTATION:
                return (xf * scale, (yf + xf * zf) * scale, 0.0)
            else:
                y_out = 0.0
                z_out = (zf + yf / xf) * scale if xf != 0 else float('inf')
                return (xf * scale, y_out, z_out)

        zf_rad = z_in / scale * math.pi

        if coord == Coord.CIRC:
            if mode == Mode.ROTATION:
                x_out = xf * math.cos(zf_rad) - yf * math.sin(zf_rad)
                y_out = xf * math.sin(zf_rad) + yf * math.cos(zf_rad)
                return (x_out * scale, y_out * scale, 0.0)
            else:
                mag   = math.sqrt(xf**2 + yf**2)
                angle = math.atan2(yf, xf)
                return (mag * scale, 0.0, angle / math.pi * scale)
        else:  # HYPR
            if mode == Mode.ROTATION:
                x_out = xf * math.cosh(zf_rad) + yf * math.sinh(zf_rad)
                y_out = yf * math.cosh(zf_rad) + xf * math.sinh(zf_rad)
                return (x_out * scale, y_out * scale, 0.0)
            else:
                if xf**2 > yf**2:
                    mag   = math.sqrt(xf**2 - yf**2)
                    angle = math.atanh(yf / xf) if xf != 0 else 0.0
                else:
                    mag, angle = 0.0, 0.0
                return (mag * scale, 0.0, angle / math.pi * scale)


# ─────────────────────────────────────────────
# Test vector generation
# ─────────────────────────────────────────────
def generate_test_vectors(n_random: int = 200, seed: int = 42,
                          data_width: int = 16) -> list:
    """
    Generate test vectors for the given data_width.
    Returns list of dicts: {x_in, y_in, z_in, coord, mode, x_out, y_out, z_out}
    """
    import random
    random.seed(seed)
    model  = CORDICModel(data_width=data_width)
    scale  = model.scale
    KINV_Q = model.circ_kinv_q
    SAFE   = scale // 2
    vectors = []

    # Directed sin/cos
    for z in [0, scale//6, scale//4, scale//3, scale//2, -scale//4, -scale//2]:
        xo, yo, zo = model.compute(KINV_Q, 0, z, Coord.CIRC, Mode.ROTATION)
        vectors.append({'label': 'sincos', 'coord': 'CIRC', 'mode': 'ROTATION',
                        'x_in': KINV_Q, 'y_in': 0, 'z_in': z,
                        'x_out': xo, 'y_out': yo, 'z_out': zo})

    # Random circular rotation
    for _ in range(n_random // 4):
        x = random.randint(-SAFE, SAFE)
        y = random.randint(-SAFE, SAFE)
        z = random.randint(-scale, scale - 1)
        xo, yo, zo = model.compute(x, y, z, Coord.CIRC, Mode.ROTATION)
        vectors.append({'label': 'circ_rot', 'coord': 'CIRC', 'mode': 'ROTATION',
                        'x_in': x, 'y_in': y, 'z_in': z,
                        'x_out': xo, 'y_out': yo, 'z_out': zo})

    # Random circular vectoring
    for _ in range(n_random // 4):
        x = random.randint(-scale, scale - 1)
        y = random.randint(-scale, scale - 1)
        xo, yo, zo = model.compute(x, y, 0, Coord.CIRC, Mode.VECTORING)
        vectors.append({'label': 'circ_vec', 'coord': 'CIRC', 'mode': 'VECTORING',
                        'x_in': x, 'y_in': y, 'z_in': 0,
                        'x_out': xo, 'y_out': yo, 'z_out': zo})

    # Random linear
    for _ in range(n_random // 4):
        x = random.randint(-scale, scale - 1)
        y = random.randint(-scale, scale - 1)
        z = random.randint(-scale, scale - 1)
        xo, yo, zo = model.compute(x, y, z, Coord.LINR, Mode.ROTATION)
        vectors.append({'label': 'linr_rot', 'coord': 'LINR', 'mode': 'ROTATION',
                        'x_in': x, 'y_in': y, 'z_in': z,
                        'x_out': xo, 'y_out': yo, 'z_out': zo})

    # Random hyperbolic
    for _ in range(n_random // 4):
        mag = random.randint(1000, SAFE)
        y   = round(mag * random.uniform(0, 0.7) * random.choice([1, -1]))
        xo, yo, zo = model.compute(mag, y, 0, Coord.HYPR, Mode.VECTORING)
        vectors.append({'label': 'hypr_vec', 'coord': 'HYPR', 'mode': 'VECTORING',
                        'x_in': mag, 'y_in': y, 'z_in': 0,
                        'x_out': xo, 'y_out': yo, 'z_out': zo})

    return vectors


def export_test_vectors_c(vectors: list, path: str):
    """Export test vectors as C header for DPI-C use."""
    with open(path, 'w') as f:
        f.write("// Auto-generated CORDIC test vectors\n")
        f.write(f"#define N_VECTORS {len(vectors)}\n\n")
        f.write("typedef struct { int x_in,y_in,z_in,coord,mode,x_out,y_out,z_out; } tv_t;\n\n")
        f.write("static const tv_t test_vectors[] = {\n")
        cmap = {'CIRC': 0, 'LINR': 1, 'HYPR': 2}
        mmap = {'ROTATION': 0, 'VECTORING': 1}
        for v in vectors:
            f.write(f"  {{{v['x_in']},{v['y_in']},{v['z_in']},"
                    f"{cmap[v['coord']]},{mmap[v['mode']]},"
                    f"{v['x_out']},{v['y_out']},{v['z_out']}}},\n")
        f.write("};\n")


# ─────────────────────────────────────────────
# Self-test
# ─────────────────────────────────────────────
def self_test():
    print("=" * 60)
    print("CORDIC Golden Model Self-Test")

    for dw in [16, 24]:
        model  = CORDICModel(data_width=dw)
        scale  = model.scale
        KINV_Q = model.circ_kinv_q
        SAFE   = scale // 2
        print(f"\n  DATA_WIDTH={dw}, FRAC_BITS={model.frac_bits}, SCALE={scale}")
        print(f"  CIRC K_inv≈{model.circ_kinv_q/scale:.6f}  ({model.circ_kinv_q})")

        test_cases = [
            (KINV_Q, 0, 0,          Coord.CIRC, Mode.ROTATION,  "cos(0°)=1, sin(0°)=0"),
            (KINV_Q, 0, scale // 4, Coord.CIRC, Mode.ROTATION,  "cos(45°)≈0.707"),
            (KINV_Q, 0, scale // 2, Coord.CIRC, Mode.ROTATION,  "cos(90°)≈0, sin(90°)≈1"),
            (SAFE,   SAFE, 0,       Coord.CIRC, Mode.VECTORING, "atan2(1,1)=π/4"),
        ]

        all_pass = True
        tol = 8  # ~0.5 LSB for 16-bit; tighter for wider
        for x_in, y_in, z_in, coord, mode, desc in test_cases:
            xo, yo, zo    = model.compute(x_in, y_in, z_in, coord, mode)
            xf, yf, zf    = model.compute_float(x_in, y_in, z_in, coord, mode)
            ex, ey        = abs(xo - xf), abs(yo - yf)
            ok = ex < tol and ey < tol
            if not ok:
                all_pass = False
            status = "PASS" if ok else "FAIL"
            print(f"    [{status}] {desc}  Δx={ex:.1f} Δy={ey:.1f}")

        print(f"  {'ALL PASS' if all_pass else 'FAILURES DETECTED'}")

    print("=" * 60)
    return True


if __name__ == "__main__":
    import sys
    ok = self_test()
    print("\nGenerating test vectors (16-bit)...")
    vecs = generate_test_vectors(200)
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_vectors.h")
    export_test_vectors_c(vecs, out_path)
    print(f"Exported {len(vecs)} vectors → model/test_vectors.h")
    sys.exit(0 if ok else 1)
