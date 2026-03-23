#!/usr/bin/env python3
# =============================================================================
# Copyright (c) 2026 Lumees Lab / Hasan Kurşun
# SPDX-License-Identifier: Apache-2.0 WITH Commons-Clause
#
# Free for non-commercial use (academic, research, hobby, education).
# Commercial use requires a Lumees Lab license: info@lumeeslab.com
# =============================================================================
"""
CORDIC UART Test Script
========================
Communicates with the CORDIC SoC over UART (LiteX wishbone bridge).
Tests all CORDIC modes and validates results against Python golden model.

Prerequisites:
  1. Build and load the SoC: python3 cordic_soc.py --build --load
  2. Find serial port: ls /dev/ttyUSB*  (usually /dev/ttyUSB1 for Arty)
  3. Run: python3 cordic_uart_test.py --port /dev/ttyUSB1

Register map (CSR base address from LiteX build):
  Run: cat build/digilent_arty/csr.csv | grep cordic
  or:  python3 -c "from litex.tools.litex_client import RemoteClient; ..."
"""

import argparse
import sys
import os
import math
import time
import random
import struct
import serial

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../model'))
from cordic_model import CORDICModel, Coord, Mode, SCALE, STAGES, CIRC_KINV

try:
    from litex.tools.litex_client import RemoteClient
    LITEX_CLIENT_AVAILABLE = True
except ImportError:
    LITEX_CLIENT_AVAILABLE = False


# ─────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────
BAUD_RATE  = 115200
TOL_LSB    = 12      # 16-stage CORDIC: up to ~11 LSB atanh_lut rounding + margin
KINV_Q14   = round(CIRC_KINV * SCALE)

COORD_CIRC, COORD_LINR, COORD_HYPR = 0, 1, 2
MODE_ROT,   MODE_VEC               = 0, 1

model = CORDICModel()


# ─────────────────────────────────────────────
# LiteX RemoteClient wrapper
# ─────────────────────────────────────────────
class CORDICClient:
    """High-level CORDIC register interface over LiteX UART bridge.

    Requires litex_server running:
        litex_server --uart --uart-port /dev/ttyUSB1 --uart-baudrate 115200
    """

    def __init__(self, host: str = "localhost", tcp_port: int = 1234,
                 base_address: int = None,
                 csr_csv: str = None):
        # Connect to litex_server (TCP bridge to the wishbone-UART)
        self.client = RemoteClient(
            host=host,
            port=tcp_port,
            csr_csv=csr_csv,
        )
        self.client.open()

        # Discover CSR base address
        self._base = base_address or self._find_csr_base()

    def _find_csr_base(self) -> int:
        """Try to read CSR base address from LiteX memory map."""
        # LiteX default CSR base is 0xF0000000; cordic is usually early in list
        # Override with --csr-base if different
        return 0xF0000000

    def _csr_addr(self, offset: int) -> int:
        """Compute absolute address for a CORDIC CSR register."""
        # LiteX CSR addresses: base + module_offset + register_offset
        # After build, check build/digilent_arty/csr.csv for exact addresses
        # Typical layout: each CSRStorage/CSRStatus is 4 bytes aligned
        CORDIC_CSR_OFFSET = 0x0800  # placeholder - update from csr.csv after build
        return self._base + CORDIC_CSR_OFFSET + offset

    def write_reg(self, offset: int, value: int):
        self.client.regs.__setattr__(f"cordic_{self._reg_name(offset)}", value)

    def read_reg(self, offset: int) -> int:
        return int(self.client.regs.__getattribute__(f"cordic_{self._reg_name(offset)}"))

    def _reg_name(self, offset: int) -> str:
        names = {0: 'ctrl', 4: 'tag', 8: 'x_in', 12: 'y_in', 16: 'z_in',
                 20: 'x_out', 24: 'y_out', 28: 'z_out', 32: 'latency', 36: 'version'}
        return names.get(offset, f'reg_{offset:02x}')

    def compute(self, x_in: int, y_in: int, z_in: int,
                coord: int, mode: int, tag: int = 0,
                timeout: float = 0.5) -> tuple:
        """
        Send inputs, trigger CORDIC, wait for done, return (x_out, y_out, z_out).
        """
        # Write inputs
        self.client.regs.cordic_x_in.write(x_in & 0xFFFF)
        self.client.regs.cordic_y_in.write(y_in & 0xFFFF)
        self.client.regs.cordic_z_in.write(z_in & 0xFFFF)
        self.client.regs.cordic_tag.write(tag & 0xFF)
        self.client.regs.cordic_coord.write(coord & 0x3)
        self.client.regs.cordic_mode.write(mode & 0x1)

        # Trigger: write ctrl[0]=1
        self.client.regs.cordic_ctrl.write(0x01)

        # Poll done CSR
        t0 = time.time()
        while True:
            done = int(self.client.regs.cordic_done.read())
            if done & 0x01:
                break
            if time.time() - t0 > timeout:
                raise TimeoutError(f"CORDIC timeout after {timeout}s")
            time.sleep(0.001)

        # Read outputs
        x_out = self._sign_extend16(int(self.client.regs.cordic_x_out.read()))
        y_out = self._sign_extend16(int(self.client.regs.cordic_y_out.read()))
        z_out = self._sign_extend16(int(self.client.regs.cordic_z_out.read()))
        return x_out, y_out, z_out

    @staticmethod
    def _sign_extend16(v: int) -> int:
        v &= 0xFFFF
        return v - 0x10000 if v >= 0x8000 else v

    def get_version(self) -> int:
        return int(self.client.regs.cordic_version.read())  # type: ignore

    def get_latency(self) -> int:
        return int(self.client.regs.cordic_latency.read())  # type: ignore

    def close(self):
        self.client.close()


# ─────────────────────────────────────────────
# Test runner
# ─────────────────────────────────────────────
class CORDICTester:
    def __init__(self, client: CORDICClient):
        self.client   = client
        self.pass_cnt = 0
        self.fail_cnt = 0
        self.results  = []

    def check(self, label: str,
              x_dut: int, y_dut: int, z_dut: int,
              x_ref: int, y_ref: int, z_ref: int) -> bool:
        ex = abs(x_dut - x_ref)
        ey = abs(y_dut - y_ref)
        ez = abs(z_dut - z_ref)
        ok = ex <= TOL_LSB and ey <= TOL_LSB and ez <= TOL_LSB
        status = "PASS" if ok else "FAIL"
        msg = (f"[{status}] {label}\n"
               f"  DUT: x={x_dut:6d} y={y_dut:6d} z={z_dut:6d}\n"
               f"  REF: x={x_ref:6d} y={y_ref:6d} z={z_ref:6d}\n"
               f"  ERR: Δx={ex} Δy={ey} Δz={ez}")
        print(msg)
        if ok:
            self.pass_cnt += 1
        else:
            self.fail_cnt += 1
        self.results.append((ok, label, msg))
        return ok

    # ── Test cases ────────────────────────────────────────────────────────

    def test_version(self):
        """Verify IP is reachable and version register readable."""
        ver = self.client.get_version()
        lat = self.client.get_latency()
        print(f"[INFO] CORDIC IP Version: 0x{ver:08X}")
        print(f"[INFO] Pipeline latency : {lat} cycles")
        assert ver != 0xDEADBEEF, "Could not read VERSION register - check connection"
        assert lat == STAGES + 2, f"Unexpected latency {lat} (expected {STAGES+2})"
        print("[PASS] Version/latency check")

    def test_sincos(self):
        """sin/cos accuracy at key angles."""
        print("\n── Circular Rotation (sin/cos) ──")
        test_angles = [
            (0,   "0°"),  (30, "30°"), (45, "45°"), (60, "60°"),
            (90,  "90°"), (135,"135°"),(180,"180°"),(-45,"-45°"),
            (-90,"-90°"), (-180,"-180°")
        ]
        for deg, name in test_angles:
            z_q = round(deg / 180.0 * SCALE)
            x_dut, y_dut, z_dut = self.client.compute(
                KINV_Q14, 0, z_q, COORD_CIRC, MODE_ROT)
            x_ref, y_ref, z_ref = model.compute(KINV_Q14, 0, z_q, Coord.CIRC, Mode.ROTATION)

            # Human-readable
            cos_dut = x_dut / SCALE
            sin_dut = y_dut / SCALE
            cos_ref = math.cos(math.radians(deg))
            sin_ref = math.sin(math.radians(deg))
            print(f"  {name:6s}: cos={cos_dut:+.4f} (ref={cos_ref:+.4f}), "
                  f"sin={sin_dut:+.4f} (ref={sin_ref:+.4f})")

            self.check(f"sincos {name}", x_dut, y_dut, z_dut, x_ref, y_ref, z_ref)

    def test_atan2(self):
        """atan2 and magnitude."""
        print("\n── Circular Vectoring (atan2 + magnitude) ──")
        points = [
            (SCALE//2,  0,         "0°"),
            (SCALE//2,  SCALE//2,  "45°"),
            (0,         SCALE//2,  "90°"),
            (-SCALE//2, 0,         "180°"),
            (SCALE//2, -SCALE//2,  "-45°"),
        ]
        for x_in, y_in, name in points:
            x_dut, y_dut, z_dut = self.client.compute(
                x_in, y_in, 0, COORD_CIRC, MODE_VEC)
            x_ref, y_ref, z_ref = model.compute(x_in, y_in, 0, Coord.CIRC, Mode.VECTORING)
            angle_dut = z_dut / SCALE * 180.0
            print(f"  {name:6s}: angle={angle_dut:+.2f}° "
                  f"mag={x_dut/SCALE:.4f}")
            self.check(f"atan2 {name}", x_dut, y_dut, z_dut, x_ref, y_ref, z_ref)

    def test_linear(self):
        """Linear multiply: y_out = y_in + x_in * z_in."""
        print("\n── Linear Rotation (multiply) ──")
        random.seed(42)
        for i in range(10):
            x = random.randint(-SCALE // 2, SCALE // 2)
            y = random.randint(-SCALE // 2, SCALE // 2)
            z = random.randint(-SCALE // 4, SCALE // 4)
            x_dut, y_dut, z_dut = self.client.compute(x, y, z, COORD_LINR, MODE_ROT)
            x_ref, y_ref, z_ref = model.compute(x, y, z, Coord.LINR, Mode.ROTATION)
            self.check(f"linear #{i}", x_dut, y_dut, z_dut, x_ref, y_ref, z_ref)

    def test_hyperbolic(self):
        """Hyperbolic vectoring: atanh."""
        print("\n── Hyperbolic Vectoring (atanh + sqrt(x²-y²)) ──")
        random.seed(99)
        for i in range(8):
            mag = random.randint(2000, SCALE // 3)
            y   = round(mag * random.uniform(0, 0.7))
            x_dut, y_dut, z_dut = self.client.compute(
                mag, y, 0, COORD_HYPR, MODE_VEC)
            x_ref, y_ref, z_ref = model.compute(mag, y, 0, Coord.HYPR, Mode.VECTORING)
            self.check(f"hypr #{i}", x_dut, y_dut, z_dut, x_ref, y_ref, z_ref)

    def test_random(self, n: int = 50):
        """Random inputs across all modes."""
        print(f"\n── Random Stress ({n} transactions) ──")
        random.seed(0xBEEF)
        all_cases = [
            (COORD_CIRC, MODE_ROT, Coord.CIRC, Mode.ROTATION),
            (COORD_CIRC, MODE_VEC, Coord.CIRC, Mode.VECTORING),
            (COORD_LINR, MODE_ROT, Coord.LINR, Mode.ROTATION),
            (COORD_HYPR, MODE_VEC, Coord.HYPR, Mode.VECTORING),
        ]
        for i in range(n):
            sv_coord, sv_mode, py_coord, py_mode = random.choice(all_cases)
            if py_coord == Coord.CIRC and py_mode == Mode.ROTATION:
                x, y, z = KINV_Q14, 0, random.randint(-SCALE, SCALE-1)
            elif py_coord == Coord.CIRC:
                x = random.randint(-SCALE//2, SCALE//2)
                y = random.randint(-SCALE//2, SCALE//2)
                z = 0
            elif py_coord == Coord.LINR:
                x = random.randint(-SCALE//2, SCALE//2)
                y = random.randint(-SCALE//2, SCALE//2)
                z = random.randint(-SCALE//4, SCALE//4)
            else:
                mag = random.randint(1000, SCALE//3)
                x, y, z = mag, round(mag * random.uniform(0, 0.7)), 0

            x_dut, y_dut, z_dut = self.client.compute(x, y, z, sv_coord, sv_mode)
            x_ref, y_ref, z_ref = model.compute(x, y, z, py_coord, py_mode)
            self.check(f"rand#{i} {py_coord.name}/{py_mode.name}",
                       x_dut, y_dut, z_dut, x_ref, y_ref, z_ref)

    def report(self):
        total = self.pass_cnt + self.fail_cnt
        print("\n" + "=" * 60)
        print(f"CORDIC Hardware Test Report")
        print(f"  TOTAL : {total}")
        print(f"  PASS  : {self.pass_cnt}")
        print(f"  FAIL  : {self.fail_cnt}")
        print("=" * 60)
        if self.fail_cnt:
            print("FAILED TESTS:")
            for ok, label, msg in self.results:
                if not ok:
                    print(f"  {label}")
        return self.fail_cnt == 0


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="CORDIC UART Hardware Test")
    parser.add_argument("--host",     default="localhost",
                        help="litex_server host (default: localhost)")
    parser.add_argument("--port",     default=1234, type=int,
                        help="litex_server TCP port (default: 1234)")
    parser.add_argument("--csr-csv",  default=None,
                        help="Path to csr.csv (auto-detected if not given)")
    parser.add_argument("--csr-base", default=None, type=lambda x: int(x, 0),
                        help="CSR base address override (from csr.csv)")
    parser.add_argument("--random-n", default=50, type=int,
                        help="Number of random test cases")
    parser.add_argument("--skip-random", action="store_true",
                        help="Skip random stress test")
    args = parser.parse_args()

    if not LITEX_CLIENT_AVAILABLE:
        print("ERROR: litex.tools.litex_client not available.")
        print("       Activate your project venv: source .venv/bin/activate")
        sys.exit(1)

    # Auto-locate csr.csv if not specified
    csr_csv = args.csr_csv
    if csr_csv is None:
        default_csv = os.path.join(os.path.dirname(__file__),
                                   "build/digilent_arty/csr.csv")
        if os.path.exists(default_csv):
            csr_csv = default_csv

    print(f"Connecting to litex_server at {args.host}:{args.port}...")
    print("(Start server with: litex_server --uart --uart-port /dev/ttyUSB1)")
    try:
        client = CORDICClient(host=args.host, tcp_port=args.port,
                              base_address=args.csr_base, csr_csv=csr_csv)
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        print("  1. Start litex_server: litex_server --uart --uart-port /dev/ttyUSB1")
        print("  2. Check that the Arty has the bitstream loaded")
        sys.exit(1)

    print("Connected.")
    tester = CORDICTester(client)

    try:
        tester.test_version()
        tester.test_sincos()
        tester.test_atan2()
        tester.test_linear()
        tester.test_hyperbolic()
        if not args.skip_random:
            tester.test_random(args.random_n)
    except KeyboardInterrupt:
        print("\nAborted by user")
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        client.close()

    ok = tester.report()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
