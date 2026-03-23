# =============================================================================
# Copyright (c) 2026 Lumees Lab / Hasan Kurşun
# SPDX-License-Identifier: Apache-2.0 WITH Commons-Clause
#
# Free for non-commercial use (academic, research, hobby, education).
# Commercial use requires a Lumees Lab license: info@lumeeslab.com
# =============================================================================
"""
CORDIC AXI4-Lite Wrapper Testbench
====================================
Tests cordic_axil.sv: write inputs, trigger, poll done, read outputs.

Register map:
  0x00 CTRL   [0]=start [1]=busy(RO) [2]=done(RO) [5:4]=coord [6]=mode
  0x04 TAG    [7:0]
  0x08 X_IN   signed 16-bit
  0x0C Y_IN   signed 16-bit
  0x10 Z_IN   signed 16-bit
  0x14 X_OUT  RO
  0x18 Y_OUT  RO
  0x1C Z_OUT  RO
  0x20 LATENCY RO
  0x24 VERSION RO
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import sys, os, random

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../model'))
from cordic_model import CORDICModel, Coord, Mode, SCALE, STAGES, CIRC_KINV

model    = CORDICModel(stages=STAGES, apply_gain=True)
KINV_Q14 = round(CIRC_KINV * SCALE)
CLK_NS   = 10
TOL      = 12   # 16-stage CORDIC: up to ~16 LSB accumulated error

COORD_MAP = {'CIRC': 0, 'LINR': 1, 'HYPR': 2}
MODE_MAP  = {'ROTATION': 0, 'VECTORING': 1}

# Register offsets
REG_CTRL    = 0x00
REG_TAG     = 0x04
REG_X_IN    = 0x08
REG_Y_IN    = 0x0C
REG_Z_IN    = 0x10
REG_X_OUT   = 0x14
REG_Y_OUT   = 0x18
REG_Z_OUT   = 0x1C
REG_LATENCY = 0x20
REG_VERSION = 0x24


# ─────────────────────────────────────────────
# AXI4-Lite bus helpers
# ─────────────────────────────────────────────
async def axil_write(dut, addr, data):
    """Single AXI4-Lite write transaction."""
    # Drive AW and W channels simultaneously (legal in AXI4-Lite)
    dut.s_axil_awaddr.value  = addr
    dut.s_axil_awvalid.value = 1
    dut.s_axil_wdata.value   = data & 0xFFFFFFFF
    dut.s_axil_wstrb.value   = 0xF
    dut.s_axil_wvalid.value  = 1
    dut.s_axil_bready.value  = 1

    # Wait for AW handshake
    while True:
        await RisingEdge(dut.clk)
        aw_done = int(dut.s_axil_awready.value) == 1
        w_done  = int(dut.s_axil_wready.value)  == 1
        if aw_done:
            dut.s_axil_awvalid.value = 0
        if w_done:
            dut.s_axil_wvalid.value = 0
        if aw_done and w_done:
            break

    # Wait for B response
    for _ in range(20):
        await RisingEdge(dut.clk)
        if int(dut.s_axil_bvalid.value) == 1:
            dut.s_axil_bready.value = 0
            return
    raise TimeoutError(f"axil_write timeout at addr=0x{addr:02X}")


async def axil_read(dut, addr) -> int:
    """Single AXI4-Lite read transaction, returns 32-bit data."""
    dut.s_axil_araddr.value  = addr
    dut.s_axil_arvalid.value = 1
    dut.s_axil_rready.value  = 1

    for _ in range(20):
        await RisingEdge(dut.clk)
        if int(dut.s_axil_arready.value) == 1:
            dut.s_axil_arvalid.value = 0
            break
    else:
        raise TimeoutError(f"axil_read AR timeout at addr=0x{addr:02X}")

    for _ in range(20):
        await RisingEdge(dut.clk)
        if int(dut.s_axil_rvalid.value) == 1:
            data = int(dut.s_axil_rdata.value)
            dut.s_axil_rready.value = 0
            return data
    raise TimeoutError(f"axil_read R timeout at addr=0x{addr:02X}")


def to_signed16(v):
    v &= 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


async def cordic_compute(dut, x, y, z, coord, mode, tag=0, timeout=100):
    """Write inputs, trigger, poll done, return (x_out, y_out, z_out)."""
    ctrl = (coord & 0x3) << 4 | (mode & 0x1) << 6
    await axil_write(dut, REG_X_IN, x & 0xFFFF)
    await axil_write(dut, REG_Y_IN, y & 0xFFFF)
    await axil_write(dut, REG_Z_IN, z & 0xFFFF)
    await axil_write(dut, REG_TAG,  tag & 0xFF)
    # Write ctrl with start bit (coord/mode + start=1)
    await axil_write(dut, REG_CTRL, ctrl | 0x01)

    # Poll done bit [2]
    for _ in range(timeout):
        ctrl_val = await axil_read(dut, REG_CTRL)
        if ctrl_val & 0x04:
            break
    else:
        raise TimeoutError("CORDIC done timeout")

    xo = to_signed16(await axil_read(dut, REG_X_OUT))
    yo = to_signed16(await axil_read(dut, REG_Y_OUT))
    zo = to_signed16(await axil_read(dut, REG_Z_OUT))
    return xo, yo, zo


async def hw_reset(dut):
    dut.s_axil_awvalid.value = 0
    dut.s_axil_wvalid.value  = 0
    dut.s_axil_bready.value  = 0
    dut.s_axil_arvalid.value = 0
    dut.s_axil_rready.value  = 0
    dut.s_axil_awaddr.value  = 0
    dut.s_axil_wdata.value   = 0
    dut.s_axil_wstrb.value   = 0xF
    dut.s_axil_araddr.value  = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)


def check(label, xd, yd, zd, xr, yr, zr):
    ex, ey, ez = abs(xd - xr), abs(yd - yr), abs(zd - zr)
    ok = ex <= TOL and ey <= TOL and ez <= TOL
    return ok, f"[{'PASS' if ok else 'FAIL'}] {label}  DUT:{xd},{yd},{zd}  REF:{xr},{yr},{zr}  ERR:{ex},{ey},{ez}"


# ─────────────────────────────────────────────
# Sub-tests
# ─────────────────────────────────────────────
async def sub_version(dut):
    lat = await axil_read(dut, REG_LATENCY)
    ver = await axil_read(dut, REG_VERSION)
    dut._log.info(f"LATENCY={lat}  VERSION=0x{ver:08X}")
    assert lat == STAGES + 2, f"Expected latency {STAGES+2}, got {lat}"
    assert ver != 0, "VERSION register is 0"


async def sub_circ_rotation(dut):
    failures = []
    for deg in [0, 30, 45, 60, 90, 135, 180, -45, -90, -180]:
        z = round(deg / 180.0 * SCALE)
        xd, yd, zd = await cordic_compute(dut, KINV_Q14, 0, z, COORD_MAP['CIRC'], MODE_MAP['ROTATION'])
        xr, yr, zr = model.compute(KINV_Q14, 0, z, Coord.CIRC, Mode.ROTATION)
        ok, msg = check(f"axil_circ_rot {deg}°", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_circ_vectoring(dut):
    HALF = SCALE // 2
    points = [(HALF, 0), (HALF, HALF), (0, HALF), (-HALF, 0), (HALF, -HALF)]
    failures = []
    for x_in, y_in in points:
        xd, yd, zd = await cordic_compute(dut, x_in, y_in, 0, COORD_MAP['CIRC'], MODE_MAP['VECTORING'])
        xr, yr, zr = model.compute(x_in, y_in, 0, Coord.CIRC, Mode.VECTORING)
        ok, msg = check(f"axil_circ_vec ({x_in},{y_in})", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_linear_rotation(dut):
    random.seed(42)
    failures = []
    for i in range(15):
        x = random.randint(-SCALE//2, SCALE//2)
        y = random.randint(-SCALE//2, SCALE//2)
        z = random.randint(-SCALE//4, SCALE//4)
        xd, yd, zd = await cordic_compute(dut, x, y, z, COORD_MAP['LINR'], MODE_MAP['ROTATION'])
        xr, yr, zr = model.compute(x, y, z, Coord.LINR, Mode.ROTATION)
        ok, msg = check(f"axil_linr #{i}", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_hypr_vectoring(dut):
    random.seed(123)
    failures = []
    for i in range(10):
        mag = random.randint(2000, SCALE//2)
        y   = round(mag * random.uniform(0, 0.7) * random.choice([1, -1]))
        xd, yd, zd = await cordic_compute(dut, mag, y, 0, COORD_MAP['HYPR'], MODE_MAP['VECTORING'])
        xr, yr, zr = model.compute(mag, y, 0, Coord.HYPR, Mode.VECTORING)
        ok, msg = check(f"axil_hypr #{i}", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


# ─────────────────────────────────────────────
# Master test
# ─────────────────────────────────────────────
@cocotb.test()
async def test_all(dut):
    """CORDIC AXI4-Lite regression."""
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    await hw_reset(dut)

    results = {}

    async def run(name, coro):
        try:
            await coro
            results[name] = "PASS"
            dut._log.info(f"[SUBTEST PASS] {name}")
        except AssertionError as e:
            results[name] = f"FAIL: {e}"
            dut._log.error(f"[SUBTEST FAIL] {name}: {e}")
        except Exception as e:
            results[name] = f"ERROR: {e}"
            dut._log.error(f"[SUBTEST ERROR] {name}: {e}")

    await run("version",         sub_version(dut))
    await run("circ_rotation",   sub_circ_rotation(dut))
    await run("circ_vectoring",  sub_circ_vectoring(dut))
    await run("linear_rotation", sub_linear_rotation(dut))
    await run("hypr_vectoring",  sub_hypr_vectoring(dut))

    passed = sum(1 for v in results.values() if v == "PASS")
    total  = len(results)
    dut._log.info(f"\n{'='*50}")
    dut._log.info(f"CORDIC AXIL REGRESSION: {passed}/{total} PASSED")
    for name, result in results.items():
        dut._log.info(f"  {'PASS' if result=='PASS' else 'FAIL'}: {name}")
    dut._log.info('='*50)

    failed = [n for n, v in results.items() if v != "PASS"]
    assert not failed, f"Failed subtests: {failed}"
