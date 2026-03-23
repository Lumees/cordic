# =============================================================================
# Copyright (c) 2026 Lumees Lab / Hasan Kurşun
# SPDX-License-Identifier: Apache-2.0 WITH Commons-Clause
#
# Free for non-commercial use (academic, research, hobby, education).
# Commercial use requires a Lumees Lab license: info@lumeeslab.com
# =============================================================================
"""
CORDIC AXI4-Stream Wrapper Testbench
======================================
Tests cordic_axis.sv: streaming interface with output FIFO.

Slave TDATA[63:0]:
  [15:0]  = x    [31:16] = y    [47:32] = z
  [55:48] = tag  [56]    = mode [58:57] = coord  [63:59] = reserved

Master TDATA[47:0]:
  [15:0]  = x_out  [31:16] = y_out  [47:32] = z_out
Master TUSER[7:0] = tag_out
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import sys, os, random

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../model'))
from cordic_model import CORDICModel, Coord, Mode, SCALE, STAGES, CIRC_KINV

model     = CORDICModel(stages=STAGES, apply_gain=True)
KINV_Q14  = round(CIRC_KINV * SCALE)
CLK_NS    = 10
TOL      = 12   # 16-stage CORDIC: up to ~16 LSB accumulated error
TOTAL_LAT = STAGES + 2

COORD_MAP = {'CIRC': 0, 'LINR': 1, 'HYPR': 2}
MODE_MAP  = {'ROTATION': 0, 'VECTORING': 1}


# ─────────────────────────────────────────────
# AXI4-Stream helpers
# ─────────────────────────────────────────────
def pack_tdata(x, y, z, tag, mode, coord):
    """Pack inputs into 64-bit slave TDATA."""
    tdata  = (x    & 0xFFFF)
    tdata |= (y    & 0xFFFF) << 16
    tdata |= (z    & 0xFFFF) << 32
    tdata |= (tag  & 0xFF)   << 48
    tdata |= (mode & 0x1)    << 56
    tdata |= (coord & 0x3)   << 57
    return tdata


def unpack_tdata_out(tdata, tuser):
    """Unpack 48-bit master TDATA and TUSER."""
    x_out = int(tdata) & 0xFFFF
    y_out = (int(tdata) >> 16) & 0xFFFF
    z_out = (int(tdata) >> 32) & 0xFFFF
    tag   = int(tuser) & 0xFF
    # sign-extend
    def s16(v):
        return v - 0x10000 if v >= 0x8000 else v
    return s16(x_out), s16(y_out), s16(z_out), tag


async def axis_send(dut, x, y, z, coord, mode, tag=0, tlast=1):
    """Send one beat on slave AXI4-Stream channel."""
    dut.s_axis_tdata.value  = pack_tdata(x, y, z, tag, mode, coord)
    dut.s_axis_tvalid.value = 1
    dut.s_axis_tlast.value  = tlast
    for _ in range(30):
        await RisingEdge(dut.clk)
        if int(dut.s_axis_tready.value) == 1:
            dut.s_axis_tvalid.value = 0
            return
    raise TimeoutError("AXIS slave ready timeout")


async def axis_recv(dut, timeout=TOTAL_LAT + 20):
    """Wait for one valid beat on master AXI4-Stream channel."""
    dut.m_axis_tready.value = 1
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.m_axis_tvalid.value) == 1:
            xo, yo, zo, tag = unpack_tdata_out(dut.m_axis_tdata.value,
                                               dut.m_axis_tuser.value)
            dut.m_axis_tready.value = 0
            return xo, yo, zo, tag
    raise TimeoutError("AXIS master valid timeout")


async def hw_reset(dut):
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tdata.value  = 0
    dut.s_axis_tlast.value  = 0
    dut.m_axis_tready.value = 1
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
async def sub_circ_rotation(dut):
    failures = []
    for i, deg in enumerate([0, 30, 45, 60, 90, 135, 180, -45, -90, -180]):
        z = round(deg / 180.0 * SCALE)
        await axis_send(dut, KINV_Q14, 0, z, COORD_MAP['CIRC'], MODE_MAP['ROTATION'], tag=i)
        xd, yd, zd, _ = await axis_recv(dut)
        xr, yr, zr    = model.compute(KINV_Q14, 0, z, Coord.CIRC, Mode.ROTATION)
        ok, msg = check(f"axis_circ_rot {deg}°", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_circ_vectoring(dut):
    HALF = SCALE // 2
    failures = []
    for i, (x_in, y_in) in enumerate([(HALF, 0), (HALF, HALF), (0, HALF), (-HALF, 0), (HALF, -HALF)]):
        await axis_send(dut, x_in, y_in, 0, COORD_MAP['CIRC'], MODE_MAP['VECTORING'], tag=i)
        xd, yd, zd, _ = await axis_recv(dut)
        xr, yr, zr    = model.compute(x_in, y_in, 0, Coord.CIRC, Mode.VECTORING)
        ok, msg = check(f"axis_circ_vec ({x_in},{y_in})", xd, yd, zd, xr, yr, zr)
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
        await axis_send(dut, x, y, z, COORD_MAP['LINR'], MODE_MAP['ROTATION'], tag=i)
        xd, yd, zd, _ = await axis_recv(dut)
        xr, yr, zr    = model.compute(x, y, z, Coord.LINR, Mode.ROTATION)
        ok, msg = check(f"axis_linr #{i}", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_hypr_vectoring(dut):
    random.seed(123)
    failures = []
    for i in range(10):
        mag = random.randint(2000, SCALE//2)
        y   = round(mag * random.uniform(0, 0.7) * random.choice([1, -1]))
        await axis_send(dut, mag, y, 0, COORD_MAP['HYPR'], MODE_MAP['VECTORING'], tag=i)
        xd, yd, zd, _ = await axis_recv(dut)
        xr, yr, zr    = model.compute(mag, y, 0, Coord.HYPR, Mode.VECTORING)
        ok, msg = check(f"axis_hypr #{i}", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_pipeline_throughput(dut, N=32):
    """Verify full throughput: N back-to-back, 1 output per clock after latency."""
    random.seed(7)
    inputs = []
    outputs = []

    async def collect():
        dut.m_axis_tready.value = 1
        for _ in range(N + TOTAL_LAT + 10):
            await RisingEdge(dut.clk)
            if int(dut.m_axis_tvalid.value) == 1:
                xo, yo, zo, tag = unpack_tdata_out(dut.m_axis_tdata.value,
                                                   dut.m_axis_tuser.value)
                outputs.append((xo, yo, zo))
            if len(outputs) >= N:
                break

    collector = cocotb.start_soon(collect())

    for i in range(N):
        z = random.randint(-SCALE, SCALE - 1)
        inputs.append(z)
        dut.s_axis_tdata.value  = pack_tdata(KINV_Q14, 0, z, i & 0xFF, 0, 0)
        dut.s_axis_tvalid.value = 1
        dut.s_axis_tlast.value  = 1 if i == N - 1 else 0
        await RisingEdge(dut.clk)
    dut.s_axis_tvalid.value = 0

    await collector

    assert len(outputs) == N, f"Expected {N} outputs, got {len(outputs)}"

    failures = []
    for i, (z, (xd, yd, zd)) in enumerate(zip(inputs, outputs)):
        xr, yr, zr = model.compute(KINV_Q14, 0, z, Coord.CIRC, Mode.ROTATION)
        ok, msg = check(f"axis_pipe #{i}", xd, yd, zd, xr, yr, zr)
        if not ok: failures.append(msg)

    dut._log.info(f"Pipeline throughput: {N-len(failures)}/{N} passed")
    assert not failures, f"{len(failures)}/{N} failures:\n" + "\n".join(failures[:3])


async def sub_backpressure(dut, N=8):
    """Test FIFO backpressure: hold m_ready=0 while pipeline fills, then drain."""
    random.seed(99)
    inputs = []
    dut.m_axis_tready.value = 0  # hold off consumer

    for i in range(N):
        z = random.randint(-SCALE, SCALE - 1)
        inputs.append(z)
        await axis_send(dut, KINV_Q14, 0, z, COORD_MAP['CIRC'], MODE_MAP['ROTATION'], tag=i)

    # Wait for pipeline to fill FIFO
    await ClockCycles(dut.clk, TOTAL_LAT + 4)

    # Now drain
    outputs = []
    dut.m_axis_tready.value = 1
    for _ in range(N + 10):
        await RisingEdge(dut.clk)
        if int(dut.m_axis_tvalid.value) == 1:
            xo, yo, zo, _ = unpack_tdata_out(dut.m_axis_tdata.value,
                                             dut.m_axis_tuser.value)
            outputs.append((xo, yo, zo))
        if len(outputs) >= N:
            break

    assert len(outputs) == N, f"Backpressure: expected {N} outputs, got {len(outputs)}"

    failures = []
    for i, (z, (xd, yd, zd)) in enumerate(zip(inputs, outputs)):
        xr, yr, zr = model.compute(KINV_Q14, 0, z, Coord.CIRC, Mode.ROTATION)
        ok, msg = check(f"axis_bp #{i}", xd, yd, zd, xr, yr, zr)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


@cocotb.test()
async def test_all(dut):
    """CORDIC AXI4-Stream regression."""
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
        await hw_reset(dut)

    await run("circ_rotation",      sub_circ_rotation(dut))
    await run("circ_vectoring",     sub_circ_vectoring(dut))
    await run("linear_rotation",    sub_linear_rotation(dut))
    await run("hypr_vectoring",     sub_hypr_vectoring(dut))
    await run("pipeline_throughput",sub_pipeline_throughput(dut))
    await run("backpressure",       sub_backpressure(dut))

    passed = sum(1 for v in results.values() if v == "PASS")
    total  = len(results)
    dut._log.info(f"\n{'='*50}")
    dut._log.info(f"CORDIC AXIS REGRESSION: {passed}/{total} PASSED")
    for name, result in results.items():
        dut._log.info(f"  {'PASS' if result=='PASS' else 'FAIL'}: {name}")
    dut._log.info('='*50)

    failed = [n for n, v in results.items() if v != "PASS"]
    assert not failed, f"Failed subtests: {failed}"
