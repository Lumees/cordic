# =============================================================================
# Copyright (c) 2026 Lumees Lab / Hasan Kurşun
# SPDX-License-Identifier: Apache-2.0 WITH Commons-Clause
#
# Free for non-commercial use (academic, research, hobby, education).
# Commercial use requires a Lumees Lab license: info@lumeeslab.com
# =============================================================================
"""
CORDIC cocotb Testbench
========================
Single top-level test that runs all sub-tests sequentially.
This avoids Verilator simulation termination between cocotb tests.

Run with:
    cd sim/
    make -f Makefile.cocotb
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import sys, os, math, random

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../model'))
from cordic_model import CORDICModel, Coord, Mode, SCALE, STAGES, CIRC_KINV

model       = CORDICModel(stages=STAGES, apply_gain=True)
COORD_MAP   = {'CIRC': 0, 'LINR': 1, 'HYPR': 2}
MODE_MAP    = {'ROTATION': 0, 'VECTORING': 1}
TOTAL_LAT   = STAGES + 2
CLK_NS      = 10
TOL         = 12   # 16-stage CORDIC: up to ~16 LSB accumulated quantization error
KINV_Q14    = round(CIRC_KINV * SCALE)


# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────
async def drive(dut, x, y, z, coord, mode, tag=0):
    dut.s_valid.value = 1
    dut.s_x.value     = int(x) & 0xFFFF
    dut.s_y.value     = int(y) & 0xFFFF
    dut.s_z.value     = int(z) & 0xFFFF
    dut.s_coord.value = coord
    dut.s_mode.value  = mode
    dut.s_tag.value   = tag & 0xFF
    await RisingEdge(dut.clk)
    dut.s_valid.value = 0


async def collect(dut, timeout=TOTAL_LAT + 15):
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.m_valid.value) == 1:
            return (dut.m_x.value.to_signed(),
                    dut.m_y.value.to_signed(),
                    dut.m_z.value.to_signed(),
                    int(dut.m_tag.value))
    raise TimeoutError("CORDIC output timeout")


def check(label, xd, yd, zd, xr, yr, zr):
    ex, ey, ez = abs(xd - xr), abs(yd - yr), abs(zd - zr)
    ok = ex <= TOL and ey <= TOL and ez <= TOL
    status = "PASS" if ok else "FAIL"
    return ok, f"[{status}] {label}  DUT:{xd},{yd},{zd}  REF:{xr},{yr},{zr}  ERR:{ex},{ey},{ez}"


async def hw_reset(dut):
    dut.s_valid.value = 0
    dut.m_ready.value = 1
    dut.s_x.value = dut.s_y.value = dut.s_z.value = 0
    dut.s_coord.value = dut.s_mode.value = dut.s_tag.value = 0
    await RisingEdge(dut.clk)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 12)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, TOTAL_LAT + 4)


# ─────────────────────────────────────────────
# Sub-tests (plain async functions, no decorator)
# ─────────────────────────────────────────────
async def sub_smoke(dut):
    await hw_reset(dut)
    await drive(dut, KINV_Q14, 0, 0, COORD_MAP['CIRC'], MODE_MAP['ROTATION'])
    xd, yd, zd, _ = await collect(dut)
    xr, yr, zr    = model.compute(KINV_Q14, 0, 0, Coord.CIRC, Mode.ROTATION)
    ok, msg = check("smoke cos(0)=1 sin(0)=0", xd, yd, zd, xr, yr, zr)
    dut._log.info(msg)
    assert ok, msg


async def sub_circ_rotation(dut):
    await hw_reset(dut)
    angles_deg = [0, 30, 45, 60, 90, 120, 135, 150, 180, -30, -90, -180]
    failures = []
    for i, deg in enumerate(angles_deg):
        z = round(deg / 180.0 * SCALE)
        await drive(dut, KINV_Q14, 0, z, COORD_MAP['CIRC'], MODE_MAP['ROTATION'], tag=i)
        xd, yd, zd, _ = await collect(dut)
        xr, yr, zr    = model.compute(KINV_Q14, 0, z, Coord.CIRC, Mode.ROTATION)
        ok, msg = check(f"circ_rot {deg}°", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_circ_vectoring(dut):
    await hw_reset(dut)
    HALF = SCALE // 2
    points = [
        (HALF, 0, "atan2(0,1)=0°"), (HALF, HALF, "atan2(1,1)=45°"),
        (0, HALF, "atan2(1,0)=90°"), (-HALF, 0, "atan2(0,-1)=180°"),
        (HALF, -HALF, "atan2(-1,1)=-45°"), (-HALF, HALF, "atan2(1,-1)=135°"),
    ]
    failures = []
    for i, (x_in, y_in, lbl) in enumerate(points):
        await drive(dut, x_in, y_in, 0, COORD_MAP['CIRC'], MODE_MAP['VECTORING'], tag=i)
        xd, yd, zd, _ = await collect(dut)
        xr, yr, zr    = model.compute(x_in, y_in, 0, Coord.CIRC, Mode.VECTORING)
        ok, msg = check(f"circ_vec {lbl}", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_linear_rotation(dut):
    await hw_reset(dut)
    random.seed(42)
    failures = []
    for i in range(20):
        x = random.randint(-SCALE//2, SCALE//2)
        y = random.randint(-SCALE//2, SCALE//2)
        z = random.randint(-SCALE//4, SCALE//4)
        await drive(dut, x, y, z, COORD_MAP['LINR'], MODE_MAP['ROTATION'], tag=i)
        xd, yd, zd, _ = await collect(dut)
        xr, yr, zr    = model.compute(x, y, z, Coord.LINR, Mode.ROTATION)
        ok, msg = check(f"linear_rot #{i}", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_hypr_vectoring(dut):
    await hw_reset(dut)
    random.seed(123)
    failures = []
    for i in range(15):
        mag  = random.randint(2000, SCALE//2)
        y    = round(mag * random.uniform(0, 0.8) * random.choice([1, -1]))
        await drive(dut, mag, y, 0, COORD_MAP['HYPR'], MODE_MAP['VECTORING'], tag=i)
        xd, yd, zd, _ = await collect(dut)
        xr, yr, zr    = model.compute(mag, y, 0, Coord.HYPR, Mode.VECTORING)
        ok, msg = check(f"hypr_vec #{i}", xd, yd, zd, xr, yr, zr)
        dut._log.info(msg)
        if not ok: failures.append(msg)
    assert not failures, "\n".join(failures)


async def sub_pipeline_throughput(dut, N=32):
    await hw_reset(dut)
    random.seed(7)
    inputs = []
    outputs = []

    # Collect outputs concurrently (must start before driving to catch early outputs)
    async def _collect():
        for _ in range(N + TOTAL_LAT + 10):
            await RisingEdge(dut.clk)
            if int(dut.m_valid.value) == 1:
                outputs.append((dut.m_x.value.to_signed(),
                                dut.m_y.value.to_signed(),
                                dut.m_z.value.to_signed()))
            if len(outputs) >= N:
                break

    collector = cocotb.start_soon(_collect())

    # Fire N back-to-back
    for i in range(N):
        z = random.randint(-SCALE, SCALE - 1)
        inputs.append(z)
        dut.s_valid.value = 1
        dut.s_x.value     = KINV_Q14 & 0xFFFF
        dut.s_y.value     = 0
        dut.s_z.value     = z & 0xFFFF
        dut.s_coord.value = 0
        dut.s_mode.value  = 0
        dut.s_tag.value   = i & 0xFF
        await RisingEdge(dut.clk)
    dut.s_valid.value = 0

    await collector

    assert len(outputs) == N, f"Expected {N} outputs, got {len(outputs)}"

    failures = []
    for i, (z, (xd, yd, zd)) in enumerate(zip(inputs, outputs)):
        xr, yr, zr = model.compute(KINV_Q14, 0, z, Coord.CIRC, Mode.ROTATION)
        ok, msg = check(f"pipe #{i}", xd, yd, zd, xr, yr, zr)
        if not ok: failures.append(msg)

    dut._log.info(f"Pipeline throughput: {N-len(failures)}/{N} passed")
    assert not failures, f"{len(failures)}/{N} failures:\n" + "\n".join(failures[:3])


async def sub_random_stress(dut, N=100):
    await hw_reset(dut)
    random.seed(0xCAFE)
    failures = []
    all_cases = [
        (COORD_MAP['CIRC'], MODE_MAP['ROTATION'],  Coord.CIRC, Mode.ROTATION),
        (COORD_MAP['CIRC'], MODE_MAP['VECTORING'], Coord.CIRC, Mode.VECTORING),
        (COORD_MAP['LINR'], MODE_MAP['ROTATION'],  Coord.LINR, Mode.ROTATION),
        (COORD_MAP['HYPR'], MODE_MAP['VECTORING'], Coord.HYPR, Mode.VECTORING),
    ]
    for i in range(N):
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

        await drive(dut, x, y, z, sv_coord, sv_mode, tag=i & 0xFF)
        xd, yd, zd, _ = await collect(dut)
        xr, yr, zr = model.compute(x, y, z, py_coord, py_mode)
        ok, msg = check(f"rand#{i} {py_coord.name}/{py_mode.name}",
                        xd, yd, zd, xr, yr, zr)
        if not ok: failures.append(msg)

    pass_cnt = N - len(failures)
    dut._log.info(f"Random stress: {pass_cnt}/{N} passed")
    assert not failures, f"{len(failures)}/{N} failures:\n" + "\n".join(failures[:5])


# ─────────────────────────────────────────────
# Master test (single @cocotb.test keeps sim alive)
# ─────────────────────────────────────────────
@cocotb.test()
async def test_all(dut):
    """
    CORDIC full regression.
    All sub-tests run in one simulation context to avoid
    Verilator re-initialization between tests.
    """
    # Start clock
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())

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

    await run("smoke",            sub_smoke(dut))
    await run("circ_rotation",    sub_circ_rotation(dut))
    await run("circ_vectoring",   sub_circ_vectoring(dut))
    await run("linear_rotation",  sub_linear_rotation(dut))
    await run("hypr_vectoring",   sub_hypr_vectoring(dut))
    await run("pipeline_throughput", sub_pipeline_throughput(dut))
    await run("random_stress",    sub_random_stress(dut))

    # Summary
    passed = sum(1 for v in results.values() if v == "PASS")
    total  = len(results)
    dut._log.info(f"\n{'='*50}")
    dut._log.info(f"CORDIC REGRESSION: {passed}/{total} PASSED")
    for name, result in results.items():
        dut._log.info(f"  {'PASS' if result=='PASS' else 'FAIL'}: {name}")
    dut._log.info('='*50)

    failed = [n for n, v in results.items() if v != "PASS"]
    assert not failed, f"Failed subtests: {failed}"
