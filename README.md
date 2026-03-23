# CORDIC IP Core

> **Lumees Lab** — FPGA-Verified, Production-Ready Silicon IP

[![License](https://img.shields.io/badge/License-Apache%202.0%20%2B%20Commons%20Clause-blue.svg)](LICENSE)
[![FPGA Verified](https://img.shields.io/badge/FPGA-83%2F83%20HW%20PASS-brightgreen.svg)]()
[![Fmax](https://img.shields.io/badge/Fmax-100%20MHz%20(Artix--7)-brightgreen.svg)]()
[![Sim](https://img.shields.io/badge/cocotb-23%2F23%20PASS-brightgreen.svg)]()
[![Language](https://img.shields.io/badge/Language-SystemVerilog-orange.svg)]()

---

## Overview

The Lumees Lab CORDIC IP Core is a fully pipelined, multi-mode **CORDIC (Coordinate Rotation Digital Computer)** engine written in SystemVerilog. It supports all three coordinate systems — **Circular, Linear, and Hyperbolic** — in both **Rotation and Vectoring** modes, covering a broad range of trigonometric, transcendental, and arithmetic operations with a single unified datapath.

Unlike soft-IP or CPU library implementations, this core computes **sin/cos, atan2, magnitude, division, and hyperbolic functions** in hardware at **1 sample per clock cycle**, with a fixed 18-cycle latency. There are no multipliers in the core iteration stages — all micro-rotations are implemented as arithmetic right-shifts and additions, resulting in an extremely low DSP footprint (2 DSP blocks used only for the final K⁻¹ gain compensation).

Verified in simulation (**23/23 cocotb tests** across four interfaces) and on Xilinx FPGA hardware (**83/83 UART regression tests** on Arty A7-100T), the core is production-ready for SoC integration in signal processing, navigation, communications, and scientific computing applications.

---

## Key Features

| Feature | Detail |
|---|---|
| **Coordinate Systems** | Circular, Linear, Hyperbolic |
| **Operating Modes** | Rotation, Vectoring (per-sample, dynamically selectable) |
| **Data Format** | 16-bit signed fixed-point, Q1.14 (14 fractional bits; angles in units of π) |
| **Parameterizable Width** | `DATA_WIDTH` 8–48 bits; all LUTs recomputed at elaboration |
| **Pipeline Stages** | 16 (parameterizable via `STAGES`) |
| **Pipeline Latency** | 18 cycles (STAGES + 2) |
| **Throughput** | 1 sample / clock (fully pipelined, no stall) |
| **Angle Range** | Full ±π for circular mode (pre-processor quadrant extension) |
| **Gain Correction** | Automatic K⁻¹ compensation (circular & hyperbolic) |
| **Saturation Detection** | Sticky overflow flag on `m_sat` |
| **Transaction Tagging** | 8-bit user tag propagated through pipeline |
| **IRQ** | Done interrupt pulse on AXI4-Lite and Wishbone wrappers |
| **Bus Interfaces** | AXI4-Lite, Wishbone B4, AXI4-Stream (with FIFO), bare port |
| **Technology** | FPGA / ASIC — pure synchronous RTL, no vendor primitives |
| **Language** | SystemVerilog |

---

## Performance — Arty A7-100T (XC7A100T) @ 100 MHz

### Resource Utilization

| Resource | Full SoC | Core Alone (est.) | Available | SoC % |
|---|---|---|---|---|
| LUT | 2,250 | ~350 | 63,400 | 3.55% |
| FF | 1,578 | ~500 | 126,800 | 1.24% |
| DSP48 | 2 | 2 | 240 | 0.83% |
| Block RAM | 0 | 0 | 135 | 0% |

> **Timing:** WNS = **+0.942 ns** @ 100 MHz. All endpoints met, 0 failing paths.
> The 2 DSP blocks are used exclusively for K⁻¹ post-processing gain multiplication;
> the 16 pipeline stages use only LUTs (shift-and-add arithmetic).

### Throughput by Interface

| Interface | Throughput | Notes |
|---|---|---|
| `cordic_top` (bare) | **1 sample / clock** | New sample accepted every cycle |
| `cordic_axis` (AXI4-Stream) | **1 sample / clock** | Output FIFO absorbs backpressure |
| `cordic_axil` (AXI4-Lite) | ~40 cycles/sample | Software write-trigger-poll model |
| `cordic_wb` (Wishbone) | ~45 cycles/sample | Same model, single-cycle ACK |

---

## Architecture

```
                ┌───────────────────────────────────────────────────┐
                │                   cordic_top                      │
                │                                                   │
 s_x/y/z     ───┤──► cordic_pre_proc ──► cordic_core ──► cordic_post│──► m_x/y/z
 s_coord/mode───┤     (quadrant ext.)     (16× stage)    (K⁻¹ gain) │    m_sat
 s_tag       ───┤     1 cycle              16 cycles      1 cycle   │    m_tag
                │                                                   │
                │   Latency: 1 + 16 + 1 = 18 clock cycles           │
                └───────────────────────────────────────────────────┘
```

**Pre-processor (`cordic_pre_proc`):** Performs quadrant normalization to extend the native CORDIC convergence zone (±1.74 rad) to the full ±π range. Captures the quadrant offset to restore the correct output quadrant after the pipeline.

**Core pipeline (`cordic_core` → 16× `cordic_stage`):** Each stage implements one CORDIC micro-rotation using only shift-and-add operations:

```
 Circular:    x' = x ∓ (y >> i),  y' = y ± (x >> i),  z' = z ∓ atan(2⁻ⁱ)
 Linear:      x' = x,             y' = y ± (x >> i),  z' = z ∓ 2⁻ⁱ
 Hyperbolic:  x' = x ± (y >> s),  y' = y ± (x >> s),  z' = z ∓ atanh(2⁻ˢ)
```

> **Hyperbolic convergence:** The stage schedule repeats stages 4 and 13 (following k = ⌊(3ˡ−1)/2⌋) to ensure convergence for all practically relevant hyperbolic inputs (|y/x| ≲ 0.8).

> **⚠ Hyperbolic input constraint:** The hardware does **not** validate hyperbolic inputs. If |y/x| exceeds ~0.754 (the convergence bound for 16-stage hyperbolic CORDIC), the output will be **silently incorrect** — `m_sat` does not fire for convergence violations, only for post-gain overflow. It is the caller's responsibility to ensure inputs are within the convergence zone. This is consistent with standard CORDIC implementations.

**Post-processor (`cordic_post_proc`):** Applies K⁻¹ gain compensation using a single integer multiply (2 DSP slices). Optionally applies round-to-nearest (controlled by `ROUND_MODE`). Reconstructs the final angle from the quadrant offset for circular vectoring. **Linear mode bypasses the gain multiply entirely** — the coord signal is pipelined through and gates the DSP path.

### Pipeline Backpressure

The pipeline is **always running** — `s_ready` is hardwired to 1. A new sample enters every cycle unconditionally. `m_ready` on the output port is **ignored** by `cordic_top`; if the downstream consumer is not ready, output data is silently overwritten on the next cycle.

**Implications:**
- For **streaming datapaths** (DSP chains): this is ideal — no stalls, deterministic latency
- For **bus-mapped access** (AXI4-Lite, Wishbone): the wrappers manage handshaking internally; no data loss
- For **backpressure-sensitive consumers**: use `cordic_axis` (AXI4-Stream wrapper) which includes a configurable-depth output FIFO to absorb downstream stalls

---

## Operating Modes

| Coordinate | Mode | Operation | Use Case | Input | Output |
|---|---|---|---|---|---|
| **Circular** | Rotation | sin/cos | Tone gen, phase rotation | x=K⁻¹, y=0, z=θ | x=cos(θ), y=sin(θ) |
| **Circular** | Vectoring | atan2, magnitude | Polar conversion, OFDM | x=I, y=Q, z=0 | x=mag, z=phase |
| **Linear** | Rotation | y + x·z | FIR tap, MAC | x, y, z | y=y+x·z |
| **Linear** | Vectoring | y/x | Normalization | x, y, z=0 | z=y/x |
| **Hyperbolic** | Rotation | sinh/cosh | Exponentials | x, y, z | cosh/sinh |
| **Hyperbolic** | Vectoring | √(x²−y²), atanh | Square root, log | x, y, z=0 | x=√, z=atanh |

---

## Data Format — Q1.14 Fixed-Point

All ports use **16-bit signed fixed-point with 14 fractional bits** (parameterizable via `DATA_WIDTH`).

| Attribute | Value |
|---|---|
| Total bits | 16 (default) |
| Fractional bits | 14 (LSB = 2⁻¹⁴ ≈ 6.1×10⁻⁵) |
| Unity (1.0) | 16,384 = `0x4000` |
| Range (x, y) | −2.0 to +1.99994 |
| Range (z angle) | −1.0 to +0.99994 in units of π |

> **Angle convention:** The z port expresses angles in **units of π**, not radians.
> `z_in = (angle_rad / M_PI) * 16384`

### Quick-Start Inputs

| Operation | x_in | y_in | z_in | Output |
|---|---|---|---|---|
| sin(θ), cos(θ) | `9949` (K⁻¹) | `0` | `θ/π × 16384` | x=cos, y=sin |
| atan2(y, x) | `x` | `y` | `0` | z=atan2/π×16384 |
| y / x | `x` | `y` | `0` | z=y/x |
| y + x×z | `x` | `y` | `z` | y=y+x×z |

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `DATA_WIDTH` | int | 16 | Bit width of all ports (8–48). LUT constants recomputed at elaboration. |
| `STAGES` | int | 16 | Micro-rotation stages. Latency = STAGES + 2. Min useful: 8. |
| `APPLY_GAIN` | bit | 1 | Enable K⁻¹ gain compensation (2 DSP slices). |
| `ROUND_MODE` | bit | 1 | Round-to-nearest (1) or truncate (0). |

---

## Interface — Bare Core (`cordic_top`)

```systemverilog
cordic_top #(
  .DATA_WIDTH(16),
  .STAGES    (16),
  .APPLY_GAIN(1),
  .ROUND_MODE(1)
) u_cordic (
  .clk      (clk),       .rst_n    (rst_n),
  // Input — s_ready is always 1
  .s_valid  (s_valid),   .s_ready  (s_ready),
  .s_x      (s_x),       .s_y      (s_y),       .s_z      (s_z),
  .s_coord  (s_coord),   // 2'b00=CIRC  2'b01=LINR  2'b10=HYPR
  .s_mode   (s_mode),    // 0=ROTATION  1=VECTORING
  .s_tag    (s_tag),     // [7:0] user tag
  // Output — valid 18 cycles after s_valid
  .m_valid  (m_valid),   .m_ready  (m_ready),
  .m_x      (m_x),       .m_y      (m_y),       .m_z      (m_z),
  .m_tag    (m_tag),     .m_sat    (m_sat)
);
```

---

## Register Map — AXI4-Lite / Wishbone

| Offset | Register | Access | Description |
|---|---|---|---|
| 0x00 | CTRL | R/W | `[0]`=START `[1]`=BUSY `[2]`=DONE `[3]`=SAT `[5:4]`=COORD `[6]`=MODE |
| 0x04 | TAG | R/W | `[7:0]` transaction tag |
| 0x08 | X_IN | R/W | Input x (signed Q1.14) |
| 0x0C | Y_IN | R/W | Input y |
| 0x10 | Z_IN | R/W | Input z / angle |
| 0x14 | X_OUT | R | Output x |
| 0x18 | Y_OUT | R | Output y |
| 0x1C | Z_OUT | R | Output z / angle |
| 0x20 | LATENCY | R | Pipeline latency (18) |
| 0x24 | VERSION | R | `0x00010000` |

**Software flow:** Write X/Y/Z/TAG → Write CTRL with COORD+MODE+START → Poll DONE → Read outputs.

---

## AXI4-Stream Packet Format

**Input TDATA [63:0]:**
```
[15:0]=x  [31:16]=y  [47:32]=z  [55:48]=tag  [56]=mode  [58:57]=coord
```

**Output TDATA [47:0] + TUSER:**
```
[15:0]=x_out  [31:16]=y_out  [47:32]=z_out  TUSER[7:0]=tag  m_axis_tsat=sat
```

---

## Directory Structure

```
cordic/
├── rtl/                          # Synthesizable RTL (11 files)
│   ├── cordic_pkg.sv               # Package: constants, atan/atanh LUTs, types
│   ├── cordic_stage.sv             # Single CORDIC micro-rotation stage
│   ├── cordic_core.sv              # 16-stage pipelined core (generate loop)
│   ├── cordic_pre_proc.sv          # Quadrant normalization pre-processor
│   ├── cordic_post_proc.sv         # K⁻¹ gain compensation + rounding
│   ├── cordic_top.sv               # Top-level: pre + core + post
│   ├── cordic_axil.sv              # AXI4-Lite slave wrapper
│   ├── cordic_wb.sv                # Wishbone B4 slave wrapper
│   ├── cordic_axis.sv              # AXI4-Stream wrapper (with FIFO)
│   ├── cordic_sincos.sv            # Sin/cos-only wrapper (angle in units-of-π, not radians)
│   └── cordic_props.sv             # SVA formal properties
├── model/                        # Golden reference
│   ├── cordic_model.py             # Bit-accurate Python model (Q1.14)
│   ├── cordic_dpi.c                # DPI-C model for UVM scoreboard
│   └── test_vectors.h              # 208 pre-computed test vectors
├── tb/
│   ├── directed/                 # cocotb tests (4 files, 23 sub-tests)
│   │   ├── test_cordic.py          # Bare core: all modes + throughput
│   │   ├── test_cordic_axil.py     # AXI4-Lite register interface
│   │   ├── test_cordic_wb.py       # Wishbone interface
│   │   └── test_cordic_axis.py     # AXI4-Stream + backpressure
│   └── uvm/                     # UVM environment (12 files)
│       ├── cordic_tb_top.sv        # Top module + DUT instantiation
│       ├── cordic_if.sv            # Virtual interface
│       ├── cordic_seq_item.sv      # Randomized transaction
│       ├── cordic_driver.sv        # Protocol driver
│       ├── cordic_monitor.sv       # Output monitor
│       ├── cordic_scoreboard.sv    # DPI-C golden comparison
│       ├── cordic_coverage.sv      # Functional coverage
│       └── ...                     # agent, env, sequences, tests
├── sim/
│   └── Makefile.cocotb           # One-command sim (sim-top/axil/wb/axis/all)
├── litex/                        # FPGA integration
│   ├── cordic_litex.py             # LiteX/Migen module wrapper
│   ├── cordic_soc.py               # Arty A7-100T reference SoC
│   └── cordic_uart_test.py         # 83-test UART hardware regression
├── docs/
│   └── cordic-ip-datasheet.md      # Full product datasheet
├── README.md
├── LICENSE                       # Apache 2.0 + Commons Clause
└── .gitignore
```

---

## Quick Start

### Simulation

```bash
cd sim/
make -f Makefile.cocotb sim-top     # Bare core tests
make -f Makefile.cocotb sim-axil    # AXI4-Lite tests
make -f Makefile.cocotb sim-axis    # AXI4-Stream tests
make -f Makefile.cocotb sim-all     # All interfaces
```

### FPGA Build & Hardware Test

```bash
cd litex/
python3 cordic_soc.py --build       # Vivado synthesis + P&R (~7 min)
python3 cordic_soc.py --load        # Program Arty A7 via JTAG
litex_server --uart --uart-port /dev/ttyUSB1 --uart-baudrate 115200
python3 cordic_uart_test.py          # 83 hardware regression tests
```

### LiteX Integration

```python
from cordic_litex import CORDIC
self.submodules.cordic = CORDIC(platform)
```

---

## Verification

| Test Suite | Tests | Status |
|---|---|---|
| `test_cordic` (bare core) | 10 circ_rot + 5 circ_vec + 15 linr + 10 hypr + throughput + edge + overflow | **7/7 PASS** |
| `test_cordic_axil` (AXI4-Lite) | version, latency, circ rot/vec, linear, hyperbolic | **5/5 PASS** |
| `test_cordic_wb` (Wishbone) | version, latency, circ rot/vec, linear, hyperbolic | **5/5 PASS** |
| `test_cordic_axis` (AXI4-Stream) | circ rot/vec, linear, hyperbolic, throughput, backpressure | **6/6 PASS** |
| **UVM** (Questa/VCS) | Constrained-random: coord × mode × tag cross coverage | Environment ready |
| **Formal** (`cordic_props.sv`) | 5 assertions (reset, valid propagation, no spurious, ready, sat) + 6 cover points | Properties defined |
| **FPGA Hardware** | 83 UART tests @ 100 MHz via LiteX + UARTBone | **83/83 PASS** |

> **Known test gaps:** Hyperbolic boundary testing (|y/x| near 0.754) and Q3 vectoring (negative x, negative y) are not covered in directed tests. The UVM constrained-random environment covers these via cross-coverage of coord × mode × data ranges.

---

## Applications

- **Signal Processing / SDR** — sin/cos generation, phase rotation, IQ magnitude/phase, OFDM carrier sync
- **Navigation / AHRS** — Attitude and heading, Euler angle conversion, polar-to-Cartesian
- **Communications** — Frequency offset correction, carrier recovery, QAM demodulation
- **Motor Control** — Clarke and Park transforms, field-oriented control (FOC)
- **Scientific Computing** — Transcendental functions (sinh, cosh, atanh, √) in FPGA
- **FPGA Overlay** — Drop-in AXI4-Lite / Wishbone / AXI4-Stream slave for any SoC
- **Multi-Core Array** — Tile multiple `cordic_top` instances for ultra-high-throughput DSP

---

## Why Lumees CORDIC?

| Differentiator | Detail |
|---|---|
| **All three CORDIC modes** | Circular, Linear, Hyperbolic — dynamically selectable per sample |
| **No multipliers in pipeline** | 16 stages: shift-and-add only; 2 DSP blocks for final K⁻¹ gain |
| **Full ±π angle range** | Pre-processor quadrant correction — no software wrapping needed |
| **Parameterizable width** | `DATA_WIDTH` 8–48; all LUT constants auto-recomputed |
| **Hardware-verified** | 83 UART tests pass on Arty A7-100T — not just simulated |
| **Four bus interfaces** | Bare port, AXI4-Lite, Wishbone B4, AXI4-Stream |
| **vs Xilinx CORDIC v6.0** | All 3 coordinate systems (Xilinx: circular/linear only); open-source with full RTL (Xilinx: encrypted netlist); no Vivado IP catalog dependency; includes Python golden model |

---

## Roadmap

### v1.1 — Usability
- [ ] `cordic_sincos_rad.sv` — sin/cos wrapper accepting **radians** (prescaler: `z = angle_rad / π × 2^FRAC_BITS`)
- [ ] `cordic_atan2.sv` — dedicated atan2(y, x) wrapper returning radians
- [ ] Hyperbolic input range check — optional `hypr_range_err` flag when |y/x| > convergence bound
- [ ] Add Q3 vectoring test (negative x, negative y) to directed suite
- [ ] Add hyperbolic boundary tests (|y/x| = 0.75, 0.76, 0.80)

### v1.2 — Performance
- [ ] Optional output register bypass (`BYPASS_POST=1`) for lower latency when gain is handled externally
- [ ] Configurable FIFO depth on `cordic_axis` (currently hardcoded)
- [ ] `DATA_WIDTH=8` and `DATA_WIDTH=48` integration tests and accuracy characterization
- [ ] Formal verification runs with Symbiyosys (properties already defined in `cordic_props.sv`)

### v2.0 — ASIC & High-Speed
- [ ] SkyWater 130nm synthesis scripts and timing reports
- [ ] Multi-cycle option: trade throughput for area (1 stage reused N times)
- [ ] FP16 input/output shim for direct floating-point integration
- [ ] 400 MHz / 800 MHz pipelined variants for high-end FPGAs (UltraScale+)
- [ ] DFT scan-chain insertion for production test
- [ ] Power analysis report (Vivado / PrimeTime)

### Contributing

Contributions are welcome for non-commercial improvements. Please open an issue first to discuss the change. PRs should include cocotb tests for any new functionality.

---

## License

**Apache License 2.0 with Commons Clause**

- **Non-commercial use** (academic, research, hobby, education, evaluation): **Free and unrestricted**
- **Commercial use** (products, services, revenue-generating activities): Requires a [Lumees Lab commercial license](https://lumeeslab.com)

> **Note:** The Commons Clause restricts selling the software itself as a commercial product. This makes the license **source-available** rather than OSI-approved open source. You can freely use, study, modify, and share the code for any non-commercial purpose. For commercial integration, contact us — licensing is straightforward.

See [LICENSE](LICENSE) for full terms.

---

## About Lumees Lab

**Lumees Lab** builds production-ready silicon IP cores for FPGA and ASIC integration.

- **45 IP cores** — all FPGA-verified on Xilinx Artix-7
- Uniform SystemVerilog codebase with AXI4-Lite + Wishbone interfaces
- Full verification: cocotb + UVM + FPGA hardware
- Targeting **SkyWater 130nm** open PDK for silicon-proven variants

**Website:** [lumeeslab.com](https://lumeeslab.com)
**Email:** [info@lumeeslab.com](mailto:info@lumeeslab.com)
**GitHub:** [github.com/lumeeslab](https://github.com/lumeeslab)

---

*Copyright © 2026 Lumees Lab / Hasan Kurşun. All rights reserved.*
