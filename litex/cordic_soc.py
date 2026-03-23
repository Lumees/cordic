#!/usr/bin/env python3
# =============================================================================
# Copyright (c) 2026 Lumees Lab / Hasan Kurşun
# SPDX-License-Identifier: Apache-2.0 WITH Commons-Clause
#
# Free for non-commercial use (academic, research, hobby, education).
# Commercial use requires a Lumees Lab license: info@lumeeslab.com
# =============================================================================

# ── Workaround: LiteX CSR auto-naming fails on Python 3.12 ──────────────────
import itertools as _it
_csr_counter = _it.count()
import litex.soc.interconnect.csr as _litex_csr
def _CSRBase_patched_init(self, size, name, n=None):
    from migen.fhdl.tracer import get_obj_var_name
    try: resolved = get_obj_var_name(name)
    except Exception: resolved = None
    if resolved is None: resolved = name if name is not None else f"_csr_{next(_csr_counter)}"
    from migen import DUID; DUID.__init__(self)
    self.n = n; self.fixed = n is not None; self.size = size; self.name = resolved
_litex_csr._CSRBase.__init__ = _CSRBase_patched_init

"""
CORDIC SoC for Arty A7-100T
============================
Builds a LiteX SoC with:
  - VexRiscv CPU
  - UART (115200 baud, for test/debug)
  - CORDIC IP (AXI4-Lite / Wishbone CSR)
  - SRAM
  - LED control CSR

Usage:
    cd litex/
    python3 cordic_soc.py --build        # synthesize + implement
    python3 cordic_soc.py --load         # program Arty via USB
    python3 cordic_soc.py --build --load # build and load

Then test:
    python3 cordic_uart_test.py          # run UART regression
"""

import argparse
import os
import sys

from migen import *

# LiteX core
from litex.soc.cores.clock            import S7PLL
from litex.soc.integration.soc_core   import SoCCore, soc_core_argdict, soc_core_args
from litex.soc.integration.builder    import Builder, builder_argdict, builder_args
from litex.soc.cores.uart             import UARTWishboneBridge
from litex.soc.interconnect.csr       import *
from litex.soc.cores.gpio             import GPIOOut

# LiteX boards
from litex_boards.platforms import digilent_arty

# CORDIC module
sys.path.insert(0, os.path.dirname(__file__))
from cordic_litex import CORDIC


# ─────────────────────────────────────────────
# CRG: Clock and Reset Generator for Arty A7
# ─────────────────────────────────────────────
class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys = ClockDomain("sys")

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(~platform.request("cpu_reset") | self.rst)
        pll.register_clkin(platform.request("clk100"), 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        platform.add_false_path_constraints(
            self.cd_sys.clk,
        )


# ─────────────────────────────────────────────
# CORDIC SoC
# ─────────────────────────────────────────────
class CORDICSoC(SoCCore):
    def __init__(self, sys_clk_freq=100e6, **kwargs):
        platform = digilent_arty.Platform(variant="a7-100")

        # No CPU needed for register access testing via uartbone.
        # uartbone speaks wishbone-over-UART, accessed by litex RemoteClient.
        kwargs["cpu_type"]              = None
        kwargs["uart_name"]             = "uartbone"
        kwargs["integrated_rom_size"]   = 0
        kwargs["integrated_sram_size"]  = 0
        SoCCore.__init__(self, platform,
            clk_freq = sys_clk_freq,
            ident    = "CORDIC IP Test SoC - Arty A7-100T",
            **kwargs
        )

        # CRG
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # CORDIC IP
        self.submodules.cordic = cordic = CORDIC(platform, sys_clk_freq)
        self.add_csr("cordic")

        # LEDs (4 green LEDs on Arty)
        leds = platform.request_all("user_led")
        self.submodules.leds = GPIOOut(leds)
        self.add_csr("leds")

        # RGB LEDs
        # rgb_leds = platform.request_all("user_rgb_led")

        # Board constraints
        platform.add_platform_command(
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]"
        )
        platform.add_platform_command(
            "set_property CONFIG_VOLTAGE 3.3 [current_design]"
        )
        platform.add_platform_command(
            "set_property CFGBVS VCCO [current_design]"
        )


# ─────────────────────────────────────────────
# Build / Load
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="CORDIC SoC for Arty A7-100T")
    parser.add_argument("--build",          action="store_true", help="Build design")
    parser.add_argument("--load",           action="store_true", help="Load bitstream via JTAG")
    parser.add_argument("--sys-clk-freq",   default=100e6, type=float, help="System clock frequency")
    builder_args(parser)
    soc_core_args(parser)
    args = parser.parse_args()

    soc = CORDICSoC(
        sys_clk_freq = int(args.sys_clk_freq),
        **soc_core_argdict(args)
    )

    builder = Builder(soc, **builder_argdict(args))
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(
            os.path.join(builder.gateware_dir, soc.build_name + ".bit")
        )


if __name__ == "__main__":
    main()
