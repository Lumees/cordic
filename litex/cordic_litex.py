# =============================================================================
# Copyright (c) 2026 Lumees Lab / Hasan Kurşun
# SPDX-License-Identifier: Apache-2.0 WITH Commons-Clause
#
# Free for non-commercial use (academic, research, hobby, education).
# Commercial use requires a Lumees Lab license: info@lumeeslab.com
# =============================================================================
"""
CORDIC LiteX Module
===================
Directly instantiates cordic_top.sv and wires it to LiteX CSR registers.
No intermediate Wishbone bridge — inputs are driven from CSR storage,
outputs are latched into CSR status when m_valid pulses.

Register map (CSR offsets, each register 32-bit wide):
  ctrl      [0]=trigger (self-clearing), [2]=output_valid (RO)
  coord     [1:0] coordinate system (0=CIRC, 1=LINR, 2=HYPR)
  mode      [0]   operation mode    (0=ROTATION, 1=VECTORING)
  tag       [7:0] transaction tag
  x_in      signed 16-bit input X (Q2.14)
  y_in      signed 16-bit input Y (Q2.14)
  z_in      signed 16-bit input Z/angle (Q2.14)
  x_out     signed 16-bit output X (RO, updated when output_valid set)
  y_out     signed 16-bit output Y (RO)
  z_out     signed 16-bit output Z (RO)
  out_tag   8-bit output tag (RO)
  latency   constant = STAGES+2 = 18 (RO)
  version   constant = 0x00010000    (RO)
"""

from migen import *
from litex.soc.interconnect.csr import *

import os

CORDIC_RTL_DIR = os.path.join(os.path.dirname(__file__), '../rtl')

# Must match RTL constants
STAGES       = 16
TOTAL_LAT    = STAGES + 2
IP_VERSION   = 0x00010000


class CORDIC(Module, AutoCSR):
    def __init__(self, platform, sys_clk_freq=100e6):
        # ── Platform sources ───────────────────────────────────────────────
        for f in ['cordic_pkg.sv', 'cordic_stage.sv', 'cordic_core.sv',
                  'cordic_pre_proc.sv', 'cordic_post_proc.sv', 'cordic_top.sv']:
            platform.add_source(os.path.join(CORDIC_RTL_DIR, f))

        # ── CSR registers ──────────────────────────────────────────────────
        # Inputs (RW)
        self.ctrl  = CSRStorage(8,  name="ctrl",
                                description="[0]=trigger(self-clearing) [2]=out_valid(RO)")
        self.coord = CSRStorage(2,  name="coord",
                                description="0=CIRC 1=LINR 2=HYPR")
        self.mode  = CSRStorage(1,  name="mode",
                                description="0=ROTATION 1=VECTORING")
        self.tag   = CSRStorage(8,  name="tag",
                                description="Transaction tag [7:0]")
        self.x_in  = CSRStorage(16, name="x_in",  description="Input X (Q2.14 signed)")
        self.y_in  = CSRStorage(16, name="y_in",  description="Input Y (Q2.14 signed)")
        self.z_in  = CSRStorage(16, name="z_in",  description="Input Z/angle (Q2.14 signed)")

        # Outputs (RO)
        self.x_out   = CSRStatus(16, name="x_out",   description="Output X")
        self.y_out   = CSRStatus(16, name="y_out",   description="Output Y")
        self.z_out   = CSRStatus(16, name="z_out",   description="Output Z")
        self.out_tag = CSRStatus(8,  name="out_tag", description="Output tag")
        self.latency = CSRStatus(32, name="latency", description="Pipeline latency (cycles)")
        self.version = CSRStatus(32, name="version", description="IP version")

        # Constant outputs
        self.comb += [
            self.latency.status.eq(TOTAL_LAT),
            self.version.status.eq(IP_VERSION),
        ]

        # ── CORDIC core signals ────────────────────────────────────────────
        s_valid = Signal()
        s_ready = Signal()
        m_valid = Signal()
        m_sat   = Signal()
        m_x     = Signal((16, True))
        m_y     = Signal((16, True))
        m_z     = Signal((16, True))
        m_tag   = Signal(8)

        # ── Instantiate cordic_top ─────────────────────────────────────────
        self.specials += Instance("cordic_top",
            p_APPLY_GAIN = 1,

            i_clk   = ClockSignal(),
            i_rst_n = ~ResetSignal(),

            i_s_valid = s_valid,
            o_s_ready = s_ready,
            i_s_x     = self.x_in.storage,
            i_s_y     = self.y_in.storage,
            i_s_z     = self.z_in.storage,
            i_s_coord = self.coord.storage,
            i_s_mode  = self.mode.storage,
            i_s_tag   = self.tag.storage,

            o_m_valid = m_valid,
            i_m_ready = 1,
            o_m_x     = m_x,
            o_m_y     = m_y,
            o_m_z     = m_z,
            o_m_tag   = m_tag,
            o_m_sat   = m_sat,
        )

        # ── Trigger: pulse s_valid for one clock when ctrl[0] written ─────
        trigger = Signal()
        out_valid = Signal()

        self.sync += [
            s_valid.eq(0),
            If(self.ctrl.re & self.ctrl.storage[0],
                s_valid.eq(1),
                out_valid.eq(0),  # clear previous result
            ),
        ]

        # ── Latch outputs when m_valid pulses ──────────────────────────────
        sat_sticky = Signal()
        self.sync += [
            If(m_valid,
                self.x_out.status.eq(m_x),
                self.y_out.status.eq(m_y),
                self.z_out.status.eq(m_z),
                self.out_tag.status.eq(m_tag),
                sat_sticky.eq(sat_sticky | m_sat),
                out_valid.eq(1),
            ),
        ]

        # ── Reflect out_valid in ctrl[2] ───────────────────────────────────
        # ctrl is a CSRStorage but we need to OR in a status bit.
        # Use a separate CSRStatus for the done flag or mix into ctrl read-back.
        # Simplest: add a dedicated done CSR status bit via ctrl.
        # We do this by wiring out_valid into the ctrl read-back register.
        # NOTE: CSRStorage read-back always returns .storage. We can't easily
        # inject out_valid into it. Use a separate mechanism: the host should
        # re-read x_out (non-zero) or poll a separate status flag.
        # Use a 1-bit CSRStatus "done" register:
        self.done = CSRStatus(1, name="done", description="1 = output is valid/ready")
        self.comb += self.done.status.eq(out_valid)
        self.sat  = CSRStatus(1, name="sat",  description="1 = saturation/overflow occurred")
        self.comb += self.sat.status.eq(sat_sticky)
