# =============================================================================
# Copyright (c) 2026 Lumees Lab
# SPDX-License-Identifier: MIT
# =============================================================================
# CORDIC UVM Testbench — Makefile
# Supports: Questa / ModelSim (default), VCS (SIM=vcs)
# =============================================================================
#
# Usage:
#   make sim              # compile + run with default test (cordic_directed_test)
#   make sim-directed     # directed test
#   make sim-random       # random transactions
#   make sim-stress       # back-to-back stress
#   make compile          # compile only
#   make run              # run only (must compile first)
#   make clean            # remove build artefacts
#
# Overrides:
#   make sim UVM_TEST=cordic_random_test SEED=42
#   make sim SIM=vcs
# =============================================================================

# -- Simulator selection -------------------------------------------------------
SIM       ?= questa

# -- Design under test --------------------------------------------------------
TOP        = cordic_tb_top
RTL_DIR    = ../../rtl
UVM_HOME  ?= $(QUESTA_HOME)/verilog_src/uvm-1.2

# -- Test / DUT parameters ----------------------------------------------------
UVM_TEST  ?= cordic_directed_test
UVM_VERB  ?= UVM_MEDIUM
SEED      ?= 1

# -- Build directories --------------------------------------------------------
BUILD_DIR  = work
LOG_DIR    = logs

# -- RTL source list ----------------------------------------------------------
RTL_SRCS = \
	$(RTL_DIR)/cordic_pkg.sv          \
	$(RTL_DIR)/cordic_stage.sv        \
	$(RTL_DIR)/cordic_pre_proc.sv     \
	$(RTL_DIR)/cordic_core.sv         \
	$(RTL_DIR)/cordic_post_proc.sv    \
	$(RTL_DIR)/cordic_top.sv

# -- TB source (top includes all others via `include) -------------------------
TB_TOP = cordic_tb_top.sv

# =============================================================================
# Questa / ModelSim targets
# =============================================================================
ifeq ($(SIM),questa)

VLOG  = vlog
VSIM  = vsim

VLOG_FLAGS  = -sv \
              +incdir+$(RTL_DIR) \
              +incdir+. \
              -timescale "1ns/1ps" \
              -work $(BUILD_DIR)

VSIM_FLAGS  = -sv_seed $(SEED) \
              -t 1ns \
              +UVM_TESTNAME=$(UVM_TEST) \
              +UVM_VERBOSITY=$(UVM_VERB) \
              -permit_unmatched_virtual_intf \
              -do "run -all; quit -f"

.PHONY: compile
compile: $(BUILD_DIR)/_lib.qdb

$(BUILD_DIR)/_lib.qdb: $(RTL_SRCS) $(TB_TOP)
	@mkdir -p $(BUILD_DIR) $(LOG_DIR)
	$(VLOG) -work $(BUILD_DIR) $(VLOG_FLAGS) $(RTL_SRCS) $(TB_TOP) \
		2>&1 | tee $(LOG_DIR)/compile.log
	@echo "Compilation complete."

.PHONY: run
run:
	@mkdir -p $(LOG_DIR)
	$(VSIM) $(VSIM_FLAGS) \
		-work $(BUILD_DIR) \
		$(TOP) \
		2>&1 | tee $(LOG_DIR)/run_$(UVM_TEST).log

.PHONY: sim
sim: compile run

# Named convenience targets
.PHONY: sim-directed
sim-directed:
	$(MAKE) sim UVM_TEST=cordic_directed_test

.PHONY: sim-random
sim-random:
	$(MAKE) sim UVM_TEST=cordic_random_test

.PHONY: sim-stress
sim-stress:
	$(MAKE) sim UVM_TEST=cordic_stress_test UVM_VERB=UVM_MEDIUM

# Run all three in sequence
.PHONY: sim-all
sim-all: sim-directed sim-random sim-stress

endif  # questa

# =============================================================================
# VCS targets
# =============================================================================
ifeq ($(SIM),vcs)

VCS      = vcs
SIMV_OUT = simv

VCS_FLAGS = -sverilog \
            -ntb_opts uvm \
            -full64 \
            -debug_access+all \
            +incdir+$(RTL_DIR) \
            +incdir+. \
            -timescale=1ns/1ps \
            -o $(SIMV_OUT)

.PHONY: compile
compile:
	@mkdir -p $(LOG_DIR)
	$(VCS) $(VCS_FLAGS) $(RTL_SRCS) $(TB_TOP) \
		2>&1 | tee $(LOG_DIR)/compile.log

.PHONY: run
run:
	@mkdir -p $(LOG_DIR)
	./$(SIMV_OUT) +UVM_TESTNAME=$(UVM_TEST) +UVM_VERBOSITY=$(UVM_VERB) \
		+ntb_random_seed=$(SEED) \
		2>&1 | tee $(LOG_DIR)/run_$(UVM_TEST).log

.PHONY: sim
sim: compile run

.PHONY: sim-directed
sim-directed:
	$(MAKE) sim UVM_TEST=cordic_directed_test

.PHONY: sim-random
sim-random:
	$(MAKE) sim UVM_TEST=cordic_random_test

.PHONY: sim-stress
sim-stress:
	$(MAKE) sim UVM_TEST=cordic_stress_test UVM_VERB=UVM_MEDIUM

.PHONY: sim-all
sim-all: sim-directed sim-random sim-stress

endif  # vcs

# =============================================================================
# Common targets
# =============================================================================
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR) $(LOG_DIR) $(SIMV_OUT) simv.daidir csrc ucli.key

.PHONY: help
help:
	@echo "Targets: compile, run, sim, sim-directed, sim-random, sim-stress, sim-all, clean"
	@echo "Variables: SIM=questa|vcs  UVM_TEST=<test>  SEED=<int>  UVM_VERB=<level>"
