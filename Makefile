# =============================================================================
# CORDIC IP - Master Makefile
# =============================================================================
# Targets:
#   make sim           - cocotb/Verilator simulation (all tests)
#   make sim-test T=   - run specific test (e.g. T=test_smoke)
#   make uvm           - UVM simulation (requires Questa/VCS/Xcelium)
#   make model         - run Python golden model self-test
#   make model-vectors - generate test vectors (C header)
#   make dpi           - compile DPI-C model
#   make build         - build LiteX SoC bitstream (requires Vivado)
#   make load          - load bitstream to Arty A7
#   make hw-test       - run UART hardware test
#   make lint          - run Verilator lint
#   make clean         - clean build artifacts
# =============================================================================

SHELL   := /bin/bash
TOP_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

# Override VENV on the command line if your venv lives elsewhere:
#   make sim VENV=/path/to/venv
VENV  ?= $(TOP_DIR)/.venv
PY    := $(VENV)/bin/python3
PIP   := $(VENV)/bin/pip
RTL_DIR    := $(TOP_DIR)/rtl
TB_DIR     := $(TOP_DIR)/tb
MODEL_DIR  := $(TOP_DIR)/model
LITEX_DIR  := $(TOP_DIR)/litex
SIM_DIR    := $(TOP_DIR)/sim
BUILD_DIR  := $(TOP_DIR)/build

UART_PORT  ?= /dev/ttyUSB1

# Verilator flags
VERILATOR      := verilator
VERILATOR_OPTS := --binary --timing --sv --top cordic_top -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC

# RTL sources (order matters for package dependencies)
RTL_SRCS := \
    $(RTL_DIR)/cordic_pkg.sv       \
    $(RTL_DIR)/cordic_stage.sv     \
    $(RTL_DIR)/cordic_core.sv      \
    $(RTL_DIR)/cordic_pre_proc.sv  \
    $(RTL_DIR)/cordic_post_proc.sv \
    $(RTL_DIR)/cordic_top.sv       \
    $(RTL_DIR)/cordic_axis.sv      \
    $(RTL_DIR)/cordic_axil.sv      \
    $(RTL_DIR)/cordic_wb.sv

# ─────────────────────────────────────────────
# Help
# ─────────────────────────────────────────────
.PHONY: help
help:
	@echo ""
	@echo "CORDIC IP Makefile"
	@echo "=================="
	@echo "  make sim           Run cocotb/Verilator simulation"
	@echo "  make sim-test T=X  Run specific test (e.g. T=test_smoke)"
	@echo "  make uvm           Run UVM simulation (Questa)"
	@echo "  make model         Python golden model self-test"
	@echo "  make model-vectors Generate C test vectors"
	@echo "  make dpi           Compile DPI-C model"
	@echo "  make lint          Verilator lint check"
	@echo "  make build         Build LiteX SoC (requires Vivado)"
	@echo "  make load          Load bitstream to Arty A7"
	@echo "  make hw-test       UART hardware test"
	@echo "  make clean         Clean build artifacts"
	@echo ""

# ─────────────────────────────────────────────
# Python golden model
# ─────────────────────────────────────────────
.PHONY: model
model:
	@echo "=== Running Python golden model self-test ==="
	$(PY) $(MODEL_DIR)/cordic_model.py

.PHONY: model-vectors
model-vectors: model
	@echo "=== Test vectors generated at model/test_vectors.h ==="

# ─────────────────────────────────────────────
# DPI-C model
# ─────────────────────────────────────────────
$(BUILD_DIR)/cordic_dpi.so: $(MODEL_DIR)/cordic_dpi.c | $(BUILD_DIR)
	gcc -shared -fPIC -O2 -lm -o $@ $<
	@echo "DPI-C shared library: $@"

$(BUILD_DIR)/cordic_dpi_test: $(MODEL_DIR)/cordic_dpi.c | $(BUILD_DIR)
	gcc -O2 -lm -DSTANDALONE_TEST -o $@ $<

.PHONY: dpi
dpi: $(BUILD_DIR)/cordic_dpi.so
	@echo "=== Testing DPI-C C model ==="
	$(BUILD_DIR)/cordic_dpi_test 2>/dev/null || \
	    (gcc -O2 -lm -DSTANDALONE_TEST -o $(BUILD_DIR)/cordic_dpi_test $(MODEL_DIR)/cordic_dpi.c && \
	     $(BUILD_DIR)/cordic_dpi_test)

# ─────────────────────────────────────────────
# Verilator lint
# ─────────────────────────────────────────────
.PHONY: lint
lint:
	@echo "=== Running Verilator lint ==="
	$(VERILATOR) --lint-only --sv --top cordic_top \
	    -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSED \
	    $(RTL_SRCS) 2>&1 | tee $(BUILD_DIR)/lint.log
	@echo "Lint complete. Log: build/lint.log"

# ─────────────────────────────────────────────
# cocotb / Verilator simulation
# ─────────────────────────────────────────────
.PHONY: sim
sim: $(BUILD_DIR) model-vectors
	@echo "=== Running cocotb simulation ==="
	cd $(SIM_DIR) && \
	    PYTHONPATH=$(MODEL_DIR):$(TB_DIR)/directed \
	    SIM=verilator \
	    TOPLEVEL=cordic_top \
	    MODULE=test_cordic \
	    VERILOG_SOURCES="$(RTL_SRCS)" \
	    $(MAKE) -f Makefile.cocotb

.PHONY: sim-test
sim-test: $(BUILD_DIR) model-vectors
	@echo "=== Running test: $(T) ==="
	cd $(SIM_DIR) && \
	    PYTHONPATH=$(MODEL_DIR):$(TB_DIR)/directed \
	    SIM=verilator \
	    TOPLEVEL=cordic_top \
	    MODULE=test_cordic \
	    TESTCASE=$(T) \
	    VERILOG_SOURCES="$(RTL_SRCS)" \
	    $(MAKE) -f Makefile.cocotb

# ─────────────────────────────────────────────
# UVM simulation (Questa)
# ─────────────────────────────────────────────
.PHONY: uvm
uvm: $(BUILD_DIR)/cordic_dpi.so
	@echo "=== Running UVM simulation ==="
	@echo "Compiling DPI library..."
	@mkdir -p $(BUILD_DIR)/uvm
	cd $(BUILD_DIR)/uvm && \
	    vlog -sv -work work +define+UVM_NO_DPI \
	         $(RTL_DIR)/cordic_pkg.sv \
	         $(RTL_DIR)/cordic_stage.sv \
	         $(RTL_DIR)/cordic_core.sv \
	         $(RTL_DIR)/cordic_pre_proc.sv \
	         $(RTL_DIR)/cordic_post_proc.sv \
	         $(RTL_DIR)/cordic_top.sv \
	         $(TB_DIR)/uvm/cordic_if.sv \
	         $(TB_DIR)/uvm/cordic_tb_top.sv && \
	    vsim -sv_lib $(BUILD_DIR)/cordic_dpi \
	         -do "run -all; quit -f" \
	         work.cordic_tb_top \
	         +UVM_TESTNAME=cordic_full_test \
	         +UVM_VERBOSITY=UVM_LOW

.PHONY: uvm-smoke
uvm-smoke: $(BUILD_DIR)/cordic_dpi.so
	@mkdir -p $(BUILD_DIR)/uvm
	cd $(BUILD_DIR)/uvm && \
	    vlog -sv -work work $(RTL_SRCS) $(TB_DIR)/uvm/cordic_if.sv $(TB_DIR)/uvm/cordic_tb_top.sv && \
	    vsim -sv_lib $(BUILD_DIR)/cordic_dpi -do "run -all; quit -f" \
	         work.cordic_tb_top +UVM_TESTNAME=cordic_smoke_test

# ─────────────────────────────────────────────
# LiteX SoC build
# ─────────────────────────────────────────────
.PHONY: build
build:
	@echo "=== Building LiteX SoC (Vivado required) ==="
	source $(VENV)/bin/activate && \
	    cd $(LITEX_DIR) && \
	    $(PY) cordic_soc.py --build \
	        --output-dir $(BUILD_DIR)/soc \
	        --no-compile-software

.PHONY: build-software
build-software:
	@echo "=== Building SoC software ==="
	source $(VENV)/bin/activate && \
	    cd $(LITEX_DIR) && \
	    $(PY) cordic_soc.py --build \
	        --output-dir $(BUILD_DIR)/soc

.PHONY: load
load:
	@echo "=== Loading bitstream to Arty A7 ==="
	source $(VENV)/bin/activate && \
	    cd $(LITEX_DIR) && \
	    $(PY) cordic_soc.py --load \
	        --output-dir $(BUILD_DIR)/soc

.PHONY: build-load
build-load: build load

# ─────────────────────────────────────────────
# Hardware test via UART
# ─────────────────────────────────────────────
.PHONY: hw-test
hw-test:
	@echo "=== Running CORDIC hardware test on $(UART_PORT) ==="
	source $(VENV)/bin/activate && \
	    $(PY) $(LITEX_DIR)/cordic_uart_test.py \
	        --port $(UART_PORT) \
	        --random-n 100

# ─────────────────────────────────────────────
# Directory creation
# ─────────────────────────────────────────────
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ─────────────────────────────────────────────
# Clean
# ─────────────────────────────────────────────
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	rm -rf $(SIM_DIR)/sim_build
	rm -rf $(SIM_DIR)/results.xml
	find . -name "*.vcd" -delete
	find . -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete
