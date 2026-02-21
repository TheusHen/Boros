PYTHON ?= python3
MATLAB ?= matlab
OCTAVE ?= octave

FW_DIR := firmware
PY_SIM_DIR := sim/flight
SIM3D_DIR := sim/3d
TOOLS_DIR := tools
ROCKET_STL := 3d/BorosRocket.stl

.PHONY: ci ci-3d check firmware python-check python-sim sim-3d-matlab sim-3d-octave dump dump-list clean

ci: python-sim

ci-3d: sim-3d-octave
	$(MAKE) python-sim PYTHON=$(PYTHON)

check: python-check

firmware:
	$(MAKE) -C $(FW_DIR) clean all

python-check: firmware
	$(PYTHON) -m compileall $(PY_SIM_DIR)
	$(PYTHON) -m pip install -r $(PY_SIM_DIR)/requirements.txt
	$(PYTHON) $(PY_SIM_DIR)/test_no_drag.py
	$(PYTHON) $(PY_SIM_DIR)/test_constant_drag.py

python-sim: python-check
	$(PYTHON) $(PY_SIM_DIR)/simulate.py --mass-total-g 80 --stl $(ROCKET_STL) --out $(PY_SIM_DIR)/out
	$(PYTHON) $(PY_SIM_DIR)/monte_carlo.py --iterations 120 --out $(PY_SIM_DIR)/out/monte_carlo
	$(PYTHON) $(PY_SIM_DIR)/detect_apogee.py $(PY_SIM_DIR)/out/firmware_like_log.csv

sim-3d-matlab:
	$(MATLAB) -batch "cd('$(SIM3D_DIR)'); test_aero_limits; run_ci_sim"

sim-3d-octave: firmware
	$(OCTAVE) --quiet --eval "cd('$(SIM3D_DIR)'); test_aero_limits; run_ci_sim;"

dump:
	$(PYTHON) -m pip install -r $(TOOLS_DIR)/requirements.txt
	$(PYTHON) $(TOOLS_DIR)/dump.py $(DUMP_ARGS)

dump-list:
	$(PYTHON) -m pip install -r $(TOOLS_DIR)/requirements.txt
	$(PYTHON) $(TOOLS_DIR)/dump.py --list-ports

clean:
	$(MAKE) -C $(FW_DIR) clean
