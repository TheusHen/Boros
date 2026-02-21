PYTHON ?= python3
MATLAB ?= matlab
OCTAVE ?= octave

FW_DIR := firmware
PY_SIM_DIR := sim/flight
SIM3D_DIR := sim/3d
TOOLS_DIR := tools
ROCKET_STL := 3d/BorosRocket.stl
SIM_MASS_FALLBACK_G := 76.978

.PHONY: ci ci-3d check firmware python-check python-sim python-firmware-playback python-mass-budget sim-3d-matlab sim-3d-octave dump dump-list clean

ci: firmware python-sim

ci-3d: sim-3d-octave
	$(MAKE) python-sim PYTHON=$(PYTHON)

check: firmware python-check

firmware:
	$(MAKE) -C $(FW_DIR) clean all

python-check:
	$(PYTHON) -m compileall $(PY_SIM_DIR)
	$(PYTHON) -m pip install -r $(PY_SIM_DIR)/requirements.txt
	$(PYTHON) $(PY_SIM_DIR)/test_no_drag.py
	$(PYTHON) $(PY_SIM_DIR)/test_constant_drag.py

python-sim: python-check python-mass-budget
	MASS_G=`$(PYTHON) -c "import json; p='$(PY_SIM_DIR)/out/mass_budget.json'; d=json.load(open(p, encoding='utf-8')); print(d.get('totals', {}).get('suggested_sim_mass_total_g', $(SIM_MASS_FALLBACK_G)))"`; \
	echo "Using mass_total_g=$$MASS_G from mass budget"; \
	$(PYTHON) $(PY_SIM_DIR)/simulate.py --mass-total-g $$MASS_G --stl $(ROCKET_STL) --out $(PY_SIM_DIR)/out; \
	$(PYTHON) $(PY_SIM_DIR)/monte_carlo.py --iterations 120 --mass-total-g $$MASS_G --out $(PY_SIM_DIR)/out/monte_carlo
	$(PYTHON) $(PY_SIM_DIR)/detect_apogee.py $(PY_SIM_DIR)/out/firmware_like_log.csv

python-firmware-playback:
	$(PYTHON) $(TOOLS_DIR)/firmware_playback.py \
		--input $(PY_SIM_DIR)/out/firmware_like_log.csv \
		--firmware-bin $(FW_DIR)/firmware.bin \
		--firmware-elf $(FW_DIR)/firmware.elf \
		--out $(PY_SIM_DIR)/out/firmware_playback_summary.json

python-mass-budget:
	$(PYTHON) $(TOOLS_DIR)/mass_budget.py \
		--stl $(ROCKET_STL) \
		--motor-mass-g 24.1 \
		--motor-max-liftoff-mass-g 113 \
		--pcb-width-mm 25 \
		--pcb-height-mm 62 \
		--pcb-copper-layers 4 \
		--out-json $(PY_SIM_DIR)/out/mass_budget.json \
		--out-md $(PY_SIM_DIR)/out/mass_budget.md

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
