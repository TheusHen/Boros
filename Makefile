PYTHON ?= python3
MATLAB ?= matlab
OCTAVE ?= octave

.PHONY: ci check firmware python-check python-sim sim-3d-matlab sim-3d-octave clean

ci: check python-sim

check: firmware python-check

firmware:
	$(MAKE) -C firmware clean all

python-check:
	$(PYTHON) -m compileall sim/flight
	$(PYTHON) -m pip install -r sim/flight/requirements.txt
	$(PYTHON) sim/flight/test_no_drag.py
	$(PYTHON) sim/flight/test_constant_drag.py

python-sim:
	$(PYTHON) sim/flight/simulate.py --mass-total-g 80 --stl 3d/BorosRocket.stl --out sim/flight/out
	$(PYTHON) sim/flight/monte_carlo.py --iterations 120 --out sim/flight/out/monte_carlo
	$(PYTHON) sim/flight/detect_apogee.py sim/flight/out/firmware_like_log.csv

sim-3d-matlab:
	$(MATLAB) -batch "cd('sim/3d'); test_aero_limits; run_ci_sim"

sim-3d-octave:
	$(OCTAVE) --quiet --eval "cd('sim/3d'); test_aero_limits; run_ci_sim;"

clean:
	$(MAKE) -C firmware clean
