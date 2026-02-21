from __future__ import annotations

import math

try:
    from .constants import default_simulation_config
    from .dynamics import run_flight_simulation
    from .forces import GeometryProperties
except ImportError:  # pragma: no cover
    from constants import default_simulation_config
    from dynamics import run_flight_simulation
    from forces import GeometryProperties


def run_test() -> None:
    cfg = default_simulation_config()
    cfg.time_step_s = 0.002
    cfg.max_time_s = 30.0
    cfg.launch.angle_deg = 90.0
    cfg.launch.rail_m = 0.0
    cfg.launch.initial_vz_mps = 50.0
    cfg.launch.initial_z_m = 0.0
    cfg.engine.thrust_curve = ((0.0, 0.0), (0.1, 0.0))
    cfg.engine.target_total_impulse_ns = None
    cfg.mass.motor_prop_mass_kg = 0.0
    cfg.recovery.enabled = False
    cfg.wind.profile = "constant"
    cfg.wind.constant_mps = 0.0
    cfg.wind.gust_sigma_mps = 0.0
    cfg.aero.use_cd_table_for_flight = False
    cfg.aero.cd_const_flight = 0.0
    cfg.aero.min_cd = 0.0

    geom = GeometryProperties(
        stl_path="synthetic",
        length_m=0.5,
        body_diameter_m=0.05,
        max_diameter_m=0.05,
        frontal_area_m2=math.pi * (0.025**2),
        wetted_area_m2=math.pi * 0.05 * 0.5,
        volume_m3=9.8e-4,
        axis_unit_vector=(0.0, 0.0, 1.0),
        bounds_min_m=(-0.025, -0.025, 0.0),
        bounds_max_m=(0.025, 0.025, 0.5),
    )

    result = run_flight_simulation(cfg, geom, seed=1)
    expected_apogee = cfg.launch.initial_vz_mps**2 / (2.0 * cfg.env.g)
    err = abs(result.summary["apogee_m"] - expected_apogee)

    assert err < 1.0, f"Apogee mismatch too high: got {result.summary['apogee_m']:.3f} expected {expected_apogee:.3f}"
    print(f"PASS test_no_drag | apogee={result.summary['apogee_m']:.3f} m | expected={expected_apogee:.3f} m")


if __name__ == "__main__":
    run_test()
