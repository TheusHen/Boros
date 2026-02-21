from __future__ import annotations

import math
import numpy as np

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
    cfg.time_step_s = 0.005
    cfg.max_time_s = 80.0
    cfg.launch.angle_deg = 90.0
    cfg.launch.rail_m = 0.0
    cfg.launch.initial_z_m = 700.0
    cfg.launch.initial_vz_mps = 0.0
    cfg.engine.thrust_curve = ((0.0, 0.0), (0.2, 0.0))
    cfg.engine.target_total_impulse_ns = None
    cfg.mass.total_liftoff_mass_kg = 0.20
    cfg.mass.motor_prop_mass_kg = 0.0
    cfg.recovery.enabled = False
    cfg.wind.profile = "constant"
    cfg.wind.constant_mps = 0.0
    cfg.wind.gust_sigma_mps = 0.0
    cfg.aero.use_cd_table_for_flight = False
    cfg.aero.cd_const_flight = 0.75
    cfg.aero.min_cd = 0.0

    area = 0.010
    geom = GeometryProperties(
        stl_path="synthetic",
        length_m=0.8,
        body_diameter_m=0.11,
        max_diameter_m=0.11,
        frontal_area_m2=area,
        wetted_area_m2=0.22,
        volume_m3=0.0012,
        axis_unit_vector=(0.0, 0.0, 1.0),
        bounds_min_m=(-0.055, -0.055, 0.0),
        bounds_max_m=(0.055, 0.055, 0.8),
    )

    result = run_flight_simulation(cfg, geom, seed=2)
    rho = cfg.env.rho0
    vt_theory = math.sqrt(2.0 * cfg.mass.total_liftoff_mass_kg * cfg.env.g / (rho * cfg.aero.cd_const_flight * area))

    t = result.time_s
    vz = result.vz_mps
    mask = t > (t[-1] - 5.0)
    vt_sim = float(np.mean(np.abs(vz[mask])))
    rel_err = abs(vt_sim - vt_theory) / vt_theory

    assert rel_err < 0.12, f"Terminal velocity mismatch: sim={vt_sim:.3f} theory={vt_theory:.3f} rel_err={rel_err:.3f}"
    print(f"PASS test_constant_drag | vt_sim={vt_sim:.3f} m/s | vt_theory={vt_theory:.3f} m/s")


if __name__ == "__main__":
    run_test()
