from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

try:
    from .atmosphere import isa_atmosphere
    from .constants import SimulationConfig, config_to_dict
    from .forces import (
        GeometryProperties,
        build_cd_table,
        build_engine_curve,
        cd_components,
        cd_from_table,
        drag_force_vector,
        mass_at_time,
        thrust_at_time,
        update_gust,
        wind_base_mps,
    )
except ImportError:  # pragma: no cover
    from atmosphere import isa_atmosphere
    from constants import SimulationConfig, config_to_dict
    from forces import (
        GeometryProperties,
        build_cd_table,
        build_engine_curve,
        cd_components,
        cd_from_table,
        drag_force_vector,
        mass_at_time,
        thrust_at_time,
        update_gust,
        wind_base_mps,
    )


@dataclass
class SimulationResult:
    config: dict
    geometry: dict
    summary: dict
    time_s: np.ndarray
    x_m: np.ndarray
    z_m: np.ndarray
    vx_mps: np.ndarray
    vz_mps: np.ndarray
    ax_mps2: np.ndarray
    az_mps2: np.ndarray
    mass_kg: np.ndarray
    thrust_n: np.ndarray
    drag_body_n: np.ndarray
    drag_chute_n: np.ndarray
    wind_x_mps: np.ndarray
    speed_air_mps: np.ndarray
    mach: np.ndarray
    cd_total: np.ndarray
    reynolds: np.ndarray
    recovery_alpha: np.ndarray
    rail_constrained: np.ndarray


def _as_dict_geometry(g: GeometryProperties) -> dict:
    return {
        "stl_path": g.stl_path,
        "length_m": g.length_m,
        "body_diameter_m": g.body_diameter_m,
        "max_diameter_m": g.max_diameter_m,
        "frontal_area_m2": g.frontal_area_m2,
        "wetted_area_m2": g.wetted_area_m2,
        "volume_m3": g.volume_m3,
        "axis_unit_vector": list(g.axis_unit_vector),
        "bounds_min_m": list(g.bounds_min_m),
        "bounds_max_m": list(g.bounds_max_m),
    }


def _chute_geometry(cfg: SimulationConfig) -> tuple[float, float]:
    if not cfg.recovery.enabled:
        return 0.0, 0.0

    if cfg.recovery.auto_size:
        dry_mass = max(cfg.mass.total_liftoff_mass_kg - cfg.mass.motor_prop_mass_kg, 1.0e-3)
        rho = cfg.env.rho0
        vt = max(cfg.recovery.v_target_mps, 0.5)
        area = 2.0 * dry_mass * cfg.env.g / (rho * cfg.recovery.cd_chute * vt * vt)
        area = max(area, 1.0e-4)
        diam = math.sqrt(4.0 * area / math.pi)
        return area, diam

    diam = max(cfg.recovery.diam_m, 1.0e-3)
    return math.pi * (diam * 0.5) ** 2, diam


def run_flight_simulation(cfg: SimulationConfig, geom: GeometryProperties, seed: int | None = None) -> SimulationResult:
    rng = np.random.default_rng(cfg.random_seed if seed is None else seed)
    dt = max(cfg.time_step_s, 1.0e-4)

    curve_t, curve_f, total_impulse = build_engine_curve(cfg.engine)
    burn_time_s = float(curve_t[-1])
    cd_table = build_cd_table(cfg.env, cfg.aero, geom)
    chute_area_m2, chute_diam_m = _chute_geometry(cfg)

    angle_rad = math.radians(cfg.launch.angle_deg)
    rail_dir = np.array([math.cos(angle_rad), math.sin(angle_rad)], dtype=np.float64)
    rail_dir /= max(float(np.linalg.norm(rail_dir)), 1.0e-12)
    rail_normal = np.array([-rail_dir[1], rail_dir[0]], dtype=np.float64)

    t = 0.0
    x = float(cfg.launch.initial_x_m)
    z = max(0.0, float(cfg.launch.initial_z_m))
    vx = float(cfg.launch.initial_vx_mps)
    vz = float(cfg.launch.initial_vz_mps)
    gust = 0.0

    off_rail = cfg.launch.rail_m <= 0.0
    burnout_time: float | None = None
    rail_exit_time: float | None = 0.0 if off_rail else None
    rail_exit_speed_mps: float = float(np.linalg.norm([vx, vz])) if off_rail else 0.0
    apogee_time: float | None = None
    apogee_alt_m: float = z
    landing_time: float | None = None
    deploy_time_target = math.inf
    deploy_time_actual: float | None = None
    inflate_start: float | None = None

    if cfg.recovery.enabled and cfg.recovery.mode == "time":
        deploy_time_target = burn_time_s + cfg.engine.delay_s + cfg.recovery.deploy_extra_s

    hist: dict[str, list[float]] = {
        "time_s": [],
        "x_m": [],
        "z_m": [],
        "vx_mps": [],
        "vz_mps": [],
        "ax_mps2": [],
        "az_mps2": [],
        "mass_kg": [],
        "thrust_n": [],
        "drag_body_n": [],
        "drag_chute_n": [],
        "wind_x_mps": [],
        "speed_air_mps": [],
        "mach": [],
        "cd_total": [],
        "reynolds": [],
        "recovery_alpha": [],
        "rail_constrained": [],
    }

    prev_vz = vz
    deployed = False
    recovery_alpha = 0.0

    while t <= cfg.max_time_s:
        z = max(0.0, z)
        atm = isa_atmosphere(z, cfg.env)

        if burnout_time is None and t >= burn_time_s:
            burnout_time = t

        if not off_rail:
            rail_progress = x * rail_dir[0] + z * rail_dir[1]
            if rail_progress >= cfg.launch.rail_m:
                off_rail = True
                rail_exit_time = t
                rail_exit_speed_mps = float(math.hypot(vx, vz))

        if cfg.recovery.enabled and not deployed:
            if cfg.recovery.mode == "apogee":
                if apogee_time is not None:
                    deploy_time_target = apogee_time + cfg.recovery.deploy_extra_s
            if t >= deploy_time_target:
                deployed = True
                deploy_time_actual = t
                inflate_start = t

        if deployed and inflate_start is not None:
            tau = max(cfg.recovery.inflation_time_s, 1.0e-3)
            recovery_alpha = min(max((t - inflate_start) / tau, 0.0), 1.0)
        else:
            recovery_alpha = 0.0

        gust = update_gust(gust, dt, cfg.wind, rng)
        wind_x = wind_base_mps(z, cfg.wind) + gust

        mass = mass_at_time(t, burn_time_s, cfg.mass)
        thrust = thrust_at_time(t, curve_t, curve_f)

        vel = np.array([vx, vz], dtype=np.float64)
        v_rel = np.array([vx - wind_x, vz], dtype=np.float64)
        speed_rel = float(np.linalg.norm(v_rel))

        if cfg.aero.use_cd_table_for_flight:
            cd = cd_from_table(speed_rel, cd_table)
            mach = speed_rel / max(atm.speed_of_sound_mps, 1.0e-6)
            reynolds = (
                atm.density_kgm3
                * speed_rel
                * max(geom.body_diameter_m, 1.0e-5)
                / max(atm.mu_pas, 1.0e-8)
            )
        else:
            cd = max(cfg.aero.cd_const_flight, cfg.aero.min_cd)
            mach = speed_rel / max(atm.speed_of_sound_mps, 1.0e-6)
            reynolds = (
                atm.density_kgm3
                * speed_rel
                * max(geom.body_diameter_m, 1.0e-5)
                / max(atm.mu_pas, 1.0e-8)
            )

        drag_body_vec = drag_force_vector(v_rel, atm.density_kgm3, cd, geom.frontal_area_m2)
        if deployed and not cfg.recovery.body_drag_after_deploy:
            drag_body_vec[:] = 0.0

        drag_chute_vec = np.zeros(2, dtype=np.float64)
        if deployed and cfg.recovery.enabled and chute_area_m2 > 0.0:
            drag_chute_vec = drag_force_vector(
                v_rel,
                atm.density_kgm3,
                cfg.recovery.cd_chute,
                chute_area_m2 * recovery_alpha,
            )

        if off_rail:
            speed_inertial = float(np.linalg.norm(vel))
            if speed_inertial > 1.0e-6:
                thrust_dir = vel / speed_inertial
            elif speed_rel > 1.0e-6:
                thrust_dir = v_rel / speed_rel
            else:
                thrust_dir = rail_dir
        else:
            thrust_dir = rail_dir

        force = (
            thrust * thrust_dir
            + drag_body_vec
            + drag_chute_vec
            + np.array([0.0, -mass * cfg.env.g], dtype=np.float64)
        )

        if not off_rail:
            # Rail constrains lateral degrees of freedom until guide departure.
            force -= np.dot(force, rail_normal) * rail_normal
            v_parallel = float(np.dot(vel, rail_dir))
            vel = max(v_parallel, 0.0) * rail_dir
            vx, vz = float(vel[0]), float(vel[1])

        ax = float(force[0] / mass)
        az = float(force[1] / mass)

        vx += ax * dt
        vz += az * dt
        x += vx * dt
        z += vz * dt

        if apogee_time is None and prev_vz > 0.0 and vz <= 0.0 and z > 0.5:
            apogee_time = t
        prev_vz = vz

        if z > apogee_alt_m:
            apogee_alt_m = z

        hist["time_s"].append(t)
        hist["x_m"].append(x)
        hist["z_m"].append(max(z, 0.0))
        hist["vx_mps"].append(vx)
        hist["vz_mps"].append(vz)
        hist["ax_mps2"].append(ax)
        hist["az_mps2"].append(az)
        hist["mass_kg"].append(mass)
        hist["thrust_n"].append(thrust)
        hist["drag_body_n"].append(float(np.linalg.norm(drag_body_vec)))
        hist["drag_chute_n"].append(float(np.linalg.norm(drag_chute_vec)))
        hist["wind_x_mps"].append(wind_x)
        hist["speed_air_mps"].append(speed_rel)
        hist["mach"].append(mach)
        hist["cd_total"].append(cd)
        hist["reynolds"].append(reynolds)
        hist["recovery_alpha"].append(recovery_alpha)
        hist["rail_constrained"].append(0.0 if off_rail else 1.0)

        t += dt
        if z <= 0.0 and t > 1.0 and off_rail:
            landing_time = t
            break

    arrays = {k: np.asarray(v, dtype=np.float64) for k, v in hist.items()}
    if arrays["time_s"].size == 0:
        raise RuntimeError("simulation produced no samples")

    idx_apogee = int(np.argmax(arrays["z_m"]))
    apogee_m = float(arrays["z_m"][idx_apogee])
    apogee_time_s = float(arrays["time_s"][idx_apogee])
    max_speed_mps = float(np.max(np.hypot(arrays["vx_mps"], arrays["vz_mps"])))
    max_air_speed_mps = float(np.max(arrays["speed_air_mps"]))
    max_mach = float(np.max(arrays["mach"]))
    max_accel_g = float(np.max(np.hypot(arrays["ax_mps2"], arrays["az_mps2"]) / cfg.env.g))
    drift_m = float(arrays["x_m"][-1])
    flight_time_s = float(arrays["time_s"][-1])
    landing_speed_mps = float(math.hypot(arrays["vx_mps"][-1], arrays["vz_mps"][-1]))

    summary = {
        "burn_time_s": burn_time_s,
        "total_impulse_ns": total_impulse,
        "apogee_m": apogee_m,
        "apogee_time_s": apogee_time_s,
        "max_speed_mps": max_speed_mps,
        "max_air_speed_mps": max_air_speed_mps,
        "max_mach": max_mach,
        "max_accel_g": max_accel_g,
        "rail_exit_time_s": rail_exit_time,
        "rail_exit_speed_mps": rail_exit_speed_mps,
        "deploy_time_s": deploy_time_actual,
        "chute_diameter_m": chute_diam_m,
        "drift_m": drift_m,
        "flight_time_s": flight_time_s,
        "landing_time_s": landing_time,
        "landing_speed_mps": landing_speed_mps,
        "burnout_time_s": burnout_time,
        "apogee_detected_time_s": apogee_time,
    }

    return SimulationResult(
        config=config_to_dict(cfg),
        geometry=_as_dict_geometry(geom),
        summary=summary,
        time_s=arrays["time_s"],
        x_m=arrays["x_m"],
        z_m=arrays["z_m"],
        vx_mps=arrays["vx_mps"],
        vz_mps=arrays["vz_mps"],
        ax_mps2=arrays["ax_mps2"],
        az_mps2=arrays["az_mps2"],
        mass_kg=arrays["mass_kg"],
        thrust_n=arrays["thrust_n"],
        drag_body_n=arrays["drag_body_n"],
        drag_chute_n=arrays["drag_chute_n"],
        wind_x_mps=arrays["wind_x_mps"],
        speed_air_mps=arrays["speed_air_mps"],
        mach=arrays["mach"],
        cd_total=arrays["cd_total"],
        reynolds=arrays["reynolds"],
        recovery_alpha=arrays["recovery_alpha"],
        rail_constrained=arrays["rail_constrained"],
    )
