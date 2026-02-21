from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

try:
    from .constants import default_simulation_config
    from .dynamics import SimulationResult, run_flight_simulation
    from .forces import build_cd_table, load_stl_geometry
    from .sensor_models import generate_firmware_like_log
except ImportError:  # pragma: no cover
    from constants import default_simulation_config
    from dynamics import SimulationResult, run_flight_simulation
    from forces import build_cd_table, load_stl_geometry
    from sensor_models import generate_firmware_like_log


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def write_timeseries_csv(result: SimulationResult, path: Path) -> None:
    _ensure_dir(path.parent)
    fieldnames = [
        "time_s",
        "x_m",
        "z_m",
        "vx_mps",
        "vz_mps",
        "ax_mps2",
        "az_mps2",
        "mass_kg",
        "thrust_n",
        "drag_body_n",
        "drag_chute_n",
        "wind_x_mps",
        "speed_air_mps",
        "mach",
        "cd_total",
        "reynolds",
        "recovery_alpha",
        "rail_constrained",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=fieldnames)
        wr.writeheader()
        for i in range(result.time_s.size):
            wr.writerow(
                {
                    "time_s": f"{result.time_s[i]:.6f}",
                    "x_m": f"{result.x_m[i]:.6f}",
                    "z_m": f"{result.z_m[i]:.6f}",
                    "vx_mps": f"{result.vx_mps[i]:.6f}",
                    "vz_mps": f"{result.vz_mps[i]:.6f}",
                    "ax_mps2": f"{result.ax_mps2[i]:.6f}",
                    "az_mps2": f"{result.az_mps2[i]:.6f}",
                    "mass_kg": f"{result.mass_kg[i]:.6f}",
                    "thrust_n": f"{result.thrust_n[i]:.6f}",
                    "drag_body_n": f"{result.drag_body_n[i]:.6f}",
                    "drag_chute_n": f"{result.drag_chute_n[i]:.6f}",
                    "wind_x_mps": f"{result.wind_x_mps[i]:.6f}",
                    "speed_air_mps": f"{result.speed_air_mps[i]:.6f}",
                    "mach": f"{result.mach[i]:.6f}",
                    "cd_total": f"{result.cd_total[i]:.6f}",
                    "reynolds": f"{result.reynolds[i]:.2f}",
                    "recovery_alpha": f"{result.recovery_alpha[i]:.6f}",
                    "rail_constrained": int(result.rail_constrained[i]),
                }
            )


def write_cd_table_csv(cd_table: np.ndarray, path: Path) -> None:
    _ensure_dir(path.parent)
    with path.open("w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["speed_mps", "cd_total", "mach", "reynolds"])
        for row in cd_table:
            wr.writerow([f"{row[0]:.6f}", f"{row[1]:.6f}", f"{row[2]:.6f}", f"{row[3]:.2f}"])


def write_summary_json(result: SimulationResult, path: Path) -> None:
    _ensure_dir(path.parent)
    payload = {
        "summary": result.summary,
        "geometry": result.geometry,
        "config": result.config,
    }
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Boros rocket flight simulation")
    p.add_argument("--stl", type=str, default=None, help="Path to BorosRocket STL")
    p.add_argument("--out", type=str, default=None, help="Output directory")
    p.add_argument("--mass-total-g", type=float, default=80.0, help="Total liftoff mass in grams")
    p.add_argument("--angle-deg", type=float, default=88.0, help="Rail angle wrt horizon (deg)")
    p.add_argument("--rail-m", type=float, default=1.0, help="Rail length (m)")
    p.add_argument("--delay-s", type=float, default=5.0, help="Motor ejection delay (s)")
    p.add_argument("--wind-profile", type=str, default="linear", choices=["constant", "linear", "piecewise"])
    p.add_argument("--wind-constant-mps", type=float, default=0.0)
    p.add_argument("--wind0-mps", type=float, default=1.0)
    p.add_argument("--windtop-mps", type=float, default=4.0)
    p.add_argument("--wind-htop-m", type=float, default=300.0)
    p.add_argument("--gust-sigma-mps", type=float, default=0.6)
    p.add_argument("--gust-tau-s", type=float, default=2.0)
    p.add_argument("--dt", type=float, default=0.002, help="Integrator step (s)")
    p.add_argument("--max-time-s", type=float, default=180.0)
    p.add_argument("--no-recovery", action="store_true", help="Disable parachute/recovery")
    p.add_argument("--recovery-mode", type=str, default="time", choices=["time", "apogee"])
    p.add_argument("--seed", type=int, default=2026)
    return p


def main() -> None:
    args = build_arg_parser().parse_args()
    cfg = default_simulation_config()

    if args.stl:
        cfg.geometry.stl_path = args.stl
    if args.out:
        cfg.output.out_dir = args.out

    cfg.mass.total_liftoff_mass_kg = args.mass_total_g / 1000.0
    cfg.launch.angle_deg = args.angle_deg
    cfg.launch.rail_m = args.rail_m
    cfg.engine.delay_s = args.delay_s
    cfg.wind.profile = args.wind_profile
    cfg.wind.constant_mps = args.wind_constant_mps
    cfg.wind.wind0_mps = args.wind0_mps
    cfg.wind.wind_top_mps = args.windtop_mps
    cfg.wind.h_top_m = args.wind_htop_m
    cfg.wind.gust_sigma_mps = args.gust_sigma_mps
    cfg.wind.gust_tau_s = args.gust_tau_s
    cfg.time_step_s = args.dt
    cfg.max_time_s = args.max_time_s
    cfg.recovery.enabled = not args.no_recovery
    cfg.recovery.mode = args.recovery_mode
    cfg.random_seed = args.seed

    geom = load_stl_geometry(cfg.geometry.stl_path, cfg.geometry.unit_scale, cfg.geometry.body_quantile)
    result = run_flight_simulation(cfg, geom, seed=args.seed)
    cd_table = build_cd_table(cfg.env, cfg.aero, geom)

    out_dir = Path(cfg.output.out_dir)
    _ensure_dir(out_dir)
    timeseries_path = out_dir / cfg.output.timeseries_csv
    summary_path = out_dir / cfg.output.summary_json
    cd_path = out_dir / cfg.output.cd_table_csv
    firmware_path = out_dir / cfg.output.firmware_like_csv

    write_timeseries_csv(result, timeseries_path)
    write_summary_json(result, summary_path)
    write_cd_table_csv(cd_table, cd_path)
    generate_firmware_like_log(result, cfg, firmware_path, seed=args.seed)

    print("=== Boros Flight Simulation ===")
    print(f"STL: {geom.stl_path}")
    print(f"Length: {geom.length_m:.3f} m | Body D: {geom.body_diameter_m*1000.0:.1f} mm")
    print(f"Apogee: {result.summary['apogee_m']:.1f} m at {result.summary['apogee_time_s']:.2f} s")
    print(f"Max speed: {result.summary['max_speed_mps']:.1f} m/s | Max Mach: {result.summary['max_mach']:.3f}")
    print(f"Drift: {result.summary['drift_m']:.1f} m | Flight time: {result.summary['flight_time_s']:.2f} s")
    print(f"Parachute diameter: {result.summary['chute_diameter_m']:.3f} m")
    print(f"Outputs: {out_dir}")


if __name__ == "__main__":
    main()
