from __future__ import annotations

import argparse
import concurrent.futures as cf
import csv
import json
import os
from pathlib import Path

import numpy as np

try:
    from .constants import clone_config, default_simulation_config
    from .dynamics import run_flight_simulation
    from .forces import load_stl_geometry
except ImportError:  # pragma: no cover
    from constants import clone_config, default_simulation_config
    from dynamics import run_flight_simulation
    from forces import load_stl_geometry


_WORKER_GEOM = None
_WORKER_GEOM_KEY: tuple[str, float, float] | None = None


def percentile_stats(values: np.ndarray) -> dict:
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "p05": float(np.percentile(values, 5)),
        "p50": float(np.percentile(values, 50)),
        "p90": float(np.percentile(values, 90)),
        "p95": float(np.percentile(values, 95)),
    }


def _simulate_case(task: tuple[int, object, tuple[str, float, float]]) -> dict:
    case_idx, cfg, geom_key = task
    global _WORKER_GEOM, _WORKER_GEOM_KEY
    if _WORKER_GEOM is None or _WORKER_GEOM_KEY != geom_key:
        stl_path, unit_scale, body_quantile = geom_key
        _WORKER_GEOM = load_stl_geometry(stl_path, unit_scale, body_quantile)
        _WORKER_GEOM_KEY = geom_key

    result = run_flight_simulation(cfg, _WORKER_GEOM, seed=cfg.random_seed)
    return {
        "case": case_idx,
        "mass_kg": cfg.mass.total_liftoff_mass_kg,
        "cd_scale": cfg.aero.flight_cd_scale,
        "angle_deg": cfg.launch.angle_deg,
        "delay_s": cfg.engine.delay_s,
        "wind0_mps": cfg.wind.wind0_mps,
        "windtop_mps": cfg.wind.wind_top_mps,
        "gust_sigma_mps": cfg.wind.gust_sigma_mps,
        "apogee_m": result.summary["apogee_m"],
        "drift_m": result.summary["drift_m"],
        "flight_time_s": result.summary["flight_time_s"],
        "landing_speed_mps": result.summary["landing_speed_mps"],
    }


def run_monte_carlo(iterations: int, out_dir: Path, seed: int = 2026, jobs: int = 0) -> None:
    base = default_simulation_config()
    out_dir.mkdir(parents=True, exist_ok=True)

    if jobs <= 0:
        jobs = max(1, (os.cpu_count() or 1) - 1)
    jobs = max(1, min(jobs, iterations))

    rng = np.random.default_rng(seed)
    geom_key = (base.geometry.stl_path, base.geometry.unit_scale, base.geometry.body_quantile)
    tasks: list[tuple[int, object, tuple[str, float, float]]] = []

    for i in range(iterations):
        cfg = clone_config(base)
        cfg.random_seed = int(rng.integers(1, 2**31 - 1))

        cfg.mass.total_liftoff_mass_kg = max(0.060, rng.normal(base.mass.total_liftoff_mass_kg, 0.003))
        cfg.aero.flight_cd_scale = max(0.85, rng.normal(base.aero.flight_cd_scale, 0.08))
        cfg.launch.angle_deg = float(np.clip(rng.normal(base.launch.angle_deg, 1.2), 82.0, 90.0))
        cfg.engine.delay_s = max(2.0, rng.normal(base.engine.delay_s, 0.35))
        cfg.wind.gust_sigma_mps = max(0.2, rng.normal(base.wind.gust_sigma_mps, 0.35))

        if cfg.wind.profile == "linear":
            sign = -1.0 if rng.random() < 0.5 else 1.0
            cfg.wind.wind0_mps = sign * max(0.0, rng.normal(abs(base.wind.wind0_mps), 0.6))
            cfg.wind.wind_top_mps = sign * max(
                abs(cfg.wind.wind0_mps),
                rng.normal(abs(base.wind.wind_top_mps), 1.0),
            )
        elif cfg.wind.profile == "constant":
            cfg.wind.constant_mps = rng.normal(base.wind.constant_mps, 1.2)

        tasks.append((i, cfg, geom_key))

    if jobs == 1:
        geom = load_stl_geometry(*geom_key)
        rows = []
        for case_idx, cfg, _ in tasks:
            result = run_flight_simulation(cfg, geom, seed=cfg.random_seed)
            rows.append(
                {
                    "case": case_idx,
                    "mass_kg": cfg.mass.total_liftoff_mass_kg,
                    "cd_scale": cfg.aero.flight_cd_scale,
                    "angle_deg": cfg.launch.angle_deg,
                    "delay_s": cfg.engine.delay_s,
                    "wind0_mps": cfg.wind.wind0_mps,
                    "windtop_mps": cfg.wind.wind_top_mps,
                    "gust_sigma_mps": cfg.wind.gust_sigma_mps,
                    "apogee_m": result.summary["apogee_m"],
                    "drift_m": result.summary["drift_m"],
                    "flight_time_s": result.summary["flight_time_s"],
                    "landing_speed_mps": result.summary["landing_speed_mps"],
                }
            )
    else:
        with cf.ProcessPoolExecutor(max_workers=jobs) as executor:
            rows = list(executor.map(_simulate_case, tasks))

    csv_path = out_dir / "monte_carlo_cases.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    apogee = np.asarray([r["apogee_m"] for r in rows], dtype=np.float64)
    drift = np.asarray([abs(r["drift_m"]) for r in rows], dtype=np.float64)
    landing = np.asarray([r["landing_speed_mps"] for r in rows], dtype=np.float64)
    flight = np.asarray([r["flight_time_s"] for r in rows], dtype=np.float64)

    summary = {
        "iterations": iterations,
        "seed": seed,
        "apogee_m": percentile_stats(apogee),
        "abs_drift_m": percentile_stats(drift),
        "landing_speed_mps": percentile_stats(landing),
        "flight_time_s": percentile_stats(flight),
    }
    summary_path = out_dir / "monte_carlo_summary.json"
    with summary_path.open("w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2)

    print("=== Monte Carlo Completed ===")
    print(f"Workers: {jobs}")
    print(f"Cases: {iterations}")
    print(f"Apogee P50/P90: {summary['apogee_m']['p50']:.1f} / {summary['apogee_m']['p90']:.1f} m")
    print(f"|Drift| P50/P90: {summary['abs_drift_m']['p50']:.1f} / {summary['abs_drift_m']['p90']:.1f} m")
    print(f"Results: {out_dir}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Boros rocket Monte Carlo flight simulation")
    ap.add_argument("--iterations", type=int, default=300)
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--out", type=str, default=None)
    ap.add_argument("--jobs", type=int, default=0, help="Parallel workers. 0 = auto")
    args = ap.parse_args()

    cfg = default_simulation_config()
    out_dir = Path(cfg.output.out_dir) / "monte_carlo" if args.out is None else Path(args.out)
    run_monte_carlo(iterations=max(args.iterations, 10), out_dir=out_dir, seed=args.seed, jobs=args.jobs)


if __name__ == "__main__":
    main()
