from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import struct
from typing import Iterable

import numpy as np

try:
    from .atmosphere import isa_atmosphere
    from .constants import AeroConfig, EngineConfig, EnvironmentConfig, MassConfig, WindConfig
except ImportError:  # pragma: no cover
    from atmosphere import isa_atmosphere
    from constants import AeroConfig, EngineConfig, EnvironmentConfig, MassConfig, WindConfig


def _trapz(y: np.ndarray, x: np.ndarray) -> float:
    trapezoid = getattr(np, "trapezoid", None)
    if trapezoid is None:  # NumPy < 2.0
        trapezoid = np.trapz
    return float(trapezoid(y, x))


@dataclass
class GeometryProperties:
    stl_path: str
    length_m: float
    body_diameter_m: float
    max_diameter_m: float
    frontal_area_m2: float
    wetted_area_m2: float
    volume_m3: float
    axis_unit_vector: tuple[float, float, float]
    bounds_min_m: tuple[float, float, float]
    bounds_max_m: tuple[float, float, float]


def _load_ascii_stl(path: Path) -> np.ndarray:
    vertices: list[tuple[float, float, float]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            txt = line.strip()
            if not txt.lower().startswith("vertex"):
                continue
            parts = txt.split()
            if len(parts) != 4:
                continue
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))

    if len(vertices) < 3 or (len(vertices) % 3) != 0:
        raise ValueError(f"ASCII STL malformed: {path}")
    arr = np.asarray(vertices, dtype=np.float64).reshape(-1, 3, 3)
    return arr


def _load_binary_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"STL too small: {path}")

    tri_count = struct.unpack("<I", raw[80:84])[0]
    expected = 84 + tri_count * 50
    if expected != len(raw):
        raise ValueError(f"Not binary STL length match: {path}")

    tri_dtype = np.dtype(
        [
            ("normal", "<f4", (3,)),
            ("v1", "<f4", (3,)),
            ("v2", "<f4", (3,)),
            ("v3", "<f4", (3,)),
            ("attr", "<u2"),
        ]
    )
    data = np.frombuffer(raw, dtype=tri_dtype, offset=84, count=tri_count)
    tris = np.stack((data["v1"], data["v2"], data["v3"]), axis=1).astype(np.float64, copy=False)
    return tris


def _load_triangles(path: Path) -> np.ndarray:
    try:
        tris = _load_binary_stl(path)
        return tris
    except Exception:
        return _load_ascii_stl(path)


def load_stl_geometry(path: str | Path, unit_scale: float, body_quantile: float = 0.90) -> GeometryProperties:
    stl_path = Path(path).resolve()
    triangles = _load_triangles(stl_path) * float(unit_scale)
    vertices = triangles.reshape(-1, 3)

    vmin = vertices.min(axis=0)
    vmax = vertices.max(axis=0)

    centered = vertices - vertices.mean(axis=0, keepdims=True)
    cov = np.cov(centered.T)
    eig_vals, eig_vecs = np.linalg.eigh(cov)
    axis = eig_vecs[:, int(np.argmax(eig_vals))]
    axis = axis / (np.linalg.norm(axis) + 1.0e-12)

    axial = centered @ axis
    length_m = float(np.ptp(axial))

    radial_vec = centered - np.outer(axial, axis)
    radii = np.linalg.norm(radial_vec, axis=1)
    q = float(np.clip(body_quantile, 0.50, 0.99))
    body_radius_m = float(np.quantile(radii, q))

    extents = vmax - vmin
    if body_radius_m < 1.0e-4:
        body_radius_m = float(0.5 * np.partition(extents, -2)[-2])

    max_radius_m = float(np.max(radii))
    body_diameter_m = 2.0 * body_radius_m
    max_diameter_m = 2.0 * max_radius_m
    frontal_area_m2 = math.pi * body_radius_m * body_radius_m

    e1 = triangles[:, 1, :] - triangles[:, 0, :]
    e2 = triangles[:, 2, :] - triangles[:, 0, :]
    cross = np.cross(e1, e2)
    tri_area = 0.5 * np.linalg.norm(cross, axis=1)
    wetted_area_m2 = float(np.sum(tri_area))

    signed = np.einsum("ij,ij->i", triangles[:, 0, :], np.cross(triangles[:, 1, :], triangles[:, 2, :]))
    volume_m3 = float(abs(np.sum(signed) / 6.0))

    return GeometryProperties(
        stl_path=str(stl_path),
        length_m=length_m,
        body_diameter_m=body_diameter_m,
        max_diameter_m=max_diameter_m,
        frontal_area_m2=frontal_area_m2,
        wetted_area_m2=wetted_area_m2,
        volume_m3=volume_m3,
        axis_unit_vector=(float(axis[0]), float(axis[1]), float(axis[2])),
        bounds_min_m=(float(vmin[0]), float(vmin[1]), float(vmin[2])),
        bounds_max_m=(float(vmax[0]), float(vmax[1]), float(vmax[2])),
    )


def build_engine_curve(engine_cfg: EngineConfig) -> tuple[np.ndarray, np.ndarray, float]:
    arr = np.asarray(engine_cfg.thrust_curve, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 2 or arr.shape[0] < 2:
        raise ValueError("engine thrust_curve must be Nx2 time/thrust")

    arr = arr[np.argsort(arr[:, 0])]
    t = arr[:, 0].copy()
    f = np.maximum(arr[:, 1], 0.0).copy()

    if t[0] > 0.0:
        t = np.insert(t, 0, 0.0)
        f = np.insert(f, 0, 0.0)
    if f[-1] != 0.0:
        t = np.append(t, t[-1] + 1.0e-3)
        f = np.append(f, 0.0)

    impulse = _trapz(f, t)
    if engine_cfg.target_total_impulse_ns is not None and impulse > 0.0:
        scale = float(engine_cfg.target_total_impulse_ns) / impulse
        f *= scale
        impulse = _trapz(f, t)

    return t, f, impulse


def thrust_at_time(t_s: float, curve_t: np.ndarray, curve_f: np.ndarray) -> float:
    if t_s <= float(curve_t[0]) or t_s >= float(curve_t[-1]):
        return 0.0
    return float(np.interp(t_s, curve_t, curve_f))


def mass_at_time(t_s: float, burn_time_s: float, mass_cfg: MassConfig) -> float:
    burn_time_s = max(1.0e-6, burn_time_s)
    frac_burned = min(max(t_s / burn_time_s, 0.0), 1.0)
    dry_kg = mass_cfg.total_liftoff_mass_kg - mass_cfg.motor_prop_mass_kg
    return max(1.0e-3, dry_kg + mass_cfg.motor_prop_mass_kg * (1.0 - frac_burned))


def wind_base_mps(altitude_m: float, cfg: WindConfig) -> float:
    h = max(0.0, altitude_m)
    if cfg.profile == "constant":
        return float(cfg.constant_mps)
    if cfg.profile == "linear":
        top = max(cfg.h_top_m, 1.0)
        alpha = min(max(h / top, 0.0), 1.0)
        return float(cfg.wind0_mps + alpha * (cfg.wind_top_mps - cfg.wind0_mps))

    h_nodes = np.asarray(cfg.h_m, dtype=np.float64)
    w_nodes = np.asarray(cfg.wind_mps, dtype=np.float64)
    if h_nodes.shape[0] != w_nodes.shape[0]:
        raise ValueError("wind.h_m and wind.wind_mps must have the same size")
    order = np.argsort(h_nodes)
    return float(np.interp(h, h_nodes[order], w_nodes[order]))


def update_gust(prev_gust_mps: float, dt_s: float, cfg: WindConfig, rng: np.random.Generator) -> float:
    if cfg.gust_sigma_mps <= 0.0:
        return 0.0
    tau = max(cfg.gust_tau_s, 1.0e-3)
    sigma = cfg.gust_sigma_mps
    a = math.exp(-dt_s / tau)
    q = sigma * math.sqrt(max(1.0 - a * a, 0.0))
    return float(a * prev_gust_mps + q * rng.normal())


def reynolds_number(rho: float, speed_mps: float, ref_len_m: float, mu_pas: float) -> float:
    return float(rho * abs(speed_mps) * max(ref_len_m, 1.0e-5) / max(mu_pas, 1.0e-8))


def cd_components(
    speed_mps: float,
    env_cfg: EnvironmentConfig,
    aero_cfg: AeroConfig,
    geom: GeometryProperties,
    rho_kgm3: float | None = None,
    mu_pas: float | None = None,
    a_mps: float | None = None,
) -> dict[str, float]:
    if rho_kgm3 is None or mu_pas is None or a_mps is None:
        atm = isa_atmosphere(0.0, env_cfg)
        rho_kgm3 = atm.density_kgm3
        mu_pas = atm.mu_pas
        a_mps = atm.speed_of_sound_mps

    v = abs(float(speed_mps))
    mach = v / max(a_mps, 1.0e-6)
    re = reynolds_number(rho_kgm3, v, geom.body_diameter_m, mu_pas)

    if re <= 0.0:
        cf = 0.0
    elif re < 5.0e5:
        cf = 1.328 / math.sqrt(re)
    else:
        cf = max(0.074 / (re ** 0.2) - 1742.0 / re, 0.0)

    if aero_cfg.roughness_m > 0.0 and geom.length_m > 0.0:
        cf_rough = 0.032 * (aero_cfg.roughness_m / geom.length_m) ** 0.2
        cf = max(cf, cf_rough)

    cd_friction = cf * (geom.wetted_area_m2 / max(geom.frontal_area_m2, 1.0e-9))
    cd_base = 0.12 + 0.13 * (mach * mach) / (1.0 + mach * mach)

    if mach < 0.80:
        cd_wave = 0.0
    elif mach < 1.20:
        phase = (mach - 0.80) / 0.40
        cd_wave = 0.22 * (math.sin(math.pi * phase) ** 2) * aero_cfg.transonic_bump_scale
    else:
        cd_wave = 0.10 / math.sqrt(max(mach, 1.0e-6))

    cd_total = cd_friction + cd_base + cd_wave
    if aero_cfg.conservative:
        cd_total *= 1.08
    cd_total *= aero_cfg.flight_cd_scale
    cd_total = max(cd_total, aero_cfg.min_cd)

    return {
        "cd_total": float(cd_total),
        "cd_friction": float(cd_friction),
        "cd_base": float(cd_base),
        "cd_wave": float(cd_wave),
        "mach": float(mach),
        "re": float(re),
    }


def build_cd_table(env_cfg: EnvironmentConfig, aero_cfg: AeroConfig, geom: GeometryProperties) -> np.ndarray:
    atm = isa_atmosphere(0.0, env_cfg)
    rows: list[tuple[float, float, float, float]] = []
    for speed in aero_cfg.v_sweep_mps:
        c = cd_components(
            speed_mps=float(speed),
            env_cfg=env_cfg,
            aero_cfg=aero_cfg,
            geom=geom,
            rho_kgm3=atm.density_kgm3,
            mu_pas=atm.mu_pas,
            a_mps=atm.speed_of_sound_mps,
        )
        rows.append((float(speed), c["cd_total"], c["mach"], c["re"]))
    return np.asarray(rows, dtype=np.float64)


def cd_from_table(speed_mps: float, table: np.ndarray) -> float:
    if table.shape[0] == 0:
        return 0.5
    return float(np.interp(abs(speed_mps), table[:, 0], table[:, 1], left=table[0, 1], right=table[-1, 1]))


def drag_force_vector(v_rel: np.ndarray, rho_kgm3: float, cd: float, ref_area_m2: float) -> np.ndarray:
    speed = float(np.linalg.norm(v_rel))
    if speed <= 1.0e-9:
        return np.zeros(2, dtype=np.float64)
    mag = 0.5 * rho_kgm3 * cd * ref_area_m2 * speed
    return -mag * v_rel
