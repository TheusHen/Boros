from __future__ import annotations

from dataclasses import dataclass
import math

try:
    from .constants import EnvironmentConfig
except ImportError:  # pragma: no cover
    from constants import EnvironmentConfig


@dataclass
class AtmosphereSample:
    altitude_m: float
    temperature_k: float
    pressure_pa: float
    density_kgm3: float
    mu_pas: float
    speed_of_sound_mps: float


def isa_atmosphere(altitude_m: float, env: EnvironmentConfig) -> AtmosphereSample:
    h = max(0.0, float(altitude_m))
    if h <= 11000.0:
        t = env.temp0_k - env.lapse_k_per_m * h
        t_ratio = max(t / env.temp0_k, 1.0e-6)
        p = env.p0_pa * t_ratio ** (env.g / (env.R * env.lapse_k_per_m))
    else:
        t = env.temp0_k - env.lapse_k_per_m * 11000.0
        p11 = env.p0_pa * (t / env.temp0_k) ** (env.g / (env.R * env.lapse_k_per_m))
        p = p11 * math.exp(-(h - 11000.0) / env.H)

    rho = p / (env.R * t)
    mu = env.mu0 * (t / env.temp0_k) ** 1.5 * ((env.temp0_k + env.sutherland_c) / (t + env.sutherland_c))
    a = math.sqrt(max(env.gamma * env.R * t, 1.0e-9))

    return AtmosphereSample(
        altitude_m=h,
        temperature_k=t,
        pressure_pa=p,
        density_kgm3=rho,
        mu_pas=mu,
        speed_of_sound_mps=a,
    )


def pressure_from_altitude(altitude_m: float, env: EnvironmentConfig) -> float:
    return isa_atmosphere(altitude_m, env).pressure_pa


def altitude_from_pressure(pressure_pa: float, env: EnvironmentConfig, base_pressure_pa: float | None = None) -> float:
    if pressure_pa <= 0.0:
        return 0.0
    p_ref = env.p0_pa if base_pressure_pa is None else max(base_pressure_pa, 1.0)
    return max(0.0, env.H * math.log(p_ref / pressure_pa))


def pressure_to_altitude_cm(pressure_pa: float, base_pressure_pa: float, env: EnvironmentConfig) -> int:
    return int(round(altitude_from_pressure(pressure_pa, env, base_pressure_pa) * 100.0))
