from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math

import numpy as np

try:
    from .atmosphere import isa_atmosphere, pressure_to_altitude_cm
    from .constants import SimulationConfig
    from .dynamics import SimulationResult
except ImportError:  # pragma: no cover
    from atmosphere import isa_atmosphere, pressure_to_altitude_cm
    from constants import SimulationConfig
    from dynamics import SimulationResult


LOG_FLAG_BMP_OK = 1 << 0
LOG_FLAG_IMU_OK = 1 << 1
LOG_FLAG_IMU_CAL_DONE = 1 << 2
LOG_FLAG_BARO_BASE_OK = 1 << 3


@dataclass
class SensorNoiseConfig:
    pressure_pa_std: float = 2.0
    accel_mps2_std: float = 0.20
    gyro_dps_std: float = 0.25
    temp_c_std: float = 0.20


def _interp(sample_t: np.ndarray, sim_t: np.ndarray, values: np.ndarray) -> np.ndarray:
    return np.interp(sample_t, sim_t, values, left=values[0], right=values[-1])


def generate_firmware_like_log(
    result: SimulationResult,
    cfg: SimulationConfig,
    out_csv: str | Path,
    sample_period_ms: int = 10,
    noise_cfg: SensorNoiseConfig | None = None,
    seed: int = 2026,
) -> Path:
    noise_cfg = noise_cfg or SensorNoiseConfig()
    out_path = Path(out_csv)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(seed)
    dt = sample_period_ms / 1000.0
    t_samples = np.arange(0.0, result.time_s[-1] + 1.0e-9, dt)

    z = _interp(t_samples, result.time_s, result.z_m)
    ax = _interp(t_samples, result.time_s, result.ax_mps2)
    az = _interp(t_samples, result.time_s, result.az_mps2)

    pressures = np.zeros_like(t_samples)
    temps = np.zeros_like(t_samples)
    for i, alt in enumerate(z):
        atm = isa_atmosphere(float(alt), cfg.env)
        pressures[i] = atm.pressure_pa + rng.normal(0.0, noise_cfg.pressure_pa_std)
        temps[i] = (atm.temperature_k - 273.15) + rng.normal(0.0, noise_cfg.temp_c_std)

    base_pressure = float(np.mean(pressures[: min(100, pressures.size)]))
    flags = LOG_FLAG_BMP_OK | LOG_FLAG_IMU_OK | LOG_FLAG_IMU_CAL_DONE | LOG_FLAG_BARO_BASE_OK

    with out_path.open("w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(
            [
                "ms",
                "seq",
                "pressure_pa",
                "altitude_cm",
                "temp_centi",
                "ax_mg",
                "ay_mg",
                "az_mg",
                "gx_dps_x10",
                "gy_dps_x10",
                "gz_dps_x10",
                "flags",
                "crc_ok",
            ]
        )

        for i, t in enumerate(t_samples):
            ax_mg = int(round(((ax[i] + rng.normal(0.0, noise_cfg.accel_mps2_std)) / cfg.env.g) * 1000.0))
            ay_mg = int(round((rng.normal(0.0, noise_cfg.accel_mps2_std) / cfg.env.g) * 1000.0))
            az_mg = int(
                round((((az[i] + cfg.env.g) + rng.normal(0.0, noise_cfg.accel_mps2_std)) / cfg.env.g) * 1000.0)
            )
            gx_dps_x10 = int(round(rng.normal(0.0, noise_cfg.gyro_dps_std) * 10.0))
            gy_dps_x10 = int(round(rng.normal(0.0, noise_cfg.gyro_dps_std) * 10.0))
            gz_dps_x10 = int(round(rng.normal(0.0, noise_cfg.gyro_dps_std) * 10.0))
            temp_centi = int(round(temps[i] * 100.0))
            alt_cm = pressure_to_altitude_cm(float(pressures[i]), base_pressure, cfg.env)

            wr.writerow(
                [
                    int(round(t * 1000.0)),
                    i,
                    int(round(pressures[i])),
                    alt_cm,
                    temp_centi,
                    ax_mg,
                    ay_mg,
                    az_mg,
                    gx_dps_x10,
                    gy_dps_x10,
                    gz_dps_x10,
                    f"0x{flags:04X}",
                    1,
                ]
            )

    return out_path
