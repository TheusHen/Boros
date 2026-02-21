from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Literal, Sequence
import copy


WindProfile = Literal["constant", "linear", "piecewise"]
RecoveryMode = Literal["time", "apogee"]


# RASP/NAR C6 curve shape (seconds, Newtons), then scaled to target impulse if requested.
DEFAULT_C6_THRUST_CURVE: tuple[tuple[float, float], ...] = (
    (0.000, 0.000),
    (0.031, 0.946),
    (0.092, 4.826),
    (0.139, 9.936),
    (0.192, 14.090),
    (0.209, 11.446),
    (0.231, 7.381),
    (0.248, 6.151),
    (0.292, 5.489),
    (0.370, 4.921),
    (0.475, 4.448),
    (0.671, 4.258),
    (0.702, 4.542),
    (0.723, 4.164),
    (0.850, 4.448),
    (1.063, 4.353),
    (1.211, 4.353),
    (1.242, 4.069),
    (1.303, 4.258),
    (1.468, 4.353),
    (1.656, 4.448),
    (1.821, 4.448),
    (1.834, 2.933),
    (1.847, 1.325),
    (1.860, 0.000),
)


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_stl_path() -> Path:
    return project_root() / "3d" / "BorosRocket.stl"


def default_output_dir() -> Path:
    return project_root() / "sim" / "flight" / "out"


@dataclass
class GeometryConfig:
    stl_path: str = str(default_stl_path())
    unit_scale: float = 1.0e-3  # STL in mm -> m
    body_quantile: float = 0.90  # suppress fins/outliers in body diameter estimate


@dataclass
class EnvironmentConfig:
    g: float = 9.80665
    p0_pa: float = 101325.0
    temp0_k: float = 288.15
    rho0: float = 1.225
    mu0: float = 1.81e-5
    a0: float = 340.3
    gamma: float = 1.4
    R: float = 287.05
    lapse_k_per_m: float = 0.0065
    H: float = 8500.0
    sutherland_c: float = 110.4


@dataclass
class AeroConfig:
    roughness_m: float = 2.0e-5
    v_sweep_mps: tuple[float, ...] = tuple(float(v) for v in range(10, 201, 10))
    conservative: bool = True
    flight_cd_scale: float = 1.10
    use_cd_table_for_flight: bool = True
    cd_const_flight: float = 0.60
    transonic_bump_scale: float = 1.00
    min_cd: float = 0.05


@dataclass
class MassConfig:
    total_liftoff_mass_kg: float = 0.076211  # Plate V4 (2-layer) estimate from mass budget (3D + motor + PCB)
    motor_total_mass_kg: float = 0.0241
    motor_prop_mass_kg: float = 0.0122


@dataclass
class EngineConfig:
    delay_s: float = 5.0
    thrust_curve: tuple[tuple[float, float], ...] = DEFAULT_C6_THRUST_CURVE
    target_total_impulse_ns: float | None = 10.0  # Estes spec sheet value


@dataclass
class LaunchConfig:
    angle_deg: float = 88.0
    rail_m: float = 1.0
    initial_x_m: float = 0.0
    initial_z_m: float = 0.0
    initial_vx_mps: float = 0.0
    initial_vz_mps: float = 0.0


@dataclass
class WindConfig:
    profile: WindProfile = "linear"
    constant_mps: float = 0.0
    wind0_mps: float = 1.0
    wind_top_mps: float = 4.0
    h_top_m: float = 300.0
    h_m: tuple[float, ...] = (0.0, 50.0, 100.0, 200.0, 300.0, 400.0)
    wind_mps: tuple[float, ...] = (1.0, 2.0, 2.5, 3.5, 4.0, 4.5)
    gust_sigma_mps: float = 0.6
    gust_tau_s: float = 2.0


@dataclass
class RecoveryConfig:
    enabled: bool = True
    mode: RecoveryMode = "time"
    cd_chute: float = 1.35
    auto_size: bool = False
    v_target_mps: float = 3.0
    diam_m: float = 0.25  # Fixed chute diameter for Plate baseline
    deploy_extra_s: float = 0.0
    inflation_time_s: float = 0.6
    body_drag_after_deploy: bool = True


@dataclass
class OutputConfig:
    out_dir: str = str(default_output_dir())
    timeseries_csv: str = "flight_timeseries.csv"
    summary_json: str = "summary.json"
    cd_table_csv: str = "cd_table.csv"
    firmware_like_csv: str = "firmware_like_log.csv"


@dataclass
class SimulationConfig:
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    env: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    aero: AeroConfig = field(default_factory=AeroConfig)
    mass: MassConfig = field(default_factory=MassConfig)
    engine: EngineConfig = field(default_factory=EngineConfig)
    launch: LaunchConfig = field(default_factory=LaunchConfig)
    wind: WindConfig = field(default_factory=WindConfig)
    recovery: RecoveryConfig = field(default_factory=RecoveryConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    time_step_s: float = 0.002
    max_time_s: float = 180.0
    random_seed: int = 2026


def default_simulation_config() -> SimulationConfig:
    return SimulationConfig()


def clone_config(cfg: SimulationConfig) -> SimulationConfig:
    return copy.deepcopy(cfg)


def config_to_dict(cfg: SimulationConfig) -> dict:
    return asdict(cfg)
