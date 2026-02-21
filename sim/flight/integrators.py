from __future__ import annotations

from typing import Callable
import numpy as np


def euler_step(deriv: Callable[[float, np.ndarray], np.ndarray], t: float, y: np.ndarray, dt: float) -> np.ndarray:
    return y + dt * deriv(t, y)


def rk4_step(deriv: Callable[[float, np.ndarray], np.ndarray], t: float, y: np.ndarray, dt: float) -> np.ndarray:
    k1 = deriv(t, y)
    k2 = deriv(t + 0.5 * dt, y + 0.5 * dt * k1)
    k3 = deriv(t + 0.5 * dt, y + 0.5 * dt * k2)
    k4 = deriv(t + dt, y + dt * k3)
    return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
