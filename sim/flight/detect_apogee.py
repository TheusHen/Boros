from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def moving_average(x: np.ndarray, window: int) -> np.ndarray:
    window = max(1, int(window))
    if window == 1:
        return x
    kernel = np.ones(window, dtype=np.float64) / float(window)
    return np.convolve(x, kernel, mode="same")


def detect_apogee_index(
    t_s: np.ndarray,
    altitude_cm: np.ndarray,
    window_s: float = 0.4,
    descent_threshold_mps: float = -0.15,
    consecutive_samples: int = 6,
) -> int | None:
    if t_s.size < 5:
        return None
    dt = float(np.median(np.diff(t_s)))
    win = max(3, int(round(window_s / max(dt, 1.0e-3))))
    alt_m = altitude_cm.astype(np.float64) / 100.0
    alt_s = moving_average(alt_m, win)
    vz = np.gradient(alt_s, t_s)

    desc_count = 0
    for i in range(win, t_s.size):
        if alt_s[i] < 1.0:
            continue
        if vz[i] <= descent_threshold_mps:
            desc_count += 1
            if desc_count >= consecutive_samples:
                return i - consecutive_samples + 1
        else:
            desc_count = 0
    return None


def load_firmware_like_log(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    times = []
    alt = []
    with Path(path).open("r", encoding="utf-8") as fh:
        rd = csv.DictReader(fh)
        for row in rd:
            times.append(float(row["ms"]) / 1000.0)
            alt.append(float(row["altitude_cm"]))
    return np.asarray(times, dtype=np.float64), np.asarray(alt, dtype=np.float64)


def main() -> None:
    ap = argparse.ArgumentParser(description="Detect apogee from firmware-like barometric log")
    ap.add_argument("log_csv", type=str)
    ap.add_argument("--window-s", type=float, default=0.4)
    ap.add_argument("--descent-threshold-mps", type=float, default=-0.15)
    ap.add_argument("--consecutive", type=int, default=6)
    args = ap.parse_args()

    t_s, altitude_cm = load_firmware_like_log(args.log_csv)
    idx = detect_apogee_index(
        t_s=t_s,
        altitude_cm=altitude_cm,
        window_s=args.window_s,
        descent_threshold_mps=args.descent_threshold_mps,
        consecutive_samples=args.consecutive,
    )
    if idx is None:
        print("Apogee not detected.")
        return
    print(
        f"Apogee detect index={idx} time={t_s[idx]:.3f}s altitude={altitude_cm[idx]/100.0:.2f}m",
    )


if __name__ == "__main__":
    main()
