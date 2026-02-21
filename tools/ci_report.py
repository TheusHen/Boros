#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_json(path: Path) -> dict | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def load_kv(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.exists():
        return out
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate concise CI markdown summary")
    ap.add_argument("--repo-root", default=".", type=str)
    ap.add_argument("--out", default="", type=str, help="Optional markdown output file")
    args = ap.parse_args()

    root = Path(args.repo_root).resolve()
    firmware_bin = root / "firmware" / "firmware.bin"
    firmware_elf = root / "firmware" / "firmware.elf"
    py_summary = load_json(root / "sim" / "flight" / "out" / "summary.json")
    py_mc = load_json(root / "sim" / "flight" / "out" / "monte_carlo" / "monte_carlo_summary.json")
    m3d = load_kv(root / "sim" / "3d" / "out" / "report_matlab_summary.txt")

    lines: list[str] = []
    lines.append("## CI Simulation Report")
    lines.append("")
    lines.append("### Firmware")
    lines.append(
        f"- `firmware.bin`: {'OK' if firmware_bin.exists() else 'missing'}"
        + (f" ({firmware_bin.stat().st_size} bytes)" if firmware_bin.exists() else "")
    )
    lines.append(
        f"- `firmware.elf`: {'OK' if firmware_elf.exists() else 'missing'}"
        + (f" ({firmware_elf.stat().st_size} bytes)" if firmware_elf.exists() else "")
    )
    lines.append("")

    lines.append("### Python Flight")
    if py_summary:
        s = py_summary.get("summary", {})
        lines.append(f"- Apogee: `{s.get('apogee_m', 'n/a')}` m")
        lines.append(f"- Max Mach: `{s.get('max_mach', 'n/a')}`")
        lines.append(f"- Drift: `{s.get('drift_m', 'n/a')}` m")
        lines.append(f"- Flight time: `{s.get('flight_time_s', 'n/a')}` s")
    else:
        lines.append("- Summary JSON not found")
    if py_mc:
        apogee = py_mc.get("apogee_m", {})
        drift = py_mc.get("abs_drift_m", {})
        lines.append(f"- Monte Carlo Apogee P50/P90: `{apogee.get('p50', 'n/a')}` / `{apogee.get('p90', 'n/a')}` m")
        lines.append(f"- Monte Carlo |Drift| P50/P90: `{drift.get('p50', 'n/a')}` / `{drift.get('p90', 'n/a')}` m")
    else:
        lines.append("- Monte Carlo summary not found")
    lines.append("")

    lines.append("### MATLAB/Octave 3D")
    if m3d:
        lines.append(f"- Apogee: `{m3d.get('apogee_m', 'n/a')}` m")
        lines.append(f"- Drift: `{m3d.get('drift_m', 'n/a')}` m")
        lines.append(f"- Flight time: `{m3d.get('flight_time_s', 'n/a')}` s")
        lines.append(f"- Chute diameter: `{m3d.get('chute_diameter_m', 'n/a')}` m")
    else:
        lines.append("- 3D summary not found")
    lines.append("")

    text = "\n".join(lines) + "\n"
    print(text)

    if args.out:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
