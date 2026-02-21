#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SIM_FLIGHT_DIR = REPO_ROOT / "sim" / "flight"
if str(SIM_FLIGHT_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_FLIGHT_DIR))

from constants import default_stl_path  # type: ignore  # noqa: E402
from forces import load_stl_geometry  # type: ignore  # noqa: E402


def oz_ft2_to_thickness_m(oz_ft2: float) -> float:
    # 1 oz/ft^2 copper ~= 34.79 um
    return max(oz_ft2, 0.0) * 34.79e-6


def build_markdown(data: dict) -> str:
    inp = data["inputs"]
    stl = data["stl_geometry"]
    airframe = data["airframe"]
    pcb = data["pcb_plate"]
    total = data["totals"]
    checks = data["checks"]

    lines: list[str] = []
    lines.append("## Mass Budget Tool")
    lines.append("")
    lines.append("### Inputs")
    lines.append(f"- STL: `{inp['stl_path']}`")
    lines.append(f"- STL unit scale: `{inp['stl_unit_scale']}`")
    lines.append(f"- Motor mass: `{inp['motor_mass_g']}` g")
    lines.append(f"- Motor max liftoff mass: `{inp['motor_max_liftoff_mass_g']}` g")
    lines.append("")
    lines.append("### 3D Model Geometry")
    lines.append(f"- Volume: `{stl['volume_cm3']}` cm^3")
    lines.append(f"- Length: `{stl['length_mm']}` mm")
    lines.append(f"- Body diameter: `{stl['body_diameter_mm']}` mm")
    lines.append("")
    lines.append("### Airframe Mass")
    lines.append(f"- Material density: `{airframe['material_density_g_cm3']}` g/cm^3")
    lines.append(f"- Solid mass (100% volume): `{airframe['solid_mass_g']}` g")
    lines.append(f"- Print factor: `{airframe['mass_factor']}`")
    lines.append(f"- Estimated 3D mass: `{airframe['estimated_mass_g']}` g")
    lines.append("")
    lines.append("### PCB Plate (25mm x 62mm, 2 layers)")
    lines.append(f"- FR4 thickness: `{pcb['fr4_thickness_mm']}` mm")
    lines.append(f"- Copper per layer: `{pcb['copper_oz_per_ft2']}` oz/ft^2")
    lines.append(f"- Copper thickness per layer: `{pcb['copper_thickness_um_per_layer']}` um")
    lines.append(f"- Copper coverage: `{pcb['copper_coverage_fraction']}`")
    lines.append(f"- FR4 mass: `{pcb['mass_fr4_g']}` g")
    lines.append(f"- Copper mass: `{pcb['mass_copper_g']}` g")
    lines.append(f"- Estimated PCB mass: `{pcb['mass_total_g']}` g")
    lines.append("")
    lines.append("### Totals")
    lines.append(f"- Estimated liftoff mass: `{total['estimated_liftoff_mass_g']}` g")
    lines.append(f"- Margin to motor limit: `{total['margin_to_limit_g']}` g")
    lines.append(f"- Suggested `simulate.py --mass-total-g`: `{total['suggested_sim_mass_total_g']}`")
    lines.append("")
    lines.append("### Check")
    lines.append(f"- Within motor limit (<= {inp['motor_max_liftoff_mass_g']} g): `{checks['within_motor_limit']}`")
    lines.append("")
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser(description="Compute mass budget from STL + motor + 2-layer PCB plate")
    ap.add_argument("--stl", type=str, default=str(default_stl_path()))
    ap.add_argument("--stl-unit-scale", type=float, default=1.0e-3, help="STL units to meters scale factor")
    ap.add_argument("--body-quantile", type=float, default=0.90)

    ap.add_argument("--motor-mass-g", type=float, default=24.1)
    ap.add_argument("--motor-max-liftoff-mass-g", type=float, default=113.0)

    ap.add_argument("--airframe-density-g-cm3", type=float, default=1.24, help="Material density (PLA default)")
    ap.add_argument("--airframe-mass-factor", type=float, default=1.0, help="Scale over solid STL mass")

    ap.add_argument("--pcb-width-mm", type=float, default=25.0)
    ap.add_argument("--pcb-height-mm", type=float, default=62.0)
    ap.add_argument("--pcb-total-thickness-mm", type=float, default=1.6)
    ap.add_argument("--pcb-copper-layers", type=int, default=2)
    ap.add_argument("--pcb-copper-oz", type=float, default=1.0)
    ap.add_argument("--pcb-copper-coverage", type=float, default=1.0)
    ap.add_argument("--pcb-fr4-density-kg-m3", type=float, default=1850.0)
    ap.add_argument("--pcb-copper-density-kg-m3", type=float, default=8960.0)

    ap.add_argument("--out-json", type=str, default="sim/flight/out/mass_budget.json")
    ap.add_argument("--out-md", type=str, default="sim/flight/out/mass_budget.md")
    args = ap.parse_args()

    requested_stl = Path(args.stl)
    stl_candidates: list[Path] = [requested_stl]
    default_candidate = Path(default_stl_path())
    named_fallbacks = [
        REPO_ROOT / "3d" / "BorosRocket3D.stl",
        REPO_ROOT / "3d" / "BorosRocket_ToPrint.stl",
    ]
    auto_fallbacks = sorted((REPO_ROOT / "3d").glob("*.stl"))
    for cand in [default_candidate, *named_fallbacks, *auto_fallbacks]:
        if cand not in stl_candidates:
            stl_candidates.append(cand)

    stl_path: Path | None = None
    for cand in stl_candidates:
        if cand.exists():
            stl_path = cand
            break
    if stl_path is None:
        joined = ", ".join(str(p) for p in stl_candidates)
        raise SystemExit(f"STL not found. Tried: {joined}")

    geom = load_stl_geometry(stl_path, args.stl_unit_scale, args.body_quantile)
    volume_cm3 = geom.volume_m3 * 1.0e6
    solid_mass_g = volume_cm3 * max(args.airframe_density_g_cm3, 0.0)
    estimated_airframe_mass_g = solid_mass_g * max(args.airframe_mass_factor, 0.0)

    area_m2 = (max(args.pcb_width_mm, 0.0) * 1.0e-3) * (max(args.pcb_height_mm, 0.0) * 1.0e-3)
    copper_layers = max(int(args.pcb_copper_layers), 0)
    copper_t_m = oz_ft2_to_thickness_m(args.pcb_copper_oz)
    copper_cov = min(max(args.pcb_copper_coverage, 0.0), 1.0)
    total_t_m = max(args.pcb_total_thickness_mm, 0.0) * 1.0e-3
    copper_total_t_m = copper_layers * copper_t_m
    fr4_t_m = max(total_t_m - copper_total_t_m, 0.0)

    mass_fr4_g = area_m2 * fr4_t_m * max(args.pcb_fr4_density_kg_m3, 0.0) * 1000.0
    mass_copper_g = area_m2 * copper_total_t_m * max(args.pcb_copper_density_kg_m3, 0.0) * 1000.0 * copper_cov
    mass_pcb_g = mass_fr4_g + mass_copper_g

    estimated_liftoff_g = estimated_airframe_mass_g + max(args.motor_mass_g, 0.0) + mass_pcb_g
    margin_g = max(args.motor_max_liftoff_mass_g, 0.0) - estimated_liftoff_g
    within_limit = estimated_liftoff_g <= max(args.motor_max_liftoff_mass_g, 0.0)

    payload = {
        "inputs": {
            "stl_path": str(stl_path.resolve()),
            "stl_unit_scale": args.stl_unit_scale,
            "body_quantile": args.body_quantile,
            "motor_mass_g": args.motor_mass_g,
            "motor_max_liftoff_mass_g": args.motor_max_liftoff_mass_g,
        },
        "stl_geometry": {
            "length_mm": round(geom.length_m * 1000.0, 3),
            "body_diameter_mm": round(geom.body_diameter_m * 1000.0, 3),
            "max_diameter_mm": round(geom.max_diameter_m * 1000.0, 3),
            "volume_cm3": round(volume_cm3, 5),
            "wetted_area_cm2": round(geom.wetted_area_m2 * 1.0e4, 5),
            "frontal_area_cm2": round(geom.frontal_area_m2 * 1.0e4, 5),
        },
        "airframe": {
            "material_density_g_cm3": args.airframe_density_g_cm3,
            "solid_mass_g": round(solid_mass_g, 5),
            "mass_factor": args.airframe_mass_factor,
            "estimated_mass_g": round(estimated_airframe_mass_g, 5),
        },
        "pcb_plate": {
            "width_mm": args.pcb_width_mm,
            "height_mm": args.pcb_height_mm,
            "area_cm2": round(area_m2 * 1.0e4, 5),
            "total_thickness_mm": args.pcb_total_thickness_mm,
            "fr4_thickness_mm": round(fr4_t_m * 1000.0, 5),
            "layers": copper_layers,
            "copper_oz_per_ft2": args.pcb_copper_oz,
            "copper_thickness_um_per_layer": round(copper_t_m * 1.0e6, 4),
            "copper_coverage_fraction": copper_cov,
            "mass_fr4_g": round(mass_fr4_g, 5),
            "mass_copper_g": round(mass_copper_g, 5),
            "mass_total_g": round(mass_pcb_g, 5),
        },
        "totals": {
            "estimated_liftoff_mass_g": round(estimated_liftoff_g, 5),
            "margin_to_limit_g": round(margin_g, 5),
            "suggested_sim_mass_total_g": round(estimated_liftoff_g, 3),
        },
        "checks": {
            "within_motor_limit": within_limit,
        },
    }

    out_json = Path(args.out_json)
    out_md = Path(args.out_md)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    out_md.write_text(build_markdown(payload), encoding="utf-8")

    print("=== Mass Budget ===")
    print(f"3D estimated mass: {payload['airframe']['estimated_mass_g']} g")
    print(f"Motor mass: {args.motor_mass_g:.3f} g")
    print(f"PCB estimated mass: {payload['pcb_plate']['mass_total_g']} g")
    print(f"Estimated liftoff mass: {payload['totals']['estimated_liftoff_mass_g']} g")
    print(f"Motor limit: {args.motor_max_liftoff_mass_g:.3f} g")
    print(f"Margin: {payload['totals']['margin_to_limit_g']} g")
    print(f"Within limit: {within_limit}")
    print(f"JSON: {out_json}")
    print(f"Markdown: {out_md}")


if __name__ == "__main__":
    main()
