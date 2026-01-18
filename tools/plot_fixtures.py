"""
Plot resolved fixture positions from a season auto JSON.

Example (robot-frame / closest to real robot path) [INCHES by default]:
  python tools/plot_fixtures.py --file src/main/deploy/auto2025.json --out plots/fixtures.png --debug-resolve --alliance-transform --field-size-x 651.25 --field-size-y 323.25 --red-start-x 0 --red-start-y 0 --red-start-yaw-deg 0 --alliance red --driver-location 1

Notes:
- When --robot-frame is enabled this forwards field/alliance setup into the Java dump tool so Utility.Initialize()
  and DS-dependent code paths match robot behavior as closely as possible.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

import matplotlib.pyplot as plt

INCH_TO_M = 0.0254


@dataclass(frozen=True)
class XY:
    x: float
    y: float


def _default_java_classpath() -> str:
    sep = ";" if sys.platform.startswith("win") else ":"
    return sep.join([
        "build/classes/java/main",
        "build/resources/main",
        "build/libs/*",
    ])


def _to_m(v: float, units: str) -> float:
    u = (units or "inches").lower()
    return v if u == "meters" else (v * INCH_TO_M)


def _run_java_resolved_dump(
    season_file: str,
    java_cmd: str,
    classpath: str,
    debug: bool,
    alliance_transform: bool,
    field_size_x: float | None,
    field_size_y: float | None,
    field_size_units: str,
    red_start_x: float,
    red_start_y: float,
    red_start_units: str,
    red_start_yaw_deg: float,
    alliance: str | None,
    driver_location: int | None,
) -> Dict[str, Any]:
    cmd = [java_cmd, "-cp", classpath, "frc.robot.auto.ResolvedFixtureDump", "--file", season_file]

    if alliance_transform:
        cmd.append("--alliance-transform")

        if field_size_x is not None:
            # CHANGED: Java expects inches; convert only if user specified meters
            cmd += ["--field-size-x", str(field_size_x if field_size_units == "inches" else field_size_x / INCH_TO_M)]
        if field_size_y is not None:
            cmd += ["--field-size-y", str(field_size_y if field_size_units == "inches" else field_size_y / INCH_TO_M)]

        cmd += ["--red-start-x", str(red_start_x if red_start_units == "inches" else red_start_x / INCH_TO_M)]
        cmd += ["--red-start-y", str(red_start_y if red_start_units == "inches" else red_start_y / INCH_TO_M)]
        cmd += ["--red-start-yaw-deg", str(red_start_yaw_deg)]

    if alliance:
        cmd += ["--alliance", alliance]
    if driver_location is not None:
        cmd += ["--driver-location", str(driver_location)]

    if debug:
        cmd.append("--debug")
        print("[java]", " ".join(cmd), flush=True)  # CHANGED

    p = subprocess.run(cmd, capture_output=True, text=True)

    # NEW: show stderr in debug mode (FixtureResolver logs live here)
    if debug and p.stderr.strip():
        print(p.stderr.strip(), file=sys.stderr, flush=True)

    if p.returncode != 0:
        raise RuntimeError(
            "Java dump failed\n"
            f"  cmd: {' '.join(cmd)}\n"
            f"  code: {p.returncode}\n"
            f"  stderr:\n{p.stderr.strip()}\n"
            f"  stdout:\n{p.stdout.strip()}"
        )

    out = (p.stdout or "").strip()
    if not out:
        raise RuntimeError(
            "Java dump produced no stdout JSON\n"
            f"  cmd: {' '.join(cmd)}\n"
            f"  stderr:\n{p.stderr.strip()}\n"
        )

    try:
        return json.loads(out)
    except json.JSONDecodeError as e:
        raise RuntimeError(
            "Java dump produced invalid JSON\n"
            f"  cmd: {' '.join(cmd)}\n"
            f"  stderr:\n{p.stderr.strip()}\n"
            f"  stdout(first 500 chars):\n{out[:500]}\n"
            f"  error: {e}"
        ) from e


def _normalize_xy_units(x: float, y: float, from_units: str, to_units: str) -> XY:
    fu = (from_units or "meters").lower()
    tu = (to_units or "inches").lower()

    if fu == tu:
        return XY(x, y)
    if fu == "meters" and tu == "inches":
        return XY(x / INCH_TO_M, y / INCH_TO_M)
    if fu == "inches" and tu == "meters":
        return XY(x * INCH_TO_M, y * INCH_TO_M)

    # unknown: assume meters coming from Java tool
    if tu == "inches":
        return XY(x / INCH_TO_M, y / INCH_TO_M)
    return XY(x, y)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="src/main/deploy/auto2025.json", help="Season JSON file path")
    ap.add_argument("--out", default="", help="Optional output image path (png)")
    ap.add_argument("--no-labels", action="store_true", help="Disable fixture text labels")
    ap.add_argument("--units", choices=["inches", "meters"], default="inches", help="Axes units for plotting")
    ap.add_argument("--debug-resolve", action="store_true", help="Print Java invocation + stderr")
    ap.add_argument("--java", default="java", help="Java executable")
    ap.add_argument("--java-classpath", default="", help="Classpath for running the dump tool (default: Gradle build outputs)")
    ap.add_argument("--alliance-transform", action="store_true", help="Match robot Resolver behavior (apply alliance transforms)")

    # CHANGED: inputs default to inches; converted to meters when forwarded to Java
    ap.add_argument("--field-size-x", type=float, default=None, help="Field size X (default units: inches)")
    ap.add_argument("--field-size-y", type=float, default=None, help="Field size Y (default units: inches)")
    ap.add_argument("--field-size-units", choices=["inches", "meters"], default="inches")

    ap.add_argument("--red-start-x", type=float, default=0.0, help="Red start transform X (default units: inches)")
    ap.add_argument("--red-start-y", type=float, default=0.0, help="Red start transform Y (default units: inches)")
    ap.add_argument("--red-start-z", type=float, default=0.0, help="Red start transform Z (default units: inches)")
    ap.add_argument("--red-start-units", choices=["inches", "meters"], default="inches")
    ap.add_argument("--red-start-yaw-deg", type=float, default=0.0)

    ap.add_argument("--alliance", choices=["red", "blue"], default=None)
    ap.add_argument("--driver-location", type=int, default=None)

    args = ap.parse_args()

    path = Path(args.file)
    cp = args.java_classpath or _default_java_classpath()

    dump = _run_java_resolved_dump(
        str(path),
        args.java,
        cp,
        args.debug_resolve,
        alliance_transform=args.alliance_transform,
        field_size_x=args.field_size_x,
        field_size_y=args.field_size_y,
        field_size_units=args.field_size_units,
        red_start_x=args.red_start_x,
        red_start_y=args.red_start_y,
        red_start_units=args.red_start_units,
        red_start_yaw_deg=args.red_start_yaw_deg,
        alliance=args.alliance,
        driver_location=args.driver_location,
    )
    fixtures_dump = dump.get("fixtures") or {}
    if not isinstance(fixtures_dump, dict):
        raise SystemExit("Java dump JSON missing 'fixtures' object")

    points_by_type: Dict[str, List[Tuple[str, XY]]] = {}
    missing: List[str] = []

    for key_str, v in fixtures_dump.items():
        if not isinstance(key_str, str) or not isinstance(v, dict):
            continue
        try:
            ftype, _idx_s = key_str.split(":", 1)
        except Exception:
            continue

        if "x" not in v or "y" not in v:
            missing.append(key_str)
            continue

        x = float(v.get("x", 0.0))
        y = float(v.get("y", 0.0))
        units = str(v.get("units", "meters"))
        xy = _normalize_xy_units(x, y, units, args.units)
        points_by_type.setdefault(ftype, []).append((key_str, xy))

    if missing:
        print("Unresolved fixtures (skipped):")
        for m in missing:
            print("  ", m)

    plt.figure(figsize=(12, 6))
    for t in sorted(points_by_type.keys()):
        pts = points_by_type[t]
        xs = [p.x for _, p in pts]
        ys = [p.y for _, p in pts]
        plt.scatter(xs, ys, label=t, s=30)
        if not args.no_labels:
            for lbl, p in pts:
                plt.text(p.x, p.y, lbl, fontsize=7)

    plt.title(f"Resolved fixtures (Java): {path.name}")
    plt.xlabel(f"X ({args.units})")
    plt.ylabel(f"Y ({args.units})")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")

    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(out, dpi=200, bbox_inches="tight")
        print(f"Wrote {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
