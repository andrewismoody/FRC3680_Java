"""
Plot resolved fixture positions from a season auto JSON.

python tools/plot_fixtures.py --file src/main/deploy/auto2025.json --out plots/fixtures.png --debug-resolve

This tool uses the Java FixtureResolver as the single source of truth.
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


def _run_java_resolved_dump(
    season_file: str,
    java_cmd: str,
    classpath: str,
    debug: bool,
) -> Dict[str, Any]:
    cmd = [java_cmd, "-cp", classpath, "frc.robot.auto.ResolvedFixtureDump", "--file", season_file]
    if debug:
        cmd.append("--debug")
        print("[java]", " ".join(cmd))

    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError(f"Java dump failed ({p.returncode}):\n{p.stderr.strip()}")

    if debug and p.stderr.strip():
        print("[java-stderr]", p.stderr.strip())

    return json.loads(p.stdout)


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
    args = ap.parse_args()

    path = Path(args.file)
    cp = args.java_classpath or _default_java_classpath()

    dump = _run_java_resolved_dump(str(path), args.java, cp, args.debug_resolve)
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
