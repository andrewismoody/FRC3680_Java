"""
Plot resolved fixture positions from a season auto JSON.

Usage:
  python tools/plot_fixtures.py
  python tools/plot_fixtures.py --file src/main/deploy/auto2025.json
  python tools/plot_fixtures.py --file ... --out fixtures.png --no-labels --units meters

Notes:
- This intentionally mirrors the current Java FixtureResolver semantics:
  * translation.position defaults to inches unless positionUnits=="meters"
  * translation.rotation defaults to degrees unless rotationUnits=="radians"
  * derivedFrom supports: none, parallel, perpendicular, bisector
  * parallel/perpendicular compute a lookat angle from A->B
  * bisector computes the perpendicular-bisector pose, intersects with a line through A along the bisector rotation
- This script does NOT apply alliance transforms; it plots whatever coordinates are authored/derived in the JSON.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Set, Tuple, Any, List

import matplotlib.pyplot as plt


INCH_TO_M = 0.0254


@dataclass(frozen=True)
class XY:
    x: float
    y: float

    def __add__(self, other: "XY") -> "XY":
        return XY(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "XY") -> "XY":
        return XY(self.x - other.x, self.y - other.y)


def inches_to_m(v: float) -> float:
    return v * INCH_TO_M


# NEW: unit conversion helper for plotting
def to_plot_units_in(value_in_inches: float, plot_units: str) -> float:
    return value_in_inches if plot_units == "inches" else inches_to_m(value_in_inches)


def get_lookat(a: XY, b: XY) -> float:
    return math.atan2(b.y - a.y, b.x - a.x)


def project_parallel(base: XY, angle: float, dist: float) -> XY:
    return XY(base.x + dist * math.cos(angle), base.y + dist * math.sin(angle))


def project_perpendicular(base: XY, angle: float, dist: float) -> XY:
    # match Java: angle + 90deg
    ang = angle + math.pi / 2.0
    return XY(base.x + dist * math.cos(ang), base.y + dist * math.sin(ang))


def perpendicular_bisector_angle(a: XY, b: XY) -> Tuple[XY, float]:
    dx = b.x - a.x
    dy = b.y - a.y
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        raise ValueError("Perpendicular bisector undefined for identical points")
    mid = XY((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)
    theta = math.atan2(dy, dx) + math.pi / 2.0
    # Java returns +180deg on theta
    return mid, theta + math.pi

# NEW: function used by bisector logic (was missing)
def perpendicular_bisector_midpoint_and_angle(a: XY, b: XY) -> Tuple[XY, float]:
    return perpendicular_bisector_angle(a, b)


def line_intersection(p1: XY, a1: float, p2: XY, a2: float) -> XY:
    # parametric lines:
    # L1 = p1 + t*(cos a1, sin a1)
    # L2 = p2 + u*(cos a2, sin a2)
    d1x, d1y = math.cos(a1), math.sin(a1)
    d2x, d2y = math.cos(a2), math.sin(a2)

    denom = d1x * d2y - d1y * d2x
    if abs(denom) < 1e-9:
        return p1  # match Java fallback

    rx = p2.x - p1.x
    ry = p2.y - p1.y
    t = (rx * d2y - ry * d2x) / denom
    return XY(p1.x + t * d1x, p1.y + t * d1y)


def parse_translation_xy(t: Dict[str, Any], plot_units: str) -> Optional[XY]:
    if not isinstance(t, dict):
        return None
    pos = t.get("position")
    if not isinstance(pos, list) or len(pos) < 2:
        return None

    x, y = float(pos[0]), float(pos[1])

    # Per schema: default is inches if omitted
    units = (t.get("positionUnits") or "inches").lower()
    if units == "inches":
        x = to_plot_units_in(x, plot_units)
        y = to_plot_units_in(y, plot_units)
    elif units == "meters":
        # JSON authored in meters; convert to inches first only if plotting inches
        if plot_units == "inches":
            x = x / INCH_TO_M
            y = y / INCH_TO_M
    else:
        # Unknown units -> assume inches
        x = to_plot_units_in(x, plot_units)
        y = to_plot_units_in(y, plot_units)

    return XY(x, y)


def parse_rotation_rad(t: Dict[str, Any]) -> Optional[float]:
    if not isinstance(t, dict):
        return None
    if "rotation" not in t:
        return None
    r = float(t.get("rotation") or 0.0)
    units = (t.get("rotationUnits") or "degrees").lower()  # schema default
    return r if units == "radians" else math.radians(r)

@dataclass(frozen=True)
class XYR:
    p: XY
    r: Optional[float]  # radians

def parse_translation_xyr(t: Dict[str, Any], plot_units: str) -> Optional[XYR]:
    p = parse_translation_xy(t, plot_units)
    if p is None:
        return None
    r = parse_rotation_rad(t)
    return XYR(p=p, r=r)

def fixture_key(ftype: str, index: int) -> str:
    return f"{ftype}:{index}"


class Resolver:
    def __init__(self, fixtures: List[Dict[str, Any]], plot_units: str, params: Optional[Dict[str, float]] = None):
        self.by_key: Dict[str, Dict[str, Any]] = {}
        for f in fixtures:
            t = f.get("type")
            idx = f.get("index")
            if isinstance(t, str) and isinstance(idx, int):
                self.by_key[fixture_key(t, idx)] = f

        self.cache: Dict[str, Optional[XY]] = {}
        self.plot_units = plot_units
        self.params: Dict[str, float] = params or {}

    def _resolve_offset(self, raw: Any) -> float:
        # supports numbers or "$paramName"
        if raw is None:
            return 0.0
        if isinstance(raw, (int, float)):
            return float(raw)
        if isinstance(raw, str):
            s = raw.strip()
            if s.startswith("$"):
                key = s[1:]
                v = self.params.get(key)
                if v is None:
                    raise ValueError(f"Unknown param referenced by offset: {raw}")
                return float(v)
            try:
                return float(s)
            except ValueError:
                return 0.0
        return 0.0

    def resolve_fixture_xyr(self, ftype: str, index: int, visiting: Optional[Set[str]] = None) -> Optional[XYR]:
        k = fixture_key(ftype, index)
        if visiting is None:
            visiting = set()
        if k in visiting:
            return None
        f = self.by_key.get(k)
        if not isinstance(f, dict):
            return None

        visiting.add(k)
        try:
            tr = f.get("translation")
            if tr is not None:
                xyr = parse_translation_xyr(tr, self.plot_units)
                if xyr is not None:
                    return xyr

            d = f.get("derivedFrom")
            p = self.eval_derived_from(d, visiting)
            return XYR(p=p, r=None) if p is not None else None
        finally:
            visiting.remove(k)

    def resolve_arg_xyr(self, fixture_ref: Any, derived_from: Any, visiting: Set[str]) -> Optional[XYR]:
        if isinstance(fixture_ref, dict):
            t = fixture_ref.get("type")
            idx = fixture_ref.get("index")
            if isinstance(t, str) and isinstance(idx, int):
                return self.resolve_fixture_xyr(t, idx, visiting)
        if derived_from is not None:
            p = self.eval_derived_from(derived_from, visiting)
            return XYR(p=p, r=None) if p is not None else None
        return None

    def resolve_fixture_xy(self, ftype: str, index: int, visiting: Optional[Set[str]] = None) -> Optional[XY]:
        k = fixture_key(ftype, index)
        if k in self.cache:
            return self.cache[k]
        if visiting is None:
            visiting = set()
        if k in visiting:
            self.cache[k] = None
            return None

        f = self.by_key.get(k)
        if not isinstance(f, dict):
            self.cache[k] = None
            return None

        visiting.add(k)
        try:
            # direct translation wins
            tr = f.get("translation")
            xy = parse_translation_xy(tr, self.plot_units) if tr is not None else None
            if xy is not None:
                self.cache[k] = xy
                return xy

            d = f.get("derivedFrom")
            xy = self.eval_derived_from(d, visiting)
            self.cache[k] = xy
            return xy
        finally:
            visiting.remove(k)

    # FIX: this method must operate on XYR (because parallel/perpendicular need angle)
    def eval_derived_from(self, d: Any, visiting: Set[str]) -> Optional[XY]:
        if not isinstance(d, dict):
            return None

        fn = (d.get("function") or "none").lower()
        offset = self._resolve_offset(d.get("offset"))

        a = self.resolve_arg_xyr(d.get("fixture"), d.get("derivedFrom"), visiting)
        b = self.resolve_arg_xyr(d.get("fixture2"), d.get("derivedFrom2"), visiting)

        if fn == "none":
            return a.p if a is not None else None

        if fn == "parallel":
            if a is None:
                return None
            ang = a.r if a.r is not None else 0.0
            return project_parallel(a.p, ang, offset)

        if fn == "perpendicular":
            if a is None:
                return None
            ang = a.r if a.r is not None else 0.0
            return project_perpendicular(a.p, ang, offset)

        if fn == "bisector":
            if a is None or b is None:
                return None
            mid, _bis_ang = perpendicular_bisector_midpoint_and_angle(a.p, b.p)
            along = get_lookat(a.p, b.p)
            return project_parallel(mid, along, offset)

        # fallback matches earlier behavior
        return a.p if a is not None else None

    def resolve_arg(self, fixture_ref: Any, derived_from: Any, visiting: Set[str]) -> Optional[XY]:
        if isinstance(fixture_ref, dict):
            t = fixture_ref.get("type")
            idx = fixture_ref.get("index")
            if isinstance(t, str) and isinstance(idx, int):
                return self.resolve_fixture_xy(t, idx, visiting)

        if derived_from is not None:
            return self.eval_derived_from(derived_from, visiting)

        return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="src/main/deploy/auto2025.json", help="Season JSON file path")
    ap.add_argument("--out", default="", help="Optional output image path (png)")
    ap.add_argument("--no-labels", action="store_true", help="Disable fixture text labels")
    # NEW: choose plot units
    ap.add_argument("--units", choices=["inches", "meters"], default="inches", help="Axes units for plotting")
    args = ap.parse_args()

    path = Path(args.file)
    data = json.loads(path.read_text(encoding="utf-8"))

    fixtures = data.get("fixtures") or []
    if not isinstance(fixtures, list):
        raise SystemExit("JSON fixtures is not a list")

    raw_params = data.get("params") or {}
    params: Dict[str, float] = {}
    if isinstance(raw_params, dict):
        for k, v in raw_params.items():
            if isinstance(k, str) and isinstance(v, (int, float)):
                params[k] = float(v)

    res = Resolver(fixtures, plot_units=args.units, params=params)

    # group fixtures by type
    points_by_type: Dict[str, List[Tuple[str, XY]]] = {}
    missing: List[str] = []

    for f in fixtures:
        if not isinstance(f, dict):
            continue
        t = f.get("type")
        idx = f.get("index")
        if not isinstance(t, str) or not isinstance(idx, int):
            continue
        xy = res.resolve_fixture_xy(t, idx)
        label = fixture_key(t, idx)
        if xy is None:
            missing.append(label)
            continue
        points_by_type.setdefault(t, []).append((label, xy))

    if missing:
        print("Unresolved fixtures (skipped):")
        for m in missing:
            print("  ", m)

    plt.figure(figsize=(12, 6))

    # stable coloring by type order
    types_sorted = sorted(points_by_type.keys())
    for t in types_sorted:
        pts = points_by_type[t]
        xs = [p.x for _, p in pts]
        ys = [p.y for _, p in pts]
        plt.scatter(xs, ys, label=t, s=30)
        if not args.no_labels:
            for lbl, p in pts:
                plt.text(p.x, p.y, lbl, fontsize=7)

    plt.title(f"Resolved fixtures: {path.name}")
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
