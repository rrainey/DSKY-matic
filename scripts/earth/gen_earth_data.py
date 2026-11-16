#!/usr/bin/env python3
"""
Convert GSHHG intermediate-resolution shapefiles into the compact binary
format consumed by MapView (DSKY-matic launch visualization).

Inputs  (expected at $GSHHG_ROOT/GSHHS_shp/i/):
  GSHHS_i_L1.shp   — continental land polygons
  GSHHS_i_L2.shp   — lakes inside continents

Outputs (written to data/earth/):
  gshhs_i_land.bin      triangles, land    (level 1)
  gshhs_i_lakes.bin     triangles, lakes   (level 2)
  gshhs_i_coastline.bin polylines, land outlines (level 1 boundary)

Binary format — triangles file:
  "ETRI"  u32 magic
   1      u32 version
   N      u32 triangle count
   repeat N times:
     f32 lat0, f32 lon0, f32 lat1, f32 lon1, f32 lat2, f32 lon2   (degrees)

Binary format — polylines file:
  "ELIN"  u32 magic
   1      u32 version
   P      u32 polyline count
   repeat P times:
     u32 vertex count
     u32 flags (bit 0 = closed loop, for future use)
     repeat: f32 lat, f32 lon   (degrees)

All numeric values are stored little-endian.  Polygons are triangulated
in (lon, lat) space using mapbox_earcut.  Triangles whose longest
great-circle edge exceeds --max-arc-deg are subdivided so the rendered
mesh stays close to the sphere surface.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
from pathlib import Path
from typing import Iterable

import mapbox_earcut as earcut
import numpy as np
import shapefile  # pyshp


# -----------------------------------------------------------------------
# Great-circle utilities
# -----------------------------------------------------------------------

def _to_unit(lat_deg: float, lon_deg: float) -> np.ndarray:
    """(lat, lon) degrees → unit 3-vector on the sphere."""
    la = math.radians(lat_deg)
    lo = math.radians(lon_deg)
    c = math.cos(la)
    return np.array([c * math.cos(lo), c * math.sin(lo), math.sin(la)])


def _from_unit(v: np.ndarray) -> tuple[float, float]:
    """Unit 3-vector → (lat, lon) degrees."""
    n = v / np.linalg.norm(v)
    return (math.degrees(math.asin(n[2])),
            math.degrees(math.atan2(n[1], n[0])))


def _midpoint_great_circle(a: tuple[float, float],
                           b: tuple[float, float]) -> tuple[float, float]:
    """Great-circle midpoint of two (lat, lon) points."""
    va = _to_unit(*a)
    vb = _to_unit(*b)
    mid = va + vb
    return _from_unit(mid)


def _arc_deg(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Angular distance along the great circle, degrees."""
    va = _to_unit(*a)
    vb = _to_unit(*b)
    dot = max(-1.0, min(1.0, float(np.dot(va, vb))))
    return math.degrees(math.acos(dot))


# -----------------------------------------------------------------------
# Triangle subdivision to keep edges short
# -----------------------------------------------------------------------

def _subdivide(tris: list[tuple], max_arc_deg: float) -> list[tuple]:
    """
    Split any triangle whose longest great-circle edge exceeds max_arc_deg.
    Each split halves the longest edge (great-circle midpoint) and produces
    two smaller triangles.  Applied iteratively via a worklist.
    """
    out: list[tuple] = []
    work: list[tuple] = list(tris)
    while work:
        a, b, c = work.pop()
        ab = _arc_deg(a, b)
        bc = _arc_deg(b, c)
        ca = _arc_deg(c, a)
        longest = max(ab, bc, ca)
        if longest <= max_arc_deg:
            out.append((a, b, c))
            continue
        if ab == longest:
            m = _midpoint_great_circle(a, b)
            work.append((a, m, c))
            work.append((m, b, c))
        elif bc == longest:
            m = _midpoint_great_circle(b, c)
            work.append((a, b, m))
            work.append((a, m, c))
        else:
            m = _midpoint_great_circle(c, a)
            work.append((a, b, m))
            work.append((m, b, c))
    return out


# -----------------------------------------------------------------------
# Shapefile → triangles / polylines
# -----------------------------------------------------------------------

def _triangulate_shape(points: list[tuple[float, float]],
                       parts: list[int]) -> list[tuple]:
    """
    Earcut-tessellate one shapefile polygon (with optional holes) in
    (lon, lat) planar space.  Returns a list of (a, b, c) triples where
    each vertex is (lat, lon) degrees.
    """
    # points come in as (lon, lat) from pyshp.
    # earcut wants an (N,2) float64 array + a u32 array of ring END
    # indices (one entry per ring, pointing to one-past the last vertex
    # of that ring).  Shapefile `parts` is ring START indices, so we
    # convert by shifting and appending the total vertex count.
    flat = np.asarray(points, dtype=np.float64)
    ring_ends = np.asarray(list(parts[1:]) + [len(points)], dtype=np.uint32)
    tri_idx = earcut.triangulate_float64(flat, ring_ends)
    tris: list[tuple] = []
    for i in range(0, len(tri_idx), 3):
        i0, i1, i2 = tri_idx[i], tri_idx[i + 1], tri_idx[i + 2]
        p0 = (points[i0][1], points[i0][0])  # (lat, lon)
        p1 = (points[i1][1], points[i1][0])
        p2 = (points[i2][1], points[i2][0])
        tris.append((p0, p1, p2))
    return tris


def _polygon_bbox_span_deg(points: list[tuple[float, float]]) -> float:
    """Maximum of (lon span, lat span) in degrees.  Cheap area proxy."""
    if not points:
        return 0.0
    lons = [p[0] for p in points]
    lats = [p[1] for p in points]
    return max(max(lons) - min(lons), max(lats) - min(lats))


def process_level(shp_path: Path,
                  min_span_deg: float,
                  max_arc_deg: float) -> tuple[list[tuple], list[list[tuple[float, float]]]]:
    """Returns (triangles, polylines).  Polylines are in (lat, lon) degrees."""
    r = shapefile.Reader(str(shp_path))
    all_tris: list[tuple] = []
    all_lines: list[list[tuple[float, float]]] = []
    kept = 0
    skipped_tiny = 0
    for shape in r.shapes():
        if not shape.points:
            continue
        span = _polygon_bbox_span_deg(shape.points)
        if span < min_span_deg:
            skipped_tiny += 1
            continue
        kept += 1
        pts = list(shape.points)
        parts = list(shape.parts) if shape.parts else [0]
        tris = _triangulate_shape(pts, parts)
        tris = _subdivide(tris, max_arc_deg)
        all_tris.extend(tris)

        # Polyline: emit each part (outer ring + holes) as its own loop.
        part_ends = parts[1:] + [len(pts)]
        start = 0
        for end in part_ends:
            seg = [(pts[i][1], pts[i][0]) for i in range(start, end)]
            all_lines.append(seg)
            start = end

    print(f"  {shp_path.name}: kept {kept}, skipped {skipped_tiny} "
          f"(< {min_span_deg}° span), triangles={len(all_tris)}, "
          f"polylines={len(all_lines)}")
    return all_tris, all_lines


# -----------------------------------------------------------------------
# Binary writers
# -----------------------------------------------------------------------

def write_triangles(path: Path, tris: list[tuple]) -> None:
    with open(path, "wb") as f:
        f.write(b"ETRI")
        f.write(struct.pack("<I", 1))
        f.write(struct.pack("<I", len(tris)))
        for (a, b, c) in tris:
            f.write(struct.pack("<ffffff",
                                a[0], a[1], b[0], b[1], c[0], c[1]))
    print(f"  wrote {path} ({path.stat().st_size // 1024} KB, {len(tris)} tri)")


def write_polylines(path: Path, lines: list[list[tuple[float, float]]]) -> None:
    with open(path, "wb") as f:
        f.write(b"ELIN")
        f.write(struct.pack("<I", 1))
        f.write(struct.pack("<I", len(lines)))
        for seg in lines:
            f.write(struct.pack("<II", len(seg), 1))  # flags: closed loop
            for (la, lo) in seg:
                f.write(struct.pack("<ff", la, lo))
    print(f"  wrote {path} ({path.stat().st_size // 1024} KB, {len(lines)} lines)")


# -----------------------------------------------------------------------
# Optional preview PNG
# -----------------------------------------------------------------------

def render_preview(out_path: Path,
                   land_tris: list[tuple],
                   lake_tris: list[tuple],
                   coastlines: list[list[tuple[float, float]]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection, LineCollection

    fig, ax = plt.subplots(figsize=(14, 7), dpi=120, facecolor="black")
    ax.set_facecolor("black")

    land_polys = [[(t[0][1], t[0][0]),
                   (t[1][1], t[1][0]),
                   (t[2][1], t[2][0])] for t in land_tris]
    pc = PolyCollection(land_polys, facecolors="#404040", edgecolors="none")
    ax.add_collection(pc)

    lake_polys = [[(t[0][1], t[0][0]),
                   (t[1][1], t[1][0]),
                   (t[2][1], t[2][0])] for t in lake_tris]
    if lake_polys:
        lc = PolyCollection(lake_polys, facecolors="#000000", edgecolors="none")
        ax.add_collection(lc)

    coast_segs = [[(p[1], p[0]) for p in line] for line in coastlines]
    lsc = LineCollection(coast_segs, colors="#808080", linewidths=0.3)
    ax.add_collection(lsc)

    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    fig.tight_layout(pad=0)
    fig.savefig(out_path, facecolor="black", bbox_inches="tight", pad_inches=0)
    print(f"  wrote preview {out_path}")


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gshhg-root", required=True,
                    help="Unpacked gshhg-shp-2.3.7 directory "
                         "(contains GSHHS_shp/).")
    ap.add_argument("--out-dir", required=True,
                    help="Output dir (e.g. DSKY-matic/data/earth).")
    ap.add_argument("--min-span-deg", type=float, default=0.3,
                    help="Skip polygons whose bbox span (max of lat/lon "
                         "extent) is below this.  0.3° ≈ 33 km at equator; "
                         "sub-pixel at our render scale.")
    ap.add_argument("--max-arc-deg", type=float, default=2.0,
                    help="Subdivide triangles whose longest great-circle "
                         "edge exceeds this.  Keeps the mesh close to the "
                         "sphere surface.")
    ap.add_argument("--preview", action="store_true",
                    help="Also emit a PNG preview of the processed data.")
    args = ap.parse_args()

    root = Path(args.gshhg_root).expanduser().resolve()
    out  = Path(args.out_dir).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    l1 = root / "GSHHS_shp" / "i" / "GSHHS_i_L1.shp"
    l2 = root / "GSHHS_shp" / "i" / "GSHHS_i_L2.shp"
    for p in (l1, l2):
        if not p.exists():
            print(f"missing shapefile: {p}", file=sys.stderr)
            return 2

    print("Level 1 (land) ...")
    land_tris, coastlines = process_level(l1, args.min_span_deg, args.max_arc_deg)
    print("Level 2 (lakes) ...")
    lake_tris, _          = process_level(l2, args.min_span_deg, args.max_arc_deg)

    write_triangles(out / "gshhs_i_land.bin",  land_tris)
    write_triangles(out / "gshhs_i_lakes.bin", lake_tris)
    write_polylines(out / "gshhs_i_coastline.bin", coastlines)

    if args.preview:
        render_preview(out / "earth_preview.png",
                       land_tris, lake_tris, coastlines)
    return 0


if __name__ == "__main__":
    sys.exit(main())
