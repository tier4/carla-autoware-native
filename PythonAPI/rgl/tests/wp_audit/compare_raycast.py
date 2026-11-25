#!/usr/bin/env python3
"""Occupancy-grid comparison between RGL hits and ray_cast_semantic hits.

Usage:
  compare_raycast.py --rgl <dir> --raycast <dir> --cell 1.0 [--mask-tags 22,23] [--static]
                     [--csv <path>]

--rgl <dir>      : debug hit dump files <sensor>_<frame>.bin (rgl.Debug.HitDumpDir; CARLA world coordinates [m]).
--raycast <dir>  : <frame>.npz with xyz (m, CARLA world coordinates) and tag, written by
                   run_scenario.py --with-raycast.
Running comparison pairs the two sides by sim_t (one revolution each). RGL dumps carry
sim_t in the header; ray_cast files carry `sim_t` in the npz. --static pairs the last
dump of each side. Prints:
  rgl_only_cells=<n> raycast_only_cells=<n> both_cells=<n>
and writes a CSV of the differing cells (x,y,z,raycast_tag).

Does not import carla. numpy is required.
"""
import argparse
import glob
import os
import struct
import sys
from collections import Counter

import numpy as np

DUMP_MAGIC = 0x52474C48
DUMP_HEADER = struct.Struct("<IId6f")  # magic, count, sim_t, pose[6]


# Keep the calculation and evaluation order unchanged.
# pylint: disable-next=too-many-locals
def local_to_world(points, location, rotation_deg):
    """Same as carla.Transform(location, Rotation(roll,pitch,yaw)).transform(p).

    CARLA (UE, left-handed): rotation order as FRotationMatrix; rotation_deg = (roll, pitch, yaw).
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    roll, pitch, yaw = (np.radians(v) for v in rotation_deg)
    sr, cr = np.sin(roll), np.cos(roll)
    sp, cp = np.sin(pitch), np.cos(pitch)
    sy, cy = np.sin(yaw), np.cos(yaw)
    ax_x = np.array([cp * cy, cp * sy, sp])
    ax_y = np.array([sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp])
    ax_z = np.array([-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp])
    rot = np.stack([ax_x, ax_y, ax_z], axis=0)  # rows = local axes in world
    return pts @ rot + np.asarray(location, dtype=np.float64)


def _cells(xyz, cell):
    return np.floor(np.asarray(xyz, dtype=np.float64).reshape(-1, 3) / cell).astype(np.int64)


def compare_points(rgl_xyz, ray_xyz, ray_tags, cell=1.0, mask_tags=None):
    """Set comparison of occupied cells. ray_cast points with a tag in mask_tags are dropped."""
    ray_xyz = np.asarray(ray_xyz, dtype=np.float64).reshape(-1, 3)
    ray_tags = np.asarray(ray_tags).reshape(-1)
    if mask_tags:
        keep = ~np.isin(ray_tags, list(mask_tags))
        ray_xyz, ray_tags = ray_xyz[keep], ray_tags[keep]
    rc = _cells(rgl_xyz, cell)
    kc = _cells(ray_xyz, cell)
    rset = set(map(tuple, rc.tolist()))
    kset = set(map(tuple, kc.tolist()))
    tag_by_cell = {}
    for c, t in zip(map(tuple, kc.tolist()), ray_tags.tolist()):
        tag_by_cell.setdefault(c, Counter())[t] += 1

    def centre(c):
        return tuple((np.array(c) + 0.5) * cell)

    rows = []
    for c in sorted(rset - kset):
        rows.append(centre(c) + ("",))
    for c in sorted(kset - rset):
        rows.append(centre(c) + (str(tag_by_cell[c].most_common(1)[0][0]),))
    return {
        "rgl_only_cells": len(rset - kset),
        "raycast_only_cells": len(kset - rset),
        "both_cells": len(rset & kset),
        "diff_rows": rows,
    }


def format_counts(res):
    """Format the occupancy-grid comparison counts."""
    # Preserve percent-format numeric coercion and invalid-input errors.
    # pylint: disable-next=consider-using-f-string
    return "rgl_only_cells=%d raycast_only_cells=%d both_cells=%d" % (
        res["rgl_only_cells"], res["raycast_only_cells"], res["both_cells"])


def write_csv(path, rows):
    """Write differing occupancy cells as a UTF-8 CSV file."""
    with open(path, "w", encoding="utf-8") as f:
        f.write("x,y,z,raycast_tag\n")
        for x, y, z, t in rows:
            f.write(f"{x:.3f},{y:.3f},{z:.3f},{t}\n")


def read_rgl_dump(path):
    """-> (sim_t, sensor_pose[6], xyz Nx3)"""
    with open(path, "rb") as f:
        data = f.read()
    magic, count, sim_t, *pose = DUMP_HEADER.unpack_from(data, 0)
    if magic != DUMP_MAGIC:
        raise ValueError(f"bad magic in {path}")
    body = np.frombuffer(data, dtype="<f4", count=count * 3, offset=DUMP_HEADER.size)
    return sim_t, pose, body.reshape(-1, 3).astype(np.float64)


def read_raycast(path):
    """Read the timestamp, coordinates and tags from a ray-cast dump."""
    z = np.load(path)
    sim_t = float(z["sim_t"]) if "sim_t" in z.files else None
    return sim_t, z["xyz"], z["tag"]


def _rgl_files(d):
    out = []
    for p in sorted(glob.glob(os.path.join(d, "*.bin"))):
        t, _, xyz = read_rgl_dump(p)
        out.append((t, xyz))
    return out


def _ray_files(d):
    out = []
    for p in sorted(glob.glob(os.path.join(d, "*.npz"))):
        t, xyz, tag = read_raycast(p)
        out.append((t, xyz, tag))
    return out


# Keep the existing interface, calculation order and control flow.
# pylint: disable-next=too-many-arguments,too-many-positional-arguments,too-many-locals
def run(rgl_dir, ray_dir, cell, mask_tags, static, tol=0.026):
    """Pair RGL and ray-cast frames and compare their occupied cells."""
    rg = _rgl_files(rgl_dir)
    rc = _ray_files(ray_dir)
    if not rg or not rc:
        return None
    if static:
        pairs = [(rg[-1][1], rc[-1][1], rc[-1][2])]
    else:
        pairs = []
        for t, rxyz in rg:
            best = min(rc, key=lambda e, t=t: abs((e[0] if e[0] is not None else 1e18) - t))
            if best[0] is not None and abs(best[0] - t) <= tol:
                pairs.append((rxyz, best[1], best[2]))
    if not pairs:
        return None
    # accumulate cell sets over all pairs (same sim_t pairs only)
    tot = {"rgl_only_cells": 0, "raycast_only_cells": 0, "both_cells": 0, "diff_rows": []}
    for rxyz, kxyz, ktag in pairs:
        r = compare_points(rxyz, kxyz, ktag, cell, mask_tags)
        for k in ("rgl_only_cells", "raycast_only_cells", "both_cells"):
            tot[k] += r[k]
        tot["diff_rows"].extend(r["diff_rows"])
    tot["pairs"] = len(pairs)
    return tot


def main(argv):
    """Main."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--rgl", required=True)
    ap.add_argument("--raycast", required=True)
    ap.add_argument("--cell", type=float, default=1.0)
    ap.add_argument("--mask-tags", default="")
    ap.add_argument("--static", action="store_true")
    ap.add_argument("--csv", default=None)
    a = ap.parse_args(argv)
    mask = {int(t) for t in a.mask_tags.split(",") if t.strip()}
    res = run(a.rgl, a.raycast, a.cell, mask, a.static)
    if res is None:
        print("status=invalid reason=no_pairs")
        return 1
    csv_path = a.csv or os.path.join(a.rgl, "diff_cells.csv")
    write_csv(csv_path, res["diff_rows"])
    # Preserve percent-format numeric coercion and invalid-input errors.
    # pylint: disable-next=consider-using-f-string
    print("%s pairs=%d csv=%s" % (format_counts(res), res["pairs"], csv_path))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
