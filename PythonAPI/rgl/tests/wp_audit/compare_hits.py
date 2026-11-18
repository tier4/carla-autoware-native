#!/usr/bin/env python3
"""Point-cloud equivalence check between two runs' hit dumps (the "Odaiba gate").

  compare_hits.py fingerprint <dump_dir> --out <csv>
  compare_hits.py diff <base> <cand>
  compare_hits.py gate --base <dir> --cand <dir> --min-frames <n> --cand-run <run.txt>
                       --base-stderr <file> --cand-stderr <file>

<dir> is a run directory written by run_scenario.py (its `hits/` is used) or a dump
directory itself. Dumps are the server's `-rgl-debug-hit-dump` files
(RGLBackendImpl.cpp): little-endian
  uint32 magic=0x52474c48, uint32 count, double sim_t, float pose[6], count x float[3]
(pose = x, y, z [m], roll, pitch, yaw [deg] of the sensor; points are CARLA world metres).

Matching rules (the file-name id and frame number change from run to run and are never used):
  - the dumps of one run are grouped by file-name id (one id = one sensor session), the groups are
    numbered by their first dump's pose, and a base sensor is paired with the candidate sensor whose
    first-dump pose is nearest (within 1 cm);
  - the k-th dump of a sensor is compared with the k-th dump of its partner;
  - sim_t is compared relative to the sensor's first dump (the absolute value drifts ~0.1 s between
    runs because the world switches to synchronous mode at a slightly different time), within 1 ms.

`diff` prints one `hits_diff ...` line (max_rel_count_diff is a fraction, not a percentage).
`gate` prints `gate verdict=pass|fail reasons=<r1,r2|none> <the hits_diff keys>`; rc 0 only on pass.
No scipy: the nearest-neighbour test uses a voxel hash over the 27 neighbouring voxels.
"""
import argparse
import collections
import glob
import hashlib
import os
import re
import struct
import sys

import numpy as np

DUMP_MAGIC = 0x52474C48
DUMP_HEADER = struct.Struct("<IId6f")   # magic, count, sim_t, pose[6]
TOL_M = 0.01
TIME_TOL_S = 0.001
RANGE_EDGES_M = (50.0, 100.0, 150.0)    # added_by_range / removed_by_range buckets: <50 / 50-100 / 100-150 / >=150
SYNC_TAG = "rgl_scene_sync t="

Dump = collections.namedtuple("Dump", "sim_t pose pts")
SetDiff = collections.namedtuple("SetDiff", "added removed added_by_range removed_by_range")

_HASH_MUL = (73856093, 19349663, 83492791)
_OFFSETS = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)]


# ---------------------------------------------------------------- dump files
def read_dump(path):
    """-> (sim_t, pose[6] float64, points Nx3 float64)"""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < DUMP_HEADER.size:
        raise ValueError("short dump %s" % path)
    magic, count, sim_t, *pose = DUMP_HEADER.unpack_from(data, 0)
    if magic != DUMP_MAGIC:
        raise ValueError("bad magic in %s" % path)
    if len(data) < DUMP_HEADER.size + 12 * count:
        raise ValueError("truncated dump %s" % path)
    pts = np.frombuffer(data, dtype="<f4", count=count * 3, offset=DUMP_HEADER.size)
    return sim_t, np.array(pose, dtype=np.float64), pts.reshape(-1, 3).astype(np.float64)


def _dump_dir(d):
    sub = os.path.join(d, "hits")
    return sub if os.path.isdir(sub) else d


def load_run(dump_dir):
    """-> {sensor_idx: [Dump, ...]} with each list in sim_t order; sensors numbered by their first pose."""
    groups = {}
    for p in sorted(glob.glob(os.path.join(_dump_dir(dump_dir), "*.bin"))):
        sid = os.path.basename(p).split("_", 1)[0]
        t, pose, pts = read_dump(p)
        groups.setdefault(sid, []).append(Dump(t, pose, pts))
    seqs = []
    for dumps in groups.values():
        dumps.sort(key=lambda d: d.sim_t)
        seqs.append(dumps)
    seqs.sort(key=lambda s: tuple(np.round(s[0].pose[:3], 3)))
    return {i: s for i, s in enumerate(seqs)}


def match_sensors(base, cand, tol_m=TOL_M):
    """Pair sensors by the nearest first-dump pose (globally closest first); farther than tol_m = no partner."""
    pairs = []
    for bi, bs in base.items():
        for ci, cs in cand.items():
            d = float(np.linalg.norm(bs[0].pose[:3] - cs[0].pose[:3]))
            if d <= tol_m:
                pairs.append((d, bi, ci))
    pairs.sort()
    used_b, used_c, out = set(), set(), []
    for _, bi, ci in pairs:
        if bi in used_b or ci in used_c:
            continue
        used_b.add(bi)
        used_c.add(ci)
        out.append((bi, ci))
    return sorted(out)


# ---------------------------------------------------------------- point sets
def _cell_hash(cells):
    return (cells[:, 0] * _HASH_MUL[0]) ^ (cells[:, 1] * _HASH_MUL[1]) ^ (cells[:, 2] * _HASH_MUL[2])


def _unmatched(src, ref, tol_m):
    """Boolean mask over src: True where no point of ref lies within tol_m."""
    if len(src) == 0:
        return np.zeros(0, dtype=bool)
    if len(ref) == 0:
        return np.ones(len(src), dtype=bool)
    rcell = np.floor(ref / tol_m).astype(np.int64)
    order = np.argsort(_cell_hash(rcell), kind="stable")
    keys = _cell_hash(rcell)[order]
    ref_sorted = ref[order]
    scell = np.floor(src / tol_m).astype(np.int64)
    found = np.zeros(len(src), dtype=bool)
    for off in _OFFSETS:
        h = _cell_hash(scell + np.array(off, dtype=np.int64))
        lo = np.searchsorted(keys, h, side="left")
        hi = np.searchsorted(keys, h, side="right")
        width = int((hi - lo).max())
        for j in range(width):
            idx = lo + j
            ok = (idx < hi) & ~found
            if not ok.any():
                continue
            cand = ref_sorted[np.minimum(idx, len(ref_sorted) - 1)]
            near = np.linalg.norm(src - cand, axis=1) <= tol_m
            found |= ok & near
    return ~found


def _range_buckets(pts, sensor_xyz):
    if len(pts) == 0:
        return (0, 0, 0, 0)
    r = np.linalg.norm(pts - np.asarray(sensor_xyz, dtype=np.float64), axis=1)
    idx = np.searchsorted(np.array(RANGE_EDGES_M), r, side="right")
    return tuple(int((idx == i).sum()) for i in range(4))


def diff_sets(base_pts, cand_pts, sensor_xyz, tol_m=TOL_M):
    """Points of cand with no base point within tol_m (added) and the other way round (removed)."""
    base = np.asarray(base_pts, dtype=np.float64).reshape(-1, 3)
    cand = np.asarray(cand_pts, dtype=np.float64).reshape(-1, 3)
    added = cand[_unmatched(cand, base, tol_m)]
    removed = base[_unmatched(base, cand, tol_m)]
    return SetDiff(len(added), len(removed), _range_buckets(added, sensor_xyz), _range_buckets(removed, sensor_xyz))


def _sorted_rows(pts):
    if len(pts) == 0:
        return pts
    return pts[np.lexsort((pts[:, 2], pts[:, 1], pts[:, 0]))]


def _identical(a, b):
    return len(a) == len(b) and np.array_equal(_sorted_rows(a), _sorted_rows(b))


# ---------------------------------------------------------------- sync lines
def sync_sequence(stderr_lines):
    """One tuple per `rgl_scene_sync` line: (t relative to the first line [s, 1 ms], trigger, scanned, queued,
    created, removed, entities, meshes)."""
    out, t0 = [], None
    for ln in stderr_lines:
        i = ln.find(SYNC_TAG)
        if i < 0:
            continue
        kv = dict(re.findall(r"(\w+)=(\S+)", ln[i + len("rgl_scene_sync "):]))
        try:
            t = float(kv["t"])
            row = (kv["trigger"],) + tuple(int(kv[k]) for k in
                                           ("scanned", "queued", "created", "removed", "entities", "meshes"))
        except (KeyError, ValueError):
            continue
        if t0 is None:
            t0 = t
        out.append((round(t - t0, 3),) + row)
    return out


def sync_sequences_equal(a, b):
    """Same length and fields; the relative t may differ by 1 ms (a rounding boundary must not fail the gate)."""
    if len(a) != len(b):
        return False
    return all(x[1:] == y[1:] and abs(x[0] - y[0]) <= TIME_TOL_S + 1e-9 for x, y in zip(a, b))


def _read_lines(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read().splitlines()
    except OSError:
        return None


# ---------------------------------------------------------------- diff of two runs
_DIFF_KEYS = ("frames_base", "frames_cand", "matched", "identical", "near_identical", "differing", "time_mismatch",
              "points_total_base", "max_abs_count_diff", "max_rel_count_diff", "points_added", "points_removed",
              "added_by_range", "removed_by_range")


def compare_runs(base_dir, cand_dir):
    base, cand = load_run(base_dir), load_run(cand_dir)
    res = dict(frames_base=sum(len(s) for s in base.values()), frames_cand=sum(len(s) for s in cand.values()),
               matched=0, identical=0, near_identical=0, differing=0, time_mismatch=0,
               points_total_base=sum(len(d.pts) for s in base.values() for d in s),
               max_abs_count_diff=0, max_rel_count_diff=0.0, points_added=0, points_removed=0)
    add_r, rem_r = [0, 0, 0, 0], [0, 0, 0, 0]
    for bi, ci in match_sensors(base, cand):
        bs, cs = base[bi], cand[ci]
        for k in range(min(len(bs), len(cs))):
            b, c = bs[k], cs[k]
            res["matched"] += 1
            if abs((b.sim_t - bs[0].sim_t) - (c.sim_t - cs[0].sim_t)) > TIME_TOL_S + 1e-9:
                res["time_mismatch"] += 1
            nb, nc = len(b.pts), len(c.pts)
            res["max_abs_count_diff"] = max(res["max_abs_count_diff"], abs(nb - nc))
            res["max_rel_count_diff"] = max(res["max_rel_count_diff"], abs(nb - nc) / max(nb, 1))
            if _identical(b.pts, c.pts):
                res["identical"] += 1
                continue
            sd = diff_sets(b.pts, c.pts, b.pose[:3])
            res["points_added"] += sd.added
            res["points_removed"] += sd.removed
            add_r = [x + y for x, y in zip(add_r, sd.added_by_range)]
            rem_r = [x + y for x, y in zip(rem_r, sd.removed_by_range)]
            if sd.added == 0 and sd.removed == 0 and nb == nc:
                res["near_identical"] += 1
            else:
                res["differing"] += 1
    res["added_by_range"] = "/".join(str(x) for x in add_r)
    res["removed_by_range"] = "/".join(str(x) for x in rem_r)
    return res


def format_diff_kv(res):
    parts = []
    for k in _DIFF_KEYS:
        v = res[k]
        parts.append("%s=%s" % (k, ("%.4f" % v) if k == "max_rel_count_diff" else v))
    return " ".join(parts)


def hits_diff_kv(base_dir, cand_dir):
    """The key=value pairs of the hits_diff line, as a dict of strings."""
    return dict(tok.split("=", 1) for tok in format_diff_kv(compare_runs(base_dir, cand_dir)).split())


# ---------------------------------------------------------------- gate
def _parse_kv(text):
    out = {}
    for tok in text.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def gate_verdict(res, min_frames, cand_run_kv, base_sync, cand_sync):
    """-> list of failure reasons (empty = pass)."""
    reasons = []
    if cand_run_kv is None or cand_run_kv.get("status") != "ok":
        reasons.append("status_not_ok")
    if cand_run_kv is not None and cand_run_kv.get("ego_bp_fallback", "0") not in ("0", ""):
        reasons.append("ego_bp_fallback")
    if res["matched"] == 0:
        reasons.append("no_match")
    if not (res["matched"] == res["frames_base"] == res["frames_cand"]):
        reasons.append("frames_mismatch")
    if res["frames_base"] < min_frames:
        reasons.append("too_few_frames")
    if res["points_total_base"] <= 0:
        reasons.append("no_points")
    if res["differing"] != 0:
        reasons.append("differing")
    if res["time_mismatch"] != 0:
        reasons.append("time_mismatch")
    # an empty sequence on either side proves nothing (no sync line = the telemetry did not run)
    if not base_sync or not cand_sync or not sync_sequences_equal(base_sync, cand_sync):
        reasons.append("sync_sequence")
    return reasons


# ---------------------------------------------------------------- cli
def cmd_fingerprint(a):
    run = load_run(a.dump_dir)
    rows = []
    for si in sorted(run):
        dumps = run[si]
        for seq, d in enumerate(dumps):
            mm = np.round(d.pts * 1000.0).astype(np.int32)
            if len(mm):
                mm = mm[np.lexsort((mm[:, 2], mm[:, 1], mm[:, 0]))]
            rows.append("%d,%d,%.3f,%d,%s" % (si, seq, d.sim_t - dumps[0].sim_t, len(d.pts),
                                              hashlib.sha1(np.ascontiguousarray(mm).tobytes()).hexdigest()))
    with open(a.out, "w", encoding="utf-8") as f:
        f.write("".join(r + "\n" for r in rows))
    print("fingerprint sensors=%d dumps=%d out=%s" % (len(run), len(rows), a.out))
    return 0


def cmd_diff(a):
    print("hits_diff " + format_diff_kv(compare_runs(a.base, a.cand)))
    return 0


def cmd_gate(a):
    res = compare_runs(a.base, a.cand)
    run_text = _read_lines(a.cand_run)
    run_kv = _parse_kv(" ".join(run_text)) if run_text is not None else None
    bl, cl = _read_lines(a.base_stderr), _read_lines(a.cand_stderr)
    reasons = gate_verdict(res, a.min_frames, run_kv, sync_sequence(bl or []), sync_sequence(cl or []))
    print("gate verdict=%s reasons=%s %s" % ("fail" if reasons else "pass", ",".join(reasons) or "none",
                                            format_diff_kv(res)))
    return 1 if reasons else 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("fingerprint")
    p.add_argument("dump_dir")
    p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_fingerprint)
    p = sub.add_parser("diff")
    p.add_argument("base")
    p.add_argument("cand")
    p.set_defaults(fn=cmd_diff)
    p = sub.add_parser("gate")
    p.add_argument("--base", required=True)
    p.add_argument("--cand", required=True)
    p.add_argument("--min-frames", type=int, required=True)
    p.add_argument("--cand-run", required=True)
    p.add_argument("--base-stderr", required=True)
    p.add_argument("--cand-stderr", required=True)
    p.set_defaults(fn=cmd_gate)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
