#!/usr/bin/env python3
"""Summarize `rgl_scene_audit` / `rgl_scene_sync` lines of one scenario run.

Usage: summarize.py <out_dir>...

Each <out_dir> is a run_scenario.py output directory holding server.stderr,
run.txt (key=value: scenario=, map=, radius_expected_m=) and optionally
tick_ms.txt (one client world.tick() wall time [ms] per line).
Prints one key=value line per directory. Does not import carla.
"""
import math
import os
import sys

AUDIT_TAG = "rgl_scene_audit "
SYNC_TAG = "rgl_scene_sync "

# Definition of "steady" for missing_sync_steady_max: lines where no level was
# added since the previous line, nothing is pending creation and the streaming
# is reported complete (stream_covered != 0). Right after a sync the live
# registration should match the world; a non-zero value there is not explained
# by in-flight streaming.


def parse_kv(text):
    out = {}
    for tok in text.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def _num(d, key, default=0.0):
    try:
        return float(d[key])
    except (KeyError, ValueError):
        return default


def percentile(values, q):
    """Nearest-rank percentile (q in 0..100). Empty -> 0."""
    if not values:
        return 0
    s = sorted(values)
    rank = max(1, int(math.ceil(q / 100.0 * len(s))))
    return s[rank - 1]


def parse_lines(lines):
    audits, syncs = [], []
    for ln in lines:
        i = ln.find(AUDIT_TAG)
        if i >= 0:
            audits.append(parse_kv(ln[i + len(AUDIT_TAG):]))
            continue
        i = ln.find(SYNC_TAG)
        if i >= 0:
            syncs.append(parse_kv(ln[i + len(SYNC_TAG):]))
    return audits, syncs


def _radius_mismatch(radii, expected):
    """1 when the streaming radius seen in the audit lines is not the one the scenario asked for.

    expected is NaN: not compared (mixed maps). expected < 0: no streaming source is expected, so every
    line must say -1. expected > 0: leading -1 lines are skipped (the source is registered a few frames
    after the sensor starts ticking); every later line must be within 1 m, and at least one must exist.
    No audit line at all is itself invalid.
    """
    if not radii:
        return 1
    if math.isnan(expected):
        return 0
    if expected < 0:
        return int(any(r != -1.0 for r in radii))
    started = [r for i, r in enumerate(radii) if any(x != -1.0 for x in radii[: i + 1])]
    if not started:
        return 1
    return int(any(abs(r - expected) >= 1.0 for r in started))


def summarize_lines(lines, scenario, map_name, radius_expected_m, tick_ms=None):
    audits, syncs = parse_lines(lines)
    g = lambda key: [_num(a, key) for a in audits]  # noqa: E731

    radii = g("parent_stream_radius_m")
    mismatch = _radius_mismatch(radii, radius_expected_m)
    # no audit line at all is itself invalid
    steady = [_num(a, "missing_sync") for a in audits
              if _num(a, "levels_added") == 0 and _num(a, "pending") == 0
              and _num(a, "stream_covered", -1) != 0]
    frame_ms = g("frame_ms")
    sync_ms = [_num(s, "elapsed_ms") for s in syncs]
    if radii:
        shown_radius = max(set(radii), key=radii.count)  # most common value
    else:
        shown_radius = -1.0

    return {
        "scenario": scenario,
        "map": map_name,
        "frames": len(audits),
        "ghost_max": int(max(g("ghost"), default=0)),
        "ghost_in_range_frames": sum(1 for v in g("ghost_in_range") if v > 0),
        "missing_in_range_frames": sum(1 for v in g("missing_in_range") if v > 0),
        "missing_in_range_p95": int(percentile(g("missing_in_range"), 95)),
        "missing_sync_steady_max": int(max(steady, default=0)),
        "mesh_mismatch_max": int(max(g("mesh_mismatch"), default=0)),
        "stale_instance_max": int(max(g("stale_instance"), default=0)),
        "hlod_registered_max": int(max(g("hlod_registered"), default=0)),
        "stream_covered0_frames": sum(1 for a in audits if _num(a, "stream_covered", -1) == 0),
        "parent_stream_radius_m": shown_radius,
        "radius_expected_m": float(radius_expected_m),
        "radius_mismatch": mismatch,
        "levels_added_total": int(sum(g("levels_added"))),
        "audit_ms_p95": percentile(g("audit_ms"), 95),
        "frame_ms_max": max(frame_ms, default=0.0),
        "frames_over_100ms": sum(1 for v in frame_ms if v > 100.0),
        "sync_elapsed_ms_max": max(sync_ms, default=0.0),
        "tick_ms_p95": percentile(list(tick_ms), 95) if tick_ms else None,
        # ISM-instance part of the misses (older lines without the key count as 0)
        "missing_ism_in_range_p95": int(percentile(g("missing_ism_in_range"), 95)),
        "missing_ism_sync_max": int(max(g("missing_ism_sync"), default=0)),
    }


_ORDER = [
    ("scenario", "%s"), ("map", "%s"), ("frames", "%d"), ("ghost_max", "%d"),
    ("ghost_in_range_frames", "%d"), ("missing_in_range_frames", "%d"),
    ("missing_in_range_p95", "%d"), ("missing_sync_steady_max", "%d"),
    ("mesh_mismatch_max", "%d"), ("stale_instance_max", "%d"),
    ("hlod_registered_max", "%d"), ("stream_covered0_frames", "%d"),
    ("parent_stream_radius_m", "%.1f"), ("radius_expected_m", "%.1f"),
    ("radius_mismatch", "%d"), ("levels_added_total", "%d"),
    ("audit_ms_p95", "%.3f"), ("frame_ms_max", "%.2f"),
    ("frames_over_100ms", "%d"), ("sync_elapsed_ms_max", "%.2f"),
    ("tick_ms_p95", "%.2f"), ("missing_ism_in_range_p95", "%d"), ("missing_ism_sync_max", "%d"),
]


def format_summary(r):
    parts = []
    for key, fmt in _ORDER:
        v = r[key]
        parts.append("%s=%s" % (key, "na" if v is None else fmt % v))
    return " ".join(parts)


def read_run_txt(path):
    try:
        with open(path, encoding="utf-8") as f:
            return parse_kv(f.read())
    except OSError:
        return {}


def summarize_dir(d):
    run = read_run_txt(os.path.join(d, "run.txt"))
    with open(os.path.join(d, "server.stderr"), encoding="utf-8", errors="replace") as f:
        lines = f.read().splitlines()
    tick = None
    tp = os.path.join(d, "tick_ms.txt")
    if os.path.exists(tp):
        with open(tp, encoding="utf-8") as f:
            tick = [float(x) for x in f.read().split()]
    r = summarize_lines(lines, run.get("scenario", "?"), run.get("map", "?"),
                        float(run.get("radius_expected_m", "-1")), tick)
    return r


def main(argv):
    if not argv:
        print("usage: summarize.py <out_dir>...", file=sys.stderr)
        return 2
    rc = 0
    for d in argv:
        try:
            r = summarize_dir(d)
        except OSError as e:
            print("summary status=invalid reason=unreadable dir=%s err=%s" % (d.replace(" ", "_"), e.__class__.__name__))
            rc = 1
            continue
        line = format_summary(r)
        if r["radius_mismatch"]:
            line += " status=invalid reason=radius_mismatch"
        print(line)
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
