#!/usr/bin/env python3
"""Summarize `rgl_scene_audit` / `rgl_scene_sync` lines of one scenario run.

Usage: summarize.py <out_dir>...

Each <out_dir> is a run_scenario.py output directory holding server.stderr,
run.txt (key=value: scenario=, map=, radius_expected_m=, no_audit=) and optionally
tick_ms.txt (one client world.tick() wall time [ms] per line). A no_audit=1 run
has no audit line: its radius check is na (not a mismatch).
Prints one key=value line per directory. Does not import carla.
"""
import math
import os
import sys

AUDIT_TAG = "rgl_scene_audit "
SYNC_TAG = "rgl_scene_sync "
STREAM_WARNING_TAG = "rgl_scene_stream_warning"
SYNC_TRIGGERS = ("init", "time", "distance", "level")

# The first definition of "steady", for missing_sync_steady_max (kept unchanged): lines
# where no level was added since the previous line, nothing is pending creation and
# the streaming is reported complete (stream_covered != 0).
#
# The later definition, for missing_steady_max / steady_rows / steady_valid, decided
# line by line in order: t_add = sim_t of the last line so far with levels_added > 0;
# a line is steady when pending == 0 and pending_ism == 0 and (t_add is None or
# sim_t - since_sync_s >= t_add - 0.01), i.e. a sync ran after the last level came in.
# stream_covered is not used: the audit counts only components of loaded levels, and a
# map that never reports full coverage (S3) would otherwise have no steady line at all.
# steady_valid = steady_rows >= 0.2 * frames. Keys missing from a line count as 0.
# The World Partition culling adds its queue: a line with wp_pending > 0 (jobs of the World Partition culling of non-Static parts
# not done yet) is not steady either.


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


def steady_lines(audits):
    """The audit lines that are steady by the later definition (see the top of this file)."""
    t_add = None
    out = []
    for a in audits:
        sim_t = _num(a, "sim_t")
        if _num(a, "levels_added") > 0:
            t_add = sim_t
        if _num(a, "pending") != 0 or _num(a, "pending_ism") != 0 or _num(a, "wp_pending") != 0:
            continue
        if t_add is None or sim_t - _num(a, "since_sync_s") >= t_add - 0.01:
            out.append(a)
    return out


def _present(rows, key):
    """The numeric values of key in rows, or None when no row has the key (an older build: na, not 0)."""
    if not any(key in r for r in rows):
        return None
    return [_num(r, key) for r in rows if key in r]


def longest_run(flags):
    best = run = 0
    for f in flags:
        run = run + 1 if f else 0
        best = max(best, run)
    return best


def summarize_lines(lines, scenario, map_name, radius_expected_m, tick_ms=None, audit=True):
    """audit=False: a --no-audit run; there is no audit line to check the radius with, so radius_mismatch is None (na)."""
    audits, syncs = parse_lines(lines)
    g = lambda key: [_num(a, key) for a in audits]  # noqa: E731

    radii = g("parent_stream_radius_m")
    # with the audit on, no audit line at all is itself invalid
    mismatch = _radius_mismatch(radii, radius_expected_m) if audit else None
    steady = [_num(a, "missing_sync") for a in audits
              if _num(a, "levels_added") == 0 and _num(a, "pending") == 0
              and _num(a, "stream_covered", -1) != 0]
    frame_ms = g("frame_ms")
    sync_ms = [_num(s, "elapsed_ms") for s in syncs]
    if radii:
        shown_radius = max(set(radii), key=radii.count)  # most common value
    else:
        shown_radius = -1.0

    st = steady_lines(audits)
    no_pending = [a for a in audits if _num(a, "pending") == 0 and _num(a, "pending_ism") == 0]
    missing_no_pending = [_num(a, "missing_in_range") for a in no_pending if _num(a, "missing_in_range") > 0]
    triggers = [s.get("trigger") for s in syncs]
    warnings = sum(1 for ln in lines if STREAM_WARNING_TAG in ln)
    wp_culled = _present(audits, "missing_wp_culled_in_range") if audit and audits else None
    wp_age = _present(syncs, "wp_queue_age_max_s")
    wp_viol = _present(syncs, "wp_violations")
    wait_ms = _present(syncs, "wait_ms")
    wp_total = _present(syncs, "wp_entities_total")
    wp_nonstatic = _present(syncs, "wp_nonstatic_registered")

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
        "sync_elapsed_ms_p50": percentile(sync_ms, 50),
        "sync_elapsed_ms_p95": percentile(sync_ms, 95),
        # tick tail (client world.tick() wall time); na without a tick log
        "tick_over_100ms": sum(1 for v in tick_ms if v > 100.0) if tick_ms else None,
        "tick_ms_p99": percentile(list(tick_ms), 99) if tick_ms else None,
        "tick_ms_max": max(tick_ms) if tick_ms else None,
        # the later steady state, pending split, breakdowns (keys missing from a line count as 0)
        "missing_steady_max": int(max((_num(a, "missing_steady") for a in st), default=0)),
        "steady_rows": len(st),
        "steady_valid": int(len(st) >= 0.2 * len(audits)),
        "missing_rows_no_pending": len(missing_no_pending),
        "missing_max_no_pending": int(max(missing_no_pending, default=0)),
        "missing_ism_p95_no_pending_ism": int(percentile(
            [_num(a, "missing_ism_in_range") for a in audits if _num(a, "pending_ism") == 0], 95)),
        "ghost_ism_max": int(max(g("ghost_ism"), default=0)),
        "ghost_nonism_max": int(max((_num(a, "ghost") - _num(a, "ghost_ism") for a in audits), default=0)),
        "cache_stale_max": int(max(g("cache_stale"), default=0)),
        "pending_ism_max": int(max(g("pending_ism"), default=0)),
        "pending_ism_run_max": longest_run(v > 0 for v in g("pending_ism")),
        "spline_registered_max": int(max(g("spline_registered"), default=0)),
        "sync_triggers": ",".join("%s:%d" % (t, triggers.count(t)) for t in SYNC_TRIGGERS),
        "stream_warnings": warnings,
        # World Partition culling of non-Static parts. Audit keys: na without audit lines or without the key.
        # missing_wp_culled_in_range is counted on every audit line, pending or not (a known part in range).
        "missing_wp_culled_in_range_frames": (None if wp_culled is None else sum(1 for v in wp_culled if v > 0)),
        "missing_wp_culled_in_range_max": (None if wp_culled is None else int(max(wp_culled, default=0))),
        # sync keys: queue age = the largest current value seen; violations = the sum of the per-line counts;
        # deadline / reset syncs = lines with that trigger; wait_ms p95; scene totals p50 (current values per line)
        "wp_queue_age_max_s": (None if wp_age is None else max(wp_age, default=0.0)),
        "wp_violations_total": (None if wp_viol is None else int(sum(wp_viol))),
        "wp_deadline_syncs": triggers.count("wp_deadline"),
        "wp_reset_syncs": triggers.count("wp_reset"),
        "wait_ms_p95": (None if wait_ms is None else percentile(wait_ms, 95)),
        "wp_entities_total_p50": (None if wp_total is None else int(percentile(wp_total, 50))),
        "wp_nonstatic_registered_p50": (None if wp_nonstatic is None else int(percentile(wp_nonstatic, 50))),
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
    ("sync_elapsed_ms_p50", "%.2f"), ("sync_elapsed_ms_p95", "%.2f"), ("tick_over_100ms", "%d"),
    ("tick_ms_p99", "%.2f"), ("tick_ms_max", "%.2f"),
    ("missing_steady_max", "%d"), ("steady_rows", "%d"), ("steady_valid", "%d"),
    ("missing_rows_no_pending", "%d"), ("missing_max_no_pending", "%d"), ("missing_ism_p95_no_pending_ism", "%d"),
    ("ghost_ism_max", "%d"), ("ghost_nonism_max", "%d"), ("cache_stale_max", "%d"),
    ("pending_ism_max", "%d"), ("pending_ism_run_max", "%d"), ("spline_registered_max", "%d"),
    ("sync_triggers", "%s"), ("stream_warnings", "%d"),
    ("missing_wp_culled_in_range_frames", "%d"), ("missing_wp_culled_in_range_max", "%d"),
    ("wp_queue_age_max_s", "%.2f"), ("wp_violations_total", "%d"), ("wp_deadline_syncs", "%d"),
    ("wp_reset_syncs", "%d"), ("wait_ms_p95", "%.2f"), ("wp_entities_total_p50", "%d"),
    ("wp_nonstatic_registered_p50", "%d"),
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
                        float(run.get("radius_expected_m", "-1")), tick,
                        audit=run.get("no_audit", "0") != "1")
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
