"""Tests for summarize.py (no CARLA server and no carla module needed)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import summarize  # noqa: E402


def _audit(frame, missing_in_range=0, radius=200.0, **kw):
    f = dict(
        ghost=0, ghost_in_range=0, missing_sync=0, missing_in_range=missing_in_range,
        mesh_mismatch=0, stale_instance=0, excluded_by_name=0, unextractable=0,
        non_sm_collision=0, hlod_registered=0, registered=10, pending=0,
        movable_registered=0, levels_added=0, since_sync_s=1.00, stream_covered=1,
        parent_stream_radius_m=radius, frame_ms=50.0, audit_ms=0.5,
    )
    f.update(kw)
    body = " ".join("%s=%s" % (k, ("%.1f" % v) if k == "parent_stream_radius_m" else v)
                    for k, v in f.items())
    return "LogCarlaRGL: rgl_scene_audit frame=%d sim_t=%.3f %s" % (frame, frame * 0.05, body)


def _lines(radius):
    out = ["LogTemp: unrelated line",
           "rgl_scene_sync entities=10 meshes=3 elapsed_ms=12.5"]
    for i in range(6):
        out.append(_audit(i + 1, missing_in_range=3 if i == 2 else 0, radius=radius))
    return out


def test_radius_mismatch_when_expectation_differs():
    r = summarize.summarize_lines(_lines(200.0), "S1p", "Town15", 150.0)
    assert r["frames"] == 6
    assert r["missing_in_range_frames"] == 1
    assert r["radius_mismatch"] == 1
    assert "missing_in_range_frames=1" in summarize.format_summary(r)
    assert "radius_mismatch=1" in summarize.format_summary(r)


def test_radius_match():
    r = summarize.summarize_lines(_lines(200.0), "S2", "Town15", 200.0)
    assert r["missing_in_range_frames"] == 1
    assert r["radius_mismatch"] == 0
    assert r["sync_elapsed_ms_max"] == 12.5


def test_non_wp_minus_one():
    r = summarize.summarize_lines(_lines(-1.0), "CONTROL", "Odaiba", -1.0)
    assert r["radius_mismatch"] == 0
    assert "parent_stream_radius_m=-1.0" in summarize.format_summary(r)


def test_leading_minus_one_before_the_source_registers():
    # the hero's streaming source appears a few frames after the sensor starts ticking
    lines = [_audit(1, radius=-1.0), _audit(2, radius=-1.0), _audit(3, radius=150.0), _audit(4, radius=150.0)]
    assert summarize.summarize_lines(lines, "S1p", "Town15", 150.0)["radius_mismatch"] == 0
    # a -1 after the source registered is a real mismatch
    lines.append(_audit(5, radius=-1.0))
    assert summarize.summarize_lines(lines, "S1p", "Town15", 150.0)["radius_mismatch"] == 1
    # never registered at all
    only = [_audit(1, radius=-1.0), _audit(2, radius=-1.0)]
    assert summarize.summarize_lines(only, "S1p", "Town15", 150.0)["radius_mismatch"] == 1
    # mixed maps are not compared
    assert summarize.summarize_lines(only, "S7", "Town15", float("nan"))["radius_mismatch"] == 0


def test_statistics():
    lines = [_audit(i + 1, missing_in_range=i, ghost=2 if i == 3 else 0,
                    frame_ms=150.0 if i == 4 else 50.0) for i in range(20)]
    r = summarize.summarize_lines(lines, "S1", "Town15", 200.0, tick_ms=[10.0] * 19 + [90.0])
    assert r["ghost_max"] == 2
    assert r["missing_in_range_p95"] == 18  # nearest rank: ceil(0.95*20)=19th of 0..19
    assert r["frames_over_100ms"] == 1
    assert r["frame_ms_max"] == 150.0
    assert r["tick_ms_p95"] == 10.0


def test_line_order_and_keys():
    r = summarize.summarize_lines(_lines(200.0), "S1p", "Town15", 200.0)
    keys = [kv.split("=")[0] for kv in summarize.format_summary(r).split()]
    assert keys == [
        "scenario", "map", "frames", "ghost_max", "ghost_in_range_frames",
        "missing_in_range_frames", "missing_in_range_p95", "missing_sync_steady_max",
        "mesh_mismatch_max", "stale_instance_max", "hlod_registered_max",
        "stream_covered0_frames", "parent_stream_radius_m", "radius_expected_m",
        "radius_mismatch", "levels_added_total", "audit_ms_p95", "frame_ms_max",
        "frames_over_100ms", "sync_elapsed_ms_max", "tick_ms_p95"]
