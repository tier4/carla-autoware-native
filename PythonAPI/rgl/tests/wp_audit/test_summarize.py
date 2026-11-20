"""Tests for summarize.py (no CARLA server and no carla module needed)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import summarize  # noqa: E402


def _audit(frame, missing_in_range=0, radius=200.0, sim_t=None, **kw):
    """A stage-0 style audit line (no ghost_ism / missing_steady / pending_ism ... unless given in kw)."""
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
    t = frame * 0.05 if sim_t is None else sim_t
    return "LogCarlaRGL: rgl_scene_audit frame=%d sim_t=%.3f %s" % (frame, t, body)


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
        "frames_over_100ms", "sync_elapsed_ms_max", "tick_ms_p95",
        "missing_ism_in_range_p95", "missing_ism_sync_max",
        "sync_elapsed_ms_p50", "sync_elapsed_ms_p95", "tick_over_100ms", "tick_ms_p99", "tick_ms_max",
        "missing_steady_max", "steady_rows", "steady_valid", "missing_rows_no_pending", "missing_max_no_pending",
        "missing_ism_p95_no_pending_ism", "ghost_ism_max", "ghost_nonism_max", "cache_stale_max",
        "pending_ism_max", "pending_ism_run_max", "spline_registered_max", "sync_triggers", "stream_warnings",
        "missing_wp_culled_in_range_frames", "missing_wp_culled_in_range_max", "wp_queue_age_max_s",
        "wp_violations_total", "wp_deadline_syncs", "wp_reset_syncs", "wait_ms_p95", "wp_entities_total_p50",
        "wp_nonstatic_registered_p50"]


def _sync(elapsed_ms, trigger="time"):
    return ("LogCarlaRGL: rgl_scene_sync t=1.000 trigger=%s scanned=5 elapsed_ms=%.2f entities=10 meshes=3"
            % (trigger, elapsed_ms))


def test_sync_percentiles_and_tick_over_100ms():
    lines = [_sync(10.0 * i) for i in range(1, 11)]           # elapsed_ms 10..100 step 10
    r = summarize.summarize_lines(lines, "S1", "Town15", 200.0, tick_ms=[99.0, 101.0])
    assert r["sync_elapsed_ms_p50"] == 50.0 and r["sync_elapsed_ms_p95"] == 100.0 and r["tick_over_100ms"] == 1
    assert r["tick_ms_max"] == 101.0 and r["tick_ms_p99"] == 101.0


def test_tick_tail_keys_are_na_without_tick_log_and_exact_at_100ms():
    r = summarize.summarize_lines([_sync(5.0)], "S1", "Town15", 200.0)
    assert r["tick_over_100ms"] is None and r["tick_ms_p99"] is None and r["tick_ms_max"] is None
    text = summarize.format_summary(r)
    assert "tick_over_100ms=na" in text and "tick_ms_p99=na" in text and "tick_ms_max=na" in text
    exact = summarize.summarize_lines([_sync(5.0)], "S1", "Town15", 200.0, tick_ms=[100.0, 100.0])
    assert exact["tick_over_100ms"] == 0              # strictly greater than 100 ms
    tail = [10.0] * 99 + [250.0]
    r2 = summarize.summarize_lines([_sync(5.0)], "S1", "Town15", 200.0, tick_ms=tail)
    assert r2["tick_ms_p99"] == 10.0 and r2["tick_ms_max"] == 250.0   # nearest rank: the 99th of 100 is still 10


def test_ism_breakdown_and_old_lines():
    lines = [_audit(1, missing_in_range=5, missing_ism_in_range=4, missing_ism_sync=3),
             _audit(2, missing_in_range=1)]          # an older line without the ISM keys counts as 0
    # _audit() writes every key it is given, so drop the ISM keys from the second line
    lines[1] = lines[1].replace(" missing_ism_in_range=0", "").replace(" missing_ism_sync=0", "")
    r = summarize.summarize_lines(lines, "S1p", "Town15", 200.0)
    assert r["missing_ism_in_range_p95"] == 4
    assert r["missing_ism_sync_max"] == 3


# ---- stage 1 (Task 0C): steady state (D13), pending split, ISM / spline / cache breakdown, sync triggers

def test_steady_requires_sync_after_level_add():
    # a level comes in at 1.0 s; the sync that saw it is the one 0.0 s before 1.2 s (the 1.1 s line's sync is older)
    lines = [_audit(1, sim_t=1.0, levels_added=1, since_sync_s=0.5, missing_steady=5),
             _audit(2, sim_t=1.1, levels_added=0, since_sync_s=0.6, missing_steady=5),
             _audit(3, sim_t=1.2, levels_added=0, since_sync_s=0.0, missing_steady=0)]
    r = summarize.summarize_lines(lines, "S1", "Town15", 200.0)
    assert r["missing_steady_max"] == 0 and r["steady_rows"] == 1


def test_steady_ignores_stream_covered():
    lines = [_audit(i + 1, stream_covered=0, pending=0, missing_steady=2) for i in range(5)]
    r = summarize.summarize_lines(lines, "S3", "Town15", 200.0)
    assert r["steady_rows"] == r["frames"] and r["steady_valid"] == 1
    assert r["missing_steady_max"] == 2


def test_steady_invalid_when_too_few():
    lines = [_audit(i + 1, pending=0 if i == 0 else 3) for i in range(10)]
    r = summarize.summarize_lines(lines, "S1", "Town15", 200.0)
    assert r["steady_rows"] == 1
    assert r["steady_valid"] == 0          # 1 < 0.2 * 10


def test_steady_excludes_pending_ism_rows():
    lines = [_audit(1, pending_ism=4, missing_steady=9), _audit(2, missing_steady=1)]
    r = summarize.summarize_lines(lines, "S1", "Town15", 200.0)
    assert r["steady_rows"] == 1 and r["missing_steady_max"] == 1


def test_missing_rows_split_by_pending():
    lines = [_audit(1, missing_in_range=3, pending=2), _audit(2, missing_in_range=4, pending=0)]
    r = summarize.summarize_lines(lines, "S1", "Town15", 200.0)
    assert r["missing_rows_no_pending"] == 1 and r["missing_max_no_pending"] == 4
    # a pending ISM instance also excuses the row
    r2 = summarize.summarize_lines([_audit(1, missing_in_range=4, pending=0, pending_ism=1)], "S1", "Town15", 200.0)
    assert r2["missing_rows_no_pending"] == 0 and r2["missing_max_no_pending"] == 0


def test_pending_ism_run_max():
    lines = [_audit(i + 1, pending_ism=v) for i, v in enumerate([0, 1, 1, 1, 0, 1])]
    r = summarize.summarize_lines(lines, "S4", "Town15", 200.0)
    assert r["pending_ism_run_max"] == 3
    assert r["pending_ism_max"] == 1


def test_sync_trigger_counts():
    lines = [_sync(5.0, "init"), _sync(5.0, "time"), _sync(5.0, "time"), _sync(5.0, "level")]
    r = summarize.summarize_lines(lines, "S1", "Town15", 200.0)
    assert r["sync_triggers"] == "init:1,time:2,distance:0,level:1"
    assert "sync_triggers=init:1,time:2,distance:0,level:1" in summarize.format_summary(r)


def test_old_lines_without_new_keys_count_zero():
    # stage-0 lines: none of the stage-1 keys; a missing key counts as 0
    lines = [_audit(1, ghost=2, missing_in_range=3, missing_sync=3), _audit(2)]
    r = summarize.summarize_lines(lines, "S1p", "Town15", 200.0)
    assert r["ghost_ism_max"] == 0 and r["missing_steady_max"] == 0 and r["pending_ism_max"] == 0
    assert r["ghost_nonism_max"] == 2 and r["cache_stale_max"] == 0 and r["spline_registered_max"] == 0
    assert r["sync_triggers"] == "init:0,time:0,distance:0,level:0" and r["stream_warnings"] == 0


def test_breakdown_maxima_and_stream_warnings():
    lines = [_audit(1, ghost=5, ghost_ism=4, cache_stale=2, spline_registered=7, missing_ism_in_range=6, pending_ism=1),
             _audit(2, ghost=3, ghost_ism=0, missing_ism_in_range=2, pending_ism=0),
             "LogCarlaRGL: rgl_scene_stream_warning t=12.000 sensor=1 uncovered_s=5.0",
             _audit(3, ghost=0, ghost_ism=0, missing_ism_in_range=1, pending_ism=0)]
    r = summarize.summarize_lines(lines, "S2", "Town15", 200.0)
    assert r["ghost_ism_max"] == 4
    assert r["ghost_nonism_max"] == 3            # max of ghost - ghost_ism per line: 1, 3, 0
    assert r["cache_stale_max"] == 2 and r["spline_registered_max"] == 7
    assert r["missing_ism_p95_no_pending_ism"] == 2   # rows with pending_ism=0: 2 and 1
    assert r["stream_warnings"] == 1
    assert r["frames"] == 3                      # the warning line is not an audit row


def _write_run(tmp_path, no_audit, lines):
    d = tmp_path / ("run-noaudit" if no_audit else "run-audit")
    d.mkdir()
    (d / "run.txt").write_text("status=ok scenario=S1 map=Town15 seed=0 port=4000 no_audit=%d "
                               "radius_expected_m=3000.0\n" % no_audit)
    (d / "server.stderr").write_text("\n".join(lines) + "\n")
    return str(d)


def test_no_audit_run_reports_radius_na(tmp_path, capsys):
    # 0B-R2: a --no-audit run has no audit line, so there is nothing to check the radius against
    d = _write_run(tmp_path, 1, [_sync(5.0, "init"), _sync(7.0)])
    r = summarize.summarize_dir(d)
    assert r["radius_mismatch"] is None
    assert summarize.main([d]) == 0
    out = capsys.readouterr().out
    assert "radius_mismatch=na" in out and "status=invalid" not in out
    assert "sync_elapsed_ms_p95=7.00" in out


def test_audit_run_without_audit_lines_stays_invalid(tmp_path, capsys):
    d = _write_run(tmp_path, 0, [_sync(5.0, "init")])
    assert summarize.summarize_dir(d)["radius_mismatch"] == 1
    summarize.main([d])
    assert "status=invalid reason=radius_mismatch" in capsys.readouterr().out


# ---- 7b: the World Partition culling of non-Static parts ----
def test_missing_wp_culled_counted_while_pending():
    # a known culled part in range is a miss even while jobs are pending: never dropped by the pending filters
    lines = [_audit(1, pending=2, pending_ism=1, missing_in_range=1, missing_wp_culled_in_range=1),
             _audit(2, missing_wp_culled_in_range=0)]
    r = summarize.summarize_lines(lines, "S1", "Town15", 200.0)
    assert r["missing_wp_culled_in_range_frames"] == 1
    assert r["missing_wp_culled_in_range_max"] == 1
    out = summarize.format_summary(r)
    assert "missing_wp_culled_in_range_frames=1" in out and "missing_wp_culled_in_range_max=1" in out


def test_missing_wp_culled_absent_key_is_na():
    # audit lines from a build without the key: na, not 0
    r = summarize.summarize_lines([_audit(1), _audit(2)], "S1", "Town15", 200.0)
    out = summarize.format_summary(r)
    assert "missing_wp_culled_in_range_frames=na" in out and "missing_wp_culled_in_range_max=na" in out


def test_missing_wp_culled_without_audit_frames_is_na():
    # audit on but no audit line: no all-zero success (and the run stays invalid through radius_mismatch)
    r = summarize.summarize_lines([_sync(5.0, "init")], "S1", "Town15", 200.0)
    assert r["frames"] == 0 and r["radius_mismatch"] == 1
    assert r["missing_wp_culled_in_range_frames"] is None and r["missing_wp_culled_in_range_max"] is None


def _wp_sync(elapsed_ms, trigger="time", age=0.4, viol=0, wait=60.0, total=1000, nonstatic=200):
    return ("LogCarlaRGL: rgl_scene_sync t=1.000 trigger=%s scanned=5 elapsed_ms=%.2f entities=10 meshes=3 "
            "skel_ms=0.00 wait_ms=%.2f wait_api=rgl_entity_create wp=1 wp_queue_age_max_s=%.2f wp_violations=%d "
            "wp_entities_total=%d wp_nonstatic_registered=%d" % (trigger, elapsed_ms, wait, age, viol, total, nonstatic))


def test_wp_sync_keys():
    lines = [_wp_sync(5.0, "init", age=0.0, viol=0, wait=1.0, total=900, nonstatic=100),
             _wp_sync(6.0, "time", age=0.7, viol=2, wait=60.0, total=1000, nonstatic=200),
             _wp_sync(7.0, "wp_deadline", age=0.2, viol=1, wait=70.0, total=1100, nonstatic=300),
             _wp_sync(8.0, "wp_reset", age=0.1, viol=0, wait=65.0, total=1200, nonstatic=400)]
    r = summarize.summarize_lines(lines, "S1", "Town15", 200.0, audit=False)
    assert r["wp_queue_age_max_s"] == 0.7
    assert r["wp_violations_total"] == 3
    assert r["wp_deadline_syncs"] == 1 and r["wp_reset_syncs"] == 1
    assert r["wait_ms_p95"] == 70.0
    assert r["wp_entities_total_p50"] == 1000 and r["wp_nonstatic_registered_p50"] == 200
    assert r["sync_triggers"] == "init:1,time:1,distance:0,level:0"   # the older key is unchanged
    assert r["missing_wp_culled_in_range_frames"] is None              # no audit (--no-audit run)


def test_wp_sync_keys_absent_are_na_and_counts_zero():
    r = summarize.summarize_lines([_sync(5.0, "init"), _sync(6.0)], "S1", "Town15", 200.0, audit=False)
    out = summarize.format_summary(r)
    for k in ("wp_queue_age_max_s", "wp_violations_total", "wait_ms_p95", "wp_entities_total_p50", "wp_nonstatic_registered_p50"):
        assert "%s=na" % k in out
    assert "wp_deadline_syncs=0" in out and "wp_reset_syncs=0" in out


def test_steady_excludes_rows_with_the_wp_queue_waiting():
    # 7b: the culling's queue not yet empty (wp_pending > 0) is no steady row, like pending / pending_ism
    lines = [_audit(1, wp_pending=1441, missing_steady=1448, missing_in_range=1441),
             _audit(2, wp_pending=0, missing_steady=0)]
    r = summarize.summarize_lines(lines, "S1p", "Town15", 200.0)
    assert r["steady_rows"] == 1
    assert r["missing_steady_max"] == 0
    # older lines without the key count as 0 (still steady)
    r2 = summarize.summarize_lines([_audit(1, missing_steady=2)], "S1p", "Town15", 200.0)
    assert r2["steady_rows"] == 1 and r2["missing_steady_max"] == 2
