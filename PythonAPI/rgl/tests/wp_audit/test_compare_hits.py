"""Tests for compare_hits.py (synthetic .bin dumps written with the stage-0 header; no CARLA, numpy only)."""
import os
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import compare_hits  # noqa: E402

HEADER = struct.Struct("<IId6f")
MAGIC = 0x52474C48
SYNC = ("LogCarlaRGL: rgl_scene_sync t=%.3f trigger=%s scanned=%d added_cached=0 queued=%d created=%d removed=%d "
        "idle_destroyed=0 pending=0 elapsed_ms=9.86 drain_ms=0.00 failed=0 drain_reused=0 dropped=0 "
        "range_skipped=5 sensors=1 entities=%d ismc=10 meshes=%d idle=0")


def write_dump(d, sensor_id, frame, sim_t, pose, pts):
    os.makedirs(d, exist_ok=True)
    pts = np.asarray(pts, dtype="<f4").reshape(-1, 3)
    name = "0x%x_%020d.bin" % (sensor_id, frame)
    with open(os.path.join(d, name), "wb") as f:
        f.write(HEADER.pack(MAGIC, len(pts), sim_t, *pose))
        f.write(pts.tobytes())


def cloud(seed, n=200):
    return np.random.default_rng(seed).uniform(-50.0, 50.0, size=(n, 3)).astype(np.float32)


POSE = (10.0, 20.0, 2.0, 0.0, 0.0, 90.0)


def make_run(d, sensor_id, n_dumps=2, t0=2.4, pose=POSE, frame0=1000, shift=None, extra=None):
    """n_dumps dumps of one sensor, 0.5 s apart; dump k holds cloud(k)."""
    for k in range(n_dumps):
        pts = cloud(k)
        if shift is not None:
            pts = pts + np.float32(shift)
        if extra is not None and k == 0:
            pts = np.vstack([pts, np.asarray(extra, dtype=np.float32).reshape(1, 3)])
        write_dump(str(d), sensor_id, frame0 + k * 7, t0 + 0.5 * k, pose, pts)


def kvs(line):
    out = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def run_cli(capsys, *argv):
    rc = compare_hits.main(list(argv))
    out = capsys.readouterr().out.strip().splitlines()
    return rc, out[-1]


def diff_kv(capsys, base, cand):
    rc, line = run_cli(capsys, "diff", str(base), str(cand))
    assert line.startswith("hits_diff ")
    return kvs(line)


def write_run_txt(path, **kw):
    kw.setdefault("status", "ok")
    kw.setdefault("ego_bp_fallback", 0)
    with open(path, "w") as f:
        f.write(" ".join("%s=%s" % (k, v) for k, v in kw.items()) + "\n")


def write_stderr(path, created=(1, 2, 3), t0=2.424):
    with open(path, "w") as f:
        f.write("LogTemp: unrelated\n")
        for i, c in enumerate(created):
            f.write(SYNC % (t0 + i * 1.0, "init" if i == 0 else "time", 100 + i, 0, c, 0, 50 + i, 7) + "\n")


def gate(capsys, tmp_path, base, cand, min_frames=1, run_kw=None, cand_created=(1, 2, 3)):
    run_kw = run_kw or {}
    write_run_txt(str(tmp_path / "run.txt"), **run_kw)
    write_stderr(str(tmp_path / "base.stderr"))
    write_stderr(str(tmp_path / "cand.stderr"), created=cand_created, t0=2.524)
    rc, line = run_cli(capsys, "gate", "--base", str(base), "--cand", str(cand), "--min-frames", str(min_frames),
                       "--cand-run", str(tmp_path / "run.txt"), "--base-stderr", str(tmp_path / "base.stderr"),
                       "--cand-stderr", str(tmp_path / "cand.stderr"))
    assert line.startswith("gate ")
    g = kvs(line)
    assert (rc == 0) == (g["verdict"] == "pass")
    return g


def test_identical_dumps_match_across_sensor_ids(tmp_path):
    make_run(tmp_path / "b", 0xAAA, frame0=100)
    make_run(tmp_path / "c", 0xBBB, frame0=9000)
    kv = compare_hits.hits_diff_kv(str(tmp_path / "b"), str(tmp_path / "c"))
    assert kv["identical"] == "2" and kv["differing"] == "0" and kv["matched"] == "2"


def test_shifted_sim_t_still_matched(tmp_path, capsys):
    make_run(tmp_path / "b", 0xAAA, t0=2.4)
    make_run(tmp_path / "c", 0xBBB, t0=2.5)             # every sim_t + 0.1 s (H1)
    kv = diff_kv(capsys, tmp_path / "b", tmp_path / "c")
    assert kv["matched"] == "2" and kv["time_mismatch"] == "0" and kv["identical"] == "2"


def test_relative_time_difference_is_counted(tmp_path, capsys):
    make_run(tmp_path / "b", 0xAAA)
    for k in range(2):                                   # second dump 5 ms late
        write_dump(str(tmp_path / "c"), 0xBBB, 50 + k, 3.0 + 0.5 * k + (0.005 if k else 0.0), POSE, cloud(k))
    kv = diff_kv(capsys, tmp_path / "b", tmp_path / "c")
    assert kv["matched"] == "2" and kv["time_mismatch"] == "1"


def test_zero_matched_fails(tmp_path, capsys):
    make_run(tmp_path / "b", 0xAAA)
    make_run(tmp_path / "c", 0xBBB, pose=(15.0, 20.0, 2.0, 0.0, 0.0, 90.0))   # 5 m away from every base sensor
    g = gate(capsys, tmp_path, tmp_path / "b", tmp_path / "c")
    assert g["verdict"] == "fail" and "no_match" in g["reasons"]
    assert g["matched"] == "0"


def test_too_few_frames_fails(tmp_path, capsys):
    make_run(tmp_path / "b", 0xAAA, n_dumps=3)
    make_run(tmp_path / "c", 0xBBB, n_dumps=3)
    g = gate(capsys, tmp_path, tmp_path / "b", tmp_path / "c", min_frames=10)
    assert g["verdict"] == "fail" and "too_few_frames" in g["reasons"]
    ok = gate(capsys, tmp_path, tmp_path / "b", tmp_path / "c", min_frames=3)
    assert ok["verdict"] == "pass" and ok["reasons"] == "none"


def test_run_status_and_fallback_fail(tmp_path, capsys):
    make_run(tmp_path / "b", 0xAAA)
    make_run(tmp_path / "c", 0xBBB)
    g1 = gate(capsys, tmp_path, tmp_path / "b", tmp_path / "c", run_kw=dict(status="invalid"))
    g2 = gate(capsys, tmp_path, tmp_path / "b", tmp_path / "c", run_kw=dict(ego_bp_fallback=1))
    assert "status_not_ok" in g1["reasons"] and "ego_bp_fallback" in g2["reasons"]
    assert "ego_bp_fallback" not in g1["reasons"] and "status_not_ok" not in g2["reasons"]


def test_unreadable_cand_run_is_status_not_ok(tmp_path, capsys):
    make_run(tmp_path / "b", 0xAAA)
    make_run(tmp_path / "c", 0xBBB)
    write_stderr(str(tmp_path / "s"))
    rc, line = run_cli(capsys, "gate", "--base", str(tmp_path / "b"), "--cand", str(tmp_path / "c"),
                       "--min-frames", "1", "--cand-run", str(tmp_path / "missing.txt"),
                       "--base-stderr", str(tmp_path / "s"), "--cand-stderr", str(tmp_path / "s"))
    assert rc == 1 and "status_not_ok" in kvs(line)["reasons"]


def test_sync_sequence_mismatch_fails(tmp_path, capsys):
    make_run(tmp_path / "b", 0xAAA)
    make_run(tmp_path / "c", 0xBBB)
    g = gate(capsys, tmp_path, tmp_path / "b", tmp_path / "c", cand_created=(1, 2, 4))   # one line created=4 vs 3
    assert g["verdict"] == "fail" and "sync_sequence" in g["reasons"]
    ok = gate(capsys, tmp_path, tmp_path / "b", tmp_path / "c")                         # same sequence, t shifted 0.1 s
    assert ok["verdict"] == "pass"


def test_extra_point_at_180m_is_added_far(tmp_path, capsys):
    make_run(tmp_path / "b", 0xAAA)
    make_run(tmp_path / "c", 0xBBB, extra=(POSE[0] + 180.0, POSE[1], POSE[2]))
    kv = diff_kv(capsys, tmp_path / "b", tmp_path / "c")
    assert kv["differing"] == "1" and kv["points_added"] == "1" and kv["added_by_range"] == "0/0/0/1" \
        and kv["points_removed"] == "0"
    g = gate(capsys, tmp_path, tmp_path / "b", tmp_path / "c")
    assert g["verdict"] == "fail" and g["reasons"] == "differing"


def test_two_sensors_matched_by_pose(tmp_path, capsys):
    pa, pb = (0.0, 0.0, 2.0, 0.0, 0.0, 0.0), (0.0, 1.0, 2.0, 0.0, 0.0, 0.0)
    write_dump(str(tmp_path / "b"), 0x1, 1, 3.0, pa, cloud(1))
    write_dump(str(tmp_path / "b"), 0x2, 1, 3.0, pb, cloud(2))
    write_dump(str(tmp_path / "c"), 0x10, 1, 7.0, pb, cloud(2))      # files sort in the opposite order
    write_dump(str(tmp_path / "c"), 0x20, 1, 7.0, pa, cloud(1))
    kv = diff_kv(capsys, tmp_path / "b", tmp_path / "c")
    assert kv["matched"] == "2" and kv["identical"] == "2"


def test_sub_cm_noise_is_near_identical(tmp_path, capsys):
    make_run(tmp_path / "b", 0xAAA, n_dumps=1)
    make_run(tmp_path / "c", 0xBBB, n_dumps=1, shift=0.003)          # every point +3 mm
    kv = diff_kv(capsys, tmp_path / "b", tmp_path / "c")
    assert kv["identical"] == "0" and kv["near_identical"] == "1" and kv["differing"] == "0"


def test_count_differences_are_reported(tmp_path, capsys):
    write_dump(str(tmp_path / "b"), 0x1, 1, 3.0, POSE, cloud(1, 100))
    write_dump(str(tmp_path / "c"), 0x2, 1, 3.0, POSE, cloud(1, 100)[:90])
    kv = diff_kv(capsys, tmp_path / "b", tmp_path / "c")
    assert kv["max_abs_count_diff"] == "10" and kv["max_rel_count_diff"] == "0.1000"
    assert kv["points_removed"] == "10" and kv["points_added"] == "0" and kv["points_total_base"] == "100"


def test_diff_sets_voxel_boundary_neighbours():
    # two points 8 mm apart that sit in different 1 cm voxels must still pair up
    base = np.array([[0.0095, 0.0, 0.0]])
    cand = np.array([[0.0175, 0.0, 0.0], [5.0, 5.0, 5.0]])
    r = compare_hits.diff_sets(base, cand, (0.0, 0.0, 0.0))
    assert r.added == 1 and r.removed == 0
    assert r.added_by_range == (1, 0, 0, 0)


def test_match_sensors_respects_tolerance():
    d = compare_hits.Dump(1.0, np.array([0, 0, 0, 0, 0, 0.0]), np.zeros((0, 3)))
    near = compare_hits.Dump(1.0, np.array([0.005, 0, 0, 0, 0, 0.0]), np.zeros((0, 3)))
    far = compare_hits.Dump(1.0, np.array([0.5, 0, 0, 0, 0, 0.0]), np.zeros((0, 3)))
    assert compare_hits.match_sensors({0: [d]}, {0: [near]}) == [(0, 0)]
    assert compare_hits.match_sensors({0: [d]}, {0: [far]}) == []


def test_read_dump_and_bad_magic(tmp_path):
    write_dump(str(tmp_path), 0xAB, 3, 12.5, POSE, cloud(0, 5))
    p = sorted(tmp_path.iterdir())[0]
    t, pose, pts = compare_hits.read_dump(str(p))
    assert t == 12.5 and list(pose) == list(POSE) and pts.shape == (5, 3)
    bad = tmp_path / "bad.bin"
    bad.write_bytes(b"\x00" * 64)
    with pytest.raises(ValueError):
        compare_hits.read_dump(str(bad))


def test_sync_sequence_parses_relative_time():
    lines = ["noise", SYNC % (2.4244, "init", 11, 0, 3, 0, 40, 9), SYNC % (3.5, "time", 12, 1, 0, 2, 41, 9)]
    seq = compare_hits.sync_sequence(lines)
    assert seq == [(0.0, "init", 11, 0, 3, 0, 40, 9), (1.076, "time", 12, 1, 0, 2, 41, 9)]


def test_fingerprint_rows(tmp_path):
    make_run(tmp_path / "b", 0xAAA)
    make_run(tmp_path / "c", 0xBBB, t0=9.9, frame0=5)
    out_b, out_c = tmp_path / "b.csv", tmp_path / "c.csv"
    assert compare_hits.main(["fingerprint", str(tmp_path / "b"), "--out", str(out_b)]) == 0
    assert compare_hits.main(["fingerprint", str(tmp_path / "c"), "--out", str(out_c)]) == 0
    rows = out_b.read_text().splitlines()
    assert len(rows) == 2 and rows[0].split(",")[:4] == ["0", "0", "0.000", "200"]
    assert rows[1].split(",")[:3] == ["0", "1", "0.500"]
    assert out_b.read_text() == out_c.read_text()      # ids, frames and absolute sim_t do not enter the row
    assert len(rows[0].split(",")[4]) == 40


# ---- 7b: the keys added to the sync line (wait_ms, wait_api, wp_*) do not change what the gate compares ----
WP_TAIL = (" scan_ms=1.00 reg_ms=0.00 remove_ms=0.00 skel_ms=0.00 wait_ms=61.20 wait_api=rgl_entity_destroy wp=1 "
           "wp_sync_bound_s=2.10 wp_create_latency_s=1.00 wp_margin_m=320.0 wp_pending=7 wp_queue_age_max_s=0.40 "
           "wp_check_n=12 wp_check_instances=900 wp_check_ms=3.10 wp_check_max_ms=3.10 wp_entity_added=5 "
           "wp_entity_removed=2 wp_entity_ms=0.80 wp_promoted=1 wp_violations=0 wp_deadline_syncs=0 "
           "wp_entities_total=12345 wp_nonstatic_registered=2345")


def test_wp_keys_do_not_change_the_sync_sequence():
    plain = [SYNC % (2.4, "init", 11, 0, 3, 0, 40, 9), SYNC % (3.5, "time", 12, 1, 0, 2, 41, 9)]
    tailed = [ln + WP_TAIL for ln in plain]
    assert compare_hits.sync_sequence(tailed) == compare_hits.sync_sequence(plain)
    assert compare_hits.sync_sequences_equal(compare_hits.sync_sequence(plain), compare_hits.sync_sequence(tailed))


def test_each_compared_key_still_fails_with_wp_keys():
    base = [SYNC % (2.4, "init", 11, 0, 3, 0, 40, 9) + WP_TAIL]
    variants = {
        "trigger": SYNC % (2.4, "time", 11, 0, 3, 0, 40, 9),
        "scanned": SYNC % (2.4, "init", 12, 0, 3, 0, 40, 9),
        "queued": SYNC % (2.4, "init", 11, 1, 3, 0, 40, 9),
        "created": SYNC % (2.4, "init", 11, 0, 4, 0, 40, 9),
        "removed": SYNC % (2.4, "init", 11, 0, 3, 1, 40, 9),
        "entities": SYNC % (2.4, "init", 11, 0, 3, 0, 41, 9),
        "meshes": SYNC % (2.4, "init", 11, 0, 3, 0, 40, 8),
    }
    b = compare_hits.sync_sequence(base)
    for key, line in variants.items():
        c = compare_hits.sync_sequence([line + WP_TAIL])
        assert not compare_hits.sync_sequences_equal(b, c), key


def test_unknown_keys_do_not_widen_what_is_ignored():
    # wp_entities_total is not entities: a change of entities still fails, a change of wp_entities_total alone does not
    base = [SYNC % (2.4, "init", 11, 0, 3, 0, 40, 9) + WP_TAIL]
    other_total = [base[0].replace("wp_entities_total=12345", "wp_entities_total=999")]
    other_entities = [(SYNC % (2.4, "init", 11, 0, 3, 0, 41, 9)) + WP_TAIL]
    b = compare_hits.sync_sequence(base)
    assert compare_hits.sync_sequences_equal(b, compare_hits.sync_sequence(other_total))
    assert not compare_hits.sync_sequences_equal(b, compare_hits.sync_sequence(other_entities))
