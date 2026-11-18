"""Tests for compare_raycast.py (no CARLA server and no carla module needed)."""
import math
import os
import sys

import pytest

np = pytest.importorskip("numpy")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import compare_raycast as cr  # noqa: E402


def _grid(n, x0, y0=0.0):
    # n points, each in the centre of its own 1 m cell
    return np.array([[x0 + i + 0.5, y0 + 0.5, 0.5] for i in range(n)], dtype=np.float64)


def test_cell_sets_synthetic():
    common = _grid(100, 0.0)
    rgl = np.vstack([common, _grid(5, 1000.0)])
    ray = np.vstack([common, _grid(3, 2000.0)])
    res = cr.compare_points(rgl, ray, np.zeros(len(ray), dtype=np.int32), cell=1.0)
    assert res["rgl_only_cells"] == 5
    assert res["raycast_only_cells"] == 3
    assert res["both_cells"] == 100
    assert cr.format_counts(res) == "rgl_only_cells=5 raycast_only_cells=3 both_cells=100"
    # CSV rows: RGL-only cells have an empty tag, ray_cast-only cells have the majority tag
    rows = {r[3] for r in res["diff_rows"]}
    assert "" in rows


def test_majority_tag_for_raycast_only():
    ray = np.array([[5.5, 5.5, 0.5], [5.6, 5.4, 0.5], [5.2, 5.2, 0.5]])
    tags = np.array([7, 7, 3], dtype=np.int32)
    res = cr.compare_points(np.zeros((0, 3)), ray, tags, cell=1.0)
    assert res["raycast_only_cells"] == 1
    assert res["diff_rows"][0][3] == "7"


def test_mask_tags():
    ray = np.array([[5.5, 5.5, 0.5], [8.5, 5.5, 0.5]])
    tags = np.array([22, 1], dtype=np.int32)
    res = cr.compare_points(np.zeros((0, 3)), ray, tags, cell=1.0, mask_tags={22})
    assert res["raycast_only_cells"] == 1


def test_negative_coordinates_floor():
    # -0.2 and +0.2 are in different 1 m cells
    a = np.array([[-0.2, 0.5, 0.5]])
    b = np.array([[0.2, 0.5, 0.5]])
    res = cr.compare_points(a, b, np.zeros(1, dtype=np.int32), cell=1.0)
    assert res["both_cells"] == 0


def test_local_to_world_yaw90():
    # CARLA left-handed: x forward, y right. yaw 90 deg turns forward (+x) into +y.
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    w = cr.local_to_world(pts, (10.0, 20.0, 30.0), (0.0, 0.0, 90.0))
    # local (1,0,0) -> world (10, 21, 30); local (0,1,0) (right) -> world (9, 20, 30)
    assert np.allclose(w[0], [10.0, 21.0, 30.0], atol=1e-9)
    assert np.allclose(w[1], [9.0, 20.0, 30.0], atol=1e-9)


def test_local_to_world_pitch_and_roll_match_carla_formula():
    # CARLA rotation matrix (UE FRotationMatrix): pitch up (positive) raises forward vector z.
    p = np.array([[1.0, 0.0, 0.0]])
    w = cr.local_to_world(p, (0, 0, 0), (0.0, 30.0, 0.0))
    assert np.allclose(w[0], [math.cos(math.radians(30)), 0.0, math.sin(math.radians(30))], atol=1e-9)
    # UE FRotationMatrix: the right axis is (sr*sp*cy - cr*sy, sr*sp*sy + cr*cy, -sr*cp),
    # so roll +90 deg sends local +y (right) to world -z (right side goes down).
    q = np.array([[0.0, 1.0, 0.0]])
    w2 = cr.local_to_world(q, (0, 0, 0), (90.0, 0.0, 0.0))
    assert np.allclose(w2[0], [0.0, 0.0, -1.0], atol=1e-9)
