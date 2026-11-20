"""Tests for the pure helpers of run_scenario.py (no carla module, no server, no GPU)."""
import dataclasses
import os
import socket
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import run_scenario  # noqa: E402
import scenarios as sc_mod  # noqa: E402


def test_udp_port_busy_refused():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    try:
        bound = s.getsockname()[1]
        assert run_scenario.udp_ports_free([bound]) == (False, bound)
        free = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        free.bind(("0.0.0.0", 0))
        fp = free.getsockname()[1]
        free.close()
        assert run_scenario.udp_ports_free([fp, bound]) == (False, bound)   # reports the busy one, not the first
    finally:
        s.close()


def test_udp_ports_free_when_nothing_listens():
    free = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    free.bind(("0.0.0.0", 0))
    fp = free.getsockname()[1]
    free.close()
    assert run_scenario.udp_ports_free([fp]) == (True, None)
    assert run_scenario.udp_ports_free([]) == (True, None)


def test_det_turns_noise_off():
    assert run_scenario.lidar_preset_kwargs(True) == {"apply_noise": False}
    assert run_scenario.lidar_preset_kwargs(False) == {}
    # F3: the deterministic set also switches off the wall-clock in-range create budget (unknown switches are ignored)
    assert run_scenario.det_server_args(True) == ["-rgl-scene-create-budget-ms=0",
                                                  "-rgl-scene-in-range-create-budget-ms=0"]
    assert run_scenario.det_server_args(False) == []


def test_npc_cycle_schedule():
    assert [run_scenario.npc_cycle_action(k) for k in (399, 400, 800, 801)] == [None, "spawn", "destroy", "respawn"]
    assert [run_scenario.npc_cycle_action(k) for k in (0, 401, 799, 802, 5000)] == [None] * 5


def test_server_args_assembly():
    a = run_scenario.build_server_args("/pkg/run.sh", 4001, audit=False, hit_dir="/o/hits", dump_every=20,
                                       extra=["-rgl-scene-create-budget-ms=0", "-foo=1"])
    assert a[0] == "/pkg/run.sh" and "-carla-rpc-port=4001" in a
    assert "-rgl-scene-audit=1" not in a
    assert "-rgl-debug-hit-dump=/o/hits" in a and "-rgl-debug-hit-dump-every=20" in a
    assert a[-2:] == ["-rgl-scene-create-budget-ms=0", "-foo=1"]
    b = run_scenario.build_server_args("/pkg/run.sh", 4000, audit=True, hit_dir=None, dump_every=10, extra=[])
    assert "-rgl-scene-audit=1" in b and not any(x.startswith("-rgl-debug-hit-dump") for x in b)


def test_nearest_route_point():
    route = ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0), (20.0, 10.0, 0.0))
    assert run_scenario.nearest_route_point(route, 9.0, 1.0) == (1, 10.0)
    assert run_scenario.nearest_route_point(route, 11.0, 12.0) == (2, 20.0)
    assert run_scenario.nearest_route_point(route, 100.0, 10.0) == (3, 30.0)
    with pytest.raises(ValueError):
        run_scenario.nearest_route_point(route, float("nan"), 0.0)


def test_parse_hold_near():
    assert run_scenario.parse_xy("12.5,-3") == (12.5, -3.0)
    for bad in ("1", "a,b", "1,2,3", ""):
        with pytest.raises(ValueError):
            run_scenario.parse_xy(bad)


def test_max_route_override_lowers_min_route():
    sc = sc_mod.SCENARIOS["CONTROL"]
    out = run_scenario.apply_max_route(sc, 2000.0)
    assert out.max_route_m == 2000.0 and out.min_route_m == 2000.0
    short = dataclasses.replace(sc, min_route_m=500.0)
    assert run_scenario.apply_max_route(short, 2000.0).min_route_m == 500.0
    assert run_scenario.apply_max_route(sc, None) is sc


def test_truncated_route_passes_the_length_check():
    # the cut route is as long as asked only up to float rounding; the check must not trip on that
    route = tuple((i * 3.7, 0.0, 0.0) for i in range(2000))
    sc = run_scenario.apply_max_route(sc_mod.SCENARIOS["CONTROL"], 2000.0)
    cut = sc_mod._truncate(route, 2000.0)
    run_scenario.check_route(sc, cut, "Odaiba")


def test_static_scenario_length_and_pose():
    sc = sc_mod.SCENARIOS["STATIC"]
    assert sc.ego_role == "hero" and sc.tile_stream_m == 3000 and sc.actor_active_m == 2000 and sc.hold
    sc = dataclasses.replace(sc, fixed_ticks=220)
    route = ((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))
    assert sc_mod.scenario_ticks(sc, route) == 220
    assert sc_mod.scenario_duration_s(sc, route) == pytest.approx(220 * sc_mod.FIXED_DT)
    # a moving scenario still derives its tick count from the route
    s1 = dataclasses.replace(sc_mod.SCENARIOS["S1p"], speed_mps=10.0)
    assert sc_mod.scenario_ticks(s1, route) == int(100.0 / 10.0 / sc_mod.FIXED_DT + 0.999999)


_CORNER = ((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (100.0, 100.0, 0.0))


def test_yaw_switches_at_a_route_vertex_by_default():
    # The original motion is kept: the heading of the new segment from the first metre after the vertex.
    p = run_scenario.Path(_CORNER)
    assert p.pose(99.0)[4] == pytest.approx(0.0)
    assert p.pose(100.5)[4] == pytest.approx(90.0)
    assert run_scenario.Path(_CORNER, yaw_blend_m=0.0).pose(100.5) == p.pose(100.5)


def test_continuous_yaw_blends_the_heading_after_a_vertex():
    p, q = run_scenario.Path(_CORNER), run_scenario.Path(_CORNER, yaw_blend_m=10.0)
    for s in (50.0, 100.0, 102.5, 105.0, 110.0, 150.0):
        assert q.pose(s)[:4] == pytest.approx(p.pose(s)[:4])   # positions and pitch are untouched
    assert q.pose(100.0)[4] == pytest.approx(0.0)
    assert q.pose(105.0)[4] == pytest.approx(45.0)
    assert q.pose(110.0)[4] == pytest.approx(90.0)
    assert q.pose(150.0)[4] == pytest.approx(90.0)
    # the shorter way round across +-180 degrees
    wrap = ((0.0, 0.0, 0.0), (-10.0, 1.0, 0.0), (-20.0, -1.0, 0.0))
    y0, y1 = run_scenario.Path(wrap).pose(5.0)[4], run_scenario.Path(wrap).pose(15.0)[4]
    mid = run_scenario.Path(wrap, yaw_blend_m=10.0).pose(10.05 + 5.0)[4]
    assert abs((mid - 180.0 + 180.0) % 360.0 - 180.0) < abs((y0 - y1 + 180.0) % 360.0 - 180.0)


def test_continuous_yaw_is_refused_where_it_would_be_ignored():
    with pytest.raises(ValueError):
        run_scenario.Path(_CORNER, pingpong=True, yaw_blend_m=10.0)
    with pytest.raises(ValueError):
        run_scenario.Path(_CORNER, yaw_blend_m=-1.0)
    run_scenario.check_continuous_yaw(sc_mod.SCENARIOS["S1"], 10.0)
    run_scenario.check_continuous_yaw(sc_mod.SCENARIOS["S7"], 0.0)
    for name in ("S7", "S5", "STATIC"):
        with pytest.raises(ValueError):
            run_scenario.check_continuous_yaw(sc_mod.SCENARIOS[name], 10.0)


def test_dash_values_are_merged_for_argparse():
    assert run_scenario.merge_dash_values(["--server-arg", "-rgl-x=1", "--det", "--hold-near", "-5,2", "--out", "o"]) == \
        ["--server-arg=-rgl-x=1", "--det", "--hold-near=-5,2", "--out", "o"]
    assert run_scenario.merge_dash_values(["--server-arg=-a=1", "--hold-near=3,4"]) == ["--server-arg=-a=1", "--hold-near=3,4"]
    assert run_scenario.merge_dash_values(["--out", "-weird"]) == ["--out", "-weird"]   # only the two options are touched


def test_npc_blueprint_falls_back_when_the_default_car_is_missing():
    """The slim package has no vehicle.lincoln.mkz_2020: spawn_npc must use the same fallback as the ego."""
    class BP:
        def __init__(self, i):
            self.id = i
            self.attrs = {}

        def set_attribute(self, k, v):
            self.attrs[k] = v

    class Lib:
        def __init__(self, ids):
            self.bps = [BP(i) for i in ids]

        def filter(self, pat):
            if pat.endswith("*"):
                return [b for b in self.bps if b.id.startswith(pat[:-1])]
            return [b for b in self.bps if b.id == pat]

        def find(self, i):
            raise IndexError("blueprint %r not found" % i)

    class World:
        def try_spawn_actor(self, bp, tf):
            class A:
                def set_simulate_physics(self, v):
                    pass
            a = A()
            a.bp = bp
            return a

    npc = run_scenario.spawn_npc(World(), Lib(["vehicle.kia.niro"]), None)
    assert npc.bp.id == "vehicle.kia.niro" and npc.bp.attrs["role_name"] == "npc_cycle"


def test_scenarios_late_sensors():
    for name, event in (("S6b", "distant_sensor_at_tick_600"), ("S6c", "slow_sensor_ahead")):
        sc = sc_mod.SCENARIOS[name]
        assert sc.map == "Town15" and sc.ego_role == "hero" and sc.sensor_mode == "ego"
        assert sc.tile_stream_m == 3000 and sc.actor_active_m == 2000
        assert sc.max_route_m == 1500.0 and sc.min_route_m == 1500.0
        assert sc.events == (event,)
        assert sc.radius_expected_m == 3000.0


def test_slow_sensor_schedule():
    # S6b: tick 600 at 8 m/s -> the ego is at 240 m, the sensor 300 m ahead of it (540 m of arc)
    assert run_scenario.late_sensor_event("distant_sensor_at_tick_600") == (600, "HesaiPandar128E4X", None, None)
    assert run_scenario.late_sensor_arc_m(600, 8.0, sc_mod.FIXED_DT) == pytest.approx(240.0 + 300.0)
    # S6c: tick 10, VLP16, range 100 m, sensor_tick 6.0 s (stale after 5 s, back at the next measurement)
    assert run_scenario.late_sensor_event("slow_sensor_ahead") == (10, "VelodyneVLP16", 100.0, 6.0)
    assert run_scenario.late_sensor_arc_m(10, 8.0, sc_mod.FIXED_DT) == pytest.approx(4.0 + 300.0)
    assert run_scenario.late_sensor_event("teleport_back") is None
    assert run_scenario.late_sensor_event("second_sensor_at_tick_10") is None
