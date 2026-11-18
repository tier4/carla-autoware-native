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


def test_dash_values_are_merged_for_argparse():
    assert run_scenario.merge_dash_values(["--server-arg", "-rgl-x=1", "--det", "--hold-near", "-5,2", "--out", "o"]) == \
        ["--server-arg=-rgl-x=1", "--det", "--hold-near=-5,2", "--out", "o"]
    assert run_scenario.merge_dash_values(["--server-arg=-a=1", "--hold-near=3,4"]) == ["--server-arg=-a=1", "--hold-near=3,4"]
    assert run_scenario.merge_dash_values(["--out", "-weird"]) == ["--out", "-weird"]   # only the two options are touched
