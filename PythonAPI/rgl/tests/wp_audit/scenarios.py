"""Scenario table for the World Partition scene-audit runner.

Does not import carla (so the table can be inspected and tested anywhere).

The routes are polylines in CARLA world coordinates [m] (left-handed, x forward /
y right / z up, the same as carla.Location), one JSON file per map in routes/
(see the frozen-routes comment below). A map without a file has no route, and
run_scenario.py stops with `status=invalid reason=route_not_set`. How they were
made:

  Town15 (WP)   : pick_route.py --mode town15 on a machine that has the package.
                  It walks world.get_map() waypoints and keeps only road
                  stretches where cast_ray hits (downwards, semantic tag)
                  contain no Terrain (= no Landscape under the road), then
                  prints a polyline: a stretch without Landscape, frozen.
  Odaiba        : the OpenDRIVE map does not exist, so waypoints are not
                  available. The polyline follows the lanelet2 map from the
                  ego start of a driving scenario on that map (lanelet 2314486
                  s=25.0127 offset=0.5; route waypoints lanelet 1501 s=30,
                  lanelet 164 s=25). `pick_route.py --mode odaiba --lanelets
                  2314486,1501,164 --osm <lanelet2_map.osm>` prints the lanelet2
                  centerlines (local_x/local_y of the osm); they are converted
                  to CARLA coordinates with the map's offset (below).
  Town10HD_Opt  : CONTROL only. Waypoints (pick_route.py --mode town10).

Route length requirements (checked by run_scenario.py, `reason=route_too_short`):
  S1 / CONTROL : >= 6000 m of path (the polyline may be repeated by the runner
                 only if `loop=True`; do not rely on it)
  S5           : one lap (forward) must take longer than 61 s at 8 m/s, i.e.
                 >= 500 m, so that the default GC interval (61 s) elapses
                 inside a lap.
  S4           : >= 1.5 km + 200 m.
"""
import json
import locale
import math
import os
from dataclasses import dataclass
from typing import Optional, Tuple

Route = Optional[Tuple[Tuple[float, float, float], ...]]

SPEED_MPS = 8.0
FIXED_DT = 0.05
LIDAR_Z = 2.0
LIDAR_MODEL = "HesaiPandar128E4X"

# Frozen routes (2026-11-18), one JSON list of [x, y, z] per map in routes/<map>.json (CARLA m):
#   Town15       : pick_route.py --mode town15 --load --length 6000 --step 10 on a packaged build
#                  (waypoints without Terrain under the road), 600 points, ~6000 m.
#   Town10HD_Opt : pick_route.py --mode town10 --load --length 6000 --step 10, 600 points, ~6000 m.
#   Odaiba       : the Odaiba lanelet2 map (lanelet2_map.osm), lanelets chained from the scenario start
#                  2314486, keeping only the 1131 lanelets whose centerline hits the CARLA ground within
#                  2 m; CARLA x = local_x - 92008.5, y = -(local_y - 45335.1), z = ele (the map's offset).
#                  2222 points, 6558 m.
# A missing file leaves the map's route unset (run_scenario.py then stops with route_not_set).
_ROUTE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "routes")


def _load_route(map_name) -> Route:
    path = os.path.join(_ROUTE_DIR, map_name + ".json")
    if not os.path.exists(path):
        return None
    with open(path, encoding=locale.getpreferredencoding(False)) as f:
        return tuple(tuple(float(c) for c in p) for p in json.load(f))


TOWN15_ROUTE: Route = _load_route("Town15")
ODAIBA_ROUTE: Route = _load_route("Odaiba")
TOWN10_ROUTE: Route = _load_route("Town10HD_Opt")

ROUTES = {"Town15": TOWN15_ROUTE, "Odaiba": ODAIBA_ROUTE, "Town10HD_Opt": TOWN10_ROUTE}


@dataclass(frozen=True)
# Preserve the existing scenario schema and constructor interface.
# pylint: disable-next=too-many-instance-attributes
class Scenario:
    """Parameters for one scene-audit scenario."""
    name: str
    map: str
    speed_mps: float = SPEED_MPS
    tile_stream_m: float = 3000.0       # EpisodeSettings.tile_stream_distance, set before the ego spawn
    actor_active_m: float = 2000.0      # EpisodeSettings.actor_active_distance
    ego_role: str = "hero"
    sensor_mode: str = "ego"            # ego | roadside | npc
    lidar_range_m: float = 200.0
    radius_expected_m: Optional[float] = -1.0   # None = do not check (S7 mixes maps)
    laps: int = 1
    pingpong: bool = False              # S5: forward and back
    min_route_m: float = 0.0
    max_route_m: Optional[float] = None  # use only the first max_route_m of the map's route (keeps the run short)
    events: Tuple[str, ...] = ()        # runner-side behaviours, see run_scenario.py
    fixed_duration_s: Optional[float] = None    # when not route-based
    fixed_ticks: Optional[int] = None   # exact tick count (STATIC: settle + hold ticks, set by the runner)
    hold: bool = False                  # the ego does not move: it stays at one point of the route
    description: str = ""


def _s(**kw):
    return Scenario(**kw)


# parent_stream_radius_m expectations: WP map + hero parent = the tile_stream_m
# value set here; S2 (role_name=npc_lidar) = 200; S3 (no parent) and non-WP = -1.
SCENARIOS = {
    "S1": _s(name="S1", map="Town15", tile_stream_m=3000, actor_active_m=2000,
             radius_expected_m=3000.0, min_route_m=6000.0,
             description="6 km at the default radii (non-Static parts, late sensors, sync cost)"),
    "S1p": _s(name="S1p", map="Town15", tile_stream_m=150, actor_active_m=150,
              radius_expected_m=150.0, min_route_m=500.0, max_route_m=1500.0,
              description="same road with 150 m radii (ghosts, mesh cache, ISM instances, level adds)"),
    "S2": _s(name="S2", map="Town15", tile_stream_m=150, actor_active_m=150,
             ego_role="npc_lidar", radius_expected_m=200.0, lidar_range_m=200.0,
             min_route_m=500.0, max_route_m=1500.0, description="non-hero vehicle carrying a 200 m LiDAR"),
    "S3": _s(name="S3", map="Town15", tile_stream_m=150, actor_active_m=150,
             sensor_mode="roadside", radius_expected_m=-1.0, min_route_m=500.0, max_route_m=1500.0,
             description="parentless roadside sensor, hero passes by"),
    "S4": _s(name="S4", map="Town15", tile_stream_m=150, actor_active_m=150,
             radius_expected_m=150.0, min_route_m=1700.0, max_route_m=1800.0,
             events=("teleport_ahead_1500", "teleport_back", "respawn"),
             description="teleport 1.5 km ahead, back, and respawn"),
    "S5": _s(name="S5", map="Town15", tile_stream_m=150, actor_active_m=150,
             radius_expected_m=150.0, laps=3, pingpong=True, min_route_m=500.0, max_route_m=600.0,
             description="S1p road, 3 round trips (a lap longer than the 61 s GC interval)"),
    "S6": _s(name="S6", map="Town15", tile_stream_m=150, actor_active_m=150,
             radius_expected_m=150.0, min_route_m=500.0, max_route_m=1500.0, events=("second_sensor_at_tick_10",),
             description="second sensor right after the initial streaming"),
    "S6b": _s(name="S6b", map="Town15", tile_stream_m=3000, actor_active_m=2000,
              radius_expected_m=3000.0, min_route_m=1500.0, max_route_m=1500.0,
              events=("distant_sensor_at_tick_600",),
              description="3 km hero; at tick 600 a parentless Pandar128E4X appears 300 m ahead on the route"),
    "S6c": _s(name="S6c", map="Town15", tile_stream_m=3000, actor_active_m=2000,
              radius_expected_m=3000.0, min_route_m=1500.0, max_route_m=1500.0,
              events=("slow_sensor_ahead",),
              description=("3 km hero; at tick 10 a parentless slow VLP16 (sensor_tick 6 s) "
                           "300 m ahead: stale, then back")),
    "S7": _s(name="S7", map="Town15", radius_expected_m=None, events=("load_world_cycle_3",),
             fixed_duration_s=480.0, tile_stream_m=150, actor_active_m=150,
             description="load_world Town15 <-> Odaiba, 3 round trips"),
    "STATIC": _s(name="STATIC", map="Town15", tile_stream_m=3000, actor_active_m=2000,
                 radius_expected_m=3000.0, hold=True,
                 description="ego parked at the route point nearest --hold-near (settle + hold ticks)"),
    "CONTROL": _s(name="CONTROL", map="Odaiba", tile_stream_m=3000, actor_active_m=2000,
                  radius_expected_m=-1.0, min_route_m=5500.0,
                  description="non-WP control: Odaiba and Town10HD_Opt, S1 speed and LiDAR"),
}

TODO_MARK = "route_not_set"


def _truncate(route, max_m):
    """The first max_m metres of the polyline (the last point interpolated)."""
    out, total = [route[0]], 0.0
    for a, b in zip(route[:-1], route[1:]):
        seg = sum((b[k] - a[k]) ** 2 for k in range(3)) ** 0.5
        if total + seg >= max_m:
            f = (max_m - total) / seg if seg > 0 else 0.0
            out.append(tuple(a[k] + f * (b[k] - a[k]) for k in range(3)))
            return tuple(out)
        out.append(b)
        total += seg
    return tuple(out)


def route_for(scenario_name, map_name, sc=None):
    """Return the polyline (cut to the scenario's max_route_m) or None when it is not set.

    sc: the scenario to use instead of the table's one (the runner passes its overridden copy).
    """
    route = ROUTES.get(map_name)
    sc = sc if sc is not None else SCENARIOS.get(scenario_name)
    if route_is_set(route) and sc is not None and sc.max_route_m is not None:
        route = _truncate(route, sc.max_route_m)
    return route


def route_is_set(route):
    """Check that a frozen route has enough vertices."""
    return bool(route) and len(route) >= 2


def route_length(route):
    """Return the total polyline length in metres."""
    total = 0.0
    for a, b in zip(route[:-1], route[1:]):
        total += ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
    return total


def scenario_ticks(sc, route):
    """Number of world ticks the scenario runs."""
    if sc.fixed_ticks is not None:
        return sc.fixed_ticks
    return int(math.ceil(scenario_duration_s(sc, route) / FIXED_DT))


def scenario_duration_s(sc, route):
    """Driving time of the scenario in seconds (without the +300 s timeout margin)."""
    if sc.fixed_ticks is not None:
        return sc.fixed_ticks * FIXED_DT
    if sc.fixed_duration_s is not None:
        return sc.fixed_duration_s
    path = route_length(route)
    return path * sc.laps * (2 if sc.pingpong else 1) / sc.speed_mps
