"""Scenario table for the World Partition scene-audit runner (stage 0).

Does not import carla (so the table can be inspected and tested anywhere).

ROUTE COORDINATES ARE PLACEHOLDERS (TODO). The maps cannot be opened on the dev
PC, so no polyline below is real. run_scenario.py detects a placeholder route
and stops with `status=invalid reason=route_not_set`. Fill them in before the
first real run (coordinates are CARLA world coordinates [m], left-handed,
x forward / y right / z up, the same as carla.Location):

  Town15 (WP)   : run pick_route.py --mode town15 on the machine that has the
                  package. It walks world.get_map() waypoints and keeps only
                  road stretches where cast_ray hits (downwards, semantic tag)
                  contain no Terrain (= no Landscape under the road), then
                  prints a polyline. Plan (Task 6, "polyline provenance"):
                  pick a stretch without Landscape and freeze its coordinates.
  Odaiba        : the OpenDRIVE map does not exist, so waypoints are not
                  available. The polyline comes from the CS Odaiba V2I scenario
                  (ndt_awsimpcd_teleport_1b_v2i.yml under
                  /mnt/dsk0/wk0/CARLA/ss2_carla_bridge/AW/pilot-auto.x2/v4.4-e2e/
                  pilot-auto.x2/src/simulator/carla_support.x2/scenario/ndt_scenarios/):
                  ego start = lanelet 2314486 s=25.0127 offset=0.5, route
                  waypoints = lanelet 1501 s=30, lanelet 164 s=25. Use
                  `pick_route.py --mode odaiba --lanelets 2314486,1501,164` to
                  print the lanelet2 centerlines (local_x/local_y of the osm),
                  then convert to CARLA coordinates (the controller decides
                  the conversion; it is not guessed here).
  Town10HD_Opt  : CONTROL only. Use waypoints (pick_route.py --mode town10).

Route length requirements (checked by run_scenario.py, `reason=route_too_short`):
  S1 / CONTROL : >= 6000 m of path (the polyline may be repeated by the runner
                 only if `loop=True`; do not rely on it)
  S5           : one lap (forward) must take longer than 61 s at 8 m/s, i.e.
                 >= 500 m, so that the default GC interval (61 s) elapses
                 inside a lap.
  S4           : >= 1.5 km + 200 m.
"""
from dataclasses import dataclass, field
from typing import Optional, Tuple

Route = Optional[Tuple[Tuple[float, float, float], ...]]

SPEED_MPS = 8.0
FIXED_DT = 0.05
LIDAR_Z = 2.0
LIDAR_MODEL = "HesaiPandar128E4X"

# TODO(route): placeholders. None means "not set"; see the module docstring.
TOWN15_ROUTE: Route = None      # TODO: freeze from pick_route.py --mode town15
ODAIBA_ROUTE: Route = None      # TODO: from the CS Odaiba V2I scenario (see above)
TOWN10_ROUTE: Route = None      # TODO: freeze from pick_route.py --mode town10

ROUTES = {"Town15": TOWN15_ROUTE, "Odaiba": ODAIBA_ROUTE, "Town10HD_Opt": TOWN10_ROUTE}


@dataclass(frozen=True)
class Scenario:
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
    events: Tuple[str, ...] = ()        # runner-side behaviours, see run_scenario.py
    fixed_duration_s: Optional[float] = None    # when not route-based
    description: str = ""


def _s(**kw):
    return Scenario(**kw)


# parent_stream_radius_m expectations: WP map + hero parent = the tile_stream_m
# value set here; S2 (role_name=npc_lidar) = 200; S3 (no parent) and non-WP = -1.
SCENARIOS = {
    "S1": _s(name="S1", map="Town15", tile_stream_m=3000, actor_active_m=2000,
             radius_expected_m=3000.0, min_route_m=6000.0,
             description="6 km at the default radii (Movable, P6, P7)"),
    "S1p": _s(name="S1p", map="Town15", tile_stream_m=150, actor_active_m=150,
              radius_expected_m=150.0, min_route_m=500.0,
              description="same road with 150 m radii (P1, P2, P3, P5)"),
    "S2": _s(name="S2", map="Town15", tile_stream_m=150, actor_active_m=150,
             ego_role="npc_lidar", radius_expected_m=200.0, lidar_range_m=200.0,
             min_route_m=500.0, description="non-hero vehicle carrying a 200 m LiDAR"),
    "S3": _s(name="S3", map="Town15", tile_stream_m=150, actor_active_m=150,
             sensor_mode="roadside", radius_expected_m=-1.0, min_route_m=500.0,
             description="parentless roadside sensor, hero passes by"),
    "S4": _s(name="S4", map="Town15", tile_stream_m=150, actor_active_m=150,
             radius_expected_m=150.0, min_route_m=1700.0,
             events=("teleport_ahead_1500", "teleport_back", "respawn"),
             description="teleport 1.5 km ahead, back, and respawn"),
    "S5": _s(name="S5", map="Town15", tile_stream_m=150, actor_active_m=150,
             radius_expected_m=150.0, laps=3, pingpong=True, min_route_m=500.0,
             description="S1p road, 3 round trips (a lap longer than the 61 s GC interval)"),
    "S6": _s(name="S6", map="Town15", tile_stream_m=150, actor_active_m=150,
             radius_expected_m=150.0, min_route_m=500.0, events=("second_sensor_at_tick_10",),
             description="second sensor right after the initial streaming"),
    "S7": _s(name="S7", map="Town15", radius_expected_m=None, events=("load_world_cycle_3",),
             fixed_duration_s=480.0, tile_stream_m=150, actor_active_m=150,
             description="load_world Town15 <-> Odaiba, 3 round trips"),
    "CONTROL": _s(name="CONTROL", map="Odaiba", tile_stream_m=3000, actor_active_m=2000,
                  radius_expected_m=-1.0, min_route_m=6000.0,
                  description="non-WP control: Odaiba and Town10HD_Opt, S1 speed and LiDAR"),
}

TODO_MARK = "route_not_set"


def route_for(scenario_name, map_name):
    """Return the polyline or None when it is still a placeholder."""
    return ROUTES.get(map_name)


def route_is_set(route):
    return bool(route) and len(route) >= 2


def route_length(route):
    total = 0.0
    for a, b in zip(route[:-1], route[1:]):
        total += ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
    return total


def scenario_duration_s(sc, route):
    """Driving time of the scenario in seconds (without the +300 s timeout margin)."""
    if sc.fixed_duration_s is not None:
        return sc.fixed_duration_s
    path = route_length(route)
    return path * sc.laps * (2 if sc.pingpong else 1) / sc.speed_mps
