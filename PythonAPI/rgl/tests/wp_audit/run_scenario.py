#!/usr/bin/env python3
"""World Partition scene-audit scenario runner (stage 0).

  run_scenario.py --pkg <dir> --map <Odaiba|Town15|Town10HD_Opt>
                  --scenario <S1|S1p|S2|S3|S4|S5|S6|S7|STATIC|CONTROL> --out <dir>
                  [--no-audit] [--with-raycast] [--seed N] [--port 4000] [--speed-mps V] [--fixed-dt DT]
                  [--det] [--server-arg=<switch>]... [--hit-dump] [--hit-dump-every N] [--raycast-every N]
                  [--max-route-m M] [--npc-cycle] [--sensor-layout <json>] [--udp-port-offset N]
                  [--hold-near=<x,y>] [--settle-ticks N] [--hold-ticks N]

Option values that start with "-" must be written with "=" (--server-arg=-rgl-scene-x=1, --hold-near=-12.5,3).

Starts the packaged server (the single <pkg>/*.sh launcher), drives a kinematic
ego (physics off, set_transform every tick at a constant speed) along the
scenario polyline with an RGL LiDAR, and stops the server by process group.
Outputs in <out>: server.stderr, server.stdout, run.txt (one key=value line),
tick_ms.txt (client world.tick() wall time per tick), and with --with-raycast
hits/ (the server's hit dumps) and raycast/<frame>.npz.

No ROS 2 is needed: the RGL LiDAR is created without a topic and nothing is
subscribed. Run this inside the venv that has the package's carla wheel.
Not runnable on the dev PC (needs a package and a GPU).
"""
import argparse
import dataclasses
import glob
import math
import os
import random
import signal
import socket
import subprocess
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", ".."))    # PythonAPI/rgl (lidar_models); HERE = PythonAPI/rgl/tests/wp_audit

import scenarios as sc_mod  # noqa: E402
import sensor_layout  # noqa: E402

RPC_READY_TIMEOUT_S = 240
CLIENT_TIMEOUT_S = 120.0
TIMEOUT_MARGIN_S = 300.0
STOP_TERM_WAIT_S = 60
S7_PHASE_TICKS = 200
RAYCAST_EVERY = 10  # default: keep every Nth ray_cast measurement (matches -rgl-debug-hit-dump-every)
HIT_DUMP_EVERY = 10
UDP_PORT_OFFSET = 40000
NPC_BLUEPRINT = "vehicle.lincoln.mkz_2020"   # also the ego fallback
NPC_AHEAD_M = 30.0
NPC_SPAWN_TICK, NPC_DESTROY_TICK, NPC_RESPAWN_TICK = 400, 800, 801
# D2: the deterministic set. The create budgets are wall-clock based; 0 removes them (the per-update count
# limit stays). A server that does not know a switch ignores it.
DET_SERVER_ARGS = ("-rgl-scene-create-budget-ms=0", "-rgl-scene-in-range-create-budget-ms=0")


class RunInvalid(Exception):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


# ---------------------------------------------------------------- output
def kv(**kw):
    return " ".join("%s=%s" % (k, str(v).replace(" ", "_")) for k, v in kw.items())


def write_run(out, **kw):
    with open(os.path.join(out, "run.txt"), "w", encoding="utf-8") as f:
        f.write(kv(**kw) + "\n")


# ---------------------------------------------------------------- pure helpers
def udp_ports_free(ports):
    """(True, None) when nothing holds any of the UDP ports, else (False, first busy port).

    The layout's sensors send to these ports; a receiver already bound there would swallow or mix the packets.
    """
    for p in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", int(p)))
        except OSError:
            return False, int(p)
        finally:
            s.close()
    return True, None


def lidar_preset_kwargs(det):
    """apply_preset keyword arguments: --det switches the Gaussian noise off (its random draws differ per run)."""
    return {"apply_noise": False} if det else {}


def det_server_args(det):
    return list(DET_SERVER_ARGS) if det else []


def npc_cycle_action(tick):
    """D3: spawn at tick 400, destroy at 800, spawn the same vehicle again at 801."""
    return {NPC_SPAWN_TICK: "spawn", NPC_DESTROY_TICK: "destroy", NPC_RESPAWN_TICK: "respawn"}.get(tick)


def build_server_args(launcher, port, audit, hit_dir, dump_every, extra):
    args = [launcher, "-RenderOffScreen", "-nosound", "-carla-rpc-port=%d" % port]
    if audit:
        args.append("-rgl-scene-audit=1")
    if hit_dir:
        args += ["-rgl-debug-hit-dump=" + hit_dir, "-rgl-debug-hit-dump-every=%d" % dump_every]
    return args + list(extra)


def merge_dash_values(argv, options=("--server-arg", "--hold-near")):
    """argparse refuses `--server-arg -rgl-x=1` (the value looks like an option); turn it into --server-arg=-rgl-x=1."""
    out, i = [], 0
    while i < len(argv):
        if argv[i] in options and i + 1 < len(argv) and argv[i + 1].startswith("-"):
            out.append("%s=%s" % (argv[i], argv[i + 1]))
            i += 2
        else:
            out.append(argv[i])
            i += 1
    return out


def parse_xy(text):
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError("expected x,y: %r" % text)
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        raise ValueError("expected x,y numbers: %r" % text) from None


def nearest_route_point(route, x, y):
    """(index, arc length [m] from the start) of the route vertex nearest to (x, y) in the horizontal plane."""
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("hold point must be finite")
    best, best_d = 0, None
    for i, p in enumerate(route):
        d = (p[0] - x) ** 2 + (p[1] - y) ** 2
        if best_d is None or d < best_d:
            best, best_d = i, d
    return best, sc_mod.route_length(route[: best + 1])


def apply_max_route(sc, max_route_m):
    """--max-route-m: use only the first max_route_m of the route; min_route_m can never exceed it."""
    if max_route_m is None:
        return sc
    return dataclasses.replace(sc, max_route_m=max_route_m, min_route_m=min(sc.min_route_m, max_route_m))


# ---------------------------------------------------------------- server
def port_listening(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


def find_launcher(pkg):
    cands = sorted(glob.glob(os.path.join(pkg, "*.sh")))
    if len(cands) != 1:
        raise RunInvalid("launcher_not_unique:%d" % len(cands))
    return cands[0]


def start_server(pkg, out, port, audit, dump_every, extra):
    """dump_every: None = no hit dump."""
    if port_listening(port):
        raise RunInvalid("port_busy")
    launcher = find_launcher(pkg)
    hits = None
    if dump_every:
        hits = os.path.join(out, "hits")
        os.makedirs(hits, exist_ok=True)
    args = build_server_args(launcher, port, audit, hits, dump_every, extra)
    so = open(os.path.join(out, "server.stdout"), "wb")
    se = open(os.path.join(out, "server.stderr"), "wb")
    # new session = new process group, so the whole launcher tree is stopped by killpg
    proc = subprocess.Popen(args, stdout=so, stderr=se, stdin=subprocess.DEVNULL,
                            start_new_session=True)
    return proc


def stop_server(proc):
    """Signal the whole process group. Verification is a separate call (verify_stopped)."""
    pgid = proc.pid  # start_new_session: pgid == pid
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return pgid
    try:
        proc.wait(timeout=STOP_TERM_WAIT_S)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    return pgid


def verify_stopped(pgid, port, wait_s=30):
    deadline = time.time() + wait_s
    while time.time() < deadline:
        try:
            os.killpg(pgid, 0)
            alive = True
        except ProcessLookupError:
            alive = False
        if not alive and not port_listening(port):
            return True
        time.sleep(1.0)
    return False


def wait_rpc(carla, port, proc):
    deadline = time.time() + RPC_READY_TIMEOUT_S
    last = None
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RunInvalid("server_died_rc%s" % proc.returncode)
        try:
            c = carla.Client("127.0.0.1", port)
            c.set_timeout(2.0)
            c.get_server_version()
            c.set_timeout(CLIENT_TIMEOUT_S)
            return c
        except RuntimeError as e:
            last = e
            time.sleep(2.0)
    raise RunInvalid("rpc_not_ready:%s" % type(last).__name__)


# ---------------------------------------------------------------- geometry
class Path:
    """Polyline with arc-length lookup. Ping-pong when `pingpong` is set."""

    def __init__(self, route, pingpong=False):
        self.pts = [tuple(map(float, p)) for p in route]
        self.cum = [0.0]
        for a, b in zip(self.pts[:-1], self.pts[1:]):
            self.cum.append(self.cum[-1] + math.dist(a, b))
        self.length = self.cum[-1]
        self.pingpong = pingpong

    def pose(self, s):
        """(x, y, z, pitch_deg, yaw_deg) at arc length s (clamped, or folded for ping-pong)."""
        L = self.length
        if self.pingpong:
            u = s % (2 * L)
            fwd = u <= L
            d = u if fwd else 2 * L - u
        else:
            fwd = True
            d = min(max(s, 0.0), L)
        i = 1
        while i < len(self.cum) - 1 and self.cum[i] < d:
            i += 1
        a, b = self.pts[i - 1], self.pts[i]
        seg = max(self.cum[i] - self.cum[i - 1], 1e-9)
        t = (d - self.cum[i - 1]) / seg
        p = [a[k] + (b[k] - a[k]) * t for k in range(3)]
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        if not fwd:
            dx, dy, dz = -dx, -dy, -dz
        yaw = math.degrees(math.atan2(dy, dx))
        pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        return p[0], p[1], p[2], pitch, yaw


# ---------------------------------------------------------------- carla helpers
def apply_world_settings(carla, world, sc):
    st = world.get_settings()
    st.synchronous_mode = True
    st.fixed_delta_seconds = sc_mod.FIXED_DT
    st.tile_stream_distance = float(sc.tile_stream_m)
    st.actor_active_distance = float(sc.actor_active_m)
    world.apply_settings(st)  # before the ego spawn


def make_lidar_bp(bp_lib, kind, det=False):
    from lidar_models import apply_preset, MODEL_REGISTRY
    m = MODEL_REGISTRY[sc_mod.LIDAR_MODEL]
    if kind == "rgl":
        bp = bp_lib.find("sensor.lidar.rgl")
        apply_preset(bp, sc_mod.LIDAR_MODEL, **lidar_preset_kwargs(det))
        bp.set_attribute("range", str(m["range"]))
    else:
        bp = bp_lib.find("sensor.lidar.ray_cast_semantic")
        for k in ("channels", "range", "upper_fov", "lower_fov", "points_per_second",
                  "rotation_frequency"):
            bp.set_attribute(k, str(m[k]))
    bp.set_attribute("sensor_tick", "0.1")
    return bp


def pick_ego_blueprint(bp_lib, wanted=None):
    """-> (blueprint, id, fallback). A wanted id that the package does not have falls back to the default car."""
    if wanted:
        found = [b for b in bp_lib.filter(wanted) if b.id == wanted]
        if found:
            return found[0], wanted, 0
    cands = bp_lib.filter(NPC_BLUEPRINT) or bp_lib.filter("vehicle.*")
    return cands[0], cands[0].id, int(bool(wanted))


def spawn_ego(carla, world, bp_lib, role, pose, max_ticks=100, wanted_bp=None, info=None):
    bp, bp_id, fallback = pick_ego_blueprint(bp_lib, wanted_bp)
    if info is not None:
        info["ego_bp"], info["ego_bp_fallback"] = bp_id, fallback
    bp.set_attribute("role_name", role)
    x, y, z, pitch, yaw = pose
    tf = carla.Transform(carla.Location(x, y, z + 0.5), carla.Rotation(pitch=pitch, yaw=yaw))
    for _ in range(max_ticks):  # WP tiles may still be streaming in
        ego = world.try_spawn_actor(bp, tf)
        if ego is not None:
            ego.set_simulate_physics(False)
            return ego
        world.tick()
    raise RunInvalid("ego_spawn_failed")


def tf6(carla, t):
    """(x, y, z, roll, pitch, yaw) [m, deg] -> carla.Transform"""
    return carla.Transform(carla.Location(x=t[0], y=t[1], z=t[2]),
                           carla.Rotation(roll=t[3], pitch=t[4], yaw=t[5]))


def spawn_layout_sensors(carla, world, bp_lib, layout, ego, det):
    """The bridge's chain: ego -> base_link -> sensor_kit (util.actor.empty), sensors on the actor they name."""
    from lidar_models import apply_preset
    base_link = world.spawn_actor(bp_lib.find("util.actor.empty"), tf6(carla, layout.base_link), attach_to=ego)
    actors = [base_link]
    sensor_kit = world.spawn_actor(bp_lib.find("util.actor.empty"), tf6(carla, layout.sensor_kit),
                                   attach_to=base_link)
    actors.append(sensor_kit)
    parents = {"ego": ego, "base_link": base_link, "sensor_kit": sensor_kit}
    for ls in layout.sensors:
        if ls.attach not in parents:
            raise RunInvalid("layout_attach:%s" % ls.attach)
        bp = bp_lib.find(ls.blueprint)
        if ls.model:
            apply_preset(bp, ls.model, **lidar_preset_kwargs(det))
        for k, v in ls.attributes.items():     # after the preset: the layout's values win
            bp.set_attribute(k, v)
        actors.append(world.spawn_actor(bp, tf6(carla, ls.transform), attach_to=parents[ls.attach]))
    return actors


def spawn_sensors(carla, world, bp_lib, sc, ego, pose, out, with_raycast, saver, opts=None):
    """Returns the list of spawned sensor actors for the scenario's sensor mode."""
    det = bool(opts and opts.det)
    if opts is not None and opts.layout is not None:
        return spawn_layout_sensors(carla, world, bp_lib, opts.layout, ego, det)
    sensors = []
    tf_roof = carla.Transform(carla.Location(z=sc_mod.LIDAR_Z))
    if sc.sensor_mode == "roadside":
        x, y, z, _, yaw = pose
        # 20 m to the right of the start point, fixed, no parent
        # CARLA right vector for heading yaw is (-sin yaw, cos yaw)
        rx = x - 20.0 * math.sin(math.radians(yaw))
        ry = y + 20.0 * math.cos(math.radians(yaw))
        tf = carla.Transform(carla.Location(rx, ry, z + sc_mod.LIDAR_Z))
        sensors.append(world.spawn_actor(make_lidar_bp(bp_lib, "rgl", det), tf))
    else:
        sensors.append(world.spawn_actor(make_lidar_bp(bp_lib, "rgl", det), tf_roof, attach_to=ego))
    if with_raycast:
        rc = world.spawn_actor(make_lidar_bp(bp_lib, "ray_cast"),
                               sensors[0].get_transform() if sc.sensor_mode == "roadside" else tf_roof,
                               attach_to=None if sc.sensor_mode == "roadside" else ego)
        rc.listen(saver)
        sensors.append(rc)
    return sensors


def make_raycast_saver(out, every=RAYCAST_EVERY):
    import numpy as np
    from compare_raycast import local_to_world
    d = os.path.join(out, "raycast")
    os.makedirs(d, exist_ok=True)
    counter = {"n": 0}

    def saver(m):
        counter["n"] += 1
        if (counter["n"] - 1) % every:
            return
        n = len(m)
        loc = np.empty((n, 3), dtype=np.float32)
        tag = np.empty(n, dtype=np.int32)
        for i, det in enumerate(m):
            loc[i] = (det.point.x, det.point.y, det.point.z)
            tag[i] = det.object_tag
        t = m.transform
        xyz = local_to_world(loc, (t.location.x, t.location.y, t.location.z),
                             (t.rotation.roll, t.rotation.pitch, t.rotation.yaw))
        np.savez(os.path.join(d, "%08d.npz" % m.frame), xyz=xyz.astype(np.float32), tag=tag,
                 sim_t=np.float64(m.timestamp))
    return saver


def destroy_all(actors):
    for a in actors:
        try:
            if hasattr(a, "stop"):
                a.stop()
        except Exception:
            pass
        try:
            a.destroy()
        except Exception:
            pass


# ---------------------------------------------------------------- scenario bodies
def check_route(sc, route, map_name):
    if not sc_mod.route_is_set(route):
        raise RunInvalid(sc_mod.TODO_MARK)
    if sc_mod.route_length(route) < sc.min_route_m - 1e-3:   # a cut route is as long as asked up to float rounding
        raise RunInvalid("route_too_short")
    if sc.name == "S5" and sc_mod.route_length(route) / sc.speed_mps <= 61.0:
        raise RunInvalid("route_too_short_for_gc")


def tick_timed(world, tick_log):
    t0 = time.perf_counter()
    world.tick()
    tick_log.append((time.perf_counter() - t0) * 1000.0)


def spawn_npc(world, bp_lib, tf):
    bp = pick_ego_blueprint(bp_lib)[0]   # NPC_BLUEPRINT, or the first vehicle of a slim package that lacks it
    bp.set_attribute("role_name", "npc_cycle")
    npc = world.try_spawn_actor(bp, tf)
    if npc is None:
        raise RunInvalid("npc_spawn_failed")
    npc.set_simulate_physics(False)
    return npc


def drive(carla, world, bp_lib, sc, route, out, with_raycast, tick_log, deadline, opts):
    path = Path(route, sc.pingpong)
    n_ticks = sc_mod.scenario_ticks(sc, route)
    saver = make_raycast_saver(out, opts.raycast_every) if with_raycast else None
    pose0 = path.pose(opts.hold_s if opts.hold_s is not None else 0.0)
    layout_bp = opts.layout.ego_blueprint if opts.layout is not None else None
    ego = spawn_ego(carla, world, bp_lib, sc.ego_role, pose0, wanted_bp=layout_bp, info=opts.info)
    actors = [ego]
    actors += spawn_sensors(carla, world, bp_lib, sc, ego, pose0, out, with_raycast, saver, opts)
    s_offset = 0.0
    respawned = False
    npc, npc_tf = None, None
    try:
        for k in range(n_ticks):
            if time.time() > deadline:
                raise RunInvalid("timeout")
            frac = k / max(n_ticks, 1)
            if "teleport_ahead_1500" in sc.events and frac >= 1 / 3 and s_offset == 0.0:
                s_offset = 1500.0
            if "teleport_back" in sc.events and frac >= 2 / 3 and s_offset == 1500.0:
                s_offset = 0.0
            if "respawn" in sc.events and frac >= 5 / 6 and not respawned:
                respawned = True
                destroy_all(actors)
                pose = path.pose(sc.speed_mps * k * sc_mod.FIXED_DT + s_offset)
                ego = spawn_ego(carla, world, bp_lib, sc.ego_role, pose, wanted_bp=layout_bp, info=opts.info)
                actors = [ego] + spawn_sensors(carla, world, bp_lib, sc, ego, pose, out,
                                               with_raycast, saver, opts)
            if "second_sensor_at_tick_10" in sc.events and k == 10:
                actors.append(world.spawn_actor(
                    make_lidar_bp(bp_lib, "rgl", opts.det),
                    carla.Transform(carla.Location(x=0.5, z=sc_mod.LIDAR_Z)), attach_to=ego))
            if opts.hold_s is not None:
                s = opts.hold_s        # STATIC: the ego never moves
            else:
                s = sc.speed_mps * k * sc_mod.FIXED_DT + s_offset
            x, y, z, pitch, yaw = path.pose(s)
            action = npc_cycle_action(k) if opts.npc_cycle else None
            if action == "spawn":      # D3: 30 m ahead on the route, physics off; the same spot is used again at 801
                nx, ny, nz, npitch, nyaw = path.pose(s + NPC_AHEAD_M)
                npc_tf = carla.Transform(carla.Location(nx, ny, nz + 0.5), carla.Rotation(pitch=npitch, yaw=nyaw))
            if action in ("spawn", "respawn"):
                npc = spawn_npc(world, bp_lib, npc_tf)
            elif action == "destroy":
                destroy_all([npc])
                npc = None
            ego.set_transform(carla.Transform(carla.Location(x, y, z + 0.5),
                                              carla.Rotation(pitch=pitch, yaw=yaw)))
            tick_timed(world, tick_log)
    finally:
        destroy_all(actors + ([npc] if npc is not None else []))
    return n_ticks


def run_load_world_cycle(carla, client, bp_lib_of, sc, out, with_raycast, tick_log, deadline, opts):
    """S7: Town15 <-> Odaiba load_world, 3 round trips."""
    ticks = 0
    for _ in range(3):
        for m in ("Town15", "Odaiba"):
            route = sc_mod.route_for(sc.name, m)
            if not sc_mod.route_is_set(route):
                raise RunInvalid(sc_mod.TODO_MARK)
            world = client.load_world(m)
            apply_world_settings(carla, world, sc)
            bp_lib = world.get_blueprint_library()
            path = Path(route)
            pose0 = path.pose(0.0)
            ego = spawn_ego(carla, world, bp_lib, sc.ego_role, pose0, info=opts.info)
            sensors = spawn_sensors(carla, world, bp_lib, sc, ego, pose0, out, False, None, opts)
            try:
                for k in range(S7_PHASE_TICKS):
                    if time.time() > deadline:
                        raise RunInvalid("timeout")
                    x, y, z, pitch, yaw = path.pose(sc.speed_mps * k * sc_mod.FIXED_DT)
                    ego.set_transform(carla.Transform(carla.Location(x, y, z + 0.5),
                                                      carla.Rotation(pitch=pitch, yaw=yaw)))
                    tick_timed(world, tick_log)
                    ticks += 1
            finally:
                destroy_all([ego] + sensors)
                st = world.get_settings()
                st.synchronous_mode = False
                world.apply_settings(st)
    return ticks


# ---------------------------------------------------------------- main
def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--pkg", required=True)
    ap.add_argument("--map", required=True, choices=["Odaiba", "Town15", "Town10HD_Opt"])
    ap.add_argument("--scenario", required=True, choices=sorted(sc_mod.SCENARIOS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--no-audit", action="store_true")
    ap.add_argument("--with-raycast", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--port", type=int, default=4000)
    ap.add_argument("--speed-mps", type=float, default=None,
                    help="override the scenario's ego speed (e.g. the range-edge gap P8 grows with speed)")
    ap.add_argument("--fixed-dt", type=float, default=None,
                    help="override fixed_delta_seconds (the end-to-end runs use 40 fps = 0.025; the RGL sync timer "
                         "only advances on frames where a sensor updates, so dt changes the sync interval)")
    ap.add_argument("--sensor-layout", default=None, metavar="JSON",
                    help="spawn the ego and sensors of this layout file (sensor_layout.py) instead of one LiDAR")
    ap.add_argument("--udp-port-offset", type=int, default=UDP_PORT_OFFSET,
                    help="added to every rgl_udp_dest_port of the layout (default %(default)s)")
    ap.add_argument("--det", action="store_true",
                    help="D2 deterministic set: server create budgets 0 and LiDAR noise off")
    ap.add_argument("--server-arg", action="append", default=[], metavar="SWITCH",
                    help="extra server switch, repeatable; write --server-arg=-rgl-scene-x=1")
    ap.add_argument("--hit-dump", action="store_true", help="have the server write hit dumps to <out>/hits")
    ap.add_argument("--hit-dump-every", type=int, default=HIT_DUMP_EVERY,
                    help="dump every Nth scan (default %(default)s)")
    ap.add_argument("--raycast-every", type=int, default=RAYCAST_EVERY,
                    help="with --with-raycast: keep every Nth ray_cast scan and dump every Nth RGL scan "
                         "(default %(default)s)")
    ap.add_argument("--max-route-m", type=float, default=None,
                    help="use only the first M metres of the scenario's route (min_route_m becomes min(old, M))")
    ap.add_argument("--npc-cycle", action="store_true",
                    help="D3: spawn an NPC 30 m ahead at tick 400, destroy it at 800, spawn it again at 801")
    ap.add_argument("--hold-near", default=None, metavar="X,Y",
                    help="STATIC only: park the ego at the route point nearest to (x, y); write --hold-near=-1,2")
    ap.add_argument("--settle-ticks", type=int, default=200, help="STATIC: ticks before the hold (default %(default)s)")
    ap.add_argument("--hold-ticks", type=int, default=20, help="STATIC: ticks of the hold (default %(default)s)")
    a = ap.parse_args(merge_dash_values(argv))

    if a.hold_near is not None and a.scenario != "STATIC":
        ap.error("--hold-near is for --scenario STATIC only")
    hold_xy = None
    if a.hold_near is not None:
        try:
            hold_xy = parse_xy(a.hold_near)
        except ValueError as e:
            ap.error("--hold-near: %s" % e)
    if a.settle_ticks < 0 or a.hold_ticks < 1:
        ap.error("--settle-ticks must be >= 0 and --hold-ticks >= 1")
    if a.max_route_m is not None and a.max_route_m <= 0:
        ap.error("--max-route-m must be > 0")
    if a.hit_dump_every < 1 or a.raycast_every < 1:
        ap.error("--hit-dump-every and --raycast-every must be >= 1")
    if a.with_raycast and a.hit_dump and a.hit_dump_every != a.raycast_every:
        ap.error("--with-raycast dumps every --raycast-every scans; --hit-dump-every must agree")
    if a.with_raycast and a.sensor_layout:
        ap.error("--with-raycast and --sensor-layout cannot be combined")
    if not 0 <= a.udp_port_offset <= 65535:
        ap.error("--udp-port-offset must be in [0, 65535]")

    os.makedirs(a.out, exist_ok=True)
    random.seed(a.seed)
    sc = sc_mod.SCENARIOS[a.scenario]
    if a.speed_mps is not None:
        if a.speed_mps <= 0:
            ap.error("--speed-mps must be > 0")
        sc = dataclasses.replace(sc, speed_mps=a.speed_mps)
    if a.fixed_dt is not None:
        if not 0.001 <= a.fixed_dt <= 0.1:
            ap.error("--fixed-dt must be in [0.001, 0.1]")
        sc_mod.FIXED_DT = a.fixed_dt   # every user reads sc_mod.FIXED_DT at call time
    sc = apply_max_route(sc, a.max_route_m)
    if sc.hold:
        sc = dataclasses.replace(sc, fixed_ticks=a.settle_ticks + a.hold_ticks)
    dump_every = (a.raycast_every if a.with_raycast else a.hit_dump_every) if (a.with_raycast or a.hit_dump) else None
    server_extra = det_server_args(a.det) + list(a.server_arg)
    wp_map = a.map == "Town15"
    expected = sc.radius_expected_m
    if expected is None:
        exp_str = "nan"
    else:
        exp_str = "%.1f" % (expected if wp_map else -1.0)
    base = dict(scenario=a.scenario, map=a.map, seed=a.seed, port=a.port,
                no_audit=int(a.no_audit), with_raycast=int(a.with_raycast),
                radius_expected_m=exp_str, speed_mps="%.1f" % sc.speed_mps, fixed_dt="%.3f" % sc_mod.FIXED_DT)
    info = dict(ego_bp="na", ego_bp_fallback=0, hold_at_m="na")
    t_start = time.time()
    proc, pgid = None, None
    tick_log = []
    status, reason, ticks = "ok", "", 0
    try:
        layout = None
        if a.sensor_layout:
            try:
                layout = sensor_layout.load_layout(a.sensor_layout, a.udp_port_offset)
            except (OSError, ValueError) as e:
                raise RunInvalid("layout_invalid:%s" % str(e).replace(" ", "_")[:120])
        opts = types.SimpleNamespace(det=a.det, layout=layout, npc_cycle=a.npc_cycle, hold_s=None,
                                     raycast_every=a.raycast_every, info=info)
        route = sc_mod.route_for(sc.name, a.map, sc)
        if sc.name == "S7":
            if not (sc_mod.route_is_set(sc_mod.ROUTES.get("Town15")) and
                    sc_mod.route_is_set(sc_mod.ROUTES.get("Odaiba"))):
                raise RunInvalid(sc_mod.TODO_MARK)
            duration = sc.fixed_duration_s
        else:
            check_route(sc, route, a.map)  # route_not_set is detected before any server starts
            duration = sc_mod.scenario_duration_s(sc, route)
        if sc.hold:
            _, hold_s = nearest_route_point(route, *(hold_xy if hold_xy is not None else route[0][:2]))
            opts.hold_s = hold_s
            info["hold_at_m"] = "%.1f" % hold_s
        deadline = t_start + duration + TIMEOUT_MARGIN_S
        if layout is not None:
            free, busy = udp_ports_free(sensor_layout.udp_ports(layout))
            if not free:
                raise RunInvalid("udp_port_busy:%d" % busy)

        import carla  # noqa: E402  (venv with the package wheel)
        proc = start_server(a.pkg, a.out, a.port, not a.no_audit, dump_every, server_extra)
        pgid = proc.pid
        client = wait_rpc(carla, a.port, proc)
        world = client.load_world(a.map)
        apply_world_settings(carla, world, sc)
        bp_lib = world.get_blueprint_library()
        world.tick()
        if sc.name == "S7":
            ticks = run_load_world_cycle(carla, client, None, sc, a.out, a.with_raycast,
                                         tick_log, deadline, opts)
        else:
            ticks = drive(carla, world, bp_lib, sc, route, a.out, a.with_raycast,
                          tick_log, deadline, opts)
        try:
            st = client.get_world().get_settings()
            st.synchronous_mode = False
            client.get_world().apply_settings(st)
        except Exception:
            pass
    except RunInvalid as e:
        status, reason = "invalid", e.reason
    except Exception as e:  # noqa: BLE001
        status, reason = "error", "%s" % type(e).__name__
        import traceback
        traceback.print_exc()   # the run.txt line keeps only the type; the stack goes to the runner's stderr
    finally:
        stopped = "na"
        if proc is not None:
            pgid = stop_server(proc)
        # verification is a separate call from the stop
        if pgid is not None:
            stopped = int(verify_stopped(pgid, a.port))
            if stopped == 0 and status == "ok":
                status, reason = "invalid", "server_not_stopped"
        with open(os.path.join(a.out, "tick_ms.txt"), "w", encoding="utf-8") as f:
            f.write("".join("%.3f\n" % v for v in tick_log))
        new = dict(det=int(a.det), layout=os.path.basename(a.sensor_layout) if a.sensor_layout else "none",
                   ego_bp=info["ego_bp"], ego_bp_fallback=info["ego_bp_fallback"],
                   udp_port_offset=a.udp_port_offset if a.sensor_layout else 0,
                   hit_dump_every=dump_every or 0, npc_cycle=int(a.npc_cycle), hold_at_m=info["hold_at_m"],
                   server_args=",".join(server_extra) or "none")
        extra = dict(ticks=ticks, elapsed_s="%.1f" % (time.time() - t_start),
                     server_stopped=stopped, pkg=a.pkg)
        if reason:
            extra["reason"] = reason
        write_run(a.out, status=status, **base, **new, **extra)
        print(kv(status=status, **base, **new, **extra))
    return 0 if status == "ok" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
