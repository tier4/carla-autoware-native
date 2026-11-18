#!/usr/bin/env python3
"""World Partition scene-audit scenario runner (stage 0).

  run_scenario.py --pkg <dir> --map <Odaiba|Town15|Town10HD_Opt>
                  --scenario <S1|S1p|S2|S3|S4|S5|S6|S7|CONTROL> --out <dir>
                  [--no-audit] [--with-raycast] [--seed N] [--port 4000] [--speed-mps V] [--fixed-dt DT]

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

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", ".."))    # PythonAPI/rgl (lidar_models); HERE = PythonAPI/rgl/tests/wp_audit

import scenarios as sc_mod  # noqa: E402

RPC_READY_TIMEOUT_S = 240
CLIENT_TIMEOUT_S = 120.0
TIMEOUT_MARGIN_S = 300.0
STOP_TERM_WAIT_S = 60
S7_PHASE_TICKS = 200
RAYCAST_EVERY = 10  # keep every Nth ray_cast measurement (matches -rgl-debug-hit-dump-every)


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


def start_server(pkg, out, port, audit, with_raycast):
    if port_listening(port):
        raise RunInvalid("port_busy")
    launcher = find_launcher(pkg)
    args = [launcher, "-RenderOffScreen", "-nosound", "-carla-rpc-port=%d" % port]
    if audit:
        args.append("-rgl-scene-audit=1")
    if with_raycast:
        hits = os.path.join(out, "hits")
        os.makedirs(hits, exist_ok=True)
        args += ["-rgl-debug-hit-dump=" + hits, "-rgl-debug-hit-dump-every=%d" % RAYCAST_EVERY]
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


def make_lidar_bp(bp_lib, kind):
    from lidar_models import apply_preset, MODEL_REGISTRY
    m = MODEL_REGISTRY[sc_mod.LIDAR_MODEL]
    if kind == "rgl":
        bp = bp_lib.find("sensor.lidar.rgl")
        apply_preset(bp, sc_mod.LIDAR_MODEL)
        bp.set_attribute("range", str(m["range"]))
    else:
        bp = bp_lib.find("sensor.lidar.ray_cast_semantic")
        for k in ("channels", "range", "upper_fov", "lower_fov", "points_per_second",
                  "rotation_frequency"):
            bp.set_attribute(k, str(m[k]))
    bp.set_attribute("sensor_tick", "0.1")
    return bp


def spawn_ego(carla, world, bp_lib, role, pose, max_ticks=100):
    cands = bp_lib.filter("vehicle.lincoln.mkz_2020") or bp_lib.filter("vehicle.*")
    bp = cands[0]
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


def spawn_sensors(carla, world, bp_lib, sc, ego, pose, out, with_raycast, saver):
    """Returns the list of spawned sensor actors for the scenario's sensor mode."""
    sensors = []
    tf_roof = carla.Transform(carla.Location(z=sc_mod.LIDAR_Z))
    if sc.sensor_mode == "roadside":
        x, y, z, _, yaw = pose
        # 20 m to the right of the start point, fixed, no parent
        # CARLA right vector for heading yaw is (-sin yaw, cos yaw)
        rx = x - 20.0 * math.sin(math.radians(yaw))
        ry = y + 20.0 * math.cos(math.radians(yaw))
        tf = carla.Transform(carla.Location(rx, ry, z + sc_mod.LIDAR_Z))
        sensors.append(world.spawn_actor(make_lidar_bp(bp_lib, "rgl"), tf))
    else:
        sensors.append(world.spawn_actor(make_lidar_bp(bp_lib, "rgl"), tf_roof, attach_to=ego))
    if with_raycast:
        rc = world.spawn_actor(make_lidar_bp(bp_lib, "ray_cast"),
                               sensors[0].get_transform() if sc.sensor_mode == "roadside" else tf_roof,
                               attach_to=None if sc.sensor_mode == "roadside" else ego)
        rc.listen(saver)
        sensors.append(rc)
    return sensors


def make_raycast_saver(out):
    import numpy as np
    from compare_raycast import local_to_world
    d = os.path.join(out, "raycast")
    os.makedirs(d, exist_ok=True)
    counter = {"n": 0}

    def saver(m):
        counter["n"] += 1
        if (counter["n"] - 1) % RAYCAST_EVERY:
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
    if sc_mod.route_length(route) < sc.min_route_m:
        raise RunInvalid("route_too_short")
    if sc.name == "S5" and sc_mod.route_length(route) / sc.speed_mps <= 61.0:
        raise RunInvalid("route_too_short_for_gc")


def tick_timed(world, tick_log):
    t0 = time.perf_counter()
    world.tick()
    tick_log.append((time.perf_counter() - t0) * 1000.0)


def drive(carla, world, bp_lib, sc, route, out, with_raycast, tick_log, deadline):
    path = Path(route, sc.pingpong)
    duration = sc_mod.scenario_duration_s(sc, route)
    n_ticks = int(math.ceil(duration / sc_mod.FIXED_DT))
    saver = make_raycast_saver(out) if with_raycast else None
    pose0 = path.pose(0.0)
    ego = spawn_ego(carla, world, bp_lib, sc.ego_role, pose0)
    actors = [ego]
    actors += spawn_sensors(carla, world, bp_lib, sc, ego, pose0, out, with_raycast, saver)
    s_offset = 0.0
    respawned = False
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
                ego = spawn_ego(carla, world, bp_lib, sc.ego_role, pose)
                actors = [ego] + spawn_sensors(carla, world, bp_lib, sc, ego, pose, out,
                                               with_raycast, saver)
            if "second_sensor_at_tick_10" in sc.events and k == 10:
                actors.append(world.spawn_actor(
                    make_lidar_bp(bp_lib, "rgl"),
                    carla.Transform(carla.Location(x=0.5, z=sc_mod.LIDAR_Z)), attach_to=ego))
            s = sc.speed_mps * k * sc_mod.FIXED_DT + s_offset
            x, y, z, pitch, yaw = path.pose(s)
            ego.set_transform(carla.Transform(carla.Location(x, y, z + 0.5),
                                              carla.Rotation(pitch=pitch, yaw=yaw)))
            tick_timed(world, tick_log)
    finally:
        destroy_all(actors)
    return n_ticks


def run_load_world_cycle(carla, client, bp_lib_of, sc, out, with_raycast, tick_log, deadline):
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
            ego = spawn_ego(carla, world, bp_lib, sc.ego_role, pose0)
            sensors = spawn_sensors(carla, world, bp_lib, sc, ego, pose0, out, False, None)
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
                    help="override fixed_delta_seconds (the X2 E2E runs at 40 fps = 0.025; the RGL sync timer "
                         "only advances on frames where a sensor updates, so dt changes the sync interval)")
    a = ap.parse_args(argv)

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
    wp_map = a.map == "Town15"
    expected = sc.radius_expected_m
    if expected is None:
        exp_str = "nan"
    else:
        exp_str = "%.1f" % (expected if wp_map else -1.0)
    base = dict(scenario=a.scenario, map=a.map, seed=a.seed, port=a.port,
                no_audit=int(a.no_audit), with_raycast=int(a.with_raycast),
                radius_expected_m=exp_str, speed_mps="%.1f" % sc.speed_mps, fixed_dt="%.3f" % sc_mod.FIXED_DT)
    t_start = time.time()
    proc, pgid = None, None
    tick_log = []
    status, reason, ticks = "ok", "", 0
    try:
        route = sc_mod.route_for(sc.name, a.map)
        if sc.name == "S7":
            if not (sc_mod.route_is_set(sc_mod.ROUTES.get("Town15")) and
                    sc_mod.route_is_set(sc_mod.ROUTES.get("Odaiba"))):
                raise RunInvalid(sc_mod.TODO_MARK)
            duration = sc.fixed_duration_s
        else:
            check_route(sc, route, a.map)  # route_not_set is detected before any server starts
            duration = sc_mod.scenario_duration_s(sc, route)
        deadline = t_start + duration + TIMEOUT_MARGIN_S

        import carla  # noqa: E402  (venv with the package wheel)
        proc = start_server(a.pkg, a.out, a.port, not a.no_audit, a.with_raycast)
        pgid = proc.pid
        client = wait_rpc(carla, a.port, proc)
        world = client.load_world(a.map)
        apply_world_settings(carla, world, sc)
        bp_lib = world.get_blueprint_library()
        world.tick()
        if sc.name == "S7":
            ticks = run_load_world_cycle(carla, client, None, sc, a.out, a.with_raycast,
                                         tick_log, deadline)
        else:
            ticks = drive(carla, world, bp_lib, sc, route, a.out, a.with_raycast,
                          tick_log, deadline)
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
        extra = dict(ticks=ticks, elapsed_s="%.1f" % (time.time() - t_start),
                     server_stopped=stopped, pkg=a.pkg)
        if reason:
            extra["reason"] = reason
        write_run(a.out, status=status, **base, **extra)
        print(kv(status=status, **base, **extra))
    return 0 if status == "ok" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
