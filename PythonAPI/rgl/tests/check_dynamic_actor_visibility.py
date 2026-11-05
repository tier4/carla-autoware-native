#!/usr/bin/env python3
"""GATE (skeletal spec §7 stage 3): are dynamic actors visible to the RGL lidar?

Method (fixes the 2026-07 false positive, spec §0.4):
  * FRESH samples only: a cloud is accepted iff it arrived after world.tick(), its
    header.stamp (RGL scene time) is strictly newer than the last accepted one, AND it
    matches the current tick's sim time to within half a step (a queued older sweep is
    also "newer", so recency alone is not enough).
    Timeout => MEASUREMENT FAILURE (exit 2). Never silently reused.
  * Equal sample counts; statistic = MEDIAN.
  * NEGATIVE CONTROL first (fixed ROI, nothing spawned, two baselines must agree within
    max(10 pts, 5 %)). Per-target: two baselines on the target's own ROI must agree too.
  * Measurement starts >= one scene-sync period (1 s) + settle after each spawn/destroy.
  * ROI = spawned actor's bounding box (XY +-0.3 m), Z in [ground+0.35, ground+2.15] m.
    Sensor at world (0,0,1.65) yaw 0 => world == sensor frame + (0,0,1.65).
  * delta = median(measured) - median(baseline).  PASS iff every REQUIRED target passes;
    a missing/unspawnable required target is exit 2, not PASS.

Requires ROS2 sourced (rclpy, sensor_msgs): sensor.listen() segfaults for sensor.lidar.rgl.
Usage:
    source /opt/ros/humble/setup.bash
    python3 check_dynamic_actor_visibility.py [--host H] [--port P] [--label NAME]
        [--targets vehicle.ue4.audi.tt,vehicle.ue4.ford.mustang,vehicle.lincoln.mkz,walker.pedestrian.0016]
        [--veh-min 200] [--walk-min 40] [--ticks 30] [--json]
"""
import argparse, json, math, struct, sys, time
try:
    import carla
except ImportError as e:
    print(f"ERROR: cannot import carla ({e}); check PYTHONPATH.", file=sys.stderr); sys.exit(2)
try:
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
except ImportError:
    print("ERROR: rclpy/sensor_msgs not importable; source ROS2 first.", file=sys.stderr); sys.exit(2)

DEFAULT_TARGETS = "vehicle.ue4.audi.tt,vehicle.ue4.ford.mustang,vehicle.lincoln.mkz,walker.pedestrian.0016"
SENSOR_Z, FRONT_M, ROI_XY_MARGIN, Z_LO, Z_HI = 1.65, 10.0, 0.3, 0.35, 2.15
SYNC_PERIOD_TICKS, SETTLE_TICKS = 20, 5
ROS2_TOPIC, TICK_WAIT_TIMEOUT_S = "/gate/rgl_dynamic_actor_visibility", 2.0
CHANNELS, PPS, ROT_HZ, UPPER, LOWER = 64, 1_300_000, 20, 15.0, -25.0
FIXED_DELTA = 1.0 / ROT_HZ          # synchronous-mode step; one sweep per tick


class MeasurementError(RuntimeError):
    pass


def stamp_s(msg): return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
def median(v):
    s = sorted(v); n = len(s)
    return 0 if n == 0 else (s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2]))
def parse_xyz(msg):
    pts, off = [], 0
    for _ in range(msg.width):
        x, y, z = struct.unpack_from("<fff", msg.data, off); pts.append((x, y, z + SENSOR_Z)); off += msg.point_step
    return pts
def count_in_roi(points, roi):
    x0, x1, y0, y1, z0, z1 = roi
    return sum(1 for (x, y, z) in points if x0 <= x <= x1 and y0 <= y <= y1 and z0 <= z <= z1)
def silhouette_rays(extent_y_m, height_m, dist_m):
    h_res = 360.0 / ((PPS / ROT_HZ) / CHANNELS); v_res = (UPPER - LOWER) / (CHANNELS - 1)
    w = math.degrees(2 * math.atan(extent_y_m / dist_m)); h = math.degrees(2 * math.atan(0.5 * height_m / dist_m))
    return int((w / h_res) * (h / v_res))
def stable(a, b): return abs(b - a) <= max(10, 0.05 * a)


class Gate:
    def __init__(self, world, node, ticks):
        self.world, self.node, self.ticks = world, node, ticks
        self.latest, self.last_stamp = {"msg": None, "seq": 0}, -1.0
        qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT; qos.durability = DurabilityPolicy.VOLATILE
        node.create_subscription(PointCloud2, ROS2_TOPIC, self._on_msg, qos)
    def _on_msg(self, msg): self.latest = {"msg": msg, "seq": self.latest["seq"] + 1}
    def sim_time(self):
        """Scene time of the tick that just completed. RGL stamps its ROS2 header with the
        CARLA SimulationTime it was given, so this is directly comparable to stamp_s(msg)."""
        return self.world.get_snapshot().timestamp.elapsed_seconds
    def warmup(self, max_ticks=400):
        """Tick until the first cloud arrives (ROS2 discovery + first sweep).
        Synchronous mode only advances on tick, so keep ticking while spinning.
        Also verifies the stamp convention: tick-matched freshness is only meaningful if
        header.stamp tracks sim time. Raises MeasurementError if no cloud arrives within
        max_ticks, or if the first stamp is nowhere near the sim clock."""
        for i in range(max_ticks):
            self.world.tick()
            t_sim = self.sim_time()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.latest["msg"] is not None:
                self.last_stamp = stamp_s(self.latest["msg"])
                print(f"warm-up: first cloud after {i + 1} ticks (stamp {self.last_stamp:.3f} sim {t_sim:.3f})")
                if abs(self.last_stamp - t_sim) > 2.0:
                    raise MeasurementError(
                        f"cloud stamp {self.last_stamp:.3f} does not track sim time {t_sim:.3f}; "
                        f"cannot do tick-matched freshness")
                return
        raise MeasurementError(f"no point cloud after {max_ticks} warm-up ticks")
    def tick_fresh(self):
        """Tick once and return the cloud belonging to THAT tick. A message that is merely
        newer than the last accepted one may still be a queued older sweep, so it must also
        match the current scene time to within half a step."""
        seq0 = self.latest["seq"]; self.world.tick(); t_sim = self.sim_time(); t0 = time.time()
        while time.time() - t0 < TICK_WAIT_TIMEOUT_S:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.latest["seq"] != seq0:
                seq0 = self.latest["seq"]
                msg = self.latest["msg"]; st = stamp_s(msg)
                if st > self.last_stamp and abs(st - t_sim) <= 0.5 * FIXED_DELTA:
                    self.last_stamp = st; return msg
        raise MeasurementError(
            f"timeout waiting for the cloud of sim time {t_sim:.3f} (last accepted stamp {self.last_stamp:.3f})")
    def skip(self, n):
        for _ in range(n): self.tick_fresh()
    def measure(self, roi): return median([count_in_roi(parse_xyz(self.tick_fresh()), roi) for _ in range(self.ticks)])


def roi_for(actor, ground_z):
    verts = actor.bounding_box.get_world_vertices(actor.get_transform())
    xs, ys = [v.x for v in verts], [v.y for v in verts]
    return (min(xs) - ROI_XY_MARGIN, max(xs) + ROI_XY_MARGIN, min(ys) - ROI_XY_MARGIN, max(ys) + ROI_XY_MARGIN, ground_z + Z_LO, ground_z + Z_HI), actor.bounding_box


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--label", default=""); ap.add_argument("--targets", default=DEFAULT_TARGETS)
    ap.add_argument("--veh-min", type=int, default=200); ap.add_argument("--walk-min", type=int, default=40)
    ap.add_argument("--ticks", type=int, default=30); ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    targets = [t.strip() for t in args.targets.split(",") if t.strip()]
    if args.ticks < 5 or not targets or args.veh_min < 0 or args.walk_min < 0:
        print("ERROR: invalid arguments (ticks>=5, non-empty targets, non-negative thresholds)", file=sys.stderr); return 2

    # Everything that can fail on a dead/absent server lives inside the try below, so a
    # connection failure is reported as a measurement failure (exit 2), not a traceback.
    world = None; original = None; node = None; actors = []; results = []; rc = 0

    def spawn_or_fail(bp_obj, tf, what):
        try:
            a = world.try_spawn_actor(bp_obj, tf)
        except Exception as e:
            raise MeasurementError(f"spawn raised for {what}: {e}")
        if a is None:
            raise MeasurementError(f"spawn failed: {what}")
        return a

    try:
        client = carla.Client(args.host, args.port); client.set_timeout(20.0); world = client.get_world()
        original = world.get_settings()
        s = world.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = FIXED_DELTA; world.apply_settings(s)
        rclpy.init(); node = rclpy.create_node("rgl_dynamic_actor_visibility_gate"); gate = Gate(world, node, args.ticks)
        bp = world.get_blueprint_library()
        try: lidar_bp = bp.find("sensor.lidar.rgl")
        except Exception: raise MeasurementError("required blueprint not found: sensor.lidar.rgl")
        for k, v in {"channels": CHANNELS, "range": 50, "upper_fov": UPPER, "lower_fov": LOWER, "points_per_second": PPS,
                     "rotation_frequency": ROT_HZ, "sensor_tick": 1.0 / ROT_HZ, "rgl_lidar_topic_name": ROS2_TOPIC,
                     "rgl_lidar_topic_frame_id": "lidar", "rgl_lidar_pointcloud_format": "PointXYZIRCAEDT"}.items():
            lidar_bp.set_attribute(k, str(v))
        actors.append(spawn_or_fail(lidar_bp, carla.Transform(carla.Location(0.0, 0.0, SENSOR_Z)), "sensor.lidar.rgl"))
        gate.warmup()
        gate.skip(SYNC_PERIOD_TICKS + SETTLE_TICKS)
        gp = world.ground_projection(carla.Location(FRONT_M, 0.0, 5.0), 20.0); ground_z = gp.location.z if gp is not None else 0.0

        nc_roi = (FRONT_M - 3.0, FRONT_M + 3.0, -1.5, 1.5, ground_z + Z_LO, ground_z + Z_HI)
        b1, b2 = gate.measure(nc_roi), gate.measure(nc_roi)
        print(f"[{args.label}] negative control: b1={b1} b2={b2} -> {'OK' if stable(b1, b2) else 'UNSTABLE'}")
        if not stable(b1, b2): raise MeasurementError("fixture unstable with nothing spawned")

        for bp_id in targets:
            try: target_bp = bp.find(bp_id)
            except Exception: raise MeasurementError(f"required blueprint not found: {bp_id}")
            is_walker = bp_id.startswith("walker.")
            z_off = 1.0 if is_walker else 0.3
            tf = carla.Transform(carla.Location(FRONT_M, 0.0, ground_z + z_off))
            print(f"[{args.label}] spawned {bp_id} at z={ground_z + z_off:.2f}")
            probe = spawn_or_fail(target_bp, tf, bp_id)
            actors.append(probe); gate.skip(SETTLE_TICKS); roi, bb = roi_for(probe, ground_z)
            probe.destroy(); actors.remove(probe); gate.skip(SYNC_PERIOD_TICKS + SETTLE_TICKS)
            base1, base2 = gate.measure(roi), gate.measure(roi)
            if not stable(base1, base2): raise MeasurementError(f"baseline unstable for {bp_id}: {base1} vs {base2}")
            base = median([base1, base2])
            actor = spawn_or_fail(target_bp, tf, bp_id)
            actors.append(actor); gate.skip(SYNC_PERIOD_TICKS + SETTLE_TICKS)
            meas = gate.measure(roi)
            actor.destroy(); actors.remove(actor); gate.skip(SYNC_PERIOD_TICKS + SETTLE_TICKS)
            delta = meas - base; theo = silhouette_rays(bb.extent.y, 2 * bb.extent.z, FRONT_M)
            need = args.walk_min if is_walker else args.veh_min; ok = delta >= need; rc = rc or (0 if ok else 1)
            print(f"[{args.label}] {bp_id:34s} base={base:7.1f} meas={meas:7.1f} delta={delta:7.1f} need>={need:4d} silhouette~{theo:5d} (30%={int(0.3*theo):4d}) -> {'PASS' if ok else 'FAIL'}")
            results.append({"id": bp_id, "status": "pass" if ok else "fail", "baseline": base, "measured": meas, "delta": delta, "need": need, "silhouette": theo})
        if len(results) != len(targets): raise MeasurementError("not every required target was measured")
        print(f"[{args.label}] GATE RESULT: {'PASS' if rc == 0 else 'FAIL'}")
        if args.json: print(json.dumps({"label": args.label, "negative_control": {"b1": b1, "b2": b2}, "results": results}))
        return rc
    except MeasurementError as e:
        print(f"[{args.label}] ERROR (measurement failure): {e}", file=sys.stderr); return 2
    except Exception as e:
        print(f"[{args.label}] ERROR (unexpected exception, treated as measurement failure): {type(e).__name__}: {e}", file=sys.stderr); return 2
    finally:
        for a in reversed(actors):
            try: a.destroy()
            except Exception: pass
        restore = (lambda: world.apply_settings(original)) if (world is not None and original is not None) else None
        for fn in ((node.destroy_node if node else None), rclpy.shutdown, restore):
            try:
                if fn: fn()
            except Exception: pass


if __name__ == "__main__":
    sys.exit(main())
