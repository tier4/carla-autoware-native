#!/usr/bin/env python3
"""Fixed scenario tick-cost driver (skeletal spec §6.2): N walkers + M vehicles, static, one RGL lidar.
Measures wall time of world.tick() (server frame incl. RGL scene update) after warm-up.
Usage: python3 measure_skeletal_tick_cost.py --label C [--walkers 7 --vehicles 3 --warmup 100 --frames 600]"""
import argparse, json, statistics, sys, time
import carla

# Explicit blueprint ids so the scenario is identical across runs and machines
# (bp.filter() order depends on the asset registry and is not stable).
WALKER_IDS = ["walker.pedestrian.0015", "walker.pedestrian.0016", "walker.pedestrian.0017",
              "walker.pedestrian.0018", "walker.pedestrian.0019", "walker.pedestrian.0020",
              "walker.pedestrian.0021"]
VEHICLE_IDS = ["vehicle.lincoln.mkz", "vehicle.ue4.audi.tt", "vehicle.mini.cooper"]


def find_bp(bp_lib, bp_id):
    try:
        return bp_lib.find(bp_id)
    except Exception as e:
        print(f"blueprint not found: {bp_id} ({e})", file=sys.stderr)
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--label", required=True); ap.add_argument("--walkers", type=int, default=7); ap.add_argument("--vehicles", type=int, default=3)
    ap.add_argument("--warmup", type=int, default=100); ap.add_argument("--frames", type=int, default=600)
    a = ap.parse_args()
    client = carla.Client(a.host, a.port); client.set_timeout(20.0); world = client.get_world()
    original = world.get_settings(); actors = []
    try:
        s = world.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05; world.apply_settings(s)
        bp = world.get_blueprint_library()
        lidar = bp.find("sensor.lidar.rgl")
        for k, v in {"channels": 64, "range": 50, "points_per_second": 1300000, "rotation_frequency": 20, "sensor_tick": 0.05,
                     "rgl_lidar_topic_name": "/perf/rgl", "rgl_lidar_pointcloud_format": "PointXYZIRCAEDT"}.items():
            lidar.set_attribute(k, str(v))
        missing = [bp_id for bp_id in WALKER_IDS + VEHICLE_IDS if find_bp(bp, bp_id) is None]
        if missing:
            for bp_id in missing:
                print(f"ERROR: required blueprint not found: {bp_id}", file=sys.stderr)
            sys.exit(2)
        actors.append(world.spawn_actor(lidar, carla.Transform(carla.Location(0, 0, 1.65))))
        gp = world.ground_projection(carla.Location(10.0, 0.0, 5.0), 20.0); gz = gp.location.z if gp is not None else 0.0
        walker_ids = []; vehicle_ids = []
        for i in range(a.walkers):
            bp_id = WALKER_IDS[i % len(WALKER_IDS)]
            wbp = find_bp(bp, bp_id)
            if wbp is None: continue
            x = 10.0 + 2.0 * i
            act = world.try_spawn_actor(wbp, carla.Transform(carla.Location(x, -3.0, gz + 1.0)))
            if not act:
                act = world.try_spawn_actor(wbp, carla.Transform(carla.Location(x + 1.0, -3.0, gz + 1.0)))
            if act: actors.append(act); walker_ids.append(bp_id)
        for i in range(a.vehicles):
            bp_id = VEHICLE_IDS[i % len(VEHICLE_IDS)]
            vbp = find_bp(bp, bp_id)
            if vbp is None: continue
            x = 12.0 + 8.0 * i
            act = world.try_spawn_actor(vbp, carla.Transform(carla.Location(x, 3.5, gz + 0.3)))
            if not act:
                act = world.try_spawn_actor(vbp, carla.Transform(carla.Location(x, -6.0, gz + 0.3)))
            if act: actors.append(act); vehicle_ids.append(bp_id)
        n_walkers = len(walker_ids); n_vehicles = len(vehicle_ids)
        print(f"spawned walkers={n_walkers}/{a.walkers} vehicles={n_vehicles}/{a.vehicles}", file=sys.stderr)
        for _ in range(a.warmup): world.tick()
        dts = []
        for _ in range(a.frames):
            t0 = time.perf_counter(); world.tick(); dts.append((time.perf_counter() - t0) * 1000.0)
        dts.sort()
        print(json.dumps({"label": a.label, "actors": len(actors) - 1, "walkers": n_walkers, "vehicles": n_vehicles,
                          "walker_ids": walker_ids, "vehicle_ids": vehicle_ids,
                          "median_ms": statistics.median(dts), "p95_ms": dts[int(0.95 * len(dts)) - 1]}))
    finally:
        for x in reversed(actors):
            try: x.destroy()
            except Exception: pass
        try: world.apply_settings(original)
        except Exception: pass

if __name__ == "__main__":
    main()
