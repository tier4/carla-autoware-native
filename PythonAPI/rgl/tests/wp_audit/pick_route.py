#!/usr/bin/env python3
"""Helper to decide the polylines in scenarios.py (prints coordinates, writes nothing).

  town mode (needs a running server and the carla module):
    pick_route.py --mode town15 --port 4000 [--load] [--length 6000] [--step 10]
    pick_route.py --mode town10 ...
  Walks world.get_map() waypoints forward and keeps stretches where a downward
  cast_ray at every waypoint has no Terrain hit (no Landscape under the road).
  Prints the longest such stretch as `route_point x= y= z=` lines plus a Python
  tuple literal to paste into scenarios.py.

  odaiba mode (no carla needed; the Odaiba map has no OpenDRIVE):
    pick_route.py --mode odaiba --lanelets 2314486,1501,164 --osm <lanelet2_map.osm>
  (--osm defaults to the environment variable ODAIBA_LANELET2_OSM; one of the two is required.)
  Prints the lanelet2 centerline (midpoint of the left/right bound) of each
  lanelet id in the osm's local_x/local_y/ele. These are NOT CARLA coordinates;
  convert them (the map's own offset) before pasting into scenarios.py.
  Source of the ids: the ego start (lanelet 2314486, s=25.0127) and route
  waypoints (lanelet 1501 s=30, lanelet 164 s=25) of a driving scenario on the
  Odaiba map.
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

DEFAULT_OSM = os.environ.get("ODAIBA_LANELET2_OSM", "")


# Keep the calculation and evaluation order unchanged.
# pylint: disable-next=too-many-locals
def odaiba_centerlines(osm_path, lanelet_ids):
    """Build the route from the requested OSM lanelet centre lines."""
    tree = ET.parse(osm_path)
    root = tree.getroot()
    nodes = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        if "local_x" in tags:
            nodes[n.get("id")] = (float(tags["local_x"]), float(tags["local_y"]),
                                  float(tags.get("ele", "0")))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}
    out = {}
    wanted = set(lanelet_ids)
    for r in root.findall("relation"):
        if r.get("id") not in wanted:
            continue
        mem = {m.get("role"): m.get("ref") for m in r.findall("member") if m.get("type") == "way"}
        left, right = ways.get(mem.get("left"), []), ways.get(mem.get("right"), [])
        n = min(len(left), len(right))
        pts = []
        for i in range(n):
            a, b = nodes[left[i]], nodes[right[i]]
            pts.append(tuple((a[k] + b[k]) / 2.0 for k in range(3)))
        out[r.get("id")] = pts
    return out


def town_mode(args):
    """Collect a route from map waypoints on the running server."""
    # Load optional runtime dependencies only when this path is used.
    # pylint: disable-next=import-outside-toplevel
    import carla
    client = carla.Client("127.0.0.1", args.port)
    client.set_timeout(120.0)
    world = client.load_world("Town15" if args.mode == "town15" else "Town10HD_Opt") if args.load \
        else client.get_world()
    amap = world.get_map()
    wps = amap.generate_waypoints(args.step)
    terrain = carla.CityObjectLabel.Terrain

    def has_terrain(loc):
        hits = world.cast_ray(carla.Location(loc.x, loc.y, loc.z + 5.0),
                              carla.Location(loc.x, loc.y, loc.z - 10.0))
        return any(h.label == terrain for h in hits)

    best = []
    for start in wps[::max(1, len(wps) // 400)]:
        cur, run = start, []
        length = 0.0
        while length < args.length:
            if has_terrain(cur.transform.location):
                break
            run.append(cur.transform.location)
            nxt = cur.next(args.step)
            if not nxt:
                break
            cur = nxt[0]
            length += args.step
        if len(run) > len(best):
            best = run
        if len(best) * args.step >= args.length:
            break
    print(f"pick_route mode={args.mode} points={len(best)} approx_length_m={len(best) * args.step:.0f}")
    for p in best:
        print(f"route_point x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}")
    print("ROUTE = (" + ", ".join(f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f})" for p in best) + ")")
    return 0 if best else 1


def main(argv):
    """Main."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", required=True, choices=["town15", "town10", "odaiba"])
    ap.add_argument("--port", type=int, default=4000)
    ap.add_argument("--load", action="store_true")
    ap.add_argument("--length", type=float, default=6000.0)
    ap.add_argument("--step", type=float, default=10.0)
    ap.add_argument("--lanelets", default="")
    ap.add_argument("--osm", default=DEFAULT_OSM)
    args = ap.parse_args(argv)
    if args.mode == "odaiba":
        if not args.osm:
            ap.error("--mode odaiba needs --osm <lanelet2_map.osm> (or ODAIBA_LANELET2_OSM)")
        ids = [s for s in args.lanelets.split(",") if s]
        cl = odaiba_centerlines(args.osm, ids)
        for i in ids:
            pts = cl.get(i, [])
            print(f"lanelet id={i} points={len(pts)}")
            for x, y, z in pts:
                print(f"local_point lanelet={i} local_x={x:.3f} local_y={y:.3f} ele={z:.3f}")
        return 0 if all(cl.get(i) for i in ids) else 1
    return town_mode(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
