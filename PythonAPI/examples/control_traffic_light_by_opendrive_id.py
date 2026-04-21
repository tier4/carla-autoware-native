#!/usr/bin/env python3
#
# Copyright (c) 2026 TIER IV, Inc.

import argparse
import sys
import time
import xml.etree.ElementTree as ET

import carla


STATES = {
    'green': carla.TrafficLightState.Green,
    'yellow': carla.TrafficLightState.Yellow,
    'red': carla.TrafficLightState.Red,
    'off': carla.TrafficLightState.Off,
}


def _load_xodr_text(world, xodr_path: str | None) -> str | None:
    if xodr_path:
        with open(xodr_path, 'r', encoding='utf-8') as f:
            return f.read()
    try:
        return world.get_map().to_opendrive()
    except Exception:
        return None


def _xodr_has_signal_id(xodr_text: str, signal_id: str) -> bool:
    """
    Best-effort check: does the OpenDRIVE contain <signal id="..."> ?
    Note: CARLA traffic light OpenDRIVE ids usually come from OpenDRIVE signals.
    """
    try:
        root = ET.fromstring(xodr_text)
    except ET.ParseError:
        return False

    for sig in root.iter('signal'):
        if sig.get('id') == signal_id:
            return True
    return False


def _iter_traffic_lights(world):
    return list(world.get_actors().filter('*traffic_light*'))


def get_traffic_light_by_opendrive_id(world, opendrive_id: str):
    """
    Returns a single carla.TrafficLight whose get_opendrive_id() matches opendrive_id.
    Uses multiple fallbacks for compatibility across server versions.
    """
    # Fast path (if supported by server).
    try:
        tl = world.get_traffic_light_from_opendrive_id(opendrive_id)
        if tl:
            return tl
    except Exception:
        pass

    # Fallback: scan all junction traffic lights and compare get_opendrive_id().
    try:
        tmap = world.get_map()
        topology = tmap.get_topology()
        seen = set()
        for (entry_wp, _) in topology:
            if entry_wp.is_junction and entry_wp.junction_id not in seen:
                seen.add(entry_wp.junction_id)
                for tl in world.get_traffic_lights_in_junction(entry_wp.junction_id):
                    try:
                        if tl.get_opendrive_id() == opendrive_id:
                            return tl
                    except Exception:
                        continue
    except Exception:
        pass

    # Last resort: scan all traffic light actors.
    for tl in _iter_traffic_lights(world):
        try:
            if tl.get_opendrive_id() == opendrive_id:
                return tl
        except Exception:
            continue
    return None


def _unique(seq):
    seen = set()
    out = []
    for x in seq:
        if x.id not in seen:
            seen.add(x.id)
            out.append(x)
    return out


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Control a CARLA traffic light by OpenDRIVE signal id '
            '(TrafficLight.get_opendrive_id()).'
        )
    )
    parser.add_argument('--host', metavar='H', default='127.0.0.1',
                        help='CARLA server host (default: 127.0.0.1)')
    parser.add_argument('-p', '--port', metavar='P', default=2000, type=int,
                        help='CARLA server port (default: 2000)')

    parser.add_argument('--id', dest='signal_id', default=None,
                        help='OpenDRIVE signal id (TrafficLight.get_opendrive_id())')
    parser.add_argument('--state', choices=list(STATES.keys()), default='red',
                        help='Set state (default: red)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='If >0, keep running for N seconds after setting state (default: 0)')
    parser.add_argument('--all-in-group', action='store_true',
                        help='Also apply to the whole group (tl.get_group_traffic_lights())')

    parser.add_argument('--green-time', type=float, default=-1.0,
                        help='If >=0, override green duration (seconds)')
    parser.add_argument('--yellow-time', type=float, default=-1.0,
                        help='If >=0, override yellow duration (seconds)')
    parser.add_argument('--red-time', type=float, default=-1.0,
                        help='If >=0, override red duration (seconds)')

    parser.add_argument('--freeze', action='store_true',
                        help='Freeze all traffic lights to prevent auto-phase overriding')
    parser.add_argument('--list', action='store_true',
                        help='List traffic lights and their OpenDRIVE ids, then exit')

    parser.add_argument('--xodr', default=None,
                        help='Optional .xodr path for id existence check (default: use world map)')
    parser.add_argument('--no-xodr-check', action='store_true',
                        help='Skip checking the id existence in OpenDRIVE')

    args = parser.parse_args()

    if not args.list and not args.signal_id:
        print('ERROR: --id is required (or use --list).', file=sys.stderr)
        return 2

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        world = client.get_world()
    except Exception as e:
        print(f'ERROR: Failed to connect to CARLA: {e}', file=sys.stderr)
        return 1

    if args.list:
        tls = _iter_traffic_lights(world)
        if not tls:
            print('No traffic_light actors found.')
            return 0
        rows = []
        for tl in tls:
            try:
                rows.append((tl.id, tl.get_opendrive_id(), tl.get_state().name))
            except Exception:
                rows.append((tl.id, '<unknown>', '<unknown>'))
        rows.sort(key=lambda r: (str(r[1]), int(r[0])))
        for actor_id, od_id, state in rows:
            print(f'actor_id={actor_id} opendrive_id={od_id} state={state}')
        return 0

    if not args.no_xodr_check:
        xodr_text = _load_xodr_text(world, args.xodr)
        if xodr_text is None:
            print('WARN: Failed to load OpenDRIVE; skipping id existence check.', file=sys.stderr)
        else:
            if not _xodr_has_signal_id(xodr_text, args.signal_id):
                print(
                    f'WARN: OpenDRIVE does not contain signal id="{args.signal_id}".\n'
                    '      (Note: this may not match CARLA TrafficLight.get_opendrive_id(); continuing anyway.)',
                    file=sys.stderr
                )

    if args.freeze:
        try:
            world.freeze_all_traffic_lights(True)
        except Exception as e:
            print(f'WARN: freeze_all_traffic_lights(True) failed: {e}', file=sys.stderr)

    tl = get_traffic_light_by_opendrive_id(world, args.signal_id)
    if not tl:
        print(
            f'ERROR: No traffic light found for OpenDRIVE id "{args.signal_id}".\n'
            '       Hint: visualize with `PythonAPI/util/show_traffic_lights.py` or run this script with --list.',
            file=sys.stderr
        )
        return 1

    targets = [tl]
    if args.all_in_group:
        try:
            targets += list(tl.get_group_traffic_lights())
        except Exception:
            pass
        targets = _unique(targets)

    if args.green_time >= 0:
        for a in targets:
            a.set_green_time(args.green_time)
    if args.yellow_time >= 0:
        for a in targets:
            a.set_yellow_time(args.yellow_time)
    if args.red_time >= 0:
        for a in targets:
            a.set_red_time(args.red_time)

    state = STATES[args.state]
    for a in targets:
        a.set_state(state)

    print(
        f'opendrive_id={args.signal_id} state={args.state} '
        f'targets={len(targets)} freeze={args.freeze}'
    )

    if args.duration and args.duration > 0:
        # Keep process alive so you can observe the state; the state itself persists server-side.
        end = time.time() + args.duration
        try:
            while time.time() < end:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    return 0


if __name__ == '__main__':
    raise SystemExit(main())

