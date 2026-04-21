#!/usr/bin/env python3
#
# Copyright (c) 2026 TIER IV, Inc.

import argparse
import sys
import time

import carla


STATES = {
    'green': carla.TrafficLightState.Green,
    'yellow': carla.TrafficLightState.Yellow,
    'red': carla.TrafficLightState.Red,
    'off': carla.TrafficLightState.Off,
}


def _iter_traffic_lights(world):
    # Matches traffic.traffic_light plus any custom traffic light actors.
    return list(world.get_actors().filter('*traffic_light*'))


def _unique(seq):
    seen = set()
    out = []
    for x in seq:
        if x.id not in seen:
            seen.add(x.id)
            out.append(x)
    return out


def _get_sign_id_best_effort(tl) -> str | None:
    """
    Best-effort: read the "SignId" assigned to the traffic light.

    In this codebase, the server stores SignId in the traffic light snapshot `sign_id`.
    Python API commonly exposes it via TrafficLight.get_opendrive_id().
    """
    try:
        sid = tl.get_opendrive_id()
        if sid is None:
            return None
        sid = str(sid)
        return sid if sid != '' else None
    except Exception:
        return None


def get_traffic_light_by_sign_id(world, sign_id: str):
    for tl in _iter_traffic_lights(world):
        sid = _get_sign_id_best_effort(tl)
        if sid == sign_id:
            return tl
    return None


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Change the state of a pre-placed traffic light actor by SignID.\n'
            'Note: OpenDRIVE (.xodr) is not required. SignID is expected to be retrievable via TrafficLight.get_opendrive_id().'
        )
    )
    parser.add_argument('--host', metavar='H', default='127.0.0.1',
                        help='CARLA server host (default: 127.0.0.1)')
    parser.add_argument('-p', '--port', metavar='P', default=2000, type=int,
                        help='CARLA server port (default: 2000)')

    parser.add_argument('--id', dest='sign_id', default=None,
                        help='SignID (pre-assigned in the level)')
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
                        help='List traffic lights and their SignIDs, then exit')

    args = parser.parse_args()

    if not args.list and not args.sign_id:
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
            sid = _get_sign_id_best_effort(tl)
            try:
                state = tl.get_state().name
            except Exception:
                state = '<unknown>'
            rows.append((tl.id, sid if sid is not None else '<none>', state))
        rows.sort(key=lambda r: (str(r[1]), int(r[0])))
        for actor_id, sid, state in rows:
            print(f'actor_id={actor_id} sign_id={sid} state={state}')
        return 0

    if args.freeze:
        try:
            world.freeze_all_traffic_lights(True)
        except Exception as e:
            print(f'WARN: freeze_all_traffic_lights(True) failed: {e}', file=sys.stderr)

    tl = get_traffic_light_by_sign_id(world, args.sign_id)
    if not tl:
        print(
            f'ERROR: No traffic light found for SignID "{args.sign_id}".\n'
            '       Hint: run this script with --list to see the sign_id values in the current world.',
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

    actual_sid = _get_sign_id_best_effort(tl)
    print(
        f'sign_id={args.sign_id} resolved_sign_id={actual_sid} '
        f'state={args.state} targets={len(targets)} freeze={args.freeze}'
    )

    if args.duration and args.duration > 0:
        end = time.time() + args.duration
        try:
            while time.time() < end:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    return 0


if __name__ == '__main__':
    raise SystemExit(main())

