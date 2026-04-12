#!/usr/bin/env python3
"""Compare CARLA and RGL LiDAR topic outputs captured via ros2 topic echo.

Usage:
    # 1. Capture topics (run simultaneously):
    #    ros2 topic echo /sensing/lidar/top/pointcloud_raw_ex > /tmp/carla_lidar.txt
    #    ros2 topic echo /sensing/lidar/top/rgl_pointcloud > /tmp/rgl_lidar.txt
    #
    # 2. Stop after a few seconds (Ctrl-C both)
    #
    # 3. Compare:
    #    python3 compare_lidar_topics.py /tmp/carla_lidar.txt /tmp/rgl_lidar.txt

    Matches messages by timestamp (sec + nanosec) and compares:
    - width (point count)
    - point_step
    - row_step
    - fields layout
    - data length
"""

import argparse
import re
import sys


def parse_messages(filepath):
    """Parse ros2 topic echo output into a list of message dicts."""
    messages = []
    current = {}

    with open(filepath, 'r') as f:
        for line in f:
            line = line.rstrip()

            # Message separator
            if line == '---':
                if current:
                    messages.append(current)
                current = {}
                continue

            # Header timestamp
            m = re.match(r'\s+sec:\s+(\d+)', line)
            if m and 'stamp_sec' not in current:
                current['stamp_sec'] = int(m.group(1))
                continue

            m = re.match(r'\s+nanosec:\s+(\d+)', line)
            if m and 'stamp_nsec' not in current:
                current['stamp_nsec'] = int(m.group(1))
                continue

            # frame_id
            m = re.match(r'\s+frame_id:\s+(.+)', line)
            if m and 'frame_id' not in current:
                current['frame_id'] = m.group(1).strip()
                continue

            # Scalar fields
            for key in ['height', 'width', 'point_step', 'row_step', 'is_bigendian', 'is_dense']:
                m = re.match(rf'{key}:\s+(.+)', line)
                if m:
                    val = m.group(1).strip()
                    try:
                        current[key] = int(val)
                    except ValueError:
                        current[key] = val
                    break

            # Data field (count bytes)
            m = re.match(r'data:\s*\[(.+)\]', line)
            if m:
                # Count comma-separated values
                data_str = m.group(1)
                current['data_len'] = data_str.count(',') + 1 if data_str.strip() else 0
                continue

            m = re.match(r'data:\s*\[\]', line)
            if m:
                current['data_len'] = 0
                continue

            # data: '...' (binary, just note it exists)
            m = re.match(r"data:\s*'", line)
            if m:
                current['data_len'] = -1  # binary, can't count
                continue

    # Last message
    if current:
        messages.append(current)

    return messages


def timestamp_key(msg):
    """Create a comparable timestamp key."""
    return (msg.get('stamp_sec', 0), msg.get('stamp_nsec', 0))


def format_timestamp(msg):
    """Format timestamp for display."""
    return f"{msg.get('stamp_sec', '?')}.{msg.get('stamp_nsec', '?'):09d}"


def compare_messages(carla_msg, rgl_msg):
    """Compare two messages and return list of differences."""
    diffs = []

    for key in ['width', 'height', 'point_step', 'row_step', 'is_dense', 'frame_id']:
        cv = carla_msg.get(key, 'N/A')
        rv = rgl_msg.get(key, 'N/A')
        if cv != rv:
            diffs.append(f"  {key}: CARLA={cv} RGL={rv}")

    cd = carla_msg.get('data_len', 0)
    rd = rgl_msg.get('data_len', 0)
    if cd != rd:
        diffs.append(f"  data_len: CARLA={cd} RGL={rd}")

    return diffs


def main():
    parser = argparse.ArgumentParser(description='Compare CARLA and RGL LiDAR topic echo outputs')
    parser.add_argument('carla_file', help='Path to CARLA topic echo output')
    parser.add_argument('rgl_file', help='Path to RGL topic echo output')
    parser.add_argument('--max-messages', type=int, default=20,
                        help='Max messages to compare (default: 20)')
    args = parser.parse_args()

    print(f"Parsing {args.carla_file}...")
    carla_msgs = parse_messages(args.carla_file)
    print(f"  Found {len(carla_msgs)} messages")

    print(f"Parsing {args.rgl_file}...")
    rgl_msgs = parse_messages(args.rgl_file)
    print(f"  Found {len(rgl_msgs)} messages")

    if not carla_msgs or not rgl_msgs:
        print("ERROR: No messages found in one or both files.")
        sys.exit(1)

    # Match by nearest timestamp within tolerance
    tolerance_ns = 50_000_000  # 50ms tolerance for matching

    def ts_to_ns(msg):
        return msg.get('stamp_sec', 0) * 1_000_000_000 + msg.get('stamp_nsec', 0)

    # Sort RGL messages by timestamp for nearest-match search
    rgl_sorted = sorted(rgl_msgs, key=ts_to_ns)
    rgl_timestamps_ns = [ts_to_ns(m) for m in rgl_sorted]

    def find_nearest_rgl(carla_ns):
        """Find the RGL message with the closest timestamp."""
        import bisect
        idx = bisect.bisect_left(rgl_timestamps_ns, carla_ns)
        best = None
        best_diff = tolerance_ns + 1
        for i in [idx - 1, idx]:
            if 0 <= i < len(rgl_sorted):
                diff = abs(rgl_timestamps_ns[i] - carla_ns)
                if diff < best_diff:
                    best_diff = diff
                    best = (rgl_sorted[i], diff)
        return best

    # Match and compare
    matched = 0
    unmatched_carla = 0
    identical = 0
    different = 0
    used_rgl = set()

    print(f"\n{'='*70}")
    print(f"Comparison Results (tolerance: {tolerance_ns/1e6:.0f}ms)")
    print(f"{'='*70}")

    for carla_msg in carla_msgs:
        if matched >= args.max_messages:
            break

        carla_ns = ts_to_ns(carla_msg)
        result = find_nearest_rgl(carla_ns)

        if result is None:
            unmatched_carla += 1
            continue

        rgl_msg, diff_ns = result
        rgl_ns = ts_to_ns(rgl_msg)

        if rgl_ns in used_rgl:
            unmatched_carla += 1
            continue

        used_rgl.add(rgl_ns)

        matched += 1
        diffs = compare_messages(carla_msg, rgl_msg)

        ts_str = format_timestamp(carla_msg)
        dt_ms = diff_ns / 1e6
        if diffs:
            different += 1
            print(f"\n[DIFF] CARLA ts={ts_str} (dt={dt_ms:.1f}ms):")
            print(f"  CARLA: width={carla_msg.get('width', '?')} point_step={carla_msg.get('point_step', '?')} "
                  f"row_step={carla_msg.get('row_step', '?')} data_len={carla_msg.get('data_len', '?')}")
            print(f"  RGL:   width={rgl_msg.get('width', '?')} point_step={rgl_msg.get('point_step', '?')} "
                  f"row_step={rgl_msg.get('row_step', '?')} data_len={rgl_msg.get('data_len', '?')}")
            for d in diffs:
                print(d)
        else:
            identical += 1
            print(f"[OK]   CARLA ts={ts_str} (dt={dt_ms:.1f}ms): width={carla_msg.get('width', '?')} - identical")

    print(f"\n{'='*70}")
    print(f"Summary:")
    print(f"  CARLA messages:    {len(carla_msgs)}")
    print(f"  RGL messages:      {len(rgl_msgs)}")
    print(f"  Matched by time:   {matched}")
    print(f"  Unmatched CARLA:   {unmatched_carla}")
    print(f"  Identical:         {identical}")
    print(f"  Different:         {different}")
    print(f"{'='*70}")


if __name__ == '__main__':
    main()
