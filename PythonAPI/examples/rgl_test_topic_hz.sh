#!/bin/bash
# Monitor LiDAR topic rates matching rgl_test_autoware_demo.py arguments.
# Usage:
#   ./rgl_test_topic_hz.sh [--lidar_type rgl|ray_cast|both] [--num_lidars N]
#
# Ctrl+C stops all background ros2 topic hz processes.

LIDAR_TYPE="rgl"
NUM_LIDARS=1
BASE_TOPIC="/sensing/lidar/top/pointcloud_raw_ex"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --lidar_type) LIDAR_TYPE="$2"; shift 2 ;;
        --num_lidars) NUM_LIDARS="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

PIDS=()

cleanup() {
    echo ""
    echo "Stopping all ros2 topic hz processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo "Done."
    exit 0
}

trap cleanup SIGINT SIGTERM

start_hz() {
    local topic="$1"
    echo "Starting: ros2 topic hz $topic"
    ros2 topic hz "$topic" &
    PIDS+=($!)
}

for ((i=0; i<NUM_LIDARS; i++)); do
    if [[ $NUM_LIDARS -ge 2 ]]; then
        SUFFIX="_${i}"
    else
        SUFFIX=""
    fi

    case "$LIDAR_TYPE" in
        rgl)
            start_hz "${BASE_TOPIC}${SUFFIX}"
            ;;
        ray_cast)
            start_hz "${BASE_TOPIC}${SUFFIX}"
            ;;
        both)
            start_hz "/raycast${BASE_TOPIC}${SUFFIX}"
            start_hz "/rgl${BASE_TOPIC}${SUFFIX}"
            ;;
        *)
            echo "Unknown lidar_type: $LIDAR_TYPE"
            exit 1
            ;;
    esac
done

echo ""
echo "Monitoring ${#PIDS[@]} topic(s). Press Ctrl+C to stop."
echo ""

wait
