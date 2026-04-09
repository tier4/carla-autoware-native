#!/bin/bash
# Monitor LiDAR topic rates matching rgl_test_autoware_demo.py arguments.
# Usage:
#   ./rgl_test_topic_hz.sh [--carla_lidar_type rgl|ray_cast|both|none]
#                          [--rgl_lidar_direct_publish]
#                          [--num_lidars N]
#                          [--carla_lidar_topic_name /topic]
#                          [--rgl_lidar_topic_name /topic]
#                          [--window-threshold N]
#
# Only the first topic's output is shown on screen.
# Ctrl+C (or reaching --window-threshold) shows total summary + last CARLA system stats.

source /mnt/dsk0/wk0/ROS2/humble/AW-OSS/1.7.1/autoware/install/setup.bash

CARLA_LIDAR_TYPE="rgl"
RGL_LIDAR_DIRECT_PUBLISH=false
NUM_LIDARS=1
CARLA_TOPIC="/sensing/lidar/top/pointcloud_raw_ex"
RGL_TOPIC="/sensing/lidar/top/pointcloud_raw_ex"
WINDOW_THRESHOLD=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --carla_lidar_type)         CARLA_LIDAR_TYPE="$2"; shift 2 ;;
        --rgl_lidar_direct_publish) RGL_LIDAR_DIRECT_PUBLISH=true; shift 1 ;;
        --num_lidars)               NUM_LIDARS="$2"; shift 2 ;;
        --carla_lidar_topic_name)   CARLA_TOPIC="$2"; shift 2 ;;
        --rgl_lidar_topic_name)     RGL_TOPIC="$2"; shift 2 ;;
        --window-threshold)         WINDOW_THRESHOLD="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# Derived flags (mirrors Python spawn logic)
case "$CARLA_LIDAR_TYPE" in
    ray_cast) SPAWN_RAY_CAST=true;  SPAWN_RGL=false; CARLA_ROS2_FOR_RGL=false ;;
    rgl)      SPAWN_RAY_CAST=false; SPAWN_RGL=true;  CARLA_ROS2_FOR_RGL=true  ;;
    both)     SPAWN_RAY_CAST=true;  SPAWN_RGL=true;  CARLA_ROS2_FOR_RGL=true  ;;
    none)     SPAWN_RAY_CAST=false; SPAWN_RGL=false; CARLA_ROS2_FOR_RGL=false ;;
    *) echo "Unknown carla_lidar_type: $CARLA_LIDAR_TYPE"; exit 1 ;;
esac

if $RGL_LIDAR_DIRECT_PUBLISH; then
    SPAWN_RGL=true
fi

if $SPAWN_RAY_CAST && $SPAWN_RGL; then
    USE_NAMESPACE=true
else
    USE_NAMESPACE=false
fi

TMP_DIR=$(mktemp -d)
PIDS=()
TOPIC_NAMES=()
TOPIC_FILES=()
MONITOR_PID=""
THRESHOLD_PID=""
MONITOR_FILE="$TMP_DIR/sysmon.txt"

avg_of() {
    echo "$1" | awk 'NF{s+=$1; c++} END{if(c>0) printf "%.4f", s/c; else print "N/A"}'
}

# Background system monitor: sample CARLA CPU/mem and GPU stats every 1s
start_sysmon() {
    local carla_pid="$1"
    (
        while kill -0 "$carla_pid" 2>/dev/null; do
            cpu=$(ps -p "$carla_pid" -o %cpu --no-headers 2>/dev/null | tr -d ' ')
            mem_kb=$(ps -p "$carla_pid" -o rss --no-headers 2>/dev/null | tr -d ' ')
            mem_mb=$(awk "BEGIN{printf \"%.0f\", ${mem_kb:-0}/1024}")
            gpu_temp=$(nvidia-smi --query-gpu=temperature.gpu \
                --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
            gpu_power=$(nvidia-smi --query-gpu=power.draw \
                --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
            gpu_mem=$(nvidia-smi --query-compute-apps=pid,used_memory \
                --format=csv,noheader,nounits 2>/dev/null \
                | awk -F',' -v p="$carla_pid" 'int($1)==p{gsub(/ /,"",$2); print $2}')
            echo "${cpu:-N/A} ${mem_mb:-N/A} ${gpu_temp:-N/A} ${gpu_power:-N/A} ${gpu_mem:-N/A}"
            sleep 1
        done
    ) >> "$MONITOR_FILE" &
    MONITOR_PID=$!
}

# Background threshold checker: sends SIGINT to main process when total window >= N
start_threshold_checker() {
    local threshold="$1"
    local tmp_dir="$2"
    local main_pid="$$"
    (
        while true; do
            sleep 0.5
            total=0
            for f in "$tmp_dir"/topic_*.txt; do
                [[ -f "$f" ]] || continue
                last_window=$(grep -oP 'window: \K[0-9]+' "$f" 2>/dev/null | tail -1)
                total=$((total + ${last_window:-0}))
            done
            if [[ $total -ge $threshold ]]; then
                echo ""
                echo "Window threshold $threshold reached (total window: $total). Stopping..."
                kill -SIGINT "$main_pid"
                break
            fi
        done
    ) &
    THRESHOLD_PID=$!
}

cleanup() {
    echo ""
    echo "Stopping all processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    [[ -n "$MONITOR_PID"    ]] && kill "$MONITOR_PID"    2>/dev/null
    [[ -n "$THRESHOLD_PID"  ]] && kill "$THRESHOLD_PID"  2>/dev/null
    wait 2>/dev/null

    # ----- Total summary -----
    echo ""
    echo "===== Summary ====="
    all_last_avg_rates=""
    all_last_mins=""
    all_last_maxs=""
    all_last_stdevs=""
    total_window=0

    for i in "${!TOPIC_FILES[@]}"; do
        local_file="${TOPIC_FILES[$i]}"

        last_avg_rate=$(grep "average rate:" "$local_file" | awk '{print $NF}' | tail -1)
        last_min=$(grep -oP 'min: \K[0-9.]+' "$local_file" | tail -1)
        last_max=$(grep -oP 'max: \K[0-9.]+' "$local_file" | tail -1)
        last_stddev=$(grep -oP 'std dev: \K[0-9.]+' "$local_file" | tail -1)
        last_window=$(grep -oP 'window: \K[0-9]+' "$local_file" | tail -1)

        [[ "$last_avg_rate" != "N/A" ]] && all_last_avg_rates="${all_last_avg_rates}"$'\n'"${last_avg_rate:-}"
        [[ "$last_min"      != "N/A" ]] && all_last_mins="${all_last_mins}"$'\n'"${last_min:-}"
        [[ "$last_max"      != "N/A" ]] && all_last_maxs="${all_last_maxs}"$'\n'"${last_max:-}"
        [[ "$last_stddev"   != "N/A" ]] && all_last_stdevs="${all_last_stdevs}"$'\n'"${last_stddev:-}"
        total_window=$((total_window + ${last_window:-0}))
    done

    echo "  total window            : $total_window"
    echo "  average rate (avg)      : $(avg_of "$all_last_avg_rates") Hz"
    echo "  min (avg)               : $(avg_of "$all_last_mins") s"
    echo "  max (avg)               : $(avg_of "$all_last_maxs") s"
    echo "  std dev (avg)           : $(avg_of "$all_last_stdevs") s"

    # ----- CARLA system stats -----
    if [[ -f "$MONITOR_FILE" ]] && [[ -s "$MONITOR_FILE" ]]; then
        read -r last_cpu last_mem_mb last_gpu_temp last_gpu_power last_gpu_mem \
            <<< "$(tail -1 "$MONITOR_FILE")"
        echo ""
        echo "===== CARLA System Stats (last sample) ====="
        echo "  CPU usage               : ${last_cpu} %"
        echo "  Memory (RSS)            : ${last_mem_mb} MB"
        echo "  GPU temperature         : ${last_gpu_temp} °C"
        echo "  GPU power               : ${last_gpu_power} W"
        echo "  GPU memory (CARLA)      : ${last_gpu_mem} MiB"
    fi

    rm -rf "$TMP_DIR"
    exit 0
}

trap cleanup SIGINT SIGTERM

# First topic (index 0): output shown on screen with total window annotation
# Other topics: output captured silently
start_hz() {
    local topic="$1"
    local idx="${#TOPIC_FILES[@]}"
    local tmpfile="$TMP_DIR/topic_${idx}.txt"
    local tmp_dir="$TMP_DIR"
    local threshold="$WINDOW_THRESHOLD"
    TOPIC_NAMES+=("$topic")
    TOPIC_FILES+=("$tmpfile")
    if [[ $idx -eq 0 ]]; then
        echo "Starting (displayed): ros2 topic hz $topic"
        # tee captures raw ros2 output to file; while loop annotates stdout with total window
        ros2 topic hz "$topic" 2>&1 | tee "$tmpfile" | \
        while IFS= read -r line; do
            echo "$line"
            if [[ "$line" =~ window: ]]; then
                total=0
                for f in "$tmp_dir"/topic_*.txt; do
                    [[ -f "$f" ]] || continue
                    lw=$(grep -oP 'window: \K[0-9]+' "$f" 2>/dev/null | tail -1)
                    total=$((total + ${lw:-0}))
                done
                if [[ -n "$threshold" ]]; then
                    printf "  [total window: %d / %d]\n" "$total" "$threshold"
                else
                    printf "  [total window: %d]\n" "$total"
                fi
            fi
        done &
    else
        echo "Starting (silent):    ros2 topic hz $topic"
        ros2 topic hz "$topic" > "$tmpfile" 2>&1 &
    fi
    PIDS+=($!)
}

# Find CARLA process
CARLA_PID=$(pgrep -f "CarlaUE5" | head -1)
if [[ -n "$CARLA_PID" ]]; then
    echo "Found CARLA process: PID=$CARLA_PID"
    start_sysmon "$CARLA_PID"
else
    echo "Warning: CARLA process not found. System stats will be unavailable."
fi

for ((i=0; i<NUM_LIDARS; i++)); do
    if [[ $NUM_LIDARS -ge 2 ]]; then
        SUFFIX="_${i}"
    else
        SUFFIX=""
    fi

    if $SPAWN_RAY_CAST; then
        if $USE_NAMESPACE; then
            start_hz "/raycast${CARLA_TOPIC}${SUFFIX}"
        else
            start_hz "${CARLA_TOPIC}${SUFFIX}"
        fi
    fi

    if $SPAWN_RGL; then
        if $CARLA_ROS2_FOR_RGL; then
            if $USE_NAMESPACE; then
                start_hz "/rgl${CARLA_TOPIC}${SUFFIX}"
            else
                start_hz "${CARLA_TOPIC}${SUFFIX}"
            fi
        fi
        if $RGL_LIDAR_DIRECT_PUBLISH; then
            if $USE_NAMESPACE; then
                start_hz "/rgl${RGL_TOPIC}${SUFFIX}"
            else
                start_hz "${RGL_TOPIC}${SUFFIX}"
            fi
        fi
    fi
done

if [[ ${#PIDS[@]} -eq 0 ]]; then
    echo "No topics to monitor (carla_lidar_type=none, rgl_lidar_direct_publish=false)."
    cleanup
fi

if [[ -n "$WINDOW_THRESHOLD" ]]; then
    echo "Window threshold: $WINDOW_THRESHOLD"
    start_threshold_checker "$WINDOW_THRESHOLD" "$TMP_DIR"
fi

echo ""
echo "Monitoring ${#PIDS[@]} topic(s). Press Ctrl+C to stop and show summary."
echo ""

wait
