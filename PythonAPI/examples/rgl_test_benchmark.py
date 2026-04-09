#!/usr/bin/env python3
"""
Automated LiDAR benchmark for CARLA + RGL.
Runs all (num_lidars, lidar_type) combinations and aggregates results.

Usage:
    python3 rgl_test_benchmark.py --ros-clock-threshold 60 --num_lidars 1,2,4
"""

import argparse
import os
import re
import shlex
import signal
import subprocess
import threading
import time

# ============================================================
# Configuration — edit these paths/values as needed
# ============================================================
CARLA_CMD = (
    "/mnt/dsk0/wk0/CARLA/T4ForkWithRGL/CarlaUE5/Build/Package/"
    "Carla-0.10.0-Linux-Shipping/Linux/CarlaUnreal.sh --ros2 -ResY=640"
)
CARLA_CLIENT_SCRIPT = (
    "/mnt/dsk0/wk0/CARLA/T4ForkWithRGL/CarlaUE5/PythonAPI/examples/"
    "rgl_test_autoware_demo.py --load_map NishishinjukuMap --lidar_spawn_delta_x 0.1"
)
TEST_TOPIC_HZ_CMD = (
    "/mnt/dsk0/wk0/CARLA/T4ForkWithRGL/CarlaUE5/PythonAPI/examples/"
    "rgl_test_topic_hz.sh"
)
CLIENT_LAUNCH_WAIT         = 20   # seconds to wait for CARLA to become ready after launch
EGO_SPAWN_TIMEOUT          = 30   # seconds to wait for "[INFO]: Ego spawned!" in client output
INTER_RUN_PAUSE            = 5    # seconds between runs for OS/GPU to settle
SIM_BEHIND_SILENCE_TIMEOUT = 5.0  # seconds; if no "Simulation is X ms behind" for this long, report 0
SIM_CLOCK_TOPIC            = "/clock"  # ROS2 topic used to measure simulation time
ROS2_SETUP_BASH            = "/mnt/dsk0/wk0/ROS2/humble/AW-OSS/1.7.1/autoware/install/setup.bash"

# ============================================================
# LiDAR type definitions: (client_extra_args, hz_extra_args)
# ============================================================
LIDAR_TYPE_CONFIGS = {
    "ray_cast":   (["--carla_lidar_type", "ray_cast"],
                   ["--carla_lidar_type", "ray_cast"]),
    "carla-rgl":  (["--carla_lidar_type", "rgl"],
                   ["--carla_lidar_type", "rgl"]),
    "rgl-direct": (["--carla_lidar_type", "none", "--rgl_lidar_direct_publish"],
                   ["--carla_lidar_type", "none", "--rgl_lidar_direct_publish"]),
}

EGO_SPAWN_KEYWORD = "Ego spawned"


# ============================================================
# Simulation-behind tracker
# ============================================================

class _SimBehindTracker:
    """
    Thread-safe tracker for client WARNING lines:
      "[WARNING]: Simulation is XXXXX.X ms behind schedule"
    Remembers the last ms value and when it was seen.
    get_final_ms() returns 0 if silent for >= SIM_BEHIND_SILENCE_TIMEOUT seconds.
    """
    _PATTERN = re.compile(r"Simulation is ([0-9.]+) ms behind schedule")

    def __init__(self):
        self._lock      = threading.Lock()
        self._last_ms   = None   # float, last parsed ms value
        self._last_time = None   # time.time() when last WARNING arrived

    def feed(self, line):
        mo = self._PATTERN.search(line)
        if mo:
            with self._lock:
                self._last_ms   = float(mo.group(1))
                self._last_time = time.time()

    def get_final_ms(self, silence_timeout=SIM_BEHIND_SILENCE_TIMEOUT):
        """
        Return the last WARNING ms value, or 0 if:
          - no WARNING was ever seen, or
          - the last WARNING was >= silence_timeout seconds ago
            (simulation caught up / is currently on schedule).
        """
        with self._lock:
            if self._last_ms is None:
                return 0.0
            if time.time() - self._last_time >= silence_timeout:
                return 0.0
            return self._last_ms


# ============================================================
# Process helpers
# ============================================================

def _ros2_cmd(args):
    """
    Wrap a command list so it runs after sourcing ROS2_SETUP_BASH.
    Returns ["bash", "-c", "source ... && <args>"] when ROS2_SETUP_BASH is set,
    otherwise returns args unchanged.
    """
    if not ROS2_SETUP_BASH:
        return args
    inner = " ".join(shlex.quote(a) for a in args)
    return ["bash", "-c", f"source {shlex.quote(ROS2_SETUP_BASH)} && {inner}"]


def _sigint_group(proc):
    """Send SIGINT to the process group of proc (if still alive)."""
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError, OSError):
            pass


def _kill(proc):
    """SIGKILL proc if still alive."""
    if proc and proc.poll() is None:
        try:
            proc.kill()
        except (ProcessLookupError, OSError):
            pass


def _wait(proc, timeout):
    """Wait for proc; SIGKILL if it doesn't finish within timeout."""
    if not proc:
        return
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        _kill(proc)
        proc.wait()


def _drain_thread(proc, lines, prefix, on_line=None):
    """Background thread: read proc stdout line by line, appending to lines.
    on_line(line) is called for each line if provided (e.g. for _SimBehindTracker.feed)."""
    try:
        for raw in iter(proc.stdout.readline, b""):
            line = raw.decode("utf-8", errors="replace").rstrip("\n")
            lines.append(line)
            if on_line:
                on_line(line)
            if prefix is not None:
                print(f"{prefix}{line}", flush=True)
    except Exception:
        pass


def _wait_for_keyword(lines, proc, keyword, timeout):
    """
    Poll lines (filled by a background thread) until keyword is found,
    timeout expires, or proc exits early.
    Returns True if keyword was found.
    """
    deadline = time.time() + timeout
    seen = 0
    while time.time() < deadline:
        while seen < len(lines):
            if keyword in lines[seen]:
                return True
            seen += 1
        if proc.poll() is not None:
            time.sleep(0.1)  # drain any last lines
            while seen < len(lines):
                if keyword in lines[seen]:
                    return True
                seen += 1
            return False
        time.sleep(0.2)
    return False


# ============================================================
# ROS clock monitor
# ============================================================

def _clock_monitor_thread(threshold_sec, stop_event, exceeded_event, proc_ref):
    """
    Background thread: subscribes to SIM_CLOCK_TOPIC via `ros2 topic echo`,
    records the first sim timestamp as t0, and sets exceeded_event when
    (current_sim_time - t0) >= threshold_sec.

    proc_ref is a one-element list; the subprocess is stored there so the
    caller can kill it from outside if stop_event is triggered externally.

    Parses the YAML-style output of `ros2 topic echo --field clock /clock`:
        sec: 1234
        nanosec: 567000000
        ---
    """
    cmd = _ros2_cmd(["ros2", "topic", "echo", "--field", "clock", SIM_CLOCK_TOPIC])
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    proc_ref.append(proc)

    start_ns  = None
    cur_sec   = None
    cur_nsec  = None

    try:
        for raw in iter(proc.stdout.readline, b""):
            if stop_event.is_set():
                break

            line = raw.decode("utf-8", errors="replace").strip()

            if line == "---":
                # Process accumulated sec/nanosec pair
                if cur_sec is not None and cur_nsec is not None:
                    now_ns = cur_sec * 1_000_000_000 + cur_nsec
                    if start_ns is None:
                        start_ns = now_ns
                        print(
                            f"  [clock] Recording started at sim time "
                            f"{cur_sec}.{cur_nsec:09d}  (threshold: {threshold_sec}s)",
                            flush=True,
                        )
                    else:
                        elapsed = (now_ns - start_ns) / 1e9
                        if elapsed >= threshold_sec:
                            print(
                                f"  [clock] {elapsed:.2f}s of sim time elapsed "
                                f"(threshold: {threshold_sec}s). Stopping measurement.",
                                flush=True,
                            )
                            exceeded_event.set()
                            break
                cur_sec  = None
                cur_nsec = None
                continue

            # Must use ^sec: to avoid matching nanosec: line
            m = re.match(r"^sec:\s*(\d+)", line)
            if m:
                cur_sec = int(m.group(1))
                continue

            m = re.match(r"^nanosec:\s*(\d+)", line)
            if m:
                cur_nsec = int(m.group(1))

    except Exception:
        pass
    finally:
        _sigint_group(proc)
        _wait(proc, timeout=3)


# ============================================================
# Result parsing
# ============================================================

def _parse_summary(lines):
    """Extract stats from the ===== Summary ===== / ===== CARLA System Stats ===== blocks."""
    result = {}
    in_block = False
    for line in lines:
        if "===== Summary =====" in line:
            in_block = True
            continue
        if not in_block:
            continue

        def m(pattern):
            mo = re.search(pattern, line)
            return mo.group(1) if mo else None

        if (v := m(r"total window\s*:\s*(\S+)")):           result["total_window"] = v
        if (v := m(r"average rate.*?:\s*(\S+)")):            result["avg_rate_hz"]  = v
        if (v := m(r"min \(avg\)\s*:\s*(\S+)")):             result["min_s"]        = v
        if (v := m(r"max \(avg\)\s*:\s*(\S+)")):             result["max_s"]        = v
        if (v := m(r"std dev.*?:\s*(\S+)")):                 result["stddev_s"]     = v
        if (v := m(r"CPU usage\s*:\s*(\S+)")):               result["cpu_pct"]      = v
        if (v := m(r"Memory \(RSS\)\s*:\s*(\S+)")):          result["mem_mb"]       = v
        if (v := m(r"GPU temperature\s*:\s*(\S+)")):         result["gpu_temp_c"]   = v
        if (v := m(r"GPU power\s*:\s*(\S+)")):               result["gpu_power_w"]  = v
        if (v := m(r"GPU memory \(CARLA\)\s*:\s*(\S+)")):    result["gpu_mem_mib"]  = v
    return result


# ============================================================
# Single benchmark run
# ============================================================

def run_one(num_lidars, lidar_type, ros_clock_threshold):
    """
    Execute one (num_lidars, lidar_type) benchmark run.
    Stops measurement after ros_clock_threshold seconds of simulation time.
    Returns a dict with 'status' key plus parsed stats on success.
    """
    client_args, hz_args = LIDAR_TYPE_CONFIGS[lidar_type]

    print(f"\n{'='*64}")
    print(f"  RUN: num_lidars={num_lidars}  lidar_type={lidar_type}")
    print(f"{'='*64}")

    proc_carla   = None
    proc_client  = None
    proc_hz      = None
    t_hz         = None
    stop_clock   = threading.Event()
    clock_done   = threading.Event()
    clock_proc   = []   # clock monitor stores its subprocess here

    try:
        # --------------------------------------------------
        # [1] Launch CARLA
        # --------------------------------------------------
        carla_cmd = shlex.split(CARLA_CMD)
        print(f"[1] Starting CARLA ...")
        print(f"    {' '.join(carla_cmd)}")
        proc_carla = subprocess.Popen(
            carla_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        print(f"    PID={proc_carla.pid}. Waiting {CLIENT_LAUNCH_WAIT}s for CARLA to become ready ...")
        time.sleep(CLIENT_LAUNCH_WAIT)

        if proc_carla.poll() is not None:
            return {
                "num_lidars": num_lidars,
                "lidar_type": lidar_type,
                "status": f"error: CARLA exited early (rc={proc_carla.returncode})",
            }

        # --------------------------------------------------
        # [2] Launch CARLA client; wait for Ego spawn
        # --------------------------------------------------
        client_cmd = _ros2_cmd(
            shlex.split(f"python3 {CARLA_CLIENT_SCRIPT}")
            + ["--num_lidars", str(num_lidars)]
            + client_args
        )
        print(f"[2] Starting client ...")
        print(f"    {' '.join(client_cmd)}")
        proc_client = subprocess.Popen(
            client_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        client_lines = []
        sim_tracker  = _SimBehindTracker()
        threading.Thread(
            target=_drain_thread,
            args=(proc_client, client_lines, "    [client] "),
            kwargs={"on_line": sim_tracker.feed},
            daemon=True,
        ).start()

        print(f"    Waiting for '{EGO_SPAWN_KEYWORD}' (timeout={EGO_SPAWN_TIMEOUT}s) ...")
        if not _wait_for_keyword(client_lines, proc_client, EGO_SPAWN_KEYWORD, EGO_SPAWN_TIMEOUT):
            return {
                "num_lidars": num_lidars,
                "lidar_type": lidar_type,
                "status": f"timeout: Ego not spawned within {EGO_SPAWN_TIMEOUT}s",
            }
        print("    Ego spawned — proceeding.")

        # --------------------------------------------------
        # [3] Launch topic-hz monitor (runs until we send SIGINT)
        # --------------------------------------------------
        hz_cmd = _ros2_cmd(
            ["bash", TEST_TOPIC_HZ_CMD]
            + ["--num_lidars", str(num_lidars)]
            + hz_args
        )
        print(f"[3] Starting topic-hz monitor ...")
        print(f"    {' '.join(hz_cmd)}")
        proc_hz = subprocess.Popen(
            hz_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        hz_lines = []
        t_hz = threading.Thread(
            target=_drain_thread,
            args=(proc_hz, hz_lines, "    [hz] "),
            daemon=True,
        )
        t_hz.start()

        # --------------------------------------------------
        # [4] Start ROS clock monitor; wait for sim-time threshold
        # --------------------------------------------------
        print(f"[4] Monitoring {SIM_CLOCK_TOPIC} for {ros_clock_threshold}s of sim time ...")
        threading.Thread(
            target=_clock_monitor_thread,
            args=(ros_clock_threshold, stop_clock, clock_done, clock_proc),
            daemon=True,
        ).start()

        clock_done.wait()   # blocks until threshold reached or KeyboardInterrupt

        # --------------------------------------------------
        # [5] Stop hz (its cleanup trap prints the summary to stdout)
        # --------------------------------------------------
        print("  Sim-time threshold reached. Stopping topic-hz ...")
        _sigint_group(proc_hz)
        _wait(proc_hz, timeout=15)
        t_hz.join(timeout=5)  # ensure all buffered output (incl. summary) is captured

        stats = _parse_summary(hz_lines)
        sim_behind_ms = sim_tracker.get_final_ms(SIM_BEHIND_SILENCE_TIMEOUT)
        return {
            "num_lidars":    num_lidars,
            "lidar_type":    lidar_type,
            "status":        "ok",
            "sim_behind_ms": f"{sim_behind_ms:.1f}",
            **stats,
        }

    except KeyboardInterrupt:
        print("\n  [benchmark] KeyboardInterrupt — aborting run")
        return {"num_lidars": num_lidars, "lidar_type": lidar_type, "status": "aborted"}

    except Exception as exc:
        print(f"  ERROR: {exc}")
        return {"num_lidars": num_lidars, "lidar_type": lidar_type, "status": f"error: {exc}"}

    finally:
        # Abort clock monitor
        stop_clock.set()
        if clock_proc:
            _sigint_group(clock_proc[0])
            _wait(clock_proc[0], timeout=3)

        # Stop hz if still running (e.g. on early error exit)
        if proc_hz and proc_hz.poll() is None:
            _sigint_group(proc_hz)
            _wait(proc_hz, timeout=15)
        if t_hz:
            t_hz.join(timeout=5)

        print("  Stopping client (SIGINT) ...")
        _sigint_group(proc_client)
        _wait(proc_client, timeout=15)

        print("  Stopping CARLA (SIGINT) ...")
        _sigint_group(proc_carla)
        _wait(proc_carla, timeout=20)

        print("  All subprocesses stopped.")


# ============================================================
# Summary table
# ============================================================

# (column header, result-dict key, printf format)
_COLS = [
    ("lidar_type",   "lidar_type",    "%-12s"),
    ("N",            "num_lidars",    "%3s"),
    ("status",       "status",        "%-10s"),
    ("window",       "total_window",  "%7s"),
    ("Hz",           "avg_rate_hz",   "%9s"),
    ("min(s)",       "min_s",         "%8s"),
    ("max(s)",       "max_s",         "%8s"),
    ("std(s)",       "stddev_s",      "%8s"),
    ("behind(ms)",   "sim_behind_ms", "%10s"),
    ("CPU%",         "cpu_pct",       "%6s"),
    ("Mem(MB)",      "mem_mb",        "%8s"),
    ("GPU C",        "gpu_temp_c",    "%6s"),
    ("GPU W",        "gpu_power_w",   "%7s"),
    ("GPU MiB",      "gpu_mem_mib",   "%8s"),
]


def print_summary(results):
    header = "  ".join(fmt % hdr for hdr, _, fmt in _COLS)
    sep    = "-" * len(header)
    print(f"\n\n{'='*len(header)}")
    print("BENCHMARK SUMMARY")
    print(f"{'='*len(header)}")
    print(header)
    print(sep)
    for r in results:
        if r is None:
            continue
        print("  ".join(fmt % r.get(key, "N/A") for _, key, fmt in _COLS))
    print(f"{'='*len(header)}")


# ============================================================
# Entry point
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Automated CARLA LiDAR benchmark across lidar types and LiDAR counts",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--ros-clock-threshold", type=float, required=True,
        help=f"Seconds of simulation time (from {SIM_CLOCK_TOPIC}) to run each measurement",
    )
    parser.add_argument(
        "--num_lidars", type=str, required=True,
        help="Comma-separated list of LiDAR counts, e.g. 1,2,4",
    )
    args = parser.parse_args()

    num_lidars_list = [int(x.strip()) for x in args.num_lidars.split(",")]
    lidar_types     = list(LIDAR_TYPE_CONFIGS.keys())
    total_runs      = len(num_lidars_list) * len(lidar_types)

    print("=" * 64)
    print("CARLA LiDAR Benchmark")
    print(f"  num_lidars list      : {num_lidars_list}")
    print(f"  lidar_types          : {lidar_types}")
    print(f"  ros-clock-threshold  : {args.ros_clock_threshold}s  (topic: {SIM_CLOCK_TOPIC})")
    print(f"  total runs           : {total_runs}")
    print("=" * 64)

    results  = []
    run_idx  = 0
    aborted  = False

    for n in num_lidars_list:
        if aborted:
            break
        for lidar_type in lidar_types:
            run_idx += 1
            print(f"\n[Run {run_idx}/{total_runs}]")
            result = run_one(n, lidar_type, args.ros_clock_threshold)
            results.append(result)

            if result.get("status") == "aborted":
                aborted = True
                print("Benchmark aborted by user.")
                break

            if run_idx < total_runs:
                print(f"  Pausing {INTER_RUN_PAUSE}s before next run ...")
                time.sleep(INTER_RUN_PAUSE)

    print_summary(results)


if __name__ == "__main__":
    main()
