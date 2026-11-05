#!/usr/bin/env bash
# Start/stop the packaged CARLA server by PID with RPC readiness wait. Never start on a busy port.
set -o pipefail
# Default derived from the script location (PythonAPI/rgl/tests/ -> repo root), not a developer path.
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
BIN=${CARLA_BIN:-$REPO/Build/Package/Carla-0.10.0-Linux-Shipping/Linux/CarlaUnreal.sh}
PORT=${CARLA_PORT:-2000}
cmd=$1; label=$2; shift 2 || true
pidf=/tmp/carla-${label}.pid; logf=/tmp/carla-${label}.log
port_busy() { ss -ltn 2>/dev/null | grep -q ":${PORT} "; }
case "$cmd" in
  start)
    port_busy && { echo "port ${PORT} busy; stop the other server first"; exit 1; }
    python3 -c "import carla" 2>/tmp/carla-${label}.precheck.err || { echo "python3 cannot import carla: $(tail -1 /tmp/carla-${label}.precheck.err)"; exit 1; }
    "$BIN" -RenderOffScreen -nosound -carla-rpc-port=${PORT} "$@" >"$logf" 2>&1 &
    pid=$!
    # "<pid> <port>": stop needs the port to describe a stale entry it refuses to signal.
    echo "$pid ${PORT}" >"$pidf"; echo "started pid=$pid port=${PORT} log=$logf args=$*"
    deadline=$((SECONDS+180))
    while [ $SECONDS -lt $deadline ]; do
      python3 - <<PY 2>/tmp/carla-${label}.rpc.err && { echo "rpc ready after $((SECONDS))s"; exit 0; }
import carla; c=carla.Client("127.0.0.1", ${PORT}); c.set_timeout(1.0); c.get_server_version()
PY
      sleep 1; kill -0 "$pid" 2>/dev/null || { echo "server died; see $logf"; exit 1; }
    done
    echo "rpc not ready after 180s: $(tail -1 /tmp/carla-${label}.rpc.err)"; exit 1 ;;
  stop)
    [ -f "$pidf" ] || { echo "no pid file"; exit 0; }
    read -r pid pidport <"$pidf"
    [ -n "$pid" ] || { echo "pid file $pidf is empty; removing"; rm -f "$pidf"; exit 1; }
    # The PID may have been recycled since the server died. Only signal a real CarlaUnreal process.
    if ! { tr '\0' ' ' </proc/$pid/cmdline | grep -q CarlaUnreal; } 2>/dev/null; then
      echo "pid $pid is not a CarlaUnreal process; refusing (removing stale pid file for port ${pidport:-?})"
      rm -f "$pidf"; exit 1
    fi
    pkill -TERM -P "$pid" 2>/dev/null; kill -TERM "$pid" 2>/dev/null
    for i in $(seq 1 60); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
    kill -0 "$pid" 2>/dev/null && { kill -KILL "$pid"; pkill -KILL -P "$pid" 2>/dev/null; }
    for i in $(seq 1 30); do port_busy || break; sleep 1; done
    port_busy && { echo "port still busy"; exit 1; }
    rm -f "$pidf"; echo "stopped ${label}" ;;
  status) port_busy && echo "port ${PORT} busy" || echo "port ${PORT} free"; ls /tmp/carla-*.pid 2>/dev/null || echo "no pid files"; exit 0 ;;
  *) echo "usage: $0 start|stop|status <label> [ue args]"; exit 2 ;;
esac
