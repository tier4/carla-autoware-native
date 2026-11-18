#!/bin/bash
# ============================================================================
# Multi-sensor distance-culling regression test (automated)
#
# Launches the packaged CARLA server, then for each sensor config resets the
# world (clean baseline), runs the RGL demo, and reads the server's
# RGLSceneManager log. Verifies the union behavior: a spatially separated 2nd
# sensor must register ADDITIONAL geometry (the mesh count rises vs single
# sensor) — the regression the old first-tick-wins single-sensor design failed.
#
# The counts come from the scene manager's `rgl_scene_sync` lines (one per
# static sync, in every build configuration): `meshes=` (RGL meshes resident in
# the mesh cache) and `sensors=` (registered sensors).
#
# Runs unattended. Requires a GPU. PKG_DIR, DEMO and PORT are required
# (exit 2 without them); override any other VAR=... at invocation, e.g.:
#   PKG_DIR=<package dir> DEMO=<.../rgl_test_autoware_demo.py> PORT=4000 SEP=250 bash test_multisensor_culling.sh
# PY must be a python with the carla module and PyKDL (the demo imports both).
# ============================================================================
set -u

# ---- config (override via env) ----
PKG_DIR="${PKG_DIR:-}"            # packaged server dir holding exactly one launcher script (*.sh)
DEMO="${DEMO:-}"                  # PythonAPI/examples/rgl_test_autoware_demo.py
PORT="${PORT:-}"                  # RPC port of the server (the demo and the reset use it too)
PY="${PY:-/mnt/dsk0/wk0/CARLA/RGLWP/venv/bin/python}"
MAP="${MAP:-Town10HD_Opt}"
DURATION="${DURATION:-25}"        # seconds each config runs before SIGINT
SEP="${SEP:-150}"                 # meters between the 2 separated sensors (config c)
NOCULL_RANGE="${NOCULL_RANGE:-5000}"  # LiDAR range (m) for the no-culling baseline (covers whole map)
RESET_WAIT="${RESET_WAIT:-9}"     # > SensorStaleTimeoutSeconds(5s): let registry clear
SERVER_EXTRA="${SERVER_EXTRA:--RenderOffScreen -nosound -quality-level=Low}"
SERVER_ROS2="${SERVER_ROS2---ros2}"   # set to empty (SERVER_ROS2=) to start the server without its ROS 2 (DDS) interface
LOG="${LOG:-/tmp/carla_server_$$.log}"
RESULT="${RESULT:-/tmp/rgl_multisensor_result.txt}"

usage() { echo "[test] ERROR: $1 (required: PKG_DIR, DEMO, PORT; optional: PY, MAP, SEP, ...)" >&2; exit 2; }
[ -n "$PKG_DIR" ] || usage "PKG_DIR is not set"
[ -n "$DEMO" ] || usage "DEMO is not set"
[ -n "$PORT" ] || usage "PORT is not set"
[ -d "$PKG_DIR" ] || usage "PKG_DIR is not a directory: $PKG_DIR"
[ -f "$DEMO" ] || usage "DEMO not found: $DEMO"
case "$PORT" in ''|*[!0-9]*) usage "PORT is not a number: $PORT";; esac
"$PY" -c "import carla, PyKDL" 2>/dev/null </dev/null || usage "PY ($PY) cannot import carla and PyKDL"
LAUNCHERS=("$PKG_DIR"/*.sh)
[ "${#LAUNCHERS[@]}" -eq 1 ] && [ -f "${LAUNCHERS[0]}" ] || usage "PKG_DIR must hold exactly one launcher script (*.sh)"
SERVER_SH=${LAUNCHERS[0]}

: > "$RESULT"
log() { echo "[test] $*" | tee -a "$RESULT"; }

SERVER_PID=
cleanup() {
    [ -n "$SERVER_PID" ] || return 0
    log "tearing down server group (pgid=$SERVER_PID)"
    # The launcher starts the real binary as a child: signal the whole process group (setsid below), then
    # check the group is gone. No name-based kill: it could hit another session's server.
    kill -INT -- "-$SERVER_PID" 2>/dev/null
    for _ in $(seq 1 15); do kill -0 -- "-$SERVER_PID" 2>/dev/null || break; sleep 1; done
    kill -9 -- "-$SERVER_PID" 2>/dev/null
    sleep 1
    if kill -0 -- "-$SERVER_PID" 2>/dev/null; then log "WARNING: server group $SERVER_PID still alive"; fi
    SERVER_PID=
}
trap cleanup EXIT
trap 'exit 130' INT TERM   # the EXIT trap stops the server

# ---- 1. launch server in its own process group (setsid) ----
if (echo > "/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; then log "ERROR: port $PORT is already in use"; exit 3; fi
log "launching CARLA: $MAP $SERVER_ROS2 -prefernvidia $SERVER_EXTRA -carla-rpc-port=$PORT"
setsid "$SERVER_SH" "$MAP" $SERVER_ROS2 -prefernvidia $SERVER_EXTRA -carla-rpc-port="$PORT" > "$LOG" 2>&1 < /dev/null &
SERVER_PID=$!   # session leader => PGID == SERVER_PID

# ---- 2. wait for RPC port ----
log "waiting for RPC port $PORT ..."
ready=0
for i in $(seq 1 90); do
    if (echo > "/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; then ready=1; break; fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then log "ERROR: server died early. tail:"; tail -20 "$LOG" | tee -a "$RESULT"; exit 3; fi
    sleep 2
done
[ "$ready" = 1 ] || { log "ERROR: server not ready after ~180s. tail:"; tail -20 "$LOG" | tee -a "$RESULT"; exit 3; }
log "server ready; warm-up 12s"
sleep 12

# ---- helpers ----
# max integer value of a \K-anchored PCRE over the rgl_scene_sync lines of a log window (from line $1)
max_sync_field() { tail -n +"$1" "$LOG" | grep -a 'rgl_scene_sync ' | grep -oP "$2" 2>/dev/null | sort -n | tail -1; }

reset_world() {  # destroy lingering actors so each config starts from a clean registry
    timeout 30 "$PY" - "$PORT" <<'PY' 2>/dev/null || true
import sys, carla
c = carla.Client('127.0.0.1', int(sys.argv[1])); c.set_timeout(20)
w = c.get_world()
for pat in ('sensor.*', 'vehicle.*', 'walker.*'):
    for a in w.get_actors().filter(pat):
        try: a.destroy()
        except Exception: pass
PY
    sleep "$RESET_WAIT"   # let UnregisterSensor + stale-TTL clear the registry & static meshes
}

declare -A CACHED SENSORS VRAM
run_config() {  # $1=name  rest=demo args
    local name="$1"; shift
    log "--- config '$name': reset + demo $* (${DURATION}s) ---"
    reset_world
    local before; before=$(( $(wc -l < "$LOG") + 1 ))
    env ${DEMO_ENV:-} timeout --signal=INT "$DURATION" "$PY" "$DEMO" --port "$PORT" --carla_lidar_type rgl "$@" \
        > "/tmp/demo_${name}.log" 2>&1 < /dev/null || true
    sleep 3
    VRAM[$name]=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
    SENSORS[$name]=$(max_sync_field "$before" ' sensors=\K[0-9]+')
    CACHED[$name]=$(max_sync_field "$before" ' meshes=\K[0-9]+')
    log "  VRAM=${VRAM[$name]}MiB  maxSensors=${SENSORS[$name]:-?}  maxMeshes=${CACHED[$name]:-?}"
    log "  last sync: $(tail -n +"$before" "$LOG" | grep -a 'rgl_scene_sync ' | tail -1 | sed 's/.*rgl_scene_sync/rgl_scene_sync/')"
}

# ---- 3. run configs ----
# Culled configs first so their measurements are captured even if the no-cull
# baseline below spikes VRAM / OOMs on a large map.
run_config a_single    --num_lidars 1
run_config b_cluster   --num_lidars 4 --lidar_spawn_delta_x 2 --lidar_spawn_delta_y 2
run_config c_separated --num_lidars 2 --lidar_spawn_delta_x "$SEP"
# no-cull baseline LAST: 1 sensor with a huge range -> registration sphere covers
# the whole map -> every static mesh registered (= without culling). On a large map this
# is the OOM scenario the feature prevents; if the server dies here, that crash is
# itself the headline (culling avoids it) and the culled data above is preserved.
DEMO_ENV="RGL_NOCULL_RANGE_M=$NOCULL_RANGE" run_config d_nocull --num_lidars 1

# ---- 4. verdict ----
nc=${CACHED[d_nocull]:-0}; ncv=${VRAM[d_nocull]:-0}
a=${CACHED[a_single]:-0};  av=${VRAM[a_single]:-0}
c=${CACHED[c_separated]:-0}
sa=${SENSORS[a_single]:-0}; sc=${SENSORS[c_separated]:-0}
pct=$(awk "BEGIN{ if ($nc+0>0) printf \"%.0f\", 100*$a/$nc; else print 0 }")
saved_m=$(( nc - a ))

log "============================================================"
log "RESULT SUMMARY  (map=$MAP, sep=${SEP}m, nocull-range=${NOCULL_RANGE}m)"
log "  (d) NO-CULL   : maxSensors=${SENSORS[d_nocull]:-?}  maxMeshes=${CACHED[d_nocull]:-?}  VRAM=${VRAM[d_nocull]:-?}MiB  (whole map registered)"
log "  (a) single    : maxSensors=${SENSORS[a_single]:-?}  maxMeshes=${CACHED[a_single]:-?}  VRAM=${VRAM[a_single]:-?}MiB"
log "  (b) cluster x4: maxSensors=${SENSORS[b_cluster]:-?}  maxMeshes=${CACHED[b_cluster]:-?}  VRAM=${VRAM[b_cluster]:-?}MiB"
log "  (c) separated : maxSensors=${SENSORS[c_separated]:-?}  maxMeshes=${CACHED[c_separated]:-?}  VRAM=${VRAM[c_separated]:-?}MiB"
log "------------------------------------------------------------"
log "CULLING BENEFIT (mesh count = the RGL/OptiX VRAM driver):"
log "  meshes : no-cull=$nc (whole map)  culled single=$a   -> culled keeps ${pct}%, ${saved_m} fewer meshes"
log "  VRAM (total GPU, informational only): no-cull=${ncv}MiB single=${av}MiB cluster=${VRAM[b_cluster]:-?}MiB sep=${VRAM[c_separated]:-?}MiB"
log "  note: nvidia-smi reports TOTAL GPU memory (engine+render dominated, ~GB) so the"
log "        mesh-registration delta is below the noise floor on a small map. The GB-scale"
log "        VRAM saving appears on large maps where no-cull exhausts OptiX memory;"
log "        the mesh count is the reliable proxy (mesh data dominates OptiX VRAM)."
log "------------------------------------------------------------"
verdict=0
[ "${a:-0}" -gt 0 ] || { log "FAIL: (a) produced no rgl_scene_sync meshes= reading"; verdict=1; }
if [ "${nc:-0}" -gt "${a:-0}" ]; then log "PASS: culling reduces registered meshes (no-cull=$nc > culled single=$a)"; else log "FAIL: expected no-cull($nc) > culled single($a) — map may fit fully within cull radius (try a larger map)"; verdict=1; fi
if [ "${sc:-0}" -gt "${sa:-0}" ]; then log "PASS: more sensors registered when separated (c=$sc > a=$sa)"; else log "FAIL: expected sensors(c) > sensors(a), got c=$sc a=$sa"; verdict=1; fi
if [ "${c:-0}" -gt "${a:-0}" ]; then log "PASS: union registers more geometry for separated sensors (c=$c > a=$a)"; else log "FAIL: expected meshes(c) > meshes(a), got c=$c a=$a — if c≈a, increase SEP and re-run"; verdict=1; fi
[ "$verdict" = 0 ] && log "OVERALL: PASS" || log "OVERALL: FAIL (full server log: $LOG)"
log "============================================================"
exit "$verdict"
