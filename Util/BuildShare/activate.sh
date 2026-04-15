# activate.sh
#
# Source this file to update CARLA_UNREAL_ENGINE_PATH to the cloned engine.
# Usage: source Util/BuildShare/activate.sh
#
# Can be run from any directory — resolves paths relative to the script location.

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
_PROJECT_ROOT="$(cd "$_SCRIPT_DIR/../.." && pwd -P)"
_CLONE_PATH_FILE="$_PROJECT_ROOT/.ue5_engine_clone_path"

if [ ! -f "$_CLONE_PATH_FILE" ]; then
    echo "ERROR: No engine clone found. Run 'Util/BuildShare/build-share.sh setup' first." >&2
    unset _SCRIPT_DIR _PROJECT_ROOT _CLONE_PATH_FILE
    return 1 2>/dev/null || true
fi

_NEW_PATH="$(cat "$_CLONE_PATH_FILE")"

if [ ! -d "$_NEW_PATH" ]; then
    echo "ERROR: Clone directory does not exist: $_NEW_PATH" >&2
    unset _SCRIPT_DIR _PROJECT_ROOT _CLONE_PATH_FILE _NEW_PATH
    return 1 2>/dev/null || true
fi

_OLD_PATH="${CARLA_UNREAL_ENGINE_PATH:-<unset>}"
export CARLA_UNREAL_ENGINE_PATH="$_NEW_PATH"

echo "CARLA_UNREAL_ENGINE_PATH updated:"
echo "  before: $_OLD_PATH"
echo "  after:  $CARLA_UNREAL_ENGINE_PATH"

unset _SCRIPT_DIR _PROJECT_ROOT _CLONE_PATH_FILE _NEW_PATH _OLD_PATH
