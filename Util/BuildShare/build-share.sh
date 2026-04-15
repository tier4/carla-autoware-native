#!/bin/bash
# build-share.sh
#
# Shared CARLA development environment setup tool.
# Creates hardlink copies of UE5 Engine, symlinks for Content,
# and optionally symlinks RobotecGPULidar from an existing 1st environment.
#
# Usage:
#   Util/BuildShare/build-share.sh setup   <1st-carla-root>   # Engine + Content + RGL
#   Util/BuildShare/build-share.sh engine  <1st-carla-root>   # Engine hardlink copy only
#   Util/BuildShare/build-share.sh content <1st-carla-root>   # Content symlink only
#   Util/BuildShare/build-share.sh check-update               # Check repos for updates
#
# Sharing model:
#   Engine (hardlink copy) — independent snapshot at clone time.
#     Each environment has its own copy of files modified by builds.
#     The clone has no .git; to update, delete and re-clone.
#   Content (symlink) — shared across all environments.
#     git pull in the Content repo affects all linked environments.
#   RGL (symlink, optional) — shared across all environments.
#     Linked only if RobotecGPULidar/ exists in the 1st environment.
#     Build artifacts (build/lib/) are also shared via the symlink.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd -P)"

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

_log_info()  { echo "INFO:  $*" >&2; }
_log_warn()  { echo "WARNING: $*" >&2; }
_log_error() { echo "ERROR: $*" >&2; }

# ---------------------------------------------------------------------------
# Validation utilities
# ---------------------------------------------------------------------------

# Check that a directory looks like a UE5 engine.
_validate_ue5_dir() {
    local dir="$1"
    if [ ! -f "$dir/Engine/Build/BatchFiles/RunUAT.sh" ]; then
        _log_error "Does not look like a UE5 engine directory: $dir"
        _log_error "  (missing Engine/Build/BatchFiles/RunUAT.sh)"
        return 1
    fi
}

# Resolve the CarlaUE5 project root from a given path.
# If the path itself is not a project root but contains a CarlaUE5/ subdir, use that.
# Prints the resolved path to stdout.
_resolve_carla_root() {
    local dir="$1"
    if [ -f "$dir/CMakeLists.txt" ] && [ -d "$dir/Unreal/CarlaUnreal" ]; then
        echo "$dir"
        return 0
    fi
    # Try CarlaUE5 subdirectory
    if [ -f "$dir/CarlaUE5/CMakeLists.txt" ] && [ -d "$dir/CarlaUE5/Unreal/CarlaUnreal" ]; then
        _log_info "Using $dir/CarlaUE5 as project root."
        echo "$dir/CarlaUE5"
        return 0
    fi
    _log_error "Does not look like a CarlaUE5 project root: $dir"
    if [ ! -f "$dir/CMakeLists.txt" ]; then
        _log_error "  (missing CMakeLists.txt)"
    fi
    if [ ! -d "$dir/Unreal/CarlaUnreal" ]; then
        _log_error "  (missing Unreal/CarlaUnreal/)"
    fi
    return 1
}

# Check that a directory looks like a CarlaUE5 project root.
_validate_carla_root() {
    local dir="$1"
    if [ ! -f "$dir/CMakeLists.txt" ]; then
        _log_error "Does not look like a CarlaUE5 project root: $dir"
        _log_error "  (missing CMakeLists.txt)"
        return 1
    fi
    if [ ! -d "$dir/Unreal/CarlaUnreal" ]; then
        _log_error "Does not look like a CarlaUE5 project root: $dir"
        _log_error "  (missing Unreal/CarlaUnreal/)"
        return 1
    fi
}

# Check that source and dest are on the same filesystem using df.
_check_same_filesystem() {
    local src="$1"
    local dest_parent="$2"
    local src_dev dest_dev
    src_dev="$(df --output=source "$src" 2>/dev/null | tail -1)"
    dest_dev="$(df --output=source "$dest_parent" 2>/dev/null | tail -1)"
    if [ "$src_dev" != "$dest_dev" ]; then
        _log_error "Source and destination are on different filesystems."
        _log_error "  Source:      $src ($src_dev)"
        _log_error "  Destination: $dest_parent ($dest_dev)"
        _log_error ""
        _log_error "  Hardlink copies require the same filesystem."
        return 1
    fi
}

# ---------------------------------------------------------------------------
# Engine source detection
# ---------------------------------------------------------------------------

# Detect the UE5 engine path used by the 1st environment.
# Checks CMakeCache.txt first, then globs for UnrealEngine5_carla*.
# Outputs the resolved engine path to stdout.
_detect_engine_source() {
    local first_root="$1"
    local engine_path=""

    # Strategy 1: Read from CMakeCache.txt
    local cache_file="$first_root/Build/CMakeCache.txt"
    if [ -f "$cache_file" ]; then
        engine_path="$(grep '^CARLA_UNREAL_ENGINE_PATH:' "$cache_file" | cut -d= -f2- || true)"
        if [ -n "$engine_path" ] && [ -d "$engine_path" ]; then
            echo "$engine_path"
            return 0
        fi
    fi

    # Strategy 2: Glob for sibling directories
    local first_parent
    first_parent="$(dirname "$(realpath "$first_root")")"
    local candidates=()
    for d in "$first_parent"/UnrealEngine5_carla*; do
        if [ -d "$d" ] && [ -f "$d/Engine/Build/BatchFiles/RunUAT.sh" ]; then
            candidates+=("$d")
        fi
    done

    if [ ${#candidates[@]} -eq 0 ]; then
        _log_error "No UE5 engine found for 1st environment."
        _log_error "  Checked: $cache_file"
        _log_error "  Checked: $first_parent/UnrealEngine5_carla*"
        _log_error ""
        _log_error "  Has the 1st environment been built? Run CarlaSetup.sh first."
        return 1
    fi

    if [ ${#candidates[@]} -eq 1 ]; then
        echo "${candidates[0]}"
        return 0
    fi

    # Multiple candidates — present them
    _log_warn "Multiple UE5 engine directories found:"
    local i=1
    for d in "${candidates[@]}"; do
        _log_warn "  [$i] $d"
        i=$((i + 1))
    done
    _log_error "Cannot auto-select. Use the 'engine' subcommand to specify directly,"
    _log_error "or ensure the 1st environment has Build/CMakeCache.txt."
    return 1
}

# ---------------------------------------------------------------------------
# Clone detection
# ---------------------------------------------------------------------------

# If the given engine path is a clone (has .carla-env-origin), resolve to the
# original. Follows the chain up to 2 levels. Prints the resolved path to
# stdout. Returns 0 on success, 1 on fatal error.
_resolve_source_origin() {
    local engine_path="$1"
    local depth=0
    local max_depth=2
    local current="$engine_path"

    while [ $depth -lt $max_depth ]; do
        local origin_file="$current/.carla-env-origin"
        if [ ! -f "$origin_file" ]; then
            # Not a clone — check for .git as quality signal
            if [ ! -d "$current/.git" ]; then
                _log_warn "No .git directory in $current"
                _log_warn "  --check-update will not work for this engine."
            fi
            echo "$current"
            return 0
        fi

        # It's a clone — read the original path
        local original
        original="$(cat "$origin_file")"
        _log_warn "Specified engine is a clone: $current"
        _log_warn "  Original recorded as: $original"

        if [ ! -d "$original" ]; then
            _log_error "Original engine directory does not exist: $original"
            _log_error "  Please specify the correct 1st environment path."
            return 1
        fi

        current="$original"
        depth=$((depth + 1))
    done

    _log_warn "Resolved to original after $depth hops: $current"
    echo "$current"
    return 0
}

# ---------------------------------------------------------------------------
# Engine hardlink copy
# ---------------------------------------------------------------------------

# Create a hardlink copy of the UE5 engine using rsync --link-dest.
# Args: $1=source engine path, $2=destination path
_do_engine_clone() {
    local source="$1"
    local dest="$2"

    # Pre-checks
    _validate_ue5_dir "$source"

    if [ -d "$dest" ] || [ -L "$dest" ]; then
        _log_error "Destination already exists: $dest"
        _log_error "  Remove it first if you want to re-clone:"
        _log_error "  rm -rf $dest"
        return 1
    fi

    local dest_parent
    dest_parent="$(dirname "$dest")"
    mkdir -p "$dest_parent"

    _check_same_filesystem "$source" "$dest_parent"

    # Trap for cleanup on interrupt
    trap '_log_error "Interrupted. Removing partial clone: $dest"; rm -rf "$dest"; exit 1' INT TERM

    echo "========================================"
    echo "UE5 Engine Hardlink Clone"
    echo "========================================"
    echo "  Source:      $source"
    echo "  Destination: $dest"
    echo ""

    _log_info "Creating hardlink copy (excluding .git and build caches)..."
    set +e
    rsync -a --link-dest="$source" \
        --exclude='.git' \
        --exclude='Engine/Programs/AutomationTool/Saved/' \
        --exclude='Engine/Programs/UnrealBuildTool/Log.uba' \
        "$source/" "$dest/"
    local rsync_rc=$?
    set -e
    if [ $rsync_rc -ne 0 ]; then
        _log_error "Hardlink copy failed (rsync exit=$rsync_rc). Removing partial clone."
        rm -rf "$dest"
        trap - INT TERM
        return 1
    fi

    # Verify
    if [ ! -f "$dest/Engine/Build/BatchFiles/RunUAT.sh" ]; then
        _log_error "Clone verification failed."
        rm -rf "$dest"
        return 1
    fi

    # Write origin marker
    echo "$source" > "$dest/.carla-env-origin"

    # Clear trap
    trap - INT TERM

    local src_size dest_size
    src_size="$(du -sh "$source" --exclude=.git 2>/dev/null | cut -f1)"
    dest_size="$(du -sh "$dest" 2>/dev/null | cut -f1)"
    _log_info "Done."
    _log_info "  Source size (excl .git): $src_size"
    _log_info "  Clone size: $dest_size (hardlinks — actual new disk usage is minimal)"
    echo ""
}

# ---------------------------------------------------------------------------
# Content symlink
# ---------------------------------------------------------------------------

# Create a symlink for Unreal/CarlaUnreal/Content pointing to 1st env.
# Args: $1=source Content path
_do_content_link() {
    local source="$1"
    local dest="$PROJECT_ROOT/Unreal/CarlaUnreal/Content"

    if [ ! -d "$source" ]; then
        _log_error "Source Content directory not found: $source"
        return 1
    fi

    # Check parent directory exists (git clone must be done first)
    if [ ! -d "$PROJECT_ROOT/Unreal/CarlaUnreal" ]; then
        _log_error "Unreal/CarlaUnreal/ does not exist in current project."
        _log_error "  Has 'git clone' been completed for this project?"
        return 1
    fi

    if [ -d "$dest" ] || [ -L "$dest" ]; then
        if [ -L "$dest" ]; then
            local existing_target
            existing_target="$(readlink -f "$dest")"
            local source_real
            source_real="$(realpath "$source")"
            if [ "$existing_target" = "$source_real" ]; then
                _log_info "Content symlink already points to the correct target."
                _log_info "  $dest → $source"
                return 0
            fi
        fi
        _log_error "Content destination already exists: $dest"
        _log_error "  Remove it first if you want to re-link:"
        _log_error "  rm -rf $dest"
        return 1
    fi

    echo "========================================"
    echo "Content Symlink"
    echo "========================================"
    echo "  Source: $source"
    echo "  Link:   $dest"
    echo ""

    ln -s "$(realpath "$source")" "$dest"

    # Verify
    if [ ! -d "$dest" ]; then
        _log_error "Symlink verification failed."
        return 1
    fi

    _log_info "Done. Content symlink created."
    echo ""
}

# ---------------------------------------------------------------------------
# RGL symlink
# ---------------------------------------------------------------------------

# Create a symlink for ../RobotecGPULidar pointing to 1st env's RGL directory.
# Args: $1=source RGL directory (absolute path)
# Skips silently if source does not exist (RGL is optional).
_do_rgl_link() {
    local source="$1"
    local dest="$(dirname "$PROJECT_ROOT")/RobotecGPULidar"

    if [ ! -d "$source" ]; then
        _log_info "RGL not found in 1st environment: $source"
        _log_info "  Skipping RGL symlink (RGL is optional)."
        return 0
    fi

    if [ -d "$dest" ] || [ -L "$dest" ]; then
        if [ -L "$dest" ]; then
            local existing_target
            existing_target="$(readlink -f "$dest")"
            local source_real
            source_real="$(realpath "$source")"
            if [ "$existing_target" = "$source_real" ]; then
                _log_info "RGL symlink already points to the correct target."
                _log_info "  $dest → $source"
                return 0
            fi
        fi
        _log_error "RGL destination already exists: $dest"
        _log_error "  Remove it first if you want to re-link:"
        _log_error "  rm -rf $dest"
        return 1
    fi

    echo "========================================"
    echo "RGL Symlink"
    echo "========================================"
    echo "  Source: $source"
    echo "  Link:   $dest"
    echo ""

    ln -s "$(realpath "$source")" "$dest"

    if [ ! -d "$dest" ]; then
        _log_error "Symlink verification failed."
        return 1
    fi

    # Verify build artifacts exist
    if [ -f "$dest/build/lib/libRobotecGPULidar.so" ]; then
        _log_info "Done. RGL symlink created (build artifacts found)."
    else
        _log_warn "Done. RGL symlink created, but build artifacts not found."
        _log_warn "  Run 'RglSetup.sh prepare' in the 1st environment to build RGL."
    fi
    echo ""
}

# ---------------------------------------------------------------------------
# Config file generation
# ---------------------------------------------------------------------------

# Write .carla-env.conf to the current project root.
# Empty arguments preserve existing values (merge semantics).
_write_config() {
    local source_project="$1"
    local engine_source="$2"
    local engine_clone="$3"
    local content_source="$4"
    local conf_file="$PROJECT_ROOT/.carla-env.conf"

    # Merge with existing config if present
    if [ -f "$conf_file" ]; then
        [ -z "$source_project" ] && source_project="$(grep '^source_project=' "$conf_file" | cut -d= -f2- || true)"
        [ -z "$engine_source" ]  && engine_source="$(grep '^engine_source=' "$conf_file" | cut -d= -f2- || true)"
        [ -z "$engine_clone" ]   && engine_clone="$(grep '^engine_clone=' "$conf_file" | cut -d= -f2- || true)"
        [ -z "$content_source" ] && content_source="$(grep '^content_source=' "$conf_file" | cut -d= -f2- || true)"
    fi

    cat > "$conf_file" <<EOF
# Auto-generated by build-share.sh on $(date -Iseconds)
source_project=$source_project
engine_source=$engine_source
engine_clone=$engine_clone
content_source=$content_source
EOF

    _log_info "Config written: $conf_file"
}

# Read .carla-env.conf. Sets global variables:
#   CONF_SOURCE_PROJECT, CONF_ENGINE_SOURCE, CONF_ENGINE_CLONE, CONF_CONTENT_SOURCE
_read_config() {
    local conf_file="$PROJECT_ROOT/.carla-env.conf"
    if [ ! -f "$conf_file" ]; then
        _log_error "No .carla-env.conf found in $PROJECT_ROOT"
        _log_error "  Run 'build-share.sh setup' first."
        return 1
    fi
    CONF_SOURCE_PROJECT="$(grep '^source_project=' "$conf_file" | cut -d= -f2-)"
    CONF_ENGINE_SOURCE="$(grep '^engine_source=' "$conf_file" | cut -d= -f2-)"
    CONF_ENGINE_CLONE="$(grep '^engine_clone=' "$conf_file" | cut -d= -f2-)"
    CONF_CONTENT_SOURCE="$(grep '^content_source=' "$conf_file" | cut -d= -f2-)"
}

# ---------------------------------------------------------------------------
# CARLA_UNREAL_ENGINE_PATH validation
# ---------------------------------------------------------------------------

# Check current env var and shell config files for CARLA_UNREAL_ENGINE_PATH.
_check_engine_path_env() {
    local expected_path="$1"

    echo "----------------------------------------"
    echo "CARLA_UNREAL_ENGINE_PATH check"
    echo "----------------------------------------"

    # Check 1: Current environment variable
    if [ -n "${CARLA_UNREAL_ENGINE_PATH:-}" ]; then
        local current_real expected_real
        current_real="$(realpath "$CARLA_UNREAL_ENGINE_PATH" 2>/dev/null || echo "$CARLA_UNREAL_ENGINE_PATH")"
        expected_real="$(realpath "$expected_path" 2>/dev/null || echo "$expected_path")"
        if [ "$current_real" = "$expected_real" ]; then
            _log_info "CARLA_UNREAL_ENGINE_PATH matches the clone path."
        else
            _log_warn "CARLA_UNREAL_ENGINE_PATH is set but does not match the clone."
            _log_warn "  Current:  $CARLA_UNREAL_ENGINE_PATH"
            _log_warn "  Expected: $expected_path"
        fi
    else
        _log_info "CARLA_UNREAL_ENGINE_PATH is not set."
        _log_info "  To set it for this session:"
        _log_info "    source Util/BuildShare/activate.sh"
        _log_info "  Or manually:"
        _log_info "    export CARLA_UNREAL_ENGINE_PATH=$expected_path"
    fi

    # Check 2: Shell config files
    local shell_files=("$HOME/.bashrc" "$HOME/.bash_profile" "$HOME/.profile" "$HOME/.zshrc")
    local found_in=()

    for f in "${shell_files[@]}"; do
        [ -f "$f" ] || continue
        # Match active (non-commented) definitions
        local matches
        matches="$(grep -n '^[^#]*CARLA_UNREAL_ENGINE_PATH=' "$f" 2>/dev/null || true)"
        if [ -n "$matches" ]; then
            found_in+=("$f")
            while IFS= read -r line; do
                _log_warn "  $f:$line"
            done <<< "$matches"
        fi
    done

    if [ ${#found_in[@]} -gt 0 ]; then
        _log_warn "CARLA_UNREAL_ENGINE_PATH is defined in shell config file(s) above."
        _log_warn "  This can cause conflicts between environments."
        _log_warn "  Recommended: comment out or remove, and use per-session:"
        _log_warn "    source Util/BuildShare/activate.sh"
    fi

    echo ""
}

# ---------------------------------------------------------------------------
# Help
# ---------------------------------------------------------------------------

_show_help() {
    cat <<'EOF'
Usage: build-share.sh <command> [<1st-carla-root>]

Commands:
  setup         <1st-carla-root>   Set up Engine (hardlink) + Content (symlink)
  engine        <1st-carla-root>   Engine hardlink copy only
  content       <1st-carla-root>   Content symlink only
  check-update                     Check shared repos for upstream updates

Options:
  -h, --help    Show this help message

Examples:
  build-share.sh setup /path/to/1stCarlaUE5
  build-share.sh check-update
EOF
}

# ---------------------------------------------------------------------------
# Subcommand implementations
# ---------------------------------------------------------------------------

cmd_engine() {
    local first_root
    first_root="$(_resolve_carla_root "$(realpath "$1")")"
    first_root="$(realpath "$first_root")"

    local engine_source
    engine_source="$(_detect_engine_source "$first_root")"
    engine_source="$(_resolve_source_origin "$engine_source")"
    engine_source="$(realpath "$engine_source")"

    local first_basename
    first_basename="$(basename "$(dirname "$first_root")")"
    local engine_basename
    engine_basename="$(basename "$engine_source")"
    local dest_engine
    dest_engine="$(dirname "$PROJECT_ROOT")/${engine_basename}.clone_of.${first_basename}"

    _do_engine_clone "$engine_source" "$dest_engine"

    _write_config "$first_root" "$engine_source" "$dest_engine" ""
    _check_engine_path_env "$dest_engine"

    # Update .ue5_engine_clone_path for activate script compatibility
    echo "$dest_engine" > "$PROJECT_ROOT/.ue5_engine_clone_path"
}

cmd_content() {
    local first_root
    first_root="$(_resolve_carla_root "$(realpath "$1")")"
    first_root="$(realpath "$first_root")"

    local content_source="$first_root/Unreal/CarlaUnreal/Content"
    if [ ! -d "$content_source" ]; then
        _log_error "Content not found in 1st environment: $content_source"
        _log_error "  Has CarlaSetup.sh been run in the 1st environment?"
        return 1
    fi

    _do_content_link "$content_source"

    _write_config "$first_root" "" "" "$content_source"
}

cmd_setup() {
    local first_root
    first_root="$(_resolve_carla_root "$(realpath "$1")")"
    first_root="$(realpath "$first_root")"

    echo "========================================"
    echo "CARLA Shared Environment Setup"
    echo "========================================"
    echo "  1st environment: $first_root"
    echo "  This project:    $PROJECT_ROOT"
    echo ""

    # --- Engine ---
    local engine_source
    engine_source="$(_detect_engine_source "$first_root")"
    engine_source="$(_resolve_source_origin "$engine_source")"
    engine_source="$(realpath "$engine_source")"

    local first_basename
    first_basename="$(basename "$(dirname "$first_root")")"
    local engine_basename
    engine_basename="$(basename "$engine_source")"
    local dest_engine
    dest_engine="$(dirname "$PROJECT_ROOT")/${engine_basename}.clone_of.${first_basename}"

    _do_engine_clone "$engine_source" "$dest_engine"

    # --- Content ---
    local content_source="$first_root/Unreal/CarlaUnreal/Content"
    if [ ! -d "$content_source" ]; then
        _log_error "Content not found in 1st environment: $content_source"
        _log_error "  Has CarlaSetup.sh been run in the 1st environment?"
        return 1
    fi

    _do_content_link "$content_source"

    # --- RGL (optional) ---
    local rgl_source
    rgl_source="$(dirname "$first_root")/RobotecGPULidar"
    _do_rgl_link "$rgl_source"

    # --- Config ---
    _write_config "$first_root" "$engine_source" "$dest_engine" "$content_source"

    # Update .ue5_engine_clone_path for activate script compatibility
    echo "$dest_engine" > "$PROJECT_ROOT/.ue5_engine_clone_path"

    # --- Env var check ---
    _check_engine_path_env "$dest_engine"

    echo "========================================"
    echo "Setup complete!"
    echo "========================================"
    echo ""
    echo "Next steps:"
    echo "  1. source Util/BuildShare/activate.sh"
    echo "  2. Run CarlaSetup.sh (Content and Engine will be skipped)"
    echo ""
}

cmd_check_update() {
    _read_config

    # --- Content ---
    echo "----------------------------------------"
    echo "[Content] Checking for updates..."
    echo "----------------------------------------"
    local content_dir="$PROJECT_ROOT/Unreal/CarlaUnreal/Content"
    if [ -L "$content_dir" ]; then
        content_dir="$(readlink -f "$content_dir")"
    fi
    if [ ! -d "$content_dir" ]; then
        _log_warn "Content directory not found: $content_dir"
    else
        # Find git repo: check Content/ itself, then Content/Carla/ (carla-content.git)
        local content_git_dir=""
        if [ -d "$content_dir/.git" ]; then
            content_git_dir="$content_dir"
        elif [ -d "$content_dir/Carla/.git" ]; then
            content_git_dir="$content_dir/Carla"
        fi

        if [ -z "$content_git_dir" ]; then
            _log_warn "No .git found in Content or Content/Carla — cannot check updates."
        else
            _log_info "Git repo found: $content_git_dir"
            (
                cd "$content_git_dir"
                git fetch --quiet 2>/dev/null || {
                    _log_warn "git fetch failed for Content"
                    exit 0
                }
                local new_commits
                new_commits="$(git log HEAD..@{upstream} --oneline 2>/dev/null || true)"
                if [ -n "$new_commits" ]; then
                    local count
                    count="$(echo "$new_commits" | wc -l)"
                    _log_info "$count new commit(s) on upstream:"
                    echo "$new_commits" | sed 's/^/  /' >&2
                    echo "" >&2
                    echo "  To update Content (shared by all environments via symlink):" >&2
                    echo "    cd $content_git_dir && git pull" >&2
                    echo "  NOTE: This affects ALL environments sharing this Content." >&2
                else
                    _log_info "Up to date."
                fi
            )
        fi
    fi
    echo "" >&2

    # --- Engine ---
    echo "----------------------------------------"
    echo "[Engine] Checking for updates..."
    echo "----------------------------------------"
    local engine_origin=""
    if [ -n "${CONF_ENGINE_CLONE:-}" ] && [ -f "${CONF_ENGINE_CLONE}/.carla-env-origin" ]; then
        engine_origin="$(cat "${CONF_ENGINE_CLONE}/.carla-env-origin")"
    elif [ -n "${CONF_ENGINE_SOURCE:-}" ]; then
        engine_origin="$CONF_ENGINE_SOURCE"
    fi

    if [ -z "$engine_origin" ]; then
        _log_warn "Cannot determine engine source. No origin recorded."
    elif [ ! -d "$engine_origin" ]; then
        _log_warn "Engine origin directory does not exist: $engine_origin"
    elif [ -d "$engine_origin/.git" ]; then
        (
            cd "$engine_origin"
            git fetch --quiet 2>/dev/null || {
                _log_warn "git fetch failed for Engine"
                exit 0
            }
            local new_commits
            new_commits="$(git log HEAD..@{upstream} --oneline 2>/dev/null || true)"
            if [ -n "$new_commits" ]; then
                local count
                count="$(echo "$new_commits" | wc -l)"
                _log_info "$count new commit(s) on upstream:"
                echo "$new_commits" | sed 's/^/  /' >&2
                echo "" >&2
                echo "  Each engine environment is independent after cloning." >&2
                echo "  The clone has no .git — it cannot be updated in place." >&2
                echo "" >&2
                echo "  To update the ORIGINAL engine (affects future clones):" >&2
                echo "    cd $engine_origin && git pull && make" >&2
                echo "" >&2
                echo "  To update THIS environment's engine clone:" >&2
                echo "    rm -rf ${CONF_ENGINE_CLONE}" >&2
                echo "    build-share.sh engine $(dirname "${CONF_SOURCE_PROJECT}")" >&2
                echo "    (then rebuild CARLA: ninja -C Build)" >&2
            else
                _log_info "Up to date."
            fi
        )
    else
        _log_warn "Engine origin has no .git: $engine_origin"
        _log_warn "  Cannot check for updates."
    fi
    echo ""
}

# ---------------------------------------------------------------------------
# Main dispatch
# ---------------------------------------------------------------------------

main() {
    if [ $# -eq 0 ]; then
        _show_help
        exit 1
    fi

    local command="$1"
    shift

    case "$command" in
        setup)
            [ $# -lt 1 ] && { _log_error "setup requires <1st-carla-root>"; exit 1; }
            cmd_setup "$1"
            ;;
        engine)
            [ $# -lt 1 ] && { _log_error "engine requires <1st-carla-root>"; exit 1; }
            cmd_engine "$1"
            ;;
        content)
            [ $# -lt 1 ] && { _log_error "content requires <1st-carla-root>"; exit 1; }
            cmd_content "$1"
            ;;
        check-update)
            cmd_check_update
            ;;
        -h|--help)
            _show_help
            exit 0
            ;;
        *)
            _log_error "Unknown command: $command"
            _show_help
            exit 1
            ;;
    esac
}

main "$@"
