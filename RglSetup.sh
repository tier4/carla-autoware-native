#!/bin/bash
# RglSetup.sh — RobotecGPULidar setup for CARLA.
#
# Two-step workflow (run CarlaSetup.sh between them):
#   1. bash RglSetup.sh prepare [OPTIONS]   # Check prerequisites, clone+build RGL
#   2. bash CarlaSetup.sh -i                # Standard CARLA setup (user runs this)
#   3. bash RglSetup.sh build [OPTIONS]     # Reconfigure CARLA with RGL, rebuild
#
# Subcommands:
#   prepare   Check prerequisites and build RGL library
#   build     Reconfigure CARLA with RGL options and rebuild
#
# Options for 'prepare':
#   --optix-dir=PATH          OptiX SDK path (or set OptiX_INSTALL_DIR env var)
#   --rgl-branch=BRANCH       RGL git branch (default: develop)
#   --rgl-repo=URL            RGL git repository URL
#   --skip-rgl-build          Skip RGL build (use pre-built library)
#   --no-pcl                  Disable PCL extension
#   --no-ros2-standalone      Disable ROS2 standalone extension
#   --no-weather              Disable weather extension
#   --no-agnocast             Disable Agnocast extension
#   --with-udp                Enable UDP extension (disabled by default)
#
# Options for 'build':
#   --package=TYPE            shipping, development, launch, none (default: none)

set -e

workspace_path="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"

# ============================================================================
# Usage
# ============================================================================

usage() {
    head -26 "$0" | tail -25
    exit 1
}

# ============================================================================
# Subcommand: prepare
# ============================================================================

cmd_prepare() {
    local optix_dir="${OptiX_INSTALL_DIR:-}"
    local rgl_branch="develop"
    local rgl_repo="https://github.com/RobotecAI/RobotecGPULidar.git"
    local skip_rgl_build=0

    # RGL extensions (1=enabled, 0=disabled)
    local ext_pcl=1
    local ext_ros2_standalone=1
    local ext_weather=1
    local ext_agnocast=1
    local ext_udp=0

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --optix-dir=*)  optix_dir="${1#*=}"; shift ;;
            --rgl-branch=*) rgl_branch="${1#*=}"; shift ;;
            --rgl-repo=*)   rgl_repo="${1#*=}"; shift ;;
            --skip-rgl-build) skip_rgl_build=1; shift ;;
            --no-pcl)             ext_pcl=0; shift ;;
            --no-ros2-standalone) ext_ros2_standalone=0; shift ;;
            --no-weather)         ext_weather=0; shift ;;
            --no-agnocast)        ext_agnocast=0; shift ;;
            --with-udp)           ext_udp=1; shift ;;
            *) echo "Unknown option for prepare: $1"; usage ;;
        esac
    done

    echo ""
    echo "========================================================================"
    echo "  RglSetup.sh prepare"
    echo "========================================================================"
    echo "  workspace:    $workspace_path"
    echo "  optix_dir:    ${optix_dir:-(not set)}"
    echo "  rgl_branch:   $rgl_branch"
    echo "  extensions:   pcl=$ext_pcl ros2=$ext_ros2_standalone weather=$ext_weather agnocast=$ext_agnocast udp=$ext_udp"
    echo "========================================================================"

    # ---- Prerequisites check ----

    echo ""
    echo "--- Prerequisites check ---"

    local errors=0

    # CUDA
    if command -v nvcc &>/dev/null; then
        echo "[OK] CUDA: $(nvcc --version | grep 'release' | sed 's/.*release //' | sed 's/,.*//')"
    else
        echo "[ERROR] CUDA (nvcc) not found."
        echo "  Install CUDA Toolkit: https://developer.nvidia.com/cuda-downloads"
        errors=1
    fi

    # OptiX SDK
    if [ -z "$optix_dir" ]; then
        echo "[ERROR] OptiX SDK not specified."
        echo "  Download OptiX 7.2 from:"
        echo "    https://developer.nvidia.com/designworks/optix/downloads/legacy"
        echo "  Then run with: --optix-dir=/path/to/NVIDIA-OptiX-SDK-7.2.0-linux64-x86_64"
        echo "  Or: export OptiX_INSTALL_DIR=/path/to/..."
        errors=1
    elif [ ! -f "$optix_dir/include/optix.h" ]; then
        echo "[ERROR] optix.h not found in $optix_dir/include/"
        errors=1
    else
        echo "[OK] OptiX SDK: $optix_dir"
    fi

    # ROS2 Humble
    if [ -f /opt/ros/humble/setup.bash ]; then
        echo "[OK] ROS2 Humble: /opt/ros/humble/"
    else
        echo "[ERROR] ROS2 Humble not found at /opt/ros/humble/setup.bash"
        echo "  Install: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
        errors=1
    fi

    # patchelf (required by RGL ros2_standalone build)
    if command -v patchelf &>/dev/null; then
        echo "[OK] patchelf: installed"
    else
        echo "[INFO] Installing patchelf..."
        sudo apt-get install -y patchelf
        echo "[OK] patchelf: installed"
    fi

    if [ $errors -ne 0 ]; then
        echo ""
        echo "Prerequisites check failed. Fix the errors above and re-run."
        exit 1
    fi

    echo "[OK] All prerequisites OK."

    # ---- RGL clone + build ----

    echo ""
    echo "--- RGL clone + build ---"

    local rgl_dir="$workspace_path/../RobotecGPULidar"

    if [ $skip_rgl_build -eq 1 ]; then
        echo "[SKIP] RGL build (--skip-rgl-build)"
        if [ ! -f "$rgl_dir/build/lib/libRobotecGPULidar.so" ]; then
            echo "[WARNING] RGL library not found at $rgl_dir/build/lib/libRobotecGPULidar.so"
            echo "  CARLA RGL build may fail. Remove --skip-rgl-build to build RGL."
        fi
        return
    fi

    # Detect symlink (shared via build-share.sh) — skip to avoid modifying shared RGL
    if [ -L "$rgl_dir" ]; then
        local rgl_target
        rgl_target="$(readlink -f "$rgl_dir")"
        echo "[INFO] RGL is a symlink to a shared environment:"
        echo "  $rgl_dir → $rgl_target"
        if [ -f "$rgl_dir/build/lib/libRobotecGPULidar.so" ]; then
            echo "[OK] Shared RGL is already built. Skipping prepare."
        else
            echo "[WARNING] Shared RGL has no build artifacts."
            echo "  Build RGL in the original environment first:"
            echo "    cd $rgl_target && bash ../CarlaUE5/RglSetup.sh prepare --optix-dir=..."
        fi
        return
    fi

    # Clone if not present
    if [ -d "$rgl_dir" ]; then
        echo "[OK] RGL repository found: $rgl_dir"
    else
        echo "Cloning RobotecGPULidar (branch: $rgl_branch)..."
        git clone -b "$rgl_branch" "$rgl_repo" "$rgl_dir"
    fi

    pushd "$rgl_dir" > /dev/null

    # Clone extension repos (separate repos, .gitignored in main RGL repo).
    # URLs and branches are parsed from extensions.repos to stay in sync with RGL upstream.
    if [ $ext_weather -eq 1 ] && [ ! -d "extensions/weather" ]; then
        local weather_url weather_branch
        weather_url=$(python3 -c "import yaml; d=yaml.safe_load(open('extensions.repos')); print(d['repositories']['extensions/weather']['url'])")
        weather_branch=$(python3 -c "import yaml; d=yaml.safe_load(open('extensions.repos')); print(d['repositories']['extensions/weather']['version'])")
        echo "Cloning RGL weather extension ($weather_branch)..."
        git clone -b "$weather_branch" "$weather_url" extensions/weather
    fi
    if [ $ext_udp -eq 1 ] && [ ! -d "extensions/udp" ]; then
        local udp_url udp_branch
        udp_url=$(python3 -c "import yaml; d=yaml.safe_load(open('extensions.repos')); print(d['repositories']['extensions/udp']['url'])")
        udp_branch=$(python3 -c "import yaml; d=yaml.safe_load(open('extensions.repos')); print(d['repositories']['extensions/udp']['version'])")
        echo "Cloning RGL UDP extension ($udp_branch)..."
        git clone -b "$udp_branch" "$udp_url" extensions/udp
    fi

    # Source ROS2
    source /opt/ros/humble/setup.bash
    export OptiX_INSTALL_DIR="$optix_dir"

    # Install core dependencies
    echo "Installing RGL core dependencies..."
    python3 setup.py --install-deps

    # Install extension dependencies
    if [ $ext_pcl -eq 1 ]; then
        echo "Installing RGL PCL dependencies..."
        python3 setup.py --install-pcl-deps
    fi
    if [ $ext_ros2_standalone -eq 1 ]; then
        echo "Installing RGL ROS2 dependencies..."
        python3 setup.py --install-ros2-deps
    fi
    if [ $ext_agnocast -eq 1 ]; then
        echo "Installing RGL Agnocast dependencies..."
        python3 setup.py --install-agnocast-deps
    fi

    # Build RGL
    local rgl_build_flags=()
    [ $ext_pcl -eq 1 ]             && rgl_build_flags+=("--with-pcl")
    [ $ext_ros2_standalone -eq 1 ] && rgl_build_flags+=("--with-ros2-standalone")
    [ $ext_weather -eq 1 ]         && rgl_build_flags+=("--with-weather")
    [ $ext_agnocast -eq 1 ]        && rgl_build_flags+=("--with-agnocast")
    [ $ext_udp -eq 1 ]             && rgl_build_flags+=("--with-udp")

    echo "Building RGL with extensions: ${rgl_build_flags[*]:-none}"
    python3 setup.py "${rgl_build_flags[@]}"

    popd > /dev/null

    # Verify
    if [ -f "$rgl_dir/build/lib/libRobotecGPULidar.so" ]; then
        echo "[OK] RGL build succeeded."
    else
        echo "[ERROR] RGL build failed: libRobotecGPULidar.so not found."
        exit 1
    fi

    echo ""
    echo "========================================================================"
    echo "  RglSetup.sh prepare completed!"
    echo "========================================================================"
    echo ""
    echo "Next steps:"
    echo ""
    echo "  1. Set up CARLA (Content, UE5, standard build):"
    echo "     bash CarlaSetup.sh -i"
    echo "     (options: -i interactive, -p skip prerequisites)"
    echo "     (do NOT use -l here — it launches the editor without RGL)"
    echo ""
    echo "  2. Reconfigure CARLA with RGL and rebuild (after step 1):"
    echo "     bash RglSetup.sh build"
    echo "     bash RglSetup.sh build --package=shipping      # + Shipping package"
    echo "     bash RglSetup.sh build --package=development   # + Development package"
    echo "     bash RglSetup.sh build --package=launch        # + open UE5 editor with RGL"
    echo ""
    echo "  Manual cmake build (after step 2):"
    echo "     export CARLA_UNREAL_ENGINE_PATH=\$(realpath ../UnrealEngine5_carla)"
    echo "     cmake --build Build                              # rebuild"
    echo "     cmake --build Build --target package             # Shipping package"
    echo "     cmake --build Build --target package-development # Development package"
    echo "     cmake --build Build --target launch              # UE5 editor"
    echo ""
}

# ============================================================================
# Subcommand: build
# ============================================================================

cmd_build() {
    local package_type="none"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --package=*) package_type="${1#*=}"; shift ;;
            *) echo "Unknown option for build: $1"; usage ;;
        esac
    done

    echo ""
    echo "========================================================================"
    echo "  RglSetup.sh build"
    echo "========================================================================"

    # Ensure CARLA_UNREAL_ENGINE_PATH is set
    if [ -z "$CARLA_UNREAL_ENGINE_PATH" ]; then
        local ue5_path
        ue5_path="$(realpath "$workspace_path/../UnrealEngine5_carla" 2>/dev/null || true)"
        if [ -d "$ue5_path" ]; then
            export CARLA_UNREAL_ENGINE_PATH="$ue5_path"
        else
            echo "[ERROR] UnrealEngine5_carla not found."
            echo "  Expected at: $workspace_path/../UnrealEngine5_carla"
            echo "  Or set: export CARLA_UNREAL_ENGINE_PATH=/path/to/..."
            exit 1
        fi
    fi
    echo "CARLA_UNREAL_ENGINE_PATH=$CARLA_UNREAL_ENGINE_PATH"

    cd "$workspace_path"

    # Reconfigure with RGL
    echo ""
    echo "--- Reconfigure CARLA with RGL ---"
    cmake -G Ninja -S . -B Build \
        --toolchain="$PWD/CMake/Toolchain.cmake" \
        -DLAUNCH_ARGS="-prefernvidia" \
        -DCMAKE_BUILD_TYPE=Release \
        -DENABLE_ROS2=ON \
        -DENABLE_RGL=ON \
        -DCARLA_DDS_VENDOR=CycloneDDS \
        -DBUILD_LIBCARLA_TESTS=OFF \
        -DCARLA_UNREAL_ENGINE_PATH="$CARLA_UNREAL_ENGINE_PATH"

    echo ""
    echo "--- Build CARLA with RGL ---"
    cmake --build Build

    # Package build
    echo ""
    echo "--- Package ---"
    case "$package_type" in
        shipping)
            echo "Building Shipping package..."
            cmake --build Build --target package
            echo "[OK] Shipping package built."
            ;;
        development)
            echo "Building Development package..."
            cmake --build Build --target package-development
            echo "[OK] Development package built."
            ;;
        launch)
            echo "Launching UE5 editor..."
            cmake --build Build --target launch
            ;;
        none)
            echo "[SKIP] Package build (--package=none)"
            ;;
        *)
            echo "[ERROR] Unknown package type: $package_type (expected: shipping, development, launch, none)"
            exit 1
            ;;
    esac

    echo ""
    echo "========================================================================"
    echo "  RglSetup.sh build completed!"
    echo "========================================================================"
    echo ""
    echo "To build packages later:"
    echo "  export CARLA_UNREAL_ENGINE_PATH=$CARLA_UNREAL_ENGINE_PATH"
    echo "  cd $workspace_path"
    echo "  cmake --build Build --target package              # Shipping"
    echo "  cmake --build Build --target package-development  # Development"
    echo ""
    echo "To run the server:"
    echo "  cd Build/Package/Carla-*-Linux-Shipping/Linux"
    echo "  ./CarlaUnreal.sh --ros2"
}

# ============================================================================
# Main: dispatch subcommand
# ============================================================================

if [ $# -eq 0 ]; then
    usage
fi

subcommand="$1"
shift

case "$subcommand" in
    prepare) cmd_prepare "$@" ;;
    build)   cmd_build "$@" ;;
    *)       echo "Unknown subcommand: $subcommand"; usage ;;
esac
