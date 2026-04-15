#!/bin/bash
# CarlaSetupWithRGL.sh — CarlaSetup.sh wrapper with RobotecGPULidar integration.
#
# Wraps CarlaSetup.sh to add RGL-specific setup phases:
#   Phase 1: Prerequisites check (CUDA, OptiX, ROS2, apt packages)
#   Phase 2: RGL clone + build (with configurable extensions)
#   Phase 3: Run CarlaSetup.sh (Content, UE5, standard CARLA build)
#   Phase 4: Reconfigure CARLA with RGL options + rebuild
#   Phase 5: Optional package build (Shipping/Development)
#
# Usage:
#   bash CarlaSetupWithRGL.sh [OPTIONS]
#
# RGL options:
#   --optix-dir=PATH          OptiX SDK path (or set OptiX_INSTALL_DIR env var)
#   --rgl-branch=BRANCH       RGL git branch (default: develop)
#   --rgl-repo=URL            RGL git repository URL
#   --package=TYPE            Build package: shipping, development, none (default: none)
#   --skip-rgl-build          Skip RGL build (use pre-built library)
#
# RGL extensions (enabled by default: pcl, ros2-standalone, weather, agnocast):
#   --no-pcl                  Disable PCL extension
#   --no-ros2-standalone      Disable ROS2 standalone extension
#   --no-weather              Disable weather extension
#   --no-agnocast             Disable Agnocast extension
#   --with-udp                Enable UDP extension (disabled by default)
#
# CarlaSetup.sh pass-through options:
#   -i, --interactive         Interactive mode (prompt for sudo password)
#   -p, --skip-prerequisites  Skip prerequisites installation
#   -l, --launch              Launch UE5 editor after build
#   --python-root=PATH        Python root path

set -e

# ============================================================================
# Defaults
# ============================================================================

optix_dir="${OptiX_INSTALL_DIR:-}"
rgl_branch="develop"
rgl_repo="https://github.com/RobotecAI/RobotecGPULidar.git"
package_type="none"
skip_rgl_build=0

# RGL extensions (1=enabled, 0=disabled)
ext_pcl=1
ext_ros2_standalone=1
ext_weather=1
ext_agnocast=1
ext_udp=0

# CarlaSetup.sh pass-through
interactive=0
skip_prerequisites=0
carla_setup_args=()

workspace_path="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"

# ============================================================================
# Argument parsing
# ============================================================================

while [[ $# -gt 0 ]]; do
    case "$1" in
        --optix-dir=*)  optix_dir="${1#*=}"; shift ;;
        --rgl-branch=*) rgl_branch="${1#*=}"; shift ;;
        --rgl-repo=*)   rgl_repo="${1#*=}"; shift ;;
        --package=*)    package_type="${1#*=}"; shift ;;
        --skip-rgl-build) skip_rgl_build=1; shift ;;
        --no-pcl)             ext_pcl=0; shift ;;
        --no-ros2-standalone) ext_ros2_standalone=0; shift ;;
        --no-weather)         ext_weather=0; shift ;;
        --no-agnocast)        ext_agnocast=0; shift ;;
        --with-udp)           ext_udp=1; shift ;;
        -i|--interactive)
            interactive=1
            carla_setup_args+=("$1"); shift ;;
        -p|--skip-prerequisites)
            skip_prerequisites=1
            carla_setup_args+=("$1"); shift ;;
        -l|--launch)
            carla_setup_args+=("$1"); shift ;;
        --python-root=*)
            carla_setup_args+=("$1"); shift ;;
        -pyroot)
            carla_setup_args+=("$1" "$2"); shift 2 ;;
        *)
            echo "Unknown argument: $1"
            echo "Run 'head -35 $0' for usage."
            exit 1 ;;
    esac
done

echo ""
echo "========================================================================"
echo "  CarlaSetupWithRGL.sh"
echo "========================================================================"
echo "  workspace:    $workspace_path"
echo "  optix_dir:    ${optix_dir:-(not set)}"
echo "  rgl_branch:   $rgl_branch"
echo "  package:      $package_type"
echo "  extensions:   pcl=$ext_pcl ros2=$ext_ros2_standalone weather=$ext_weather agnocast=$ext_agnocast udp=$ext_udp"
echo "========================================================================"

# ============================================================================
# Phase 1: Prerequisites check
# ============================================================================

echo ""
echo "========================================"
echo "  Phase 1: Prerequisites check"
echo "========================================"

errors=0

# --- CUDA ---
if command -v nvcc &>/dev/null; then
    echo "[OK] CUDA: $(nvcc --version | grep 'release' | sed 's/.*release //' | sed 's/,.*//')"
else
    echo "[ERROR] CUDA (nvcc) not found."
    echo "  Install CUDA Toolkit: https://developer.nvidia.com/cuda-downloads"
    errors=1
fi

# --- OptiX SDK ---
if [ -z "$optix_dir" ]; then
    echo "[ERROR] OptiX SDK not specified."
    echo "  Download OptiX 7.2 from:"
    echo "    https://developer.nvidia.com/designworks/optix/downloads/legacy"
    echo "  Then run with: --optix-dir=/path/to/NVIDIA-OptiX-SDK-7.2.0-linux64-x86_64"
    echo "  Or: export OptiX_INSTALL_DIR=/path/to/..."
    errors=1
elif [ ! -f "$optix_dir/include/optix.h" ]; then
    echo "[ERROR] optix.h not found in $optix_dir/include/"
    echo "  Expected: $optix_dir/include/optix.h"
    errors=1
else
    echo "[OK] OptiX SDK: $optix_dir"
fi

# --- ROS2 Humble ---
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "[OK] ROS2 Humble: /opt/ros/humble/"
else
    echo "[ERROR] ROS2 Humble not found at /opt/ros/humble/setup.bash"
    echo "  Install: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    errors=1
fi

# --- apt packages (pigz, zstd) ---
missing_pkgs=()
for pkg in pigz zstd; do
    if dpkg -s "$pkg" &>/dev/null; then
        echo "[OK] $pkg: installed"
    else
        missing_pkgs+=("$pkg")
    fi
done

if [ ${#missing_pkgs[@]} -gt 0 ]; then
    if [ $skip_prerequisites -eq 1 ]; then
        echo "[SKIP] Missing packages: ${missing_pkgs[*]} (--skip-prerequisites)"
    else
        echo "[INFO] Installing missing packages: ${missing_pkgs[*]}"
        if [ $interactive -eq 1 ]; then
            sudo apt-get install -y "${missing_pkgs[@]}"
        else
            if sudo -n apt-get install -y "${missing_pkgs[@]}" 2>/dev/null; then
                echo "[OK] Installed: ${missing_pkgs[*]}"
            else
                echo "[ERROR] Failed to install: ${missing_pkgs[*]}"
                echo "  Run: sudo apt-get install -y ${missing_pkgs[*]}"
                echo "  Or run this script with --interactive"
                errors=1
            fi
        fi
    fi
fi

if [ $errors -ne 0 ]; then
    echo ""
    echo "Prerequisites check failed. Fix the errors above and re-run."
    exit 1
fi

echo ""
echo "All prerequisites OK."

# ============================================================================
# Phase 2: RGL clone + build
# ============================================================================

echo ""
echo "========================================"
echo "  Phase 2: RGL clone + build"
echo "========================================"

rgl_dir="$workspace_path/../RobotecGPULidar"

if [ $skip_rgl_build -eq 1 ]; then
    echo "[SKIP] RGL build (--skip-rgl-build)"
    if [ ! -f "$rgl_dir/build/lib/libRobotecGPULidar.so" ]; then
        echo "[WARNING] RGL library not found at $rgl_dir/build/lib/libRobotecGPULidar.so"
        echo "  CARLA RGL build may fail. Remove --skip-rgl-build to build RGL."
    fi
else
    # Clone if not present
    if [ -d "$rgl_dir" ]; then
        echo "[OK] RGL repository found: $rgl_dir"
    else
        echo "Cloning RobotecGPULidar (branch: $rgl_branch)..."
        git clone -b "$rgl_branch" "$rgl_repo" "$rgl_dir"
    fi

    pushd "$rgl_dir" > /dev/null

    # Source ROS2 (required for RGL ROS2 extension build)
    source /opt/ros/humble/setup.bash

    # Set OptiX
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

    # Build RGL with selected extensions
    rgl_build_flags=()
    [ $ext_pcl -eq 1 ]             && rgl_build_flags+=("--with-pcl")
    [ $ext_ros2_standalone -eq 1 ] && rgl_build_flags+=("--with-ros2-standalone")
    [ $ext_weather -eq 1 ]         && rgl_build_flags+=("--with-weather")
    [ $ext_agnocast -eq 1 ]        && rgl_build_flags+=("--with-agnocast")
    [ $ext_udp -eq 1 ]             && rgl_build_flags+=("--with-udp")

    echo "Building RGL with extensions: ${rgl_build_flags[*]:-none}"
    python3 setup.py "${rgl_build_flags[@]}"

    popd > /dev/null

    # Verify build output
    if [ -f "$rgl_dir/build/lib/libRobotecGPULidar.so" ]; then
        echo "[OK] RGL build succeeded: $rgl_dir/build/lib/libRobotecGPULidar.so"
    else
        echo "[ERROR] RGL build failed: libRobotecGPULidar.so not found."
        exit 1
    fi
fi

# ============================================================================
# Phase 3: Run CarlaSetup.sh
# ============================================================================

echo ""
echo "========================================"
echo "  Phase 3: CarlaSetup.sh"
echo "========================================"

echo "Running CarlaSetup.sh with args: ${carla_setup_args[*]:-none}"
bash "$workspace_path/CarlaSetup.sh" "${carla_setup_args[@]}"

# ============================================================================
# Phase 4: Reconfigure with RGL options
# ============================================================================

echo ""
echo "========================================"
echo "  Phase 4: Reconfigure with RGL"
echo "========================================"

# Ensure CARLA_UNREAL_ENGINE_PATH is set
if [ -z "$CARLA_UNREAL_ENGINE_PATH" ]; then
    ue5_path="$(realpath "$workspace_path/../UnrealEngine5_carla" 2>/dev/null || true)"
    if [ -d "$ue5_path" ]; then
        export CARLA_UNREAL_ENGINE_PATH="$ue5_path"
    else
        echo "[ERROR] UnrealEngine5_carla not found."
        echo "  Expected at: $workspace_path/../UnrealEngine5_carla"
        exit 1
    fi
fi
echo "CARLA_UNREAL_ENGINE_PATH=$CARLA_UNREAL_ENGINE_PATH"

cd "$workspace_path"

echo "Reconfiguring CARLA with RGL support..."
cmake -G Ninja -S . -B Build \
    --toolchain="$PWD/CMake/Toolchain.cmake" \
    -DLAUNCH_ARGS="-prefernvidia" \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_ROS2=ON \
    -DENABLE_RGL=ON \
    -DCARLA_DDS_VENDOR=CycloneDDS \
    -DCARLA_PACKAGE_COMPRESSION=pigz \
    -DBUILD_LIBCARLA_TESTS=OFF \
    -DCARLA_UNREAL_ENGINE_PATH="$CARLA_UNREAL_ENGINE_PATH"

echo "Rebuilding CARLA with RGL..."
cmake --build Build

# ============================================================================
# Phase 5: Package build (optional)
# ============================================================================

echo ""
echo "========================================"
echo "  Phase 5: Package build"
echo "========================================"

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
    none)
        echo "[SKIP] Package build (--package=none)"
        ;;
    *)
        echo "[ERROR] Unknown package type: $package_type"
        echo "  Expected: shipping, development, none"
        exit 1
        ;;
esac

# ============================================================================
# Done
# ============================================================================

echo ""
echo "========================================================================"
echo "  CarlaSetupWithRGL.sh completed!"
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
