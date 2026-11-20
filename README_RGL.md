CARLA with RobotecGPULidar (RGL)
================================

[日本語](README_RGL.ja.md)

This document describes how to build and run CARLA with
[RobotecGPULidar (RGL)](https://github.com/RobotecAI/RobotecGPULidar) — a
GPU-accelerated ray-traced LiDAR simulation library. The setup is driven by
`RglSetup.sh`, which complements the standard `CarlaSetup.sh` flow with the
steps required to clone, build and integrate RGL and its dependencies.

> [!NOTE]
> RGL integration requires an NVIDIA GPU, CUDA, NVIDIA OptiX SDK 7.2 and
> ROS 2 (Humble on Ubuntu 22.04, Jazzy on Ubuntu 24.04) on the host machine. The GPU LiDAR pipeline also uses the
> `RclcppBridge` helper library under
> `Unreal/CarlaUnreal/Plugins/CarlaRGL/ThirdParty/RclcppBridge` to coordinate
> rclcpp/DDS domain ownership between CARLA and RGL.

## Recommended system

* Intel i7 gen 9th - 11th / Intel i9 gen 9th - 11th / AMD Ryzen 7 / AMD Ryzen 9
* +32 GB RAM
* NVIDIA RTX 3070 / 3080 / 3090 / RTX 4090 / RTX 5090 or better
* 16 GB or more VRAM
* Ubuntu 22.04 or Ubuntu 24.04
* CUDA Toolkit 12.x (tested with 12.9 on Ubuntu 24.04; RGL needs 12.6+ there)
* NVIDIA OptiX SDK 7.2
* ROS 2 Humble (Ubuntu 22.04) or ROS 2 Jazzy (Ubuntu 24.04), plus
  `ros-<distro>-radar-msgs`, `ros-<distro>-fastrtps`,
  `ros-<distro>-rmw-fastrtps-cpp`, `ros-<distro>-cyclonedds`,
  `ros-<distro>-rmw-cyclonedds-cpp`, `python3-colcon-common-extensions` and
  `patchelf`

> [!NOTE]
> Windows is not yet supported by `RglSetup.sh` — **TBD**.

## Workflow overview

RGL integration uses a three-step workflow. `CarlaSetup.sh` is run between
the two `RglSetup.sh` invocations:

```
1. bash RglSetup.sh prepare [OPTIONS]   # Check prerequisites, clone+build RGL
2. bash CarlaSetup.sh -i                # Standard CARLA setup
3. bash RglSetup.sh build  [OPTIONS]    # Reconfigure CARLA with RGL, rebuild
```

Step 1 stages the RGL library and its colcon-built dependencies
(`agnocast`, `radar_msgs`, …). Step 2 runs the standard CARLA setup (Unreal
Engine download, content, initial build). Step 3 reconfigures CMake with
`ENABLE_RGL=ON` and rebuilds CARLA so that the CarlaRGL plugin is linked in.

## Building CARLA with RGL

### Step 1 — `prepare`

Run from the CARLA root directory (`CarlaUE5/`):

```sh
cd CarlaUE5
bash RglSetup.sh prepare --optix-dir=/path/to/NVIDIA-OptiX-SDK-7.2.0-linux64-x86_64
```

Options worth knowing:

* `--ros-distro=humble|jazzy` selects the ROS 2 distribution. The default is
  `$ROS_DISTRO` when a ROS environment is sourced, otherwise Humble on
  Ubuntu 22.04 and Jazzy on Ubuntu 24.04. Pass the same value to
  `RglSetup.sh build`.
* `--cuda-dir=PATH` points at a CUDA Toolkit whose `nvcc` is not on `PATH`,
  e.g. a toolkit-only install under your home directory.
* `--no-pcl` skips the PCL extension (CARLA's RGL plugin does not use it) and
  saves the long vcpkg build.

What it does:

1. Verifies CUDA, OptiX SDK, ROS 2 (the selected distro) and `patchelf` are available.
2. Clones `RobotecGPULidar/` alongside `CarlaUE5/` (git branch: `develop`).
3. Sources `/opt/ros/<distro>/setup.bash` and installs RGL core, PCL, ROS 2 and
   Agnocast dependencies via `python3 setup.py --install-*-deps`.
4. Builds RGL with the selected extensions and produces
   `RobotecGPULidar/build/lib/libRobotecGPULidar.so`.

> [!NOTE]
> You can also set `OptiX_INSTALL_DIR` in the environment instead of passing
> `--optix-dir=`.

### Step 2 — `CarlaSetup.sh`

Run the standard CARLA setup to download Unreal Engine 5, content and
produce a first CARLA build:

```sh
bash CarlaSetup.sh -i
```

See [README.md](README.md) for full `CarlaSetup.sh` usage.

> [!IMPORTANT]
> Do **not** use `CarlaSetup.sh -l` here — it launches the editor *without*
> RGL. RGL is only enabled after Step 3.

### Step 3 — `build`

Reconfigure CMake with `ENABLE_RGL=ON` and rebuild:

```sh
bash RglSetup.sh build                       # rebuild only
bash RglSetup.sh build --package=shipping    # + build Shipping package
bash RglSetup.sh build --package=development # + build Development package
bash RglSetup.sh build --package=launch      # + open UE5 editor with RGL
```

The script:

1. Detects and wipes stale CMake caches under `Build/` (see
   [Stale cache detection](#stale-cache-detection)).
2. Runs `cmake -G Ninja -S . -B Build --toolchain=$PWD/CMake/Toolchain.cmake
   -DENABLE_ROS2=ON -DENABLE_RGL=ON …`. The ROS 2 middleware is chosen when
   the server starts (`-rmw=fastdds` by default, `-rmw=cyclonedds` for CycloneDDS).
3. Invokes `cmake --build Build` and, if requested, the package target.

`RclcppBridge` is built as an ExternalProject under `Build/` using the system
compiler (not the UE toolchain), linking against `rclcpp` from
`$AMENT_PREFIX_PATH`, else `/opt/ros/$ROS_DISTRO` (`RglSetup.sh build` sets it
from `--ros-distro`), else whichever of `/opt/ros/jazzy` and `/opt/ros/humble`
is installed. Use the same distro RGL was prepared with.

## Rebuilding CARLA (manual)

Once `RglSetup.sh build` has succeeded once, subsequent builds can be done
with the standard CMake commands:

```sh
export CARLA_UNREAL_ENGINE_PATH=$(realpath ../UnrealEngine5_carla)
cmake --build Build                              # rebuild
cmake --build Build --target package             # Shipping package
cmake --build Build --target package-development # Development package
cmake --build Build --target launch              # UE5 editor
```

## Running a packaged build

```sh
cd Build/Package/Carla-*-Linux-Shipping
./CarlaUnreal.sh --ros2
```

## Command reference

### `prepare` options

| Option | Description |
|---|---|
| `--optix-dir=PATH`       | OptiX SDK path (or set `OptiX_INSTALL_DIR`) |
| `--cuda-dir=PATH`        | CUDA Toolkit path when `nvcc` is not on `PATH` |
| `--ros-distro=NAME`      | ROS 2 distro, `humble` or `jazzy` (default: `$ROS_DISTRO`, else by Ubuntu release) |
| `--rgl-branch=BRANCH`    | RGL git branch (default: `develop`) |
| `--rgl-repo=URL`         | RGL git repository URL |
| `--skip-rgl-build`       | Skip RGL build (use pre-built library) |
| `--no-pcl`               | Disable PCL extension |
| `--no-ros2-standalone`   | Disable ROS 2 standalone extension |
| `--no-agnocast`          | Disable Agnocast extension |
| `--with-weather`         | Enable weather extension (private repo) |
| `--with-udp`             | Enable UDP extension (private repo) |
| `--ext-patch-dir=DIR`    | Apply `DIR/*.patch` to the UDP extension after cloning; only with `--with-udp` (see [UDP Raw Packet extension](#udp-raw-packet-extension-optional)) |

### `build` options

| Option | Description |
|---|---|
| `--package=shipping`    | Build the CARLA Shipping package after rebuild |
| `--package=development` | Build the CARLA Development package after rebuild |
| `--package=launch`      | Launch the UE5 editor after rebuild |
| `--package=none`        | (default) Rebuild only, no package step |
| `--ros-distro=NAME`     | ROS 2 distro RclcppBridge builds against; use the one RGL was prepared with |
| `--no-clear-ddc`        | Skip the automatic DDC clear on `--package=launch` |

## Stale cache detection

`RglSetup.sh` detects three classes of stale build artifacts that arise when
the workspace is renamed, copied, or when `SOURCE_DIR` changes in CMake:

| Detector | Trigger | Action |
|---|---|---|
| `verify_cmake_cache_path`              | top-level `CMAKE_HOME_DIRECTORY` mismatch | wipe `build_dir` |
| `verify_cmake_subcache_sources_exist`  | ExternalProject sub-cache points to a missing directory | wipe the affected sub-build only |
| `verify_colcon_install_path`           | colcon `setup.sh` has a baked-in path from a previous workspace | wipe `build/`, `install/`, `log/` |

These are invoked automatically:

- `prepare`: scans `RobotecGPULidar/build/` and every colcon workspace under
  `RobotecGPULidar/external/*/` and `RobotecGPULidar/extensions/*/`.
- `build`:  scans `CarlaUE5/Build/` and every ExternalProject sub-cache
  underneath it.

No flags are needed — relocating the workspace is handled transparently.

## Package compression

### When compression runs

Whether the `package*` target compresses the staged build is controlled by the
upstream `CARLA_UNREAL_PACKAGE_NO_COMPRESSION` option. Its default depends on
the build type (inherited from upstream CARLA, not RGL-specific):

| `CARLA_UNREAL_PACKAGE_BUILD_TYPE` | `NO_COMPRESSION` default | Behavior |
|---|---|---|
| `Shipping`       | `OFF` | Compression runs |
| others (Development, Debug, …) | `ON`  | Compression is skipped |

Override with `-DCARLA_UNREAL_PACKAGE_NO_COMPRESSION=ON|OFF`.

### Compression method

When compression runs, the method is selectable via `CARLA_PACKAGE_COMPRESSION`
(default: `pigz` — parallel gzip, auto-detects CPU cores):

```sh
cmake ... -DCARLA_PACKAGE_COMPRESSION=pigz   # .tar.gz, parallel gzip (default)
cmake ... -DCARLA_PACKAGE_COMPRESSION=zstd   # .tar.zst, parallel zstd (pzstd)
cmake ... -DCARLA_PACKAGE_COMPRESSION=gzip   # .tar.gz, single-threaded (compatibility)
```

`pigz` and `zstd` both auto-detect CPU cores and parallelize. Install via
`apt install pigz` or `apt install zstd`.

## Troubleshooting

### `agnocastlib` not found

```
Could not find a package configuration file provided by "agnocastlib" …
```

The colcon install for Agnocast is from a different workspace. `prepare`
detects and removes it automatically; rerun `bash RglSetup.sh prepare`.

### `CMakeCache.txt directory is different than the directory …`

A build directory was generated in a different workspace. `prepare` / `build`
detect this automatically and wipe the cache; rerun the same command.

### iceoryx link errors when building CycloneDDS

Fixed upstream in this fork by setting `-DENABLE_SHM=OFF` in CycloneDDS's
ExternalProject — CycloneDDS no longer picks up the system iceoryx built
against libstdc++ while the UE toolchain uses libc++.

### Editor crash on launch (Nanite / SkeletalMesh assertion)

Symptom — editor process aborts shortly after `cmake --build Build --target launch`
with a message similar to:

```
Assertion failed: ExternalEdgeOffset == ExternalEdges.Num()
  [File:./Developer/NaniteBuilder/Private/ClusterDAG.cpp] [Line: 126]

  Stack: Nanite::BuildDAG ← FSkeletalMeshBuilder::Build
       ← USkeletalMesh::ExecutePostLoadInternal
```

Root cause — the project's `DerivedDataCache/` (DDC) holds Nanite / asset
artefacts produced by a previous build whose internal layout no longer
matches the current code. Common after an upstream sync, branch switch, or
rebase that touches asset post-load paths. DDC is an internal cache: it
does not invalidate itself when the surrounding CARLA/UE5 code changes.

Recovery — wipe the project DDC and let UE5 rebuild it on next launch:

```sh
# 1. Kill any lingering editor / crash reporter
pkill -9 -f UnrealEditor
pkill -9 -f CrashReportClient

# 2. Remove the project DDC (typically <100 MB on a cold build)
rm -rf Unreal/CarlaUnreal/DerivedDataCache

# 3. Relaunch — initial start is slow (~5–10 min) while DDC is rebuilt
cmake --build Build --target launch
```

`RglSetup.sh build --package=launch` automates this: it tracks the last
launch's git HEAD in `Build/.last-launch-head` and clears DDC when HEAD has
moved. Suppress with `--no-clear-ddc` if you want to keep the warm cache
across switches. Manually clear with the steps above when the heuristic
misses (e.g. uncommitted local edits that change asset behaviour).

If clearing the project DDC does not resolve the crash, also check:

- Engine-shared DDC (rare): `~/Documents/Unreal Engine/Common/DerivedDataCache/`
- Engine-internal DDC (rare): `UnrealEngine5_carla/Engine/DerivedDataCache/`

Both are safe to delete; expect a longer rebuild.

## Windows

**TBD** — `RglSetup.sh` is currently a Bash script and targets Linux
(Ubuntu 22.04). A Windows equivalent (e.g. `RglSetup.bat` or PowerShell) is
not yet implemented.

## See also

- [README.md](README.md) — standard CARLA UE5 build instructions.
- [RobotecGPULidar](https://github.com/RobotecAI/RobotecGPULidar) — upstream
  RGL repository.

## UDP Raw Packet extension (optional)

To enable AWSIM-compatible UDP Raw Packet emission, build RGL with the
private `RobotecAI/RGL-extension-udp` extension. `RglSetup.sh` exposes
a `--with-udp` flag for this; see
[`Docs/rgl/udp_raw_packets.md`](Docs/rgl/udp_raw_packets.md) for the
full procedure, Python recipes, attribute reference, and the per-model
return-mode whitelist.

CARLA itself still builds without the UDP extension. In that case the
UDP-related features auto-disable at runtime with a warning log and the
rest of the RGL pipeline (ROS 2 publish, Yield) is unaffected.

> [!IMPORTANT]
> The UDP extension needs fixes that are not in RobotecAI's repository yet.
> They are kept as patches outside this public repository (a patch carries
> context lines of the private extension's source) and are applied with
> `--ext-patch-dir=DIR` together with `--with-udp`:
>
> ```bash
> bash RglSetup.sh prepare --with-udp --ext-patch-dir=/path/to/udp-patches ...
> ```
>
> `--with-udp` alone still builds a library that loads and sends packets, but
> the packets may not be accepted downstream (for example, Autoware does not
> engage). `RglSetup.sh` cannot tell the two builds apart, so pass the patch
> directory whenever you build the UDP extension. Once RobotecAI ships the
> fixes, the patches become no-ops (already-applied patches are skipped) and
> `--ext-patch-dir` can be dropped. `--ext-patch-dir` without `--with-udp` is
> ignored with a warning.

## Phase 2 verification with tier4/nebula

Once a shipping package is built and the UDP extension is in the
loaded `libRobotecGPULidar.so`, run the Phase 2 end-to-end test to
verify all eight supported LiDAR models against the production ROS 2
driver `tier4/nebula`:

```bash
source /opt/ros/<distro>/setup.bash   # humble or jazzy
source /path/to/colcon_workspace/install/setup.bash

cd PythonAPI/rgl/tests
python3 test_phase2_nebula_e2e.py
```

The test auto-launches CARLA, generates a per-model Nebula config
YAML in `/tmp`, sequentially exercises one Nebula process per model,
and prints a final `N/8 passed` summary. See
[`Docs/rgl/phase2_nebula_e2e.md`](Docs/rgl/phase2_nebula_e2e.md) for
prerequisites, validation indicators, return-mode mapping, and
troubleshooting (including the HesaiPandarQT-specific
`enable_hesai_udp_sequence` + `ensure_hesai_pandar_driver_compat`
requirements).

## Skeletal meshes (vehicle bodies, walkers)

Since 2026-09 the RGL scene sync also registers pawn-owned `USkeletalMeshComponent`s
(CARLA vehicles and walkers). Rest-pose vertices, 4-influence bone weights and inverse
bind poses are uploaded once per (mesh, LOD); every tick only bone world matrices are
sent (`rgl_entity_set_pose_world`) and RGL skins on the GPU.

| CVar | Default | Effect |
|---|---|---|
| `rgl.SkeletalMesh.Enable` | 1 | 0 = legacy static-mesh-only behaviour (live: tears skeletal entities down). |
| `rgl.SkeletalMesh.AlwaysTickPose` | 1 | Force `AlwaysTickPoseAndRefreshBones` + disable URO on registered components. Live. |
| `rgl.SkeletalMesh.Scope` | 0 | 0 = pawn-owned only; 1 = all skeletal components. |
| `rgl.SkeletalMesh.MaxEntities` | 64 | Budget; nearest-to-sensor wins (counts entities kept by unregistration hysteresis too). |
| `rgl.SkeletalMesh.MinLOD` | 0 | Lowest LOD index the extractor may use; raise to trade fidelity for VRAM/skinning cost. |

Device memory is roughly **24 B x vertices per registered skeletal entity** (RGL's
SkeletonAnimator + GAS), e.g. ~8 MB for a 347k-vertex LOD0 vehicle; use `MaxEntities` and
`MinLOD` to bound it.

Set via the packaged `Config/DefaultEngine.ini` `[ConsoleVariables]` section (works in
every build configuration, including Shipping), or, in Development/editor builds only,
`-ExecCmds="rgl.SkeletalMesh.Enable 0"`.

> [!IMPORTANT]
> `-ExecCmds=` and `-ini:` command-line overrides are compiled out of Shipping builds —
> they only work in Development/editor. For a packaged Shipping server use the CVar in
> `Config/DefaultEngine.ini`, or the dedicated Shipping-safe switches added on this
> branch: `-rgl-skeletal-mesh-enable=<0|1>`, `-rgl-skeletal-mesh-always-tick-pose=<0|1>`,
> `-rgl-skeletal-mesh-scope=<0|1>`, `-rgl-skeletal-mesh-max-entities=<n>`,
> `-rgl-skeletal-mesh-min-lod=<n>`.

Known limits:

* No morph/cloth/deformer displacement; LOD-dependent bone evaluation may leave distal
  bones of far NPCs stale; discovery latency <= 1 s (scene sync period).
* **Ego self-visibility**: the ego vehicle's own skeletal body is registered too (before
  this change only its door static meshes were). A roof-mounted lidar with a negative
  `lower_fov` therefore returns roof/hood/trunk points at 0-2 m. Filter by range or by the
  ego bounding box downstream if your consumer does not expect them.
* **Ego self-occlusion costs returns**: those rays used to pass through the unregistered
  ego body and hit the scene beyond it. They now terminate on the body and come back below
  the sensor's minimum range, so they are dropped rather than reported. Measured on an
  empty Odaiba with a test ego vehicle and its two roof Pandars, the raw cloud shrinks 17.8%
  (1,186,458 -> 975,408 points over 20 frames) with the skeletal path on. This matches a
  real vehicle, which does occlude its own lidar, but it is a large enough change that
  detector behaviour shifts: in that same scene a roadside object 82 m away went from
  being detected in 2 of 629 frames to 658 of 718, although its own return count was
  unchanged (about 30 per frame either way).
* **Kill-switch latency**: `rgl.SkeletalMesh.Enable 0` stops posing immediately, but the
  existing entities are only removed at the next scene sync (<= 1 s).

Tests: `Automation RunTests CarlaRGL.Skeletal` (editor -game),
`PythonAPI/rgl/tests/check_dynamic_actor_visibility.py` (needs ROS2).

Measured on the Shipping package (`check_dynamic_actor_visibility.py`, same binary,
`rgl.SkeletalMesh.Enable=0` vs `=1`):

| blueprint | OFF | ON |
|---|---|---|
| vehicle.ue4.audi.tt (Door=0) | 12 | 372 |
| vehicle.ue4.ford.mustang (Door=0) | 6 | 367 |
| vehicle.lincoln.mkz (doors) | 207 | 415 |
| walker.pedestrian.0016 | 0 | 78 |

Tick-cost impact (7 walkers + 3 vehicles + 1 RGL lidar, 20 Hz sync, world.tick() wall
time): +11.4% median vs skeletal OFF (within the +20% budget).

## World Partition maps

On World Partition maps (for example Town15), Unreal Engine loads the map in cells around streaming sources, and the RGL scene follows what is loaded. Static meshes retain their sensor-sphere selection and leave the scene when their cell unloads. What is not loaded is not in the RGL scene, just as it is not in the physics scene that `ray_cast` lidars trace. To see an area, get it loaded (see below). Maps without World Partition (for example Town10HD_Opt) keep their existing registration.

On World Partition maps only, non-Static static-mesh components (both Movable and Stationary) and ISM/HISM instances that have not been observed moving are selected against the union of all sensor spheres, with a speed-dependent registration margin. Pawn-owned, physics-driven and attached vehicle parts retain their existing registration; the protection also follows attachment ancestors, movement/skeletal owners and sensor carriers. Skeletal mesh distance, budget and self-cutout policies are unchanged. Enabling `rgl.Scene.WpMovableCulling` requires an actual World Partition world; map names and streaming coverage do not enable it. Setting it to 0 restores the old path.

Each sync still scans the loaded world, including candidates previously left out. Nonzero component/owner velocity or a changed component/instance transform promotes the component to protection for its lifetime; one moving ISM instance protects the whole component. This check is at syncs, not a new full scan on every Update. Registered entity transforms continue to update before raytracing. Regular components use their bounds sphere. Each instance uses the mesh bounds origin transformed to world space and its sphere radius multiplied by the largest absolute scale, including offset pivots and negative/nonuniform scale. Registration needs an intersection with any sensor's load sphere; removal needs to be outside every unload sphere.

The margin uses configured sensor and object speed bounds `V_s` and `V_o` (50 m/s each by default), including objects that may start moving after the scan. With distances in metres and times in simulation seconds, `I` is the effective sync interval setting, `d` the world dt, `U` the maximum scene Update/active sensor pose-refresh gap, and `L` the creation deadline:

```text
T_sync_max = ceil(I / d) * max(U, d) + U   (Update clock, rgl.Scene.SyncClock = 0)
           = I + U                       (simulation clock, the default)
H = T_sync_max + L + U
M = max(MinMarginM, (V_s + V_o) * H)
R_load = max(RegistrationDistance, SensorRange + M)
R_unload = R_load * UnloadDistanceFactor + UnregistrationHysteresis
```

The default simulation clock gives `T_sync_max = 1.1 s` and a 220 m margin at any `d` with the default `I=1 s`, `U=0.1` and `L=1`. The Update clock (the original timer) advances by world dt per scene Update: `I=1 s` is about 2 s between syncs at `d=0.05`, or 4 s at `d=0.025`, when Updates are 0.1 s apart, so the bounds are 2.1/4.1 s and the margins 320/520 m. A WP-only simulation-time deadline forces a sync when `sim_now - last_wp_sync_sim + U > T_assumed + 1e-6`, using the bound saved by the last culling sync. This protects against dt shrinking between syncs. New sensors, pose jumps, time rewind and live changes to the horizon inputs also force a sync; non-WP triggers keep their existing behaviour.

New jobs whose bounds already intersect a sensor's raw range at discovery are created in that Update without entity or mesh-upload caps or ms budgets. This can produce a start-up burst. Pre-emptive creation in the margin and queued removals use the WP queue, prioritizing the earliest range-arrival deadline; cached meshes and individual ISM entities also consume its entity budget. `L` includes all queue/upload/partial-creation waits and is not the 4 ms wall-time budget. That budget is checked between changes after the first change; at least one change proceeds, so it is not a hard upper bound on an Update's wall time.

The continuity contract requires Update/pose gaps within `U`, speeds within the configured bounds, and covered creation jobs completed within `L`. Exceeding these records `rgl_scene_wp_cull_violation` (including `update_gap`, `pose_gap`, `sensor_speed`, `object_speed`, `queue_age` and `late_create`). Only the first interval immediately after a new sensor's init sync may exceed `U` by at most one world dt; its raw value remains in `rgl_scene_wp_cull_startup_gap`. The scene-wide exemption uses the sensors present at the interval's start, so sensors joining at its end do not invalidate that first interval. Later or larger gaps remain violations. Start-up, new sensors, arbitrary teleports and instantaneous mass appearances have no general guarantee of both zero missing entities and a sub-100 ms tick. Unloaded objects remain outside the contract. The audit still counts known WP candidates missing within raw range, even while pending or being promoted.

> [!NOTE]
> Landscape (terrain heightfield) components are not registered in the RGL scene, while
> `ray_cast` lidars do hit them. Where the ground is a Landscape, the RGL lidar gets no
> ground returns.

### Which areas are loaded

Cells are loaded around World Partition streaming sources. On a World Partition map,
CARLA gives every vehicle and walker it spawns a spherical streaming source:

| Spawned actor | Radius of its streaming source |
|---|---|
| Vehicle or walker whose `role_name` contains `hero` or `ego_vehicle` (in any letter case) | `tile_stream_distance` at spawn time (default 3000 m) |
| Any other vehicle or walker (for example `role_name` = `ego`) | 200 m |

The spectator (the player controller's view) is a streaming source too. Sensors and the
other actors CARLA spawns are not.

* Attach RGL lidars to the hero vehicle. A `role_name` such as `ego` does not make a hero:
  that vehicle loads only 200 m around it, and a sensor on it with a longer range sees
  only what other streaming sources load beyond that.
* `tile_stream_distance` applies to the hero only, and only when the hero spawns: the
  radius is taken at spawn time, and changing the setting later does not resize it.
* An `actor_active_distance` larger than `tile_stream_distance` raises
  `tile_stream_distance` to the same value (CARLA logs a warning). The default
  `actor_active_distance` is 2000 m, so a `tile_stream_distance` below 2 km takes effect
  only with a smaller `actor_active_distance`. Set both before spawning the hero:

  ```python
  settings = world.get_settings()
  settings.tile_stream_distance = 500.0   # m: the hero's streaming radius
  settings.actor_active_distance = 500.0  # m: not larger than tile_stream_distance
  world.apply_settings(settings)
  # then spawn the hero (role_name='hero') and attach the lidars to it
  ```

* A roadside sensor (attached to nothing, or to an actor without a streaming source) has
  its range loaded only when another streaming source covers it. Raise the hero's
  `tile_stream_distance` until the hero's sphere holds the sensor's whole range (the
  distance from the hero to the sensor plus the sensor's range is at most the radius), or
  mount the sensor on a vehicle.

### The stream warning

On a World Partition map, after each static scene sync (periodic, and sooner when a sensor
moves), the scene manager checks whether each sensor's range sphere is streamed in (its
World Partition cells are activated). A sensor whose range stays not streamed in for 5 s
of simulation time prints one line, once per registration of the sensor:

```text
rgl_scene_stream_warning t=12.400 sensor=0x7f3a2c001230 range_m=200 parent=BP_Car_C_1 parent_stream_radius_m=150 hint=attach_the_sensor_to_a_hero_vehicle_or_raise_tile_stream_distance
```

| Key | Meaning |
|---|---|
| `t` | Simulation time (s) of the check that printed the line |
| `sensor` | Sensor id (the RGL session handle, as on the `rgl_hit_dump` line) |
| `range_m` | Radius of the checked sphere: the sensor's range (m) |
| `parent` | Root actor of the sensor's attachment chain, or `none` |
| `parent_stream_radius_m` | Radius of that actor's streaming source (m); `-1` when it has none |
| `hint` | What to do: attach the sensor to a hero vehicle, or raise `tile_stream_distance` |

* No line is printed on maps without World Partition, nor while the parent's own
  streaming source holds the whole range sphere: that is streaming still in progress (the
  default 3 km hero radius can take more than 5 s to load at start-up), which a setting
  cannot fix.
* The check runs at syncs only, so the line comes at the first sync at least 5 s after
  the first check that found the range not streamed in.
* The line is printed once per registration of the sensor, even if the range is loaded
  and lost again later. A sensor that stops ticking for more than 5 s is dropped from the
  scene manager; when it ticks again it is registered anew and can print the line again.

### Sync cost with the default 3 km hero radius

Each sync still scans every actor of the loaded world. With the default `tile_stream_distance` (3000 m), the hero keeps a 3 km disc loaded. Measured on 2026-11-20 on pc4090 (RTX 4090), Town15, one 200 m lidar on a driving hero, dt 0.05 s, audit off, the WP knob defaults below, seeds 0/1/2 in run order. These measurements used the original sync settings: Update clock, 1 s interval, uncapped 0.2 R distance trigger and no in-range create priority. The main route uses a 3000 m streaming radius; the separate check uses 150 m. Values are sync `elapsed_ms`:

| Scenario / run | Hero streaming radius | p95, all syncs (ms) | Max excluding first init (ms) | Max including first init (ms) |
|---|---|---|---|---|
| Town15 (3 km) / 1 | 3000 m (default) | 35.73 | 38.56 | 192.75 |
| Town15 (3 km) / 2 | 3000 m (default) | 36.67 | 38.54 | 180.33 |
| Town15 (3 km) / 3 | 3000 m (default) | 36.26 | 40.71 | 190.65 |
| Town15 (150 m) / 1 | 150 m | 25.58 | 28.22 | 185.93 |
| Town15 (150 m) / 2 | 150 m | 24.51 | 26.06 | 194.25 |
| Town15 (150 m) / 3 | 150 m | 23.83 | 27.31 | 199.16 |

The Town15 3 km p95 is 35.73–36.67 ms, down from about 81 ms before WP non-Static culling. The sync cost meets the limits with p95 measured over all syncs (≤50 ms) and max over all except the scene's first init sync (<100 ms), using a start-up window. Later sensors' init syncs remain included. Apply the same window to both the original baseline and the preceding build without WP culling. The old path did its initial registration in `InitializeFromWorld`, outside `elapsed_ms`; this path includes the in-range discovery burst inside the first init sync. The last column equals the first init sync in each of these runs. `wait_ms` times the first RGL API call, including both waiting and that API's work (`wait_api=none`, 0 ms if no call); it is diagnostic. `elapsed_ms`, without subtracting `wait_ms`, determines the sync result.

Start-up is also assessed using raw whole-run ticks and the same fixed window `[60,N)` on both sides (zero-based, only the first 60 ticks excluded):

| Scenario | Whole-run ticks >100 ms, runs 1/2/3 | `[60,N)` ticks >100 ms, runs 1/2/3 |
|---|---|---|
| Town15 (3 km) | 2 / 2 / 2 | 0 / 0 / 0 |
| Town15 (150 m) | 2 / 2 / 2 | 0 / 0 / 0 |
| Odaiba | 1 / 1 / 1 | 0 / 0 / 0 |

Town15 3 km tick maxima are 814.32 / 803.27 / 787.76 ms (the preceding build without WP culling peaked at 810.53 ms; the original baseline at 818.76 ms). For the same route, baseline counts were 3/6/4 whole-run and 1/4/2 after tick 60 in the original baseline, and 4/2/3 and 2/0/1 in the preceding build without WP culling. Odaiba sync maxima remain 8.95 / 8.58 / 8.49 ms (below its 10.27 ms limit). `wp_deadline` fired zero times in all nine runs.

The separate audit comparison used the same binary, route segment and seed 0 with culling off/on; the on runs also include seeds 1/2. These are sync-line p50 counts, not gate `entities` or VRAM MB:

| Scenario | Total entities, off → on | Non-Static registered, off → on | Non-Static reduction |
|---|---|---|---|
| Town15 (3 km) | 169,968 → 28,423–28,793 | 155,514 → 14,109–14,189 | about 90.9% |
| Town15 (150 m) | 85,400 → 27,772 | 71,035 → 13,554 | about 80.9% |

Entity reduction narrows the scene targeted by the later VRAM work; mesh sharing, idle meshes and acceleration structures still require separate MB measurements.

To cut the loaded area and the sync time, lower both distances to what the sensors need:
their range plus their distance from the hero's origin, with some margin.

### Scene sync knobs

The `rgl.Scene.*` and `rgl.Debug.*` knobs are CVars (see
[Skeletal meshes](#skeletal-meshes-vehicle-bodies-walkers) for how to set CVars). A
packaged Shipping server also takes the switch in the second column, for example
`-rgl-scene-audit=10`. The first eleven scene controls and all eight WP controls are printed on an `rgl_scene_config` line at start-up and whenever one changes, together with the WP policy state.

| CVar | Shipping switch | Default | Effect |
|---|---|---|---|
| `rgl.Scene.MaxMeshCreatesPerUpdate` | `-rgl-scene-max-mesh-creates-per-update=` | 8 | Most new RGL meshes (or idle reactivations) per scene Update; the rest wait in a nearest-first queue. 0 = unlimited. |
| `rgl.Scene.CreateBudgetMs` | `-rgl-scene-create-budget-ms=` | 4 | Wall-time budget (ms) for mesh creation per scene Update. 0 = unlimited. |
| `rgl.Scene.UnloadDistanceFactor` | `-rgl-scene-unload-distance-factor=` | 1.0 | Static meshes unload only beyond the registration radius × factor + 20 m. Values below 1 count as 1. |
| `rgl.Scene.IdleMeshCacheMax` | `-rgl-scene-idle-mesh-cache-max=` | 0 | RGL meshes kept resident after their last entity has left, least recently idled destroyed first. 0 = destroy at once. |
| `rgl.Scene.MaxIsmInstanceChangesPerUpdate` | `-rgl-scene-max-ism-instance-changes-per-update=` | 512 | Most Static ISM instances added to or removed from the scene per scene Update when a sync has the ISM groups re-evaluated against the sensor spheres (on the Updates without a sync). 0 = unlimited. |
| `rgl.Scene.SyncClock` | `-rgl-scene-sync-clock=` | 1 | Clock of the periodic sync: 1 = simulation time; 0 = the world dt of the scene Updates only (the original timer: with a 0.1 s lidar at dt 0.025 s, a 1 s interval took about 4 s). Other values count as 1. Legacy value: 0. |
| `rgl.Scene.SyncIntervalSeconds` | `-rgl-scene-sync-interval-seconds=` | 1.0 | Seconds of that clock between periodic syncs, ≥0.1. Legacy value: 1. |
| `rgl.Scene.SyncTriggerRatio` | `-rgl-scene-sync-trigger-ratio=` | 0.2 | Distance trigger: a sensor that moved more than this ratio × R since the last sync syncs at once (R = its registration radius, its range + 20 m for a lidar). >0. Legacy value: 0.2. |
| `rgl.Scene.SyncTriggerMarginSlackM` | `-rgl-scene-sync-trigger-margin-slack-m=` | 5.0 | Caps the distance trigger at max(1 m, (R − range) − this value), so that the sync comes before a part just beyond R at the last sync can reach the range: 15 m when R = range + 20 m. Negative = no cap. Legacy value: −1. |
| `rgl.Scene.InRangeCreatesPerUpdate` | `-rgl-scene-in-range-creates-per-update=` | 16 | Queued static parts whose bounds reach a sensor's range are created first, outside `MaxMeshCreatesPerUpdate` and `CreateBudgetMs`: at most this many per scene Update; the rest wait for that budget. ≥0, 0 = no priority. Legacy value: 0. |
| `rgl.Scene.InRangeCreateBudgetMs` | `-rgl-scene-in-range-create-budget-ms=` | 8.0 | Wall-time cap (ms) of those in-range creations per scene Update, checked between creations; at least one proceeds. ≥0, 0 = no ms cap. Not used with the legacy values. |
| `rgl.Scene.WpMovableCulling` | `-rgl-scene-wp-movable-culling=` | 1 | WP only: 0 = old registration, 1 = distance selection of non-Static static meshes. No effect on non-WP maps. |
| `rgl.Scene.MovableMinMarginM` | `-rgl-scene-movable-min-margin-m=` | 20 | WP only: minimum registration margin (m), ≥20; 0 is invalid. |
| `rgl.Scene.MovableSensorSpeedBoundMps` | `-rgl-scene-movable-sensor-speed-bound-mps=` | 50 | WP only: assumed sensor speed bound (m/s), >0; 0 is invalid. |
| `rgl.Scene.MovableObjectSpeedBoundMps` | `-rgl-scene-movable-object-speed-bound-mps=` | 50 | WP only: assumed speed bound for map parts that may start moving (m/s), >0; 0 is invalid. Observed movement promotes protection. |
| `rgl.Scene.MovableMaxUpdateGapSeconds` | `-rgl-scene-movable-max-update-gap-seconds=` | 0.1 | WP only: maximum scene Update and active sensor pose-refresh gap (sim s), >0; 0 is invalid. |
| `rgl.Scene.MovableCreateLatencySeconds` | `-rgl-scene-movable-create-latency-seconds=` | 1.0 | WP only: covered job completion deadline including all creation waits (sim s), >0; 0 is invalid. |
| `rgl.Scene.MaxMovableEntityChangesPerUpdate` | `-rgl-scene-max-movable-entity-changes-per-update=` | 512 | WP only: entity additions/removals per Update, including cached regular meshes and individual ISM instances; ≥0, 0 = no count cap. In-range-at-discovery jobs bypass caps. |
| `rgl.Scene.MovableEntityBudgetMs` | `-rgl-scene-movable-entity-budget-ms=` | 4.0 | WP only: wall-time budget (ms) for those changes after the first one; ≥0, 0 = no ms cap. In-range-at-discovery jobs bypass budgets. |
| `rgl.Scene.Audit` | `-rgl-scene-audit=` | 0 | Read-only audit: N >= 1 prints an `rgl_scene_audit` line every N-th frame. Scans every actor, so leave it off when timing frames. |
| `rgl.Debug.HitDumpDir` | `-rgl-debug-hit-dump=` | (empty) | Debug: directory that receives one file of hit points per dumped scan. Empty = off. |
| `rgl.Debug.HitDumpEvery` | `-rgl-debug-hit-dump-every=` | 10 | Debug: with a dump directory set, dump each sensor's first scan and then every N-th one. |

Invalid values of the WP, sync-clock and range-edge controls (including non-finite floats or negative count limits) produce a warning and fall back to their defaults. A live enable change is reconciled at the next WP sync; use a fresh server for off/on comparisons. Horizon input changes force re-evaluation.

The sync-clock and range-edge controls close a gap at the edge of a sensor's range. A sensor registers the static parts within R, 20 m beyond its range. With the legacy values (the Update clock and a distance trigger of 0.2 R, 44 m for a 200 m lidar), a part beyond R at one sync could enter the range before the next sync and stay missing until then, and the create budget could hold it in the queue for a few more Updates. Each sensor's numbers are printed once, at its first sync, on an `rgl_scene_sensor` line (`registration_m`, `range_m`, `trigger_m`); the sync line carries `in_range_creates` and `in_range_ms` for that Update's create step. The legacy values restore the original behaviour:

```text
-rgl-scene-sync-clock=0 -rgl-scene-sync-interval-seconds=1 -rgl-scene-sync-trigger-ratio=0.2 -rgl-scene-sync-trigger-margin-slack-m=-1 -rgl-scene-in-range-creates-per-update=0
```

### ISM re-evaluation keys on the log lines

Two keys on different lines have similar names and count the same queue at different
moments: the queue of Static instanced static mesh (ISM) groups waiting to have their
instances compared with the sensor spheres. Every sync refills the queue with all
registered Static ISM groups; the Updates without a sync work through it, capped by
`rgl.Scene.MaxIsmInstanceChangesPerUpdate`.

| Line | Key | Meaning |
|---|---|---|
| `rgl_scene_sync` | `ism_pending` | Entries left in the queue when this sync refilled it: groups the previous round had not reached. Non-zero when the Updates since the previous sync did not finish the round: the cap stopped it, or the syncs came back to back (the round runs only on Updates without a sync). |
| `rgl_scene_audit` | `pending_ism` | Entries in the queue at the audited frame: groups not yet compared with the current sensor spheres. |

Both counts include the entries of components that have been removed since they were
queued; such an entry is dropped when the queue reaches it.

The WP non-Static queue is separate: `wp_pending` is its current job count and `wp_queue_age_max_s` the current oldest pending creation age in sim seconds. `wp_check_n`, `wp_check_instances`, `wp_check_ms`, `wp_entity_added`, `wp_entity_removed` and `wp_entity_ms` accumulate since the previous sync log line; `wp_check_max_ms` is the largest single check in that interval. `wp_inrange_added`/`wp_inrange_ms` describe only this Update's raw-range discovery burst, including upload and the first API wait. `wp_sync_bound_s`/`wp_create_latency_s`/`wp_margin_m` describe this sync's horizon; `wp_violations` counts violations since the previous line, while `wp_deadline_syncs` is cumulative. `wp_promoted` is the current promoted-candidate count. `wp_entities_total` counts regular + all ISM instances + live skeletal entities (excluding synthetic ground); `wp_nonstatic_registered` counts regular/ISM non-Static entities including Stationary, excluding skeletal. The old `entities` key remains the regular entity-map count. Audit `missing_wp_culled_in_range` counts known WP candidates missing within raw range, including pending jobs; it is not a Static mesh creation queue count.
