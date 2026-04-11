# CARLA Docker Development Environment

Development container for building, testing, and automating CARLA with GPU support.

## Prerequisites

- Docker Engine
- NVIDIA Container Toolkit
- NVIDIA GPU + driver

## Setup (first time only)

```bash
cd CarlaUE5/docker/

# Create config file and edit paths for your environment
cp .env.example .env
vim .env

# Build the image
docker compose build
```

## Start and login

```bash
# Start the container (background)
docker compose up -d

# Login
docker compose exec carla bash

# Logout (container keeps running after exit)
exit
```

## Usage inside the container

```bash
# ROS2 environment is auto-sourced (.bashrc)

# CARLA configure
cmake -G Ninja -S . -B Build \
  --toolchain=$PWD/CMake/Toolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_ROS2=ON -DENABLE_RGL=ON \
  -DCARLA_DDS_VENDOR=CycloneDDS \
  -DCARLA_PACKAGE_COMPRESSION=zstd

# Build
cmake --build Build --target carla-unreal-editor

# Package build
cmake --build Build --target package-development

# Run tests
python3 PythonAPI/examples/rgl_test_regression.py --test spawn

# Claude Code
claude
```

## Stop and restart

```bash
# Stop (removes container)
docker compose down

# Restart (e.g. after config changes)
docker compose down && docker compose up -d
```

## .env configuration

| Variable | Description | Example |
|----------|-------------|---------|
| `WORK_PATH` | Parent directory of UE5/CARLA/RGL (required) | `/mnt/dsk0/wk0/CARLA/T4ForkWithRGL` |
| `UE5_SUBDIR` | UE5 subdirectory name | `UnrealEngine5_carla` |
| `CARLA_SUBDIR` | CARLA subdirectory name | `CarlaUE5` |
| `AWSIM_PATH` | AWSIM source (optional) | `/mnt/dsk0/wk0/AWSIM/OSS/AWSIM` |
| `AUTOWARE_PATH` | Autoware installation (optional) | `/mnt/dsk0/wk0/ROS2/humble/AW-OSS/1.7.1/autoware` |
| `CLAUDE_CONFIG` | Claude Code config directory | `~/.claude` |
| `ANTHROPIC_API_KEY` | Claude API key (optional) | `sk-ant-...` |
| `NETWORK_MODE` | `bridge` (default, safe) / `host` (for ROS2 DDS) | `bridge` |
| `UID` / `GID` | Host user/group ID (`id -u` / `id -g`) | `1000` |

## Network mode

- **bridge** (default): Only ports 2000-2002 and 1985 are exposed. Isolated from host network.
- **host**: Direct host network access. Required for ROS2 DDS multicast discovery.

Switch via `NETWORK_MODE` in `.env`.

## Bind mount layout

```
Host                                      Container (same absolute path)
${WORK_PATH}/                        →  ${WORK_PATH}/          (read-write)
  ├── UnrealEngine5_carla/                  UE5 engine
  ├── CarlaUE5/                             CARLA source + build
  └── RobotecGPULidar/                      RGL library

${AWSIM_PATH}                        →  ${AWSIM_PATH}          (read-only)
${AUTOWARE_PATH}                     →  ${AUTOWARE_PATH}       (read-only)
~/.claude/                           →  /home/carla/.claude    (read-write)
```

Mounted at the same absolute path as the host so that symlinks (e.g. Content created by `carla-env.sh`) resolve correctly inside the container.

## Parallel environments

Create a hardlink copy of UE5 with `carla-env.sh` and point to a different WORK_PATH:

```bash
# Create 2nd environment on host
cd /disk1/carla-2nd/CarlaUE5
tools/carla-env.sh setup /path/to/1st-env

# Start with a separate .env (-p sets project name)
docker compose -p carla-2nd --env-file .env.2nd up -d
```

## Rebuilding the image

After modifying the Dockerfile (e.g. adding packages):

```bash
docker compose build
```

## Notes

- UE5 source is not included in the image (bind mounted). Access requires linking your GitHub account to Epic Games.
- `--privileged` is never used. Container impact is limited to bind-mounted directories and the container filesystem.
