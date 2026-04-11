# CARLA UE5 + RGL development container
# Private use only — do not publish (UE5 EULA restriction on redistribution)
#
# Build:
#   docker compose build
# Or:
#   docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -t carla-dev .

FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# ---------------------------------------------------------------------------
# Build tools + UE5 dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build essentials
    cmake ninja-build g++ clang lld \
    python3 python3-pip python3-dev \
    git curl wget ca-certificates gnupg lsb-release \
    # UE5 runtime/build dependencies
    libsdl2-dev libvulkan-dev libomp-dev xdg-user-dirs \
    # Package compression
    zstd \
    # Misc
    sudo locales \
    && rm -rf /var/lib/apt/lists/*

# Locale (UE5 needs UTF-8)
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ---------------------------------------------------------------------------
# ROS2 Humble
# ---------------------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Python packages
# ---------------------------------------------------------------------------
RUN pip3 install --no-cache-dir numpy

# ---------------------------------------------------------------------------
# Node.js + Claude Code
# ---------------------------------------------------------------------------
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && npm install -g @anthropic-ai/claude-code \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# User setup (match host UID/GID for bind mount permissions)
# ---------------------------------------------------------------------------
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} carla 2>/dev/null || true \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash carla \
    && echo "carla ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/carla

USER carla

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/carla/.bashrc

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
# CARLA_UNREAL_ENGINE_PATH is set via docker-compose environment
# (must match host path for symlink compatibility)

# Default workdir; overridden by docker-compose working_dir
WORKDIR /workspace

CMD ["/bin/bash"]
