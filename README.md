Carla & Autoware integration
===============

![Carla Logo](https://carla.org//img/logo/carla-black-m.png)

CARLA is an open-source simulator for autonomous driving research. CARLA has been developed from the ground up to support development, training, and
validation of autonomous driving systems. In addition to open-source code and protocols, CARLA provides open digital assets (urban layouts, buildings,
vehicles) that were created for this purpose and can be used freely. The simulation platform supports flexible specification of sensor suites and
environmental conditions.

![Carla Official Documentation](https://carla-ue5.readthedocs.io)

![Autoware Logo](https://autoware.org/wp-content/uploads/2023/02/984e93_61c0e31c437c4be294009b8effdd3fe0_mv2.webp)

Autoware is the world's leading open-source autonomous driving framework. 
Autoware provides a comprehensive, production-ready software stack designed to accelerate the commercial deployment of autonomous vehicles across diverse platforms and use cases.

![Autoware Official Documentation](https://autowarefoundation.github.io/autoware-documentation/main/home/)

> This branch was created from the carla development branch `ue5-dev` for the **Unreal Engine 5.5 version of CARLA**. 
> This branch exists in parallel with the Unreal Engine 4.26 version of CARLA, in the `ue4-dev` branch.
> Please be sure that this version of CARLA is suitable for your needs as there are significant differences between the UE 5.5 and UE 4.26 versions of CARLA. 

# Installation Guide

## System Requirements
- Ubuntu 22.04
- NVIDIA Drivers 560 (or above)
- CUDA 12 (or above)
- NVIDIA Graphics card with 16Gb VRAM (or more)
- 32Gb RAM Memory (or more for Unreal Editor)
- Intel i7 gen 9th - 11th / Intel i9 gen 9th - 11th / AMD Ryzen 7 / AMD Ryzen 9
- Clang 14 (or above)
- Default system Python 3.10 ONLY (otherwise work with Python environment)
- Around 300Gb free disk space

Reference: https://carla-ue5.readthedocs.io/en/latest/start_quickstart/#before-you-begin

## Prerequisites

Please install prerequisites and dependencies:
```shell
sudo apt update
sudo apt install build-essential ninja-build libvulkan1 python3 python3-dev python3-pip git git-lfs
```

Install from source:
- ROS2 humble - https://docs.ros.org/en/humble/Installation.html
- Autoware - [release version 0.45.1](github.com/autowarefoundation/autoware/tree/0.45.1) or relatively new version

> Please note, `ROS_LOCALHOST_ONLY` is not supported.
> Make sure to use a unique `ROS_DOMAIN_ID` instead, which is optional.

Before proceeding any further, you must link your GitHub account with the Epic Games organization to be able to access Unreal Engine source code.
Please follow the instructions at https://github.com/EpicGames/Signup.

After successful linkage of Epic Account + Github proceed with the next steps.

