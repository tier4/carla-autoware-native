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

# Installation Guide

1. **Clone tier4 Carla Autoware support repo**
    ```shell
    git clone -b autoware-support git@github.com:tier4/carla-autoware-native.git CarlaUE5
    ```
   
2. **Modify a Carla setup script**
    ```shell
    cd CarlaUE5
    nano ./CarlaSetup.sh
    ```
   Then find line 108 and change `https` to `ssh`:
    ```diff
    - UE5_URL=https://github.com/CarlaUnreal/UnrealEngine.git
    + UE5_URL=git@github.com:CarlaUnreal/UnrealEngine.git
    ```

3. **Run installation script**
    ```shell
    ./CarlaSetup.sh --interactive
    ```
    
> [!IMPORTANT]
> _UnrealEngine5_carla is built alongside the CarlaUE5 - in the same root directory.
> Building Unreal Engine from source can take 3-4 hours, depending on your machine!_

The following actions will be taken by running the script:
- installation of dependencies
- downloading the Carla source repository
-  the Carla Unreal Content
- downloading and building the dedicated fork of Unreal Engine 5.5 for Carla
- building the cmake project - Lib Carla
- building and installing the Python API - carla package
- building the Carla Unreal Project ( including built Lib Carla )

4. **Build Carla** 
    ```shell
    cmake --build Build
    ```

5. **Install Python API**
    ```shell
    pip3 install Build/PythonAPI/dist/carla-***.whl
    ```

> [!NOTE] 
> _Step 4. and 5. can be combined into one command:_
```shell
cmake --build Build --target carla-python-api-install
```

## Open Carla in Unreal Editor
> [!IMPORTANT]
> _If it's the first time running the project in the editor, it might take up to 1 hour to build the project_

There are two ways to open Carla in Unreal Editor:
1. Using cmake command - Carla predefined command

    This process takes longer, because all carla dependencies are being built and linked before lunching UE Editor
    ```shell
    cmake --build Build --target launch
    ```
2. Using Unreal Build Tool

    Previous cmake command runs this part to open editor. Remember to run this command in `CarlaUE5` directory, otherwise provide absolute paths.
    ```shell
    $CARLA_UNREAL_ENGINE_PATH/Engine/Binaries/Linux/UnrealEditor $PWD/Unreal/CarlaUnreal/CarlaUnreal.uproject
    ```
   
### Recommended approach to save time
Run the first command (with cmake) only and only when you've made changes inside LibCarla or PythonAPI.

If you've made only changes inside Carla Unreal - use the 2nd command to only launch editor.

## Shipping Carla in a package
Carla provides option to ship builds into packages.
Before running command ensure you are in the `CarlaUE5` directory (root where you’ve cloned the project).

The package will be generated in the directory `/Build/Package`.
```shell
cmake --build Build --target package
```

### Launching shipped project
```shell
cd Build/Package/Carla-0.10.0-Linux-Shipping/Linux
./CarlaUnreal.sh --ros2
```

> [!IMPORTANT]
> This script runs only the Carla Server!
> Please follow next section to understand how to connect a client, to spawn ego vehicles with sensors.

## Launching Carla simulation
All Carla demos and showcases are available at PythonAPI/examples. 
Feel free to explore them. To run our autoware demo, follow these steps:

1. Either run the Carla in Unreal Editor or the packaged version
    ```shell
    # Unreal Editor, then click play (ALT+P)
    cmake --build Build --target launch
   
    # Or packed version
    cd Carla-0.10.0-Linux-Shipping/Linux
    ./CarlaUnreal.sh --ros2
    ```
2. Run the Python client script
   This script spawns Ego and attaches all sensors to it at runtime.
    ```shell
    # Source build
    cd CarlaUE5
    python3 PythonAPI/examples/autoware_demo.py
    
    # Or packaged version
    cd Carla-0.10.0-Linux-Shipping/
    python3 PythonAPI/examples/autoware_demo.py
    ```
### Example - Test Environment
1. Source ros2 and Autoware
    ```shell
    source /opt/ros/humble/setup.bash
    # Autoware Workspace is path where you've installed autoware
    source AutowareWorkspace/release-0.45.1/install/setup.bash
    ```

2. Run ros2 command to drive forward
    ```shell
    ros2 topic pub --once /control/command/control_cmd autoware_control_msgs/msg/Control "{
      stamp: {sec: 0, nanosec: 0},
      control_time: {sec: 0, nanosec: 0},
      lateral: {
        stamp: {sec: 0, nanosec: 0},
        control_time: {sec: 0, nanosec: 0},
        steering_tire_angle: 0.0,
        steering_tire_rotation_rate: 0.0,
        is_defined_steering_tire_rotation_rate: false
      },
      longitudinal: {
        stamp: {sec: 0, nanosec: 0},
        control_time: {sec: 0, nanosec: 0},
        velocity: 20.0,
        acceleration: 1.0,
        jerk: 0.5,
        is_defined_acceleration: true,
        is_defined_jerk: true
      }
    }"
    ```

## Launching Carla with Autoware
1. Navigate to the Autoware repo and source Autoware

    Open 1st terminal window and source environment:
    ```shell
    cd AutowareWorkspace/release-0.45.1
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ```
   
2. Run Carla either in the editor or the packaged version

    In the 2nd terminal run:
    ```shell
    # Unreal Editor, then click play (ALT+P)
    cd CarlaUE5
    cmake --build Build --target launch
   
    # Or packed version
    cd Carla-0.10.0-Linux-Shipping/Linux
    ./CarlaUnreal.sh --ros2
    ```
   
3. Run the Python client

    In the 3rd terminal window run demo scenario script to spawn the Ego with sensors:
    ```shell
    cd CarlaUE5
    python3 PythonAPI/examples/autoware_demo.py
    ```

4. Get the path of the Autoware map

   We will be launching the `Town10HD_Opt` since it’s a default map for `CarlaUE5`. Reference: https://github.com/autowarefoundation/autoware_universe/tree/main/simulator/autoware_carla_interface/#Setup

   To obtain compatible map, follow steps:

   1. Download Town10 lanelet2 and point cloud from this link: https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/
   Put those 2 files in one directory, f.e autoware_map/Town10/
   2. Rename files:
    ```shell
    mv Town10HD.pcd pointcloud_map.pcd
    mv Town10HD.osm lanelet2_map.osm
    ```

   3. Create a map projector
    ```shell
    touch map_projector_info.yaml
    echo "projector_type: Local" >> map_projector_info.yaml
    ```

   4. Save the path of <path_to>/autoware_map/Town10/ for later

5. Launch Autoware with the map file
   Go back the 1st terminal, where you sourced Autoware.
   Replace accordingly a `<path_to>/autoware_map/Town10/` - path of the directory with lanelet2 map, pointcloud map, and map projector file.

    ```shell
    ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=<path_to>/autoware_map/Town10/
    ```