import carla
import math
import random
import argparse
import time
import PyKDL as kdl

# Sim rate has to be 100 to make /clock tick with 100Hz (it ticks each frame)
DESIRED_SIM_RATE = 100.0  # Hz
SIM_DT = 1.0 / DESIRED_SIM_RATE  # Simulation delta time


def log_info(text):
    print("[INFO]: %s" % text)


def log_warning(text):
    print("[WARNING]: %s" % text)


def log_error(text):
    print("[ERROR]: %s" % text)


class ROS2:
    """
    ROS2 coordinate system to CARLA coordinate system conversions
    ROS2 uses right-handed system and CARLA uses left-handed system.
    Also converts rotation units
    - ROS2 uses radians
    - CARLA uses degrees
    https://github.com/carla-simulator/ros-bridge/blob/e9063d97ff5a724f76adbb1b852dc71da1dcfeec/carla_common/src/carla_common/transforms.py#L307-L310
    """

    def Location(x: float = 0.0, y: float = 0.0, z: float = 0.0):
        """In meters"""
        return carla.Location(x=x,
                              y=-y,
                              z=z)

    def Rotation(roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        """In radians"""
        return carla.Rotation(roll=math.degrees(roll),
                              pitch=-math.degrees(pitch),
                              yaw=-math.degrees(yaw))

    class Transform:
        """
        Transform in ROS2 coordinate system
        """
        x = 0.0
        y = 0.0
        z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        def __init__(self,
                     x: float = 0.0,
                     y: float = 0.0,
                     z: float = 0.0,
                     roll: float = 0.0,
                     pitch: float = 0.0,
                     yaw: float = 0.0):
            self.x = x
            self.y = y
            self.z = z
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw

        def to_carla(self):
            return carla.Transform(ROS2.Location(x=self.x,
                                                 y=self.y,
                                                 z=self.z),
                                   ROS2.Rotation(roll=self.roll,
                                                 pitch=self.pitch,
                                                 yaw=self.yaw))

        def to_kdl(self):
            return kdl.Frame(kdl.Rotation.RPY(self.roll, self.pitch, self.yaw),
                             kdl.Vector(self.x, self.y, self.z))

        @staticmethod
        def from_kdl(F: kdl.Frame):
            x, y, z = F.p[0], F.p[1], F.p[2]
            roll, pitch, yaw = F.M.GetRPY()

            return ROS2.Transform(x, y, z, roll, pitch, yaw)


def chain_transforms(transforms):
    """
    :param transforms: list of ROS2.Transform
    :return: ROS2.Transform
    """
    F = transforms[0].to_kdl()
    for t in transforms[1:]:
        F = F * t.to_kdl()
    return ROS2.Transform.from_kdl(F)


def generate_vlp16_blueprint(blueprint_library):
    """Generates a blueprint for VLP16

    The following assumptions were made based on the configuration in AWSIM:
    - 10 Hz publish frequency
    - 0.2 deg horizontal resolution"""

    blueprint = blueprint_library.find("sensor.lidar.ray_cast")

    # The following values are taken from VLP16 datasheet
    blueprint.set_attribute("channels", "16")
    blueprint.set_attribute("range", "100.0")
    blueprint.set_attribute("upper_fov", "10.0")
    blueprint.set_attribute("lower_fov", "-20.0")

    # Calculated as: horizontal_fov / horizontal_resolution / sensor_tick * channels
    blueprint.set_attribute("points_per_second", "288000")

    blueprint.set_attribute("sensor_tick", "0.1")

    # ROS settings
    blueprint.set_attribute("ros_name", "velodyne_top")  # frame_id
    blueprint.set_attribute("ros_topic_name", "/sensing/lidar/top/pointcloud_raw_ex")

    return blueprint


def generate_traffic_light_camera_blueprint(blueprint_library):
    """Generates a traffic light camera

    The following assumptions were made based on the configuration in AWSIM:
    - 10 Hz publish frequency
    - 1920 x 1080 image resolution
    - 90 deg horizontal field of view"""

    blueprint = blueprint_library.find("sensor.camera.rgb")

    blueprint.set_attribute("fov", "90.0")
    blueprint.set_attribute("image_size_x", "1920")
    blueprint.set_attribute("image_size_y", "1080")
    blueprint.set_attribute("sensor_tick", "0.1")

    # ROS settings
    blueprint.set_attribute("ros_name", "traffic_light_left_camera/camera_optical_link")  # frame_id
    blueprint.set_attribute("ros_topic_name", "/sensing/camera/traffic_light")

    return blueprint


def generate_imu_blueprint(blueprint_library):
    """Generates a blueprint for IMU"""

    blueprint = blueprint_library.find("sensor.other.imu")

    blueprint.set_attribute("sensor_tick", f"{1.0 / 30.0}")

    # ROS settings
    blueprint.set_attribute("ros_name", "tamagawa/imu_link")  # frame_id
    blueprint.set_attribute("ros_topic_name", "/sensing/imu/tamagawa/imu_raw")

    return blueprint


def generate_gnss_blueprint(blueprint_library):
    """Generates a blueprint for GNSS"""

    blueprint = blueprint_library.find("sensor.other.gnss")

    blueprint.set_attribute("sensor_tick", "1.0")

    # ROS settings
    blueprint.set_attribute("ros_name", "map")  # frame_id
    blueprint.set_attribute("ros_topic_name", "/sensing/gnss")

    return blueprint


def spawn_sensors(world, base_link, ego):
    """Spawns sensors relatively to the provided base_link actor

    Positioning of the sensors is taken from the URDF for awsim_sensor_kit_description package at:
    https://github.com/autowarefoundation/autoware_launch/tree/0.45.3/sensor_kit/awsim_sensor_kit_launch/awsim_sensor_kit_description
    """

    blueprint_library = world.get_blueprint_library()

    sensor_kit_blueprint = blueprint_library.find("util.actor.empty")
    sensor_kit_blueprint.set_attribute("ros_name", "sensor_kit_base_link")
    vlp16_blueprint = generate_vlp16_blueprint(blueprint_library)
    traffic_light_camera_blueprint \
        = generate_traffic_light_camera_blueprint(blueprint_library)
    imu_blueprint = generate_imu_blueprint(blueprint_library)
    gnss_receiver_blueprint = generate_gnss_blueprint(blueprint_library)
    vehicle_status_blueprint = blueprint_library.find("sensor.other.vehicle_status")

    sensor_kit_to_base_link_transform = ROS2.Transform(
        x=0.9,
        z=2.0,
        roll=-0.001,
        pitch=0.015,
        yaw=-0.0364)
    sensor_kit = world.spawn_actor(
        sensor_kit_blueprint,
        sensor_kit_to_base_link_transform.to_carla(),
        attach_to=base_link)

    # Spawn top lidar
    lidar_top_to_sensor_kit_transform = ROS2.Transform(yaw=1.575)
    lidar_top = world.spawn_actor(
        vlp16_blueprint,
        lidar_top_to_sensor_kit_transform.to_carla(),
        attach_to=sensor_kit)
    lidar_top.enable_for_ros()

    # Spawn traffic light camera
    traffic_light_left_camera_to_sensor_kit_transform = carla.Transform(
        ROS2.Location(x=0.05, y=0.0175, z=-0.1))
    traffic_light_left_camera = world.spawn_actor(
        traffic_light_camera_blueprint,
        traffic_light_left_camera_to_sensor_kit_transform,
        attach_to=sensor_kit)
    traffic_light_left_camera.enable_for_ros()

    # Spawn IMU
    # NOTE: IMU is mounted to Ego directly, because this is required for angular velocity to work
    base_link_to_pivot_transform = ROS2.Transform(x=-1.39706787)
    sensor_kit_to_base_link_transform = ROS2.Transform(x=0.9,
                                                       z=2.0,
                                                       roll=-0.001,
                                                       pitch=0.015,
                                                       yaw=-0.0364)
    imu_to_sensor_kit_transform = ROS2.Transform(roll=3.14159265359,
                                                 yaw=3.14159265359)

    imu_to_pivot_transform = chain_transforms([base_link_to_pivot_transform,
                                               sensor_kit_to_base_link_transform,
                                               imu_to_sensor_kit_transform])
    imu = world.spawn_actor(
        imu_blueprint,
        imu_to_pivot_transform.to_carla(),
        attach_to=ego)
    imu.enable_for_ros()

    # Spawn GNSS receiver
    gnss_receiver_to_sensor_kit_transform = ROS2.Transform(x=-0.1, z=-0.2)
    gnss_receiver = world.spawn_actor(
        gnss_receiver_blueprint,
        gnss_receiver_to_sensor_kit_transform.to_carla(),
        attach_to=sensor_kit)
    gnss_receiver.enable_for_ros()

    # Spawn Vehicle Status Sensor
    vehicle_status_sensor = world.spawn_actor(
        vehicle_status_blueprint,
        carla.Transform(),
        attach_to=base_link)  # Attach to base_link with no offset, because velocities should come from rear axle
    # # NOTE: Enable for ros is not needed, because this sensor uses a global publisher
    # vehicle_status_sensor.enable_for_ros()


def spawn_ego_with_sensors(world, spawn_point):
    """Spawns a controllable vehicle with a basic sensor configuration

    The sensor configuration is compatible with the one for Lexus RX450h in AWSIM.
    The vehicle itself is replaced by Lincoln MKZ available in CARLA."""

    blueprint_library = world.get_blueprint_library()

    ego_blueprint = blueprint_library.find("vehicle.lincoln.mkz")
    ego_blueprint.set_attribute("role_name", "ego")
    ego_blueprint.set_attribute("ros_topic_name", "/carla/input")  # Default Carla ROS input topic name

    ego = world.spawn_actor(ego_blueprint, spawn_point)

    base_link_blueprint = blueprint_library.find("util.actor.empty")
    base_link_blueprint.set_attribute("ros_name", "sensor_kit_base_link")

    # Transformation between vehicle pivot and projection of the rear
    # axis on the ground (base link) as measured in Unreal Editor
    base_link_to_pivot_transform = ROS2.Transform(x=-1.39706787)
    base_link = world.spawn_actor(
        base_link_blueprint,
        base_link_to_pivot_transform.to_carla(),
        attach_to=ego)

    spawn_sensors(world, base_link, ego)

    return ego


def move_spectator(world, ego_vehicle):
    spectator = world.get_spectator()

    spectator_tf = ego_vehicle.get_transform()
    spectator_offset = carla.Transform(ROS2.Location(x=-6.0, z=1.5))

    spectator_with_offset_position = spectator_tf.transform(spectator_offset.location)

    spectator_tf = carla.Transform(spectator_with_offset_position, spectator_tf.rotation)

    spectator.set_transform(spectator_tf)


def apply_world_settings(client, world, map_name="Town10HD_Opt"):
    """
    Applies Synchronous mode + fixed time-step into world settings.

    Store here all settings related to simulation world.
    """

    # Load the desired map
    client.load_world(map_name)

    # Get Settings
    settings = world.get_settings()

    # Set synchronous mode
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = SIM_DT

    # Set physics substepping
    settings.substepping = True
    settings.max_substep_delta_time = 0.001  # max 1 ms per physics substep, swap to 0.01 if no neet of extreme physics realism
    settings.max_substeps = 10


    world.apply_settings(settings)

    # Disable TF publishing in CARLA to avoid conflicts.
    world.set_publish_tf(
        False)  # Autoware will be publishing TF information based on the URDF files of the vehicle and sensor kit.
    # client.reload_world(False)  # reload map keeping the world settings


def run_simulation_loop(target_time_scale=1.0, acceptable_lag=0.05, should_resync=False, world=None, ego=None, follow_ego=False):
    """
    Runs a real-time simulation loop for a synchronous simulation environment.

    The loop advances the simulation world in fixed time steps determined by
    `target_time_scale`, maintaining real-time pacing. It adjusts for timing
    and can optionally resynchronize if the simulation falls behind.

    Parameters
    ----------
    world : object
        The simulation world instance.
    ego : object, optional
        The main vehicle actor - player pawn.
    target_time_scale : float, default=1.0
        The simulation speed multiplier. Higher values make the simulation run faster, and lower to run slower
        relative to real time (2.0 = twice real-time speed, 0.5 = half the real-time speed).
    acceptable_lag : float, default=0.05
        The maximum acceptable lag. If the loop falls behind by more than
        this value, a warning is logged.
    should_resync : bool, default=False
        If True, the simulation will resynchronize to the current time when lag exceeds
        the acceptable threshold.
    follow_ego : bool, default=False
        If True, moves the spectator camera to follow the ego actor each tick.

    Notes
    -----
    - This function runs indefinitely until interrupted (e.g., with Ctrl+C).
    - When interrupted, it restores the world to asynchronous mode to prevent crashes.
    """

    real_dt = SIM_DT / target_time_scale
    frame_count = 0
    next_tick_time = time.time()

    try:
        while True:
            current_time = time.time()

            # Sleep until it's time for the next tick
            wait_time = next_tick_time - current_time
            if wait_time > 0:
                time.sleep(wait_time)

            # Advance simulation
            world.tick()
            frame_count += 1

            # Optional spectator movement
            if follow_ego and ego is not None:
                move_spectator(world, ego)

            # Schedule next tick
            next_tick_time += real_dt

            # Detect and handle lag
            lag = time.time() - next_tick_time
            if lag > acceptable_lag:
                log_warning(f"Simulation is {lag * 1000:.1f} ms behind schedule")
                if should_resync:
                    next_tick_time = time.time()

    except KeyboardInterrupt:
        log_info("Exiting simulation loop, restoring asynchronous mode...")
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose', action='store_true', dest='debug',
        help='Print debug information')
    argparser.add_argument(
        '--host', metavar='H', default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port', metavar='P', default=2000, type=int,
        help='TCP port of the host server (default: 2000)')
    argparser.add_argument(
        '--time_scale', metavar='T', default=1.0, type=float,
        help='Simulation timescale (default: 1.0)')
    argparser.add_argument(
        '--follow', action='store_true',
        help='Follow Ego vehicle')
    args = argparser.parse_args()

    # Get Client info
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()

    # Apply Settings
    apply_world_settings(client, world)
    log_info("Simulation time scale is %f" % args.time_scale)

    # Spawn Ego
    spawn_point = random.choice(world.get_map().get_spawn_points())
    ego = spawn_ego_with_sensors(world, spawn_point)

    world.tick()  # tick to process the changes (settings, ego + sensors spawn)
    move_spectator(world, ego)

    log_info('Ego spawned!')
    log_warning('Kill this script before stopping simulation!')

    # Run simulation loop
    run_simulation_loop(target_time_scale=args.time_scale, follow_ego=True, world=world, ego=ego)


if __name__ == '__main__':
    main()
