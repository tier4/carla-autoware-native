import carla
import math
import random
import argparse
import time
import PyKDL as kdl

# Sim rate has to be 100 to make /clock tick with 100Hz (it ticks each frame)
DESIRED_SIM_RATE = 100

def info(text):
    print("[INFO]: %s" % text)

def warning(text):
    print("[WARNING]: %s" % text)

def error(text):
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
    def Location(x : float = 0.0, y : float = 0.0, z : float = 0.0):
        """In meters"""
        return carla.Location(x =  x,
                              y = -y,
                              z =  z)

    def Rotation(roll : float = 0.0, pitch : float = 0.0, yaw : float = 0.0):
        """In radians"""
        return carla.Rotation(roll  =  math.degrees(roll),
                              pitch = -math.degrees(pitch),
                              yaw   = -math.degrees(yaw))

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
                     x : float = 0.0,
                     y : float = 0.0,
                     z : float = 0.0,
                     roll : float = 0.0,
                     pitch : float = 0.0,
                     yaw : float = 0.0):
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
    The vehicle itself is replaced by Linkoln MKZ available in CARLA."""

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

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()

    # Autoware will be publishing TF information based on the URDF files
    # of the vehicle and sensor kit. Disable TF publishing in CARLA
    # to avoid conflicts.
    world.set_publish_tf(False)

    spawn_point = carla.Transform(
        carla.Location(x=-266.9, y=216.1, z=1.0),
        carla.Rotation(pitch=0.0, yaw=-33.0, roll=0)
    )

    # spawn_point = random.choice(custom_spawn_points)
    ego = spawn_ego_with_sensors(world, spawn_point)
    move_spectator(world, ego)

    info('Ego spawned!')
    time.sleep(0.05)  # Without this sometimes spectator would not move

    # Set synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / DESIRED_SIM_RATE
    world.apply_settings(settings)

    info("Simulation time scale is %f" % args.time_scale)

    warning('Kill this script before stopping simulation!')

    frame_count = 0

    start_time = time.time()
    last_behind_error_time = start_time
    # Tick world to follow desired time_scale
    try:
        while True:
            current_time = time.time()
            real_time_passed = current_time - start_time
            sim_time_passed = real_time_passed * args.time_scale
            desired_frame_count = math.floor(sim_time_passed * DESIRED_SIM_RATE)

            if desired_frame_count <= frame_count:
                continue

            frame_behind_count = max(desired_frame_count - frame_count, 0)
            if (frame_behind_count > 10) and \
                (current_time > last_behind_error_time + 2.0):  # Error every 2 seconds
                    last_behind_error_time = current_time
                    error(f'Simulation cannot keep up with the desired time scale!!! ({frame_behind_count} frames behind)')

            world.tick()
            frame_count += 1

            if args.follow:
                move_spectator(world, ego)

    except:  # KeyboardInterrupt:
        info("Exiting, restoring asynchronous mode...")
        # Set asynchronous mode, otherwise simulation will crash
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

if __name__ == '__main__':
    main()
