import carla
import math
import random
import argparse

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

def spawn_sensors(world, base_link):
    """Spawns sensors relatively to the provided base_link actor

    Positioning of the sensors is taken from the URDF for awsim_sensor_kit_description package at:
    https://github.com/autowarefoundation/autoware_launch/tree/0.45.3/sensor_kit/awsim_sensor_kit_launch/awsim_sensor_kit_description
    """

    blueprint_library = world.get_blueprint_library()

    empty_blueprint = blueprint_library.find("util.actor.empty")
    vlp16_blueprint = generate_vlp16_blueprint(blueprint_library)
    traffic_light_camera_blueprint \
        = generate_traffic_light_camera_blueprint(blueprint_library)
    imu_blueprint = generate_imu_blueprint(blueprint_library)
    gnss_receiver_blueprint = generate_gnss_blueprint(blueprint_library)
    vehicle_status_blueprint = blueprint_library.find("sensor.other.vehicle_status")

    sensor_kit_to_base_link_transform = carla.Transform(
        carla.Location(x=0.9, z=2.0),
        carla.Rotation(
            roll=math.degrees(-0.001),
            pitch=math.degrees(0.015),
            yaw=math.degrees(-0.0364)))
    sensor_kit = world.spawn_actor(
        empty_blueprint,
        sensor_kit_to_base_link_transform,
        attach_to=base_link)

    # Spawn top lidar
    lidar_top_to_sensor_kit_transform = carla.Transform(
        carla.Location(),
        carla.Rotation(
            yaw=math.degrees(1.575)))
    lidar_top = world.spawn_actor(
        vlp16_blueprint,
        lidar_top_to_sensor_kit_transform,
        attach_to=sensor_kit)
    lidar_top.enable_for_ros()

    # Spawn traffic light camera
    traffic_light_left_camera_to_sensor_kit_transform = carla.Transform(
        carla.Location(x=0.05, y=0.0175, z=-0.1))
    traffic_light_left_camera = world.spawn_actor(
        traffic_light_camera_blueprint,
        traffic_light_left_camera_to_sensor_kit_transform,
        attach_to=sensor_kit)
    traffic_light_left_camera.enable_for_ros()

    # Spawn IMU
    imu_to_sensor_kit_transform = carla.Transform(
        carla.Location(),
        carla.Rotation(
            roll=math.degrees(3.14159265359),
            yaw=math.degrees(3.14159265359)))
    imu = world.spawn_actor(
        imu_blueprint,
        imu_to_sensor_kit_transform,
        attach_to=sensor_kit)
    imu.enable_for_ros()

    # Spawn GNSS receiver
    gnss_receiver_to_sensor_kit_transform = carla.Transform(
        carla.Location(x=-0.1, z=-0.2))
    gnss_receiver = world.spawn_actor(
        gnss_receiver_blueprint,
        gnss_receiver_to_sensor_kit_transform,
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

    empty_blueprint = blueprint_library.find("util.actor.empty")

    # Transformation between vehicle pivot and projection of the rear
    # axis on the ground (base link) as measured in Unreal Editor
    base_link_to_pivot_transform = carla.Transform(
        carla.Location(x=-1.39706787))
    base_link = world.spawn_actor(
        empty_blueprint,
        base_link_to_pivot_transform,
        attach_to=ego)

    spawn_sensors(world, base_link)

    return ego

def move_spectator(world, ego_vehicle):
    spectator = world.get_spectator()

    spectator_tf = ego_vehicle.get_transform()
    spectator_offset = carla.Transform(carla.Location(x=-6.0, z=1.5))

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

    spawn_point = random.choice(world.get_map().get_spawn_points())
    ego = spawn_ego_with_sensors(world, spawn_point)
    move_spectator(world, ego)

    print('Ego spawned!')

    if args.follow:
        print('Kill this script before stopping simulation!')
    while args.follow:
        world.wait_for_tick()
        move_spectator(world, ego)

if __name__ == '__main__':
    main()
