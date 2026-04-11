"""LiDAR preset: VelodyneVLP16 (extracted from AWSIM RGLUnityPlugin)."""

MODEL = {
    "name": "VelodyneVLP16",
    "channels": 16,
    "range": 100.0,
    "min_range": 0.0,
    "upper_fov": 15.0,
    "lower_fov": -15.0,
    "horizontal_fov": 360.0,
    "rotation_frequency": 10.0,
    "points_per_second": 288000,
    "vertical_angles": [
        15.0, -1.0, 13.0, -3.0, 11.0, -5.0, 9.0, -7.0,
        7.0, -9.0, 5.0, -11.0, 3.0, -13.0, 1.0, -15.0,
    ],
    "horizontal_angle_offsets": [],
    "ring_ids": [
        0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15,
    ],
}

NOISE = {
    "angular_type": "ray",
    "angular_mean": 0.0,
    "angular_stddev": 0.05730,    # degrees (= 0.001 rad)
    "distance_mean": 0.0,
    "distance_stddev_base": 0.02,
    "distance_stddev_rise": 0.0,
    "angular_axis": "Y",
}
