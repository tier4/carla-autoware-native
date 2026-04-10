"""AWSIM-compatible LiDAR presets for CARLA RGL.

Usage:
    from rgl_lidar_models import apply_preset, list_models

    list_models()  # prints available model names
    blueprint = world.get_blueprint_library().find("sensor.lidar.rgl")
    apply_preset(blueprint, "VelodyneVLP16")
"""

from .range_meter import MODEL as _RangeMeter
from .sick_mrs6000 import MODEL as _SickMRS6000
from .velodyne_vlp16 import MODEL as _VelodyneVLP16
from .velodyne_vlp32c import MODEL as _VelodyneVLP32C
from .velodyne_vls128 import MODEL as _VelodyneVLS128
from .hesai_pandar40p import MODEL as _HesaiPandar40P
from .hesai_pandarqt import MODEL as _HesaiPandarQT
from .hesai_pandarxt32 import MODEL as _HesaiPandarXT32
from .hesai_at128e2x import MODEL as _HesaiAT128E2X
from .hesai_qt128c2x import MODEL as _HesaiQT128C2X
from .hesai_pandar128e4x import MODEL as _HesaiPandar128E4X
from .ouster_os1_64 import MODEL as _OusterOS1_64

MODEL_REGISTRY = {m["name"]: m for m in [
    _RangeMeter, _SickMRS6000,
    _VelodyneVLP16, _VelodyneVLP32C, _VelodyneVLS128,
    _HesaiPandar40P, _HesaiPandarQT, _HesaiPandarXT32,
    _HesaiAT128E2X, _HesaiQT128C2X, _HesaiPandar128E4X,
    _OusterOS1_64,
]}


def list_models():
    """Print all available LiDAR model names."""
    for name, m in sorted(MODEL_REGISTRY.items()):
        print(f"  {name:24s} {m['channels']:>3d}ch  range={m['min_range']}-{m['range']}m  "
              f"H-FOV={m['horizontal_fov']}")


def apply_preset(blueprint, model_name):
    """Apply a LiDAR preset to a CARLA blueprint.

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        model_name: one of the names from list_models() (e.g. "VelodyneVLP16")
    """
    if model_name not in MODEL_REGISTRY:
        available = ", ".join(sorted(MODEL_REGISTRY.keys()))
        raise ValueError(f"Unknown model '{model_name}'. Available: {available}")

    m = MODEL_REGISTRY[model_name]

    blueprint.set_attribute("channels", str(m["channels"]))
    blueprint.set_attribute("range", str(m["range"]))
    blueprint.set_attribute("upper_fov", str(m["upper_fov"]))
    blueprint.set_attribute("lower_fov", str(m["lower_fov"]))
    blueprint.set_attribute("horizontal_fov", str(m["horizontal_fov"]))
    blueprint.set_attribute("rotation_frequency", str(m["rotation_frequency"]))
    blueprint.set_attribute("points_per_second", str(m["points_per_second"]))

    if m.get("min_range", 0) > 0:
        blueprint.set_attribute("min_range", str(m["min_range"]))

    if m["vertical_angles"]:
        blueprint.set_attribute("vertical_angles",
                                ",".join(str(a) for a in m["vertical_angles"]))

    if m["horizontal_angle_offsets"]:
        blueprint.set_attribute("horizontal_angle_offsets",
                                ",".join(str(a) for a in m["horizontal_angle_offsets"]))

    if m["ring_ids"]:
        blueprint.set_attribute("ring_ids",
                                ",".join(str(r) for r in m["ring_ids"]))
