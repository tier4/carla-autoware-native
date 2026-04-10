"""AWSIM-compatible LiDAR presets for CARLA RGL.

Usage:
    from rgl_lidar_models import apply_preset, list_models

    list_models()  # prints available model names
    blueprint = world.get_blueprint_library().find("sensor.lidar.rgl")
    apply_preset(blueprint, "VelodyneVLP16")
"""

import base64
import struct
import zlib

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
from .hesai_pandar128e4x_highres import MODEL as _HesaiPandar128E4XHighRes
from .ouster_os1_64 import MODEL as _OusterOS1_64

MODEL_REGISTRY = {m["name"]: m for m in [
    _RangeMeter, _SickMRS6000,
    _VelodyneVLP16, _VelodyneVLP32C, _VelodyneVLS128,
    _HesaiPandar40P, _HesaiPandarQT, _HesaiPandarXT32,
    _HesaiAT128E2X, _HesaiQT128C2X, _HesaiPandar128E4X,
    _HesaiPandar128E4XHighRes, _OusterOS1_64,
]}


def _encode_float_array(floats):
    """Encode float array as delta-int32(×1e6) + zlib + base64.

    CARLA's actor attribute values pass through UE5's FName, which has a hard
    limit of 1023 characters.  A plain comma-separated text representation of
    128-channel float32 angles (e.g. HesaiQT128C2X) exceeds this limit at
    ~1600 chars, making it impossible to transmit per-channel data at full
    float32 precision without compression.

    Encoding pipeline:
      float × 1e6 → int32 (fixed-point, 0.000001° precision ≈ float32)
      → delta encoding (exploits near-monotonic angle sequences)
      → zlib level-9 compression
      → base64 (ASCII-safe for attribute transport)

    Result: 128ch worst case ≈ 572 chars, well within the 1023 char limit.
    """
    ints = [round(f * 1e6) for f in floats]
    deltas = [ints[0]] + [ints[i] - ints[i - 1] for i in range(1, len(ints))]
    raw = struct.pack(f">{len(deltas)}i", *deltas)
    return base64.b64encode(zlib.compress(raw, 9)).decode("ascii")


def _encode_int_array(ints):
    """Encode int array as delta-int32 + zlib + base64.

    Same encoding as _encode_float_array but without the ×1e6 scaling.
    See _encode_float_array docstring for rationale (FName 1023 char limit).
    """
    deltas = [ints[0]] + [ints[i] - ints[i - 1] for i in range(1, len(ints))]
    raw = struct.pack(f">{len(deltas)}i", *deltas)
    return base64.b64encode(zlib.compress(raw, 9)).decode("ascii")


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

    # These attributes require the RGL preset C++ changes.
    # Use try/except so presets degrade gracefully on older builds.
    def _try_set(attr, val):
        try:
            blueprint.set_attribute(attr, val)
        except RuntimeError:
            pass  # attribute not available in this build

    if m.get("min_range", 0) > 0:
        _try_set("min_range", str(m["min_range"]))

    # Encode arrays as delta+zlib+base64 to stay within UE5 FName 1023 char limit.
    if m["vertical_angles"]:
        _try_set("vertical_angles", _encode_float_array(m["vertical_angles"]))

    if m["horizontal_angle_offsets"]:
        _try_set("horizontal_angle_offsets", _encode_float_array(m["horizontal_angle_offsets"]))

    if m["ring_ids"]:
        _try_set("ring_ids", _encode_int_array(m["ring_ids"]))

    if m.get("per_channel_min_ranges"):
        _try_set("per_channel_min_ranges",
                 _encode_float_array(m["per_channel_min_ranges"]))
        _try_set("per_channel_max_ranges",
                 _encode_float_array(m["per_channel_max_ranges"]))
        _try_set("range_pattern_period",
                 str(m.get("range_pattern_period", 0)))

    if m.get("horizontal_step_offsets"):
        _try_set("horizontal_step_offsets",
                 _encode_float_array(m["horizontal_step_offsets"]))
