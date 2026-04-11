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
    compressed = zlib.compress(raw, 9)
    # Prepend 4-byte big-endian uncompressed size for C++ FCompression::UncompressMemory
    # which requires the exact output size.
    payload = struct.pack(">I", len(raw)) + compressed
    return base64.b64encode(payload).decode("ascii")


def _encode_int_array(ints):
    """Encode int array as delta-int32 + zlib + base64.

    Same encoding as _encode_float_array but without the ×1e6 scaling.
    See _encode_float_array docstring for rationale (FName 1023 char limit).
    """
    deltas = [ints[0]] + [ints[i] - ints[i - 1] for i in range(1, len(ints))]
    raw = struct.pack(f">{len(deltas)}i", *deltas)
    compressed = zlib.compress(raw, 9)
    payload = struct.pack(">I", len(raw)) + compressed
    return base64.b64encode(payload).decode("ascii")


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


def set_azimuth_fov(blueprint, sections):
    """Set azimuth FOV sections (whitelist). Rays outside all sections are masked.

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        sections: list of [start, end] pairs (degrees, 0-360).
            Max 5 sections. Wrap-around supported (start > end).
    Example:
        set_azimuth_fov(bp, [[0, 120], [240, 360]])
    """
    if len(sections) > 5:
        raise ValueError("Maximum 5 azimuth sections supported")
    for s, e in sections:
        if not (0 <= s <= 360 and 0 <= e <= 360):
            raise ValueError(f"Azimuth must be 0-360, got ({s}, {e})")
    blueprint.set_attribute("ray_mask_azimuth",
                            ";".join(f"{s},{e}" for s, e in sections))


def set_disabled_rings(blueprint, ring_ids):
    """Disable specific ring IDs (fault injection).

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        ring_ids: list of ring IDs to disable
    """
    for r in ring_ids:
        if not isinstance(r, int) or r < 0:
            raise ValueError(f"Ring ID must be non-negative integer, got {r}")
    blueprint.set_attribute("ray_mask_rings",
                            ",".join(str(r) for r in ring_ids))


def set_mask_rectangles(blueprint, rects):
    """Mask rectangular regions (self-occlusion).

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        rects: list of dicts with keys: az_start, az_end, el_start, el_end
            azimuth: 0-360 degrees, elevation: -90 to 90 degrees.
            Rays inside any rectangle are masked.
    Example:
        set_mask_rectangles(bp, [
            {"az_start": 150, "az_end": 210, "el_start": -25, "el_end": 5},
        ])
    """
    parts = []
    for r in rects:
        for key in ("az_start", "az_end", "el_start", "el_end"):
            if key not in r:
                raise ValueError(f"Rectangle missing key: {key}")
        parts.append(f"{r['az_start']},{r['az_end']},{r['el_start']},{r['el_end']}")
    blueprint.set_attribute("ray_mask_rects", ";".join(parts))


def set_raw_mask(blueprint, channel_mask):
    """Set per-channel raw mask (advanced/fault injection).

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        channel_mask: list of int (1=enabled, 0=masked), one per channel.
            Applied to all horizontal steps of each channel.
    """
    blueprint.set_attribute("ray_mask_raw", _encode_int_array(channel_mask))
