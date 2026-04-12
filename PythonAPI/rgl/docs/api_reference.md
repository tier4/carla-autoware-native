# RGL LiDAR Python API Reference

Python API for configuring RobotecGPULidar (RGL) LiDAR sensors in CARLA.

## Quick Start

```python
import carla
from lidar_models import apply_preset, list_models

client = carla.Client("localhost", 2000)
world = client.get_world()

# List available models
list_models()

# Spawn an RGL LiDAR with VelodyneVLP16 preset
bp = world.get_blueprint_library().find("sensor.lidar.rgl")
apply_preset(bp, "VelodyneVLP16")
sensor = world.spawn_actor(bp, carla.Transform(carla.Location(z=2.5)))
```

---

## Presets

### `list_models()`

Print all available LiDAR model names with channel count, range, and horizontal FOV.

### `apply_preset(blueprint, model_name, apply_noise=True, apply_beam_divergence=False)`

Apply a LiDAR preset to a CARLA blueprint.

**Args:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `blueprint` | `carla.ActorBlueprint` | (required) | Blueprint for `sensor.lidar.rgl` |
| `model_name` | `str` | (required) | Model name from `list_models()` |
| `apply_noise` | `bool` | `True` | Apply preset noise parameters |
| `apply_beam_divergence` | `bool` | `False` | Apply preset beam divergence (58x ray cost) |

**Raises:** `ValueError` if `model_name` is not found.

**Example:**

```python
bp = world.get_blueprint_library().find("sensor.lidar.rgl")
apply_preset(bp, "VelodyneVLP16")                                # noise ON, BD OFF
apply_preset(bp, "HesaiAT128E2X", apply_noise=False)             # noise OFF
apply_preset(bp, "VelodyneVLP16", apply_beam_divergence=True)     # noise ON, BD ON
```

### Available Models (13)

| Model | Channels | Range (m) | H-FOV | Special |
|-------|----------|-----------|-------|---------|
| RangeMeter | 1 | 0-40 | 0° | Single ray |
| SickMRS6000 | 24 | 0-40 | 240° | |
| VelodyneVLP16 | 16 | 0-100 | 360° | Non-uniform angles |
| VelodyneVLP32C | 32 | 0-200 | 360° | Horizontal offsets |
| VelodyneVLS128 | 128 | 0-220 | 360° | |
| HesaiPandar40P | 40 | 0.3-200 | 360° | Horizontal offsets |
| HesaiPandarQT | 64 | 0.1-20 | 360° | Wide FOV ±52° |
| HesaiPandarXT32 | 32 | 0.05-80 | 360° | Uniform 1° |
| HesaiAT128E2X | 128 | 0.5-200 | 120° | Per-channel range (NF/far-field) |
| HesaiQT128C2X | 128 | 0.05-20 | 360° | Bank C/D step offset |
| HesaiPandar128E4X | 128 | 0.3-200 | 360° | |
| HesaiPandar128E4XHighRes | 192 | 0.3-200 | 360° | High-res step offset |
| OusterOS1_64 | 64 | 0.5-90 | 360° | |

---

## Noise Model

### `set_noise(blueprint, **kwargs)`

Override individual noise parameters. Only specified parameters are changed.

**Args:**

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `angular_type` | `str` | - | `"ray"` (pre-raytrace) or `"hitpoint"` (post-raytrace) |
| `angular_mean` | `float` | degrees | Angular noise mean |
| `angular_stddev` | `float` | degrees | Angular noise standard deviation (>= 0) |
| `distance_mean` | `float` | meters | Distance noise mean |
| `distance_stddev_base` | `float` | meters | Distance noise base standard deviation (>= 0) |
| `distance_stddev_rise` | `float` | m/m | Distance noise rise per meter (>= 0) |
| `angular_axis` | `str` | - | Rotation axis: `"X"`, `"Y"`, or `"Z"` |

**Raises:** `ValueError` for invalid values.

**Example:**

```python
apply_preset(bp, "VelodyneVLP16")                       # preset noise applied
set_noise(bp, angular_type="hitpoint")                   # switch to hitpoint-based
set_noise(bp, distance_stddev_base=0.05)                 # increase distance noise
set_noise(bp, angular_stddev=0.1, angular_axis="X")      # custom angular noise
```

### `disable_noise(blueprint)`

Disable all noise (set all parameters to zero).

### Preset Noise Defaults (AWSIM TypicalNoiseParams)

All models share the same defaults:

| Parameter | Value |
|-----------|-------|
| angular_type | `"ray"` |
| angular_mean | 0.0° |
| angular_stddev | 0.05730° (= 0.001 rad) |
| distance_mean | 0.0 m |
| distance_stddev_base | 0.02 m |
| distance_stddev_rise | 0.0 m/m |
| angular_axis | `"Y"` |

---

## Beam Divergence

### `set_beam_divergence(blueprint, horizontal, vertical)`

Set beam divergence in degrees. Both values must be > 0 or both 0.

When enabled, RGL shoots 58 sub-rays per primary ray (3 layers × 19 vertices + center) to simulate beam width. This is a prerequisite for multi-return modes.

**Args:**

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `horizontal` | `float` | degrees | Horizontal beam divergence (>= 0) |
| `vertical` | `float` | degrees | Vertical beam divergence (>= 0) |

**Raises:** `ValueError` if only one is > 0, or if negative.

### `disable_beam_divergence(blueprint)`

Disable beam divergence (set both to 0).

### Preset Beam Divergence Defaults

| Model | Horizontal | Vertical | Source |
|-------|-----------|----------|--------|
| Velodyne VLP16/VLP32C/VLS128 | 0.171887° | 0.0859435° | Manufacturer manual |
| All other models | 0.13° | 0.13° | AWSIM default |

**Note:** `apply_beam_divergence=False` by default in `apply_preset()` due to 58x ray cost.

---

## Multi-Return Mode

### `set_return_mode(blueprint, mode)`

Set the LiDAR return mode. Dual return modes require `beam_divergence > 0`.

**Args:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `mode` | `str` | Return mode string (see table below) |

**Single return modes** (no beam divergence required):

| Mode | Description |
|------|-------------|
| `"first"` | First hit (default) |
| `"second"` | Second hit |
| `"last"` | Last hit |
| `"strongest"` | Strongest intensity hit |

**Dual return modes** (require `beam_divergence > 0`):

| Mode | Description |
|------|-------------|
| `"first_last"` | First + Last |
| `"first_strongest"` | First + Strongest |
| `"first_second"` | First + Second |
| `"last_strongest"` | Last + Strongest |
| `"strongest_second_strongest"` | Strongest + Second Strongest |

**Raises:** `ValueError` for invalid mode string.

**Note:** Dual return modes double the output point count. Each point's `return_type` field indicates which return it is (`RGL_RETURN_TYPE_FIRST=4`, `RGL_RETURN_TYPE_LAST=2`, etc.).

**Example:**

```python
apply_preset(bp, "VelodyneVLP16", apply_beam_divergence=True)
set_return_mode(bp, "first_last")    # dual return
```

---

## Ray Masks

### `set_azimuth_fov(blueprint, sections)`

Set azimuth FOV sections (whitelist). Rays outside all sections are masked.

**Args:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `sections` | `list[list[float, float]]` | List of `[start, end]` pairs in degrees (0-360). Max 5 sections. Wrap-around supported (`start > end`). |

**Example:**

```python
set_azimuth_fov(bp, [[0, 180]])                    # front half only
set_azimuth_fov(bp, [[0, 120], [240, 360]])         # two sectors
set_azimuth_fov(bp, [[270, 90]])                    # wrap-around
```

### `set_disabled_rings(blueprint, ring_ids)`

Disable specific ring IDs (blacklist). For fault injection.

**Args:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `ring_ids` | `list[int]` | Ring IDs to disable (non-negative integers) |

### `set_mask_rectangles(blueprint, rects)`

Mask rectangular regions (blacklist). For self-occlusion exclusion.

**Args:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `rects` | `list[dict]` | List of rectangles with keys: `az_start`, `az_end` (0-360°), `el_start`, `el_end` (-90 to 90°) |

**Example:**

```python
set_mask_rectangles(bp, [
    {"az_start": 150, "az_end": 210, "el_start": -25, "el_end": 5},
])
```

### `set_raw_mask(blueprint, channel_mask)`

Set per-channel raw mask (advanced). 1 = enabled, 0 = masked.

**Args:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `channel_mask` | `list[int]` | One value per channel, applied to all horizontal steps |

### Mask Combination

Multiple mask types can be combined. All conditions are ANDed — a ray must pass all active masks to be enabled.

---

## Registries

### `MODEL_REGISTRY`

`dict[str, dict]` — Maps model name to MODEL dict containing channels, range, angles, ring_ids, etc.

### `NOISE_REGISTRY`

`dict[str, dict]` — Maps model name to NOISE dict containing angular/distance noise parameters.

### `BEAM_DIVERGENCE_REGISTRY`

`dict[str, dict]` — Maps model name to BEAM_DIVERGENCE dict containing horizontal/vertical divergence values.
