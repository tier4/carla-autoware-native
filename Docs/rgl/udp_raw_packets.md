# LiDAR UDP Raw Packet (AWSIM Compatible)

Emit raw UDP packets from a CARLA RGL LiDAR sensor in the wire format
of real Velodyne / Hesai products. Production ROS 2 drivers such as
tier4/nebula can consume the simulated stream unchanged.

## Supported LiDAR Models

| Model name | Packet format | Default dst_port |
|---|---|---|
| `VelodyneVLP16` | Velodyne Legacy (1206 B) | 2368 |
| `VelodyneVLP32C` | Velodyne Legacy (1206 B) | 2368 |
| `VelodyneVLS128` | Velodyne Legacy (1206 B) | 2368 |
| `HesaiPandar40P` | Hesai Standard | 2368 |
| `HesaiPandarQT` | Hesai PandarQT | 2368 |
| `HesaiPandarXT32` | Hesai XT32 (UDP seq) | 2368 |
| `HesaiQT128C2X` | Hesai QT128 (UDP seq) | 2368 |
| `HesaiPandar128E4X` | Hesai Pandar128 (UDP seq) | 2368 |

## Prerequisite: build RGL with the UDP extension

The UDP extension lives in the private repository
`RobotecAI/RGL-extension-udp`. An SSH key with read access to
`git@github.com:` is required to clone it.

The CARLA RGL setup script `RglSetup.sh` already exposes a `--with-udp`
flag that clones and builds the extension automatically (the same
mechanism used by `--with-weather`):

```bash
cd /path/to/CarlaUE5

# Add --with-udp to the usual prepare arguments (--optix-dir, etc.)
bash RglSetup.sh prepare --optix-dir=/path/to/optix --with-udp

# Standard CARLA setup
bash CarlaSetup.sh -i

# Build CARLA linked against the UDP-enabled RGL
bash RglSetup.sh build
```

CARLA itself still builds without the UDP extension; UDP features then
disable themselves at runtime with a warning, leaving every other RGL
feature (ROS 2 publish, Yield) untouched. The presence of the UDP
extension in `libRobotecGPULidar.so` can be checked at simulation
startup by looking for the log line:

`RGLBackendImpl: UDP publishing requested but RGL_EXTENSION_UDP not present ...`

## Python usage

```python
import carla
from lidar_models import apply_preset

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(5.0)
world = client.get_world()
bp    = world.get_blueprint_library().find("sensor.lidar.rgl")

# Example 1: Velodyne VLP16 -> localhost:2368
# VLP16 does not accept "first" in the UDP whitelist, so set "strongest".
apply_preset(bp, "VelodyneVLP16",
             udp_publish={"dest_ip": "127.0.0.1", "dest_port": 2368})
bp.set_attribute("return_mode", "strongest")

# Example 2: Hesai Pandar40P with ROS 2 driver coordinate compat + UDP
# hesai_ros_driver_compat defaults to None (auto-ON for Hesai), so the
# -90 deg sweep start is applied automatically. The explicit True below is
# equivalent and remains valid.
apply_preset(bp, "HesaiPandar40P",
             hesai_ros_driver_compat=True,  # redundant for Hesai (default is auto-ON)
             udp_publish={"dest_ip": "127.0.0.1",
                          "dest_port": 2368,
                          "ensure_hesai_pandar_driver_compat": True})
bp.set_attribute("return_mode", "strongest")  # Pandar40P also rejects "first"

# Example 3: Hesai QT128C2X (UDP sequence + blockage detection)
# -90 deg sweep start is applied automatically (Hesai default).
# QT128C2X accepts "first", so explicit return_mode is optional.
apply_preset(bp, "HesaiQT128C2X",
             udp_publish={"dest_ip": "127.0.0.1",
                          "enable_hesai_udp_sequence": True,
                          "enable_hesai_blockage_detection": True})

spawn_point = world.get_map().get_spawn_points()[0]
sensor = world.spawn_actor(bp, spawn_point)
```

### Return mode whitelist (important)

Each model has a whitelist of `return_mode` values that the UDP encoder
accepts (mirrored from AWSIM's
`LidarUdpPublisher.cs::SupportedLidarsAndReturnModes`):

| Model | Accepted return_mode values |
|---|---|
| VelodyneVLP16 / VLP32C / VLS128 | `strongest`, `last`, `last_strongest` |
| HesaiPandar40P | `strongest`, `last`, `last_strongest` |
| HesaiPandarXT32 | `strongest`, `last`, `last_strongest` |
| HesaiPandarQT | `first`, `last`, `first_last` |
| HesaiQT128C2X | `first`, `second`, `strongest`, `last`, `last_strongest`, `first_last`, `first_strongest`, `strongest_second_strongest`, `first_second` |
| HesaiPandar128E4X | `first`, `strongest`, `last`, `last_strongest`, `first_last`, `first_strongest` |

CARLA's default `return_mode = "first"` is **rejected by Velodyne,
Pandar40P, and PandarXT32**. For those models you must set an accepted
mode explicitly, for example with
`bp.set_attribute("return_mode", "strongest")`. Without it the UDP node
fails the whitelist check, logs a warning, and stays inactive while the
rest of the LiDAR continues to operate normally.

## Blueprint attribute reference

| Attribute | Type | Default | Description |
|---|---|---|---|
| `rgl_lidar_model_name` | string | `""` | Model name from the table above |
| `horizontal_start_angle` | float | `0.0` | Sweep start angle (deg). For Hesai models, `apply_preset` now defaults to `-90.0` (Hesai ROS driver convention) instead of `0.0`. **Changed default:** this is a behavior change — existing code relying on `0.0` must pass `hesai_ros_driver_compat=False` to restore the old behavior. |
| `rgl_udp_enabled` | bool | `false` | UDP publish toggle |
| `rgl_udp_source_ip` | string | `"0.0.0.0"` | Source IP |
| `rgl_udp_dest_ip` | string | `""` | Destination IP (empty disables UDP) |
| `rgl_udp_dest_port` | int | `2368` | Destination port |
| `rgl_udp_hesai_enable_udp_sequence` | bool | `false` | Add UDP sequence-number field |
| `rgl_udp_hesai_blockage_detection` | bool | `false` | Up-close blockage detection (HesaiQT128C2X only) |
| `rgl_udp_hesai_pandar_driver_compat` | bool | `false` | Pandar driver compatibility (HesaiPandarQT only) |
| `rgl_scan_timing` | string | `"firing"` | How a scan's points are timed: `firing` or `velocity_distortion` (see below) |

UDP publishing activates only when `rgl_udp_enabled = true` **and**
`rgl_udp_dest_ip` is non-empty. With either condition unsatisfied UDP
stays off regardless of the other attributes.

### Scan timing

`rgl_scan_timing` selects how sensor motion is modeled and how the scan's
UDP packet times relate to the simulation frame that triggers capture.

| Value | Motion and packet time |
|---|---|
| `firing` (default) | Captures all rays at frame time T. Packet times advance over one rotation starting at T. Existing behavior is unchanged. |
| `velocity_distortion` | Uses RGL velocity distortion to approximate sensor motion over the preceding rotation. Packet times retain firing intervals, starting at max(0, T − P), where P is the rotation period. For T ≥ P, the nominal azimuth-step window covers the preceding rotation. |

With `firing`, the points do not move during the rotation, but their times
do. A driver that compensates motion per point, such as Autoware's distortion
corrector, then shifts them by the vehicle speed times their time offset: up to
one rotation of travel (about 1 m at 10 m/s and 10 Hz). Packet times also run
up to one rotation ahead of the simulation time when they arrive.

With `velocity_distortion`, rays use negative time offsets from −P toward
zero, in azimuth order. Linear velocity and RPY angular rates are estimated
from successive capture poses in the current sensor frame. This approximates
only the sensor's own motion; moving objects remain at their current frame
poses. During the first rotation after simulation start (T < P), distortion
is disabled and the packet epoch stays at zero, even if capture history is
already available. The first capture without pose history also uses zero
velocity, but still shifts the packet epoch to max(0, T − P).

A velocity-distortion LiDAR ignores the vehicle it is attached to, including
its registered body, wheels, attached components and child actors. All
velocity-distortion LiDARs on that vehicle share an ignore id. LiDARs on
other vehicles can still see it. A `firing` LiDAR keeps its default id,
including when it shares a vehicle with a velocity-distortion LiDAR.
The existing `rgl_self_cutout_radius` behavior is retained.

Everything attached to the vehicle, directly or through other actors, or
owned by it is ignored as well: a trailer, a towed vehicle or a prop attached
to the ego vehicle is invisible to that vehicle's velocity-distortion LiDARs
(LiDARs on other vehicles and `firing` LiDARs still see it). The search from
an object's actor through its attachment parents and owners looks at no more
than 16 actors; an object whose vehicle lies beyond that is not ignored and
stays visible.

Ignoring the vehicle also removes its occlusion: rays can reach surfaces
behind the body. If Autoware's self crop removes the real sensor's body
returns, the remaining difference is visibility into that body shadow.
This does not reproduce physical body occlusion; body-shadow masking is
future work.

A ray from a sensor inside or on a vehicle crosses many faces of its body.
`velocity_distortion` therefore needs an RGL library that exports
`rgl_get_ignored_faces_limit`, one that lets a beam sample pass 64 or more
faces of ignored entities. RobotecAI's RGL (the `develop` branch that
`RglSetup.sh` clones by default, 0.21.0 at the time of writing) does not have
it yet: it continues a ray behind an ignored face with nested traces, which
can exceed the ray tracing pipeline's trace depth when the ray crosses a whole
vehicle (in our tests, an illegal memory access). With such a library,
`velocity_distortion` session creation is rejected
(`reason=rgl_ignored_faces_limit_insufficient ignored_faces_limit=missing`,
see below), and `firing` LiDARs work as before. `RglSetup.sh prepare` can
point at an RGL that has the change with `--rgl-repo` and `--rgl-branch`. Such a library passes up
to the limit it reports (256 at the time of writing); a beam sample that
crosses more ignored faces than that is a non-hit.

Non-increasing capture times or gaps greater than 2.5 rotations use zero
velocity. Translation speed above 70 m/s or angular-rate magnitude above
3 rad/s is treated as a teleport and uses zero velocity. A pose with scale,
shear or reflection is invalid for velocity estimation and also uses zero
velocity; rotation columns must have unit length within 1e-3, be mutually
orthogonal within 1e-3, and have a positive determinant.

For UDP output, this mode supports **HesaiPandar128E4X in standard mode**
and **HesaiQT128C2X**, with single or dual returns. The rotation period divided
by the actual number of CARLA azimuth steps must match the model's interval
within **1%**:

| Model | Ray entries per CARLA step | Model step interval | Example at 10 Hz |
|---|---:|---:|---:|
| HesaiPandar128E4X | 128 | 55.556 µs | 1800 steps |
| HesaiQT128C2X | 192 | 222.220 µs | 450 steps |

Pandar's standard block interval is 2 × 27778 ns. QT128C2X uses two
96-firing sequences per CARLA step; each sequence advances by 111110 ns.
The QT128C2X preset (864000 points/s, 192 ray entries, 10 Hz) with
`sensor_tick=0.1` meets this constraint. At 20 Hz, 900 Pandar steps are
accepted. At 10 Hz, 900 or 3600 Pandar steps are rejected.

Ray offsets distribute the configured rotation period across its azimuth
steps, keeping them in [−P, 0). The preset QT128C2X ray interval is
222.222222 µs, differing from its packet interval by about 1 µs per rotation.
All entries in a CARLA step share one time. Sequence, channel and return
firing delays within a step are omitted; exact per-point timing agreement
with a decoder is not guaranteed. The QT128C2X decoder also assigns
single-return block offsets in reverse block order.

The sweep must cover 360° per capture (`horizontal_fov=360` and an effective
capture interval of at least one rotation). Other models, the HighRes mode,
partial sweeps and incompatible step timing are rejected at session creation
with an error reason. The velocity-distortion and sensor-ignore APIs must
both be available in the loaded RGL library. With UDP disabled, there is no
model-specific constraint on step count and rotation period, and partial
FoV scans are allowed.

Set **`sensor_tick` to `1/rotation_frequency`** (0.1 at 10 Hz) on a
`velocity_distortion` LiDAR. The capture interval is checked at session
creation and again against the actual first ray pattern. With
`sensor_tick=0` the world's frame delta at that moment is used instead, so
the same configuration can be accepted or rejected depending on when the
sensor is created and on the frame step.

The results are written to stdout, one line each, also in Shipping builds.
Every value is a token without spaces or quotes, so a line can be split on
spaces; descriptions stay in the engine log. `name` is the sensor actor's
name, `sensor` its session handle and `world` the path name of the world
whose scene time was disabled.

```text
rgl_scan_timing_rejected sensor=... name=... model=... reason=... [ignored_faces_limit=<n|missing|error>]
rgl_scan_timing sensor=... name=... mode=velocity_distortion model=... steps=... step_us=... ignore=<id|none>
rgl_scan_timing_ignore sensor=... name=... ignore=<id>
rgl_scan_timing_warning sensor=... name=... reason=no_parent_vehicle
rgl_scan_timing_stopped sensor=... name=... mode=<firing|velocity_distortion> reason=...
rgl_scene_time_disabled world=... reason=restore_failed
```

A rejection is written when the session is created, or once at the first
capture whose actual ray pattern is incompatible (the pattern is generated
and checked again on later captures until one is compatible). For
`reason=rgl_ignored_faces_limit_insufficient`, `ignored_faces_limit` is
`missing` when the library does not export `rgl_get_ignored_faces_limit`,
`error` when the call fails, and otherwise the limit it reported.

The accepted line follows the first capture whose graph run succeeds: the
vehicle to ignore is known only once the sensor is attached, and RGL checks
its node inputs when the graph runs. `step_us` reports the model interval
for UDP. `ignore=none` means that no vehicle was found up the sensor's
attachment chain: the LiDAR then sees its own vehicle, and the warning line
is written once. If a vehicle is found later, `rgl_scan_timing_ignore` is
written once with its id.

When a capture ends early for one of the following reasons,
`rgl_scan_timing_stopped` is written once per sensor and reason; a different
reason is written again: `scene_time_disabled`, `scene_time_restore_failed`
(see below), `vehicle_ignore_refresh_failed`, `raytrace_id_failed`,
`sensor_id_exhausted`, `ray_pattern_failed`, `ray_time_offsets_failed`,
`velocity_failed`, `scan_time_failed` or `graph_run_failed`. Except for the
two scene-time reasons, the capture is tried again at the next tick, and a
recovery is not written. A failure to update the vehicle's ignore ids only
skips `velocity_distortion` captures. A `velocity_distortion` sensor with no
accepted, rejected or stopped line has not completed a capture yet; a few
other early returns (for example an empty ray pattern) are not reported.

Direct RGL ROS 2 PointCloud2 output cannot be combined with
`velocity_distortion`, regardless of the point format or whether UDP is
also enabled. Leave **`rgl_lidar_topic_name` empty**; with a non-empty
value, session creation is rejected
(`reason=direct_ros2_output_unsupported`). This avoids exporting the
internal negative ray offsets as point timestamps.

Per-point motion compensation, such as Autoware's, can approximately restore
static shapes when its motion estimate and firing-time model agree with the
simulated sensor. This is not an exact firing simulation: channel microsecond
delays, within-scan acceleration and moving-object distortion are not modeled.
The 1% step-time tolerance also permits a small residual mismatch. Decoded
channel times can slightly exceed T; T is not a strict upper timestamp bound.

Each sensor temporarily changes RGL's shared scene time around its graph run.
When available, the previous frame time is set before both the scan epoch and
the restored frame time. The first frame only sets the scan epoch, then the
frame time. Restoration retries the complete sequence up to four times. If
it still fails, capture history is not updated and further captures using
that shared scene stop until the scene is recreated; an error is logged,
`rgl_scene_time_disabled` is written once for the scene and
`rgl_scan_timing_stopped` once for each sensor that stops.
These time changes synchronize graph execution, so multiple sensors can be
processed serially. The performance impact has not been measured. Sharing
this scene with object-velocity or radar consumers needs integration testing.

In Python, `apply_preset(..., scan_timing="velocity_distortion")` sets the attribute.

## Smoke test

```bash
# Phase 1 verification
cd /path/to/CarlaUE5/PythonAPI/rgl/tests
python3 test_udp_raw_packets.py
```

Exit codes:

- `0`: all tests passed
- `1`: at least one test failed
- `2`: UDP extension missing from the loaded `libRobotecGPULidar.so`
  (the run is skipped, not failed)

When the UDP-enabled `.so` is loaded the script runs three tests: an
all-eight-models smoke test, a VLP16 deep decode (Velodyne Legacy
Format packet structure), and a HesaiPandar40P horizontal-start-angle
end-to-end check.

## tier4/nebula integration (Phase 2)

End-to-end verification against the production ROS 2 driver
`tier4/nebula` is implemented as Phase 2. See
[`phase2_nebula_e2e.md`](phase2_nebula_e2e.md) for prerequisites,
test invocation, per-model thresholds, return-mode notes, and the
HesaiPandarQT-specific UDP flag requirements.

## Troubleshooting

| Symptom | What to check |
|---|---|
| `UDP publishing requested but RGL_EXTENSION_UDP not present` warning | Confirm `libRobotecGPULidar.so` was built with the UDP extension (`nm -D` and grep for `rgl_node_points_udp_publish`) |
| No packets received | If `rgl_udp_dest_ip` is not `127.0.0.1`, verify firewall and NIC binding on the receiver side |
| Hesai azimuth looks shifted | For Hesai models the -90 deg sweep start is applied by default. If you opted out with `hesai_ros_driver_compat=False`, remove that override. Also confirm `ensure_hesai_pandar_driver_compat` is inside the `udp_publish` dict when applicable. |
| `Return mode '...' not supported by model '...'` warning | Use the return-mode whitelist above and set the attribute with `bp.set_attribute("return_mode", ...)` |
| `UDP requires HorizontalFov=360.0 but is ...` warning | Set the LiDAR `HorizontalFov` to 360 (UDP raw packets assume a full rotation) |
