/*
 * Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
 * de Barcelona (UAB).
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */

/*
 * C interface between the simulator's high-precision IMU and an external
 * "raw sink" shared library (set with the sensor's raw_sink_library
 * attribute). The simulator owns this contract; a sink library implements
 * the four functions below. Only C types cross the boundary, so the sink may
 * be built with any toolchain and C++ standard library.
 *
 * The simulator loads the library with RTLD_GLOBAL, so its exported names
 * must be unique to the sink type (keep the carla_imu_raw_sink_ prefix and
 * export nothing else). Give raw_sink_library as an absolute path; the server
 * runs the library's code, so only point it at trusted files.
 *
 * The simulator and every sink library each carry a copy of this file. Keep
 * the copies identical and bump CARLA_IMU_RAW_SINK_ABI_VERSION on any change.
 */

#ifndef CARLA_IMU_RAW_SINK_H
#define CARLA_IMU_RAW_SINK_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CARLA_IMU_RAW_SINK_ABI_VERSION 1

/* One IMU sample in the sensor's REP-103 frame (x forward, y left, z up). */
typedef struct carla_imu_raw_sample {
  double sim_time_s;     /* simulation time, increasing */
  double gyro_radps[3];  /* angular rate [rad/s] */
  double accel_mps2[3];  /* specific force incl. gravity [m/s^2] */
} carla_imu_raw_sample;

#ifdef __cplusplus
static_assert(sizeof(carla_imu_raw_sample) == 56, "carla_imu_raw_sample layout changed: bump the ABI version");
#else
_Static_assert(sizeof(carla_imu_raw_sample) == 56, "carla_imu_raw_sample layout changed: bump the ABI version");
#endif

enum {
  CARLA_IMU_RAW_SINK_LOG_INFO = 0,
  CARLA_IMU_RAW_SINK_LOG_WARNING = 1,
  CARLA_IMU_RAW_SINK_LOG_ERROR = 2
};

/* May be called from a sink-owned thread, until destroy returns. */
typedef void (*carla_imu_raw_sink_log_fn)(void *user, int level, const char *message);

/* Returns CARLA_IMU_RAW_SINK_ABI_VERSION of the library. */
typedef int (*carla_imu_raw_sink_abi_version_fn)(void);

/* Creates a sink from a sink-specific configuration string, which needs to
 * stay valid only during the call. Returns NULL and writes a NUL-terminated
 * message into `error` (when not NULL) on failure. */
typedef void *(*carla_imu_raw_sink_create_fn)(
    const char *config, carla_imu_raw_sink_log_fn log, void *log_user,
    char *error, size_t error_size);

/* Queues one sample; called from the simulator's game thread, never blocks. */
typedef void (*carla_imu_raw_sink_push_fn)(void *sink, const carla_imu_raw_sample *sample);

/* Stops the sink and frees it. The log callback is not called after return. */
typedef void (*carla_imu_raw_sink_destroy_fn)(void *sink);

int carla_imu_raw_sink_abi_version(void);
void *carla_imu_raw_sink_create(
    const char *config, carla_imu_raw_sink_log_fn log, void *log_user,
    char *error, size_t error_size);
void carla_imu_raw_sink_push(void *sink, const carla_imu_raw_sample *sample);
void carla_imu_raw_sink_destroy(void *sink);

#ifdef __cplusplus
}
#endif

#endif /* CARLA_IMU_RAW_SINK_H */
