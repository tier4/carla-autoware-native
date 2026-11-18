/*
 * Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
 * de Barcelona (UAB).
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */

/*
 * C interface between the simulator's GNSS sensor and an external "raw sink"
 * shared library (set with the sensor's raw_sink_library attribute). The
 * simulator owns this contract; a sink library implements the functions
 * below. Only C types cross the boundary, so the sink may be built with any
 * toolchain and C++ standard library.
 *
 * A sink is the truth source of a receiver that the library keeps for the
 * whole process: destroying the sink detaches it, and the receiver stays
 * until carla_gnss_raw_sink_shutdown_all. The simulator may unload the
 * library after the last destroy, so a library that keeps running code by
 * then must keep itself loaded.
 *
 * The simulator loads the library with RTLD_GLOBAL, so its exported names
 * must be unique to the sink type (keep the carla_gnss_raw_sink_ prefix and
 * export nothing else). Give raw_sink_library as an absolute path; the server
 * runs the library's code, so only point it at trusted files.
 *
 * The simulator and every sink library each carry a copy of this file. Keep
 * the copies identical and bump CARLA_GNSS_RAW_SINK_ABI_VERSION on any change.
 */

#ifndef CARLA_GNSS_RAW_SINK_H
#define CARLA_GNSS_RAW_SINK_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CARLA_GNSS_RAW_SINK_ABI_VERSION 1

/* The ground truth of the antenna at one simulation step. */
typedef struct carla_gnss_raw_truth {
  double sim_time_s;                /* simulation time, increasing */
  double position_m[3];             /* antenna position in the map frame (ENU, metres inside the MGRS square) */
  double orientation_xyzw[4];       /* vehicle attitude in the map frame (REP-103 body: x forward, y left, z up) */
  double velocity_mps[3];           /* antenna velocity in the map frame */
  double angular_velocity_radps[3]; /* in the map frame */
} carla_gnss_raw_truth;

#ifdef __cplusplus
static_assert(sizeof(carla_gnss_raw_truth) == 112, "carla_gnss_raw_truth layout changed: bump the ABI version");
#else
_Static_assert(sizeof(carla_gnss_raw_truth) == 112, "carla_gnss_raw_truth layout changed: bump the ABI version");
#endif

enum {
  CARLA_GNSS_RAW_SINK_LOG_INFO = 0,
  CARLA_GNSS_RAW_SINK_LOG_WARNING = 1,
  CARLA_GNSS_RAW_SINK_LOG_ERROR = 2
};

/* May be called from any thread (a sink-owned one too), until destroy returns.
 * `message` is valid only during the call. Must not call into the library. */
typedef void (*carla_gnss_raw_sink_log_fn)(void *user, int level, const char *message);

/* Returns CARLA_GNSS_RAW_SINK_ABI_VERSION of the library. */
typedef int (*carla_gnss_raw_sink_abi_version_fn)(void);

/* Creates a sink from a sink-specific configuration string and the MGRS
 * 100 km square that anchors the map frame (e.g. "54SUE"); both need to stay
 * valid only during the call. Sinks whose configuration names the same
 * receiver share it: the newest one is its truth source, and the pushes of
 * earlier ones are ignored. Returns NULL and writes a NUL-terminated message
 * into `error` (when not NULL) on failure. */
typedef void *(*carla_gnss_raw_sink_create_fn)(
    const char *config, const char *mgrs_grid,
    carla_gnss_raw_sink_log_fn log, void *log_user,
    char *error, size_t error_size);

/* Hands over the newest truth; called from the simulator's game thread, never
 * blocks. The sink stamps the wall-clock time itself. */
typedef void (*carla_gnss_raw_sink_push_fn)(void *sink, const carla_gnss_raw_truth *truth);

/* Detaches the sink from its receiver and frees it; the receiver and its
 * connections stay. The log callback is not called after return. */
typedef void (*carla_gnss_raw_sink_destroy_fn)(void *sink);

/* Optional (look it up with dlsym, skip it when absent): stops every receiver
 * of the library. Call it once at process shutdown. */
typedef void (*carla_gnss_raw_sink_shutdown_all_fn)(void);

int carla_gnss_raw_sink_abi_version(void);
void *carla_gnss_raw_sink_create(
    const char *config, const char *mgrs_grid,
    carla_gnss_raw_sink_log_fn log, void *log_user,
    char *error, size_t error_size);
void carla_gnss_raw_sink_push(void *sink, const carla_gnss_raw_truth *truth);
void carla_gnss_raw_sink_destroy(void *sink);
void carla_gnss_raw_sink_shutdown_all(void);

#ifdef __cplusplus
}
#endif

#endif /* CARLA_GNSS_RAW_SINK_H */
