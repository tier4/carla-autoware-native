#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize rclcpp. Returns true on success. Safe to call multiple times. */
bool rclcpp_bridge_init(void);

/** Check if rclcpp is initialized. */
bool rclcpp_bridge_ok(void);

/** Shutdown rclcpp. Safe to call if not initialized. */
void rclcpp_bridge_shutdown(void);

#ifdef __cplusplus
}
#endif
