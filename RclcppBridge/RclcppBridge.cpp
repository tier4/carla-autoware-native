#include "RclcppBridge.h"
#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <memory>

// Keep the node alive so the DDS domain stays open.
static std::shared_ptr<rclcpp::Node> g_node;

extern "C" bool rclcpp_bridge_init(void)
{
    try {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        // Create a node to force DDS domain creation NOW.
        // This must happen before CARLA's dds_create_participant() calls.
        // RGL's Ros2InitGuard will see rclcpp::ok()==true and reuse this context.
        if (!g_node) {
            g_node = std::make_shared<rclcpp::Node>("rclcpp_bridge");
        }
        fprintf(stdout, "RclcppBridge: rclcpp initialized and DDS domain created.\n");
        return rclcpp::ok();
    } catch (const std::exception& e) {
        fprintf(stderr, "RclcppBridge: init failed: %s\n", e.what());
        return false;
    } catch (...) {
        fprintf(stderr, "RclcppBridge: init failed with unknown error.\n");
        return false;
    }
}

extern "C" bool rclcpp_bridge_ok(void)
{
    return rclcpp::ok();
}

extern "C" void rclcpp_bridge_shutdown(void)
{
    try {
        g_node.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
            fprintf(stdout, "RclcppBridge: rclcpp shutdown.\n");
        }
    } catch (...) {
        // Ignore shutdown errors
    }
}
