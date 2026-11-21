#pragma once

#include <cstdint>

// Dynamic loader for libRobotecGPULidar.so
// Removes link-time dependency so the module can load even when the library is unavailable.
namespace RGLDynLoader
{
    // Try to load libRobotecGPULidar.so from the given path.
    // Returns true if the library was loaded and all function pointers resolved.
    bool Load(const char* LibPath);

    // Unload the library.
    void Unload();

    // Check if the library is loaded.
    bool IsLoaded();

    // True when all three skeleton-animation entry points resolved. Core RGL API
    // (not an extension); the skeletal scene path auto-disables when false.
    bool IsSkeletalApiAvailable();

    // True when ray time offsets and both velocity-distortion entry points resolved.
    bool IsVelocityDistortionApiAvailable();

    // Both per-vehicle visibility entry points must be available.
    bool IsSensorIgnoreApiAvailable();

    // True when the library exports rgl_get_ignored_faces_limit (libraries that loop past ignored
    // faces); libraries that continue with nested traces do not.
    bool HasIgnoredFacesLimit();

    // Faces of ignored entities one beam sample may pass (rgl_get_ignored_faces_limit).
    // False when the library does not export it or the call fails.
    bool GetIgnoredFacesLimit(int32_t& OutLimit);
}

// Dynamic loader for libRclcppBridge.so
// Initializes rclcpp before CARLA's DDS so RGL's Ros2InitGuard reuses the existing context.
namespace RclcppBridge
{
    bool Load(const char* LibPath);
    bool Init();
    void Shutdown();
    void Unload();
    bool IsLoaded();
}
