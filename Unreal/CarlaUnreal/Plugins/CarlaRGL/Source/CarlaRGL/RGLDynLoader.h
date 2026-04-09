#pragma once

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
