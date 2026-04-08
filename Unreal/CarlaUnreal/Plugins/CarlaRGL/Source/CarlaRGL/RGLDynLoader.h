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
