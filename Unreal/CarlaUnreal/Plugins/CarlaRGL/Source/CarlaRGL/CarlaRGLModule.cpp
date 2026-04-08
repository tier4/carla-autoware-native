#include "CarlaRGLModule.h"
#include "RGLDynLoader.h"
#include "Carla/RGL/IRGLBackend.h"
#include "Misc/Paths.h"

DEFINE_LOG_CATEGORY(LogCarlaRGL);

#ifdef WITH_RGL
#include "RGLBackendImpl.h"
#endif

void FCarlaRGLModule::StartupModule()
{
#ifdef WITH_RGL
    // Find libRobotecGPULidar.so in this plugin's Binaries.
    // __FILE__ is Source/CarlaRGL/CarlaRGLModule.cpp → go up to plugin root.
    FString PluginDir = FPaths::GetPath(FPaths::GetPath(FPaths::GetPath(FString(__FILE__))));
    FString LibPath = FPaths::Combine(PluginDir, TEXT("Binaries"), TEXT("Linux"), TEXT("libRobotecGPULidar.so"));

    if (!RGLDynLoader::Load(TCHAR_TO_UTF8(*LibPath)))
    {
        UE_LOG(LogCarlaRGL, Warning,
            TEXT("CarlaRGL: Failed to load libRobotecGPULidar.so from %s. "
                 "RGL LiDAR will not function. "
                 "Set LD_LIBRARY_PATH to include ROS2 standalone libraries if needed."),
            *LibPath);
        return;
    }

    UE_LOG(LogCarlaRGL, Log, TEXT("CarlaRGL: libRobotecGPULidar.so loaded successfully."));

    FRGLBackendRegistry::Register(new FRGLBackendImpl());
    UE_LOG(LogCarlaRGL, Log, TEXT("CarlaRGL: RGL backend registered."));
#else
    UE_LOG(LogCarlaRGL, Log, TEXT("CarlaRGL: WITH_RGL not defined, no backend registered."));
#endif
}

void FCarlaRGLModule::ShutdownModule()
{
    FRGLBackendRegistry::Unregister();
    RGLDynLoader::Unload();
}

IMPLEMENT_MODULE(FCarlaRGLModule, CarlaRGL)
