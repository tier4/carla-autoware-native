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
    // __FILE__ is Source/CarlaRGL/CarlaRGLModule.cpp → go up to plugin root.
    FString PluginDir = FPaths::GetPath(FPaths::GetPath(FPaths::GetPath(FString(__FILE__))));
    FString BinDir = FPaths::Combine(PluginDir, TEXT("Binaries"), TEXT("Linux"));

    // 1. Load and init RclcppBridge FIRST (before RGL and before CARLA's DDS).
    //    This calls rclcpp::init() so RGL's Ros2InitGuard sees rclcpp::ok()==true.
    FString BridgePath = FPaths::Combine(BinDir, TEXT("libRclcppBridge.so"));
    if (RclcppBridge::Load(TCHAR_TO_UTF8(*BridgePath)))
    {
        if (RclcppBridge::Init())
        {
            UE_LOG(LogCarlaRGL, Log, TEXT("CarlaRGL: RclcppBridge initialized (rclcpp ready)."));
        }
        else
        {
            UE_LOG(LogCarlaRGL, Warning,
                TEXT("CarlaRGL: RclcppBridge init failed. RGL ROS2 publish may not work."));
        }
    }
    else
    {
        UE_LOG(LogCarlaRGL, Log,
            TEXT("CarlaRGL: libRclcppBridge.so not found at %s (optional)."), *BridgePath);
    }

    // 2. Load libRobotecGPULidar.so
    FString LibPath = FPaths::Combine(BinDir, TEXT("libRobotecGPULidar.so"));
    if (!RGLDynLoader::Load(TCHAR_TO_UTF8(*LibPath)))
    {
        UE_LOG(LogCarlaRGL, Warning,
            TEXT("CarlaRGL: Failed to load libRobotecGPULidar.so from %s. "
                 "RGL LiDAR will not function."),
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
    RclcppBridge::Shutdown();
    RclcppBridge::Unload();
}

IMPLEMENT_MODULE(FCarlaRGLModule, CarlaRGL)
