#include "CarlaRGLModule.h"
#include "RGLDynLoader.h"
#include "Carla/RGL/IRGLBackend.h"
#include "Misc/Paths.h"

DEFINE_LOG_CATEGORY(LogCarlaRGL);

#ifdef WITH_RGL
#include "RGLBackendImpl.h"
#include "RGLSceneManager.h"

#include <util/ue-header-guard-begin.h>
#include "Engine/World.h"
#include <util/ue-header-guard-end.h>
#endif

static FString GetCarlaRGLBinDir()
{
    // In packaged builds: <ProjectDir>/Plugins/CarlaRGL/Binaries/Linux
    // FPaths::ProjectPluginsDir() resolves correctly in both editor and package.
    FString PluginBinDir = FPaths::Combine(
        FPaths::ProjectPluginsDir(), TEXT("CarlaRGL"), TEXT("Binaries"), TEXT("Linux"));
    if (FPaths::DirectoryExists(PluginBinDir))
    {
        return PluginBinDir;
    }
    // Fallback: __FILE__ based (editor with source tree, resolved at compile time)
    FString PluginDir = FPaths::GetPath(FPaths::GetPath(FPaths::GetPath(FString(__FILE__))));
    return FPaths::Combine(PluginDir, TEXT("Binaries"), TEXT("Linux"));
}

void FCarlaRGLModule::StartupModule()
{
#ifdef WITH_RGL
    FString BinDir = GetCarlaRGLBinDir();

    // 1. Load and init RclcppBridge FIRST (before RGL and before CARLA's DDS).
    //    This calls rclcpp::init() so RGL's Ros2InitGuard sees rclcpp::ok()==true.
    //    The bridge only serves RGL's own ROS 2 publisher, so it is used only when RGL's ROS 2
    //    runtime (ros2_standalone, copied next to RGL by Unreal/CMakeLists.txt) is present, and
    //    never in commandlets such as the cook. Without that runtime, rclcpp would resolve to the
    //    sourced system ROS 2, whose middleware then runs against the Fast-DDS that CARLA already
    //    bundles (same soname, different build) and crashes in rmw create_participant.
    FString BridgePath = FPaths::Combine(BinDir, TEXT("libRclcppBridge.so"));
    const bool bHasRglRos2Runtime = FPaths::FileExists(FPaths::Combine(BinDir, TEXT("librclcpp.so")));
    if (IsRunningCommandlet() || !bHasRglRos2Runtime)
    {
        UE_LOG(LogCarlaRGL, Log,
            TEXT("CarlaRGL: RclcppBridge skipped (commandlet=%d, rgl_ros2_runtime=%d)."),
            IsRunningCommandlet() ? 1 : 0, bHasRglRos2Runtime ? 1 : 0);
    }
    else if (RclcppBridge::Load(TCHAR_TO_UTF8(*BridgePath)))
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

    // Deterministic per-world teardown (episode reload / map change): restores skeletal tick policies, frees entities.
    WorldCleanupHandle = FWorldDelegates::OnWorldCleanup.AddStatic(
        [](UWorld* World, bool /*bSessionEnded*/, bool /*bCleanupResources*/) { FRGLSceneManager::DestroyInstance(World); });
#else
    UE_LOG(LogCarlaRGL, Log, TEXT("CarlaRGL: WITH_RGL not defined, no backend registered."));
#endif
}

void FCarlaRGLModule::ShutdownModule()
{
#ifdef WITH_RGL
    if (WorldCleanupHandle.IsValid())
    {
        FWorldDelegates::OnWorldCleanup.Remove(WorldCleanupHandle);
        WorldCleanupHandle.Reset();
    }
#endif
    FRGLBackendRegistry::Unregister();
#ifdef WITH_RGL
    FRGLSceneManager::DestroyAllInstances();   // before Unload: destructors call rgl_* through the loader
#endif
    RGLDynLoader::Unload();
    RclcppBridge::Shutdown();
    RclcppBridge::Unload();
}

IMPLEMENT_MODULE(FCarlaRGLModule, CarlaRGL)
