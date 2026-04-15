#pragma once
#include "CoreMinimal.h"
#include "Carla/Sensor/LidarDescription.h"

using FRGLSessionHandle = void*;

struct FRGLSessionConfig
{
    FLidarDescription LidarDesc;
    FString Ros2Topic;
    FString Ros2FrameId;
    FString Ros2Reliability;
    FString Ros2Durability;
    FString Ros2History;
    int32 Ros2HistoryDepth = 5;
    FString Ros2Format;
};

struct FRGLTickResult
{
    TArray<FVector> HitPoints;       // sensor-local, meters (RGL coords)
    TArray<float> Distances;          // meters
    TArray<float> Intensities;
    int32 TotalHits = 0;
    TArray<FVector> WorldPoints;      // UE coords, cm (for debug draw)
    TArray<float> WorldIntensities;
};

class IRGLBackend
{
public:
    virtual ~IRGLBackend() = default;
    virtual FRGLSessionHandle CreateSession(const FRGLSessionConfig& Config, UWorld* World) = 0;
    virtual void DestroySession(FRGLSessionHandle Handle) = 0;
    /// Set simulation time (elapsed since episode start) for ROS2 timestamp synchronization.
    virtual void SetSimulationTime(double InSimulationTime) = 0;
    virtual FRGLTickResult Tick(
        FRGLSessionHandle Handle,
        const FTransform& SensorWorldTransform,
        float EffectiveDelta,
        float& InOutHorizontalAngle,
        bool bNeedResults,
        bool bNeedWorldPoints) = 0;
};

class CARLA_API FRGLBackendRegistry
{
public:
    static void Register(IRGLBackend* Backend);
    static void Unregister();
    static IRGLBackend* Get();
private:
    static IRGLBackend* Instance;
};
