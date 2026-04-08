// Copyright (c) 2026 RGL Integration for CARLA.
// Implementation of ARGLLidar: GPU-accelerated LiDAR sensor using RGL.
//
// All RGL-specific logic is delegated to the IRGLBackend interface.
// When no backend is registered (CarlaRGL module not loaded), the sensor
// produces no output.

#include "Carla/Sensor/RGLLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/RGL/IRGLBackend.h"

#include <util/ue-header-guard-begin.h>
#include "Engine/World.h"
#include "Math/UnrealMathUtility.h"
#include "Components/LineBatchComponent.h"
#include <util/ue-header-guard-end.h>

#include <util/disable-ue4-macros.h>
#include <carla/Logging.h>
#include <carla/geom/Math.h>
#include <carla/sensor/data/LidarData.h>
#if defined(WITH_ROS2)
#include <carla/ros2/ROS2.h>
#endif
#include <carla/streaming/detail/Token.h>
#include <util/enable-ue4-macros.h>

#include <cmath>

// ============================================================================
// Constructor / Destructor
// ============================================================================

ARGLLidar::ARGLLidar(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    PrimaryActorTick.bCanEverTick = true;
}

ARGLLidar::~ARGLLidar()
{
    DestroyRGLGraph();
}

// ============================================================================
// Sensor Definition (used by SensorFactory for auto-discovery)
// ============================================================================

FActorDefinition ARGLLidar::GetSensorDefinition()
{
    return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("rgl"));
    // Registered as sensor.lidar.rgl in CARLA Python API
}

// ============================================================================
// Set: configure from actor description
// ============================================================================

void ARGLLidar::Set(const FActorDescription& ActorDescription)
{
    Super::Set(ActorDescription);
    UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, Description);

    // Read optional visualization attributes
    auto ShowAttr = GetAttribute(TEXT("rgl_show_lidar_points"));
    if (ShowAttr.has_value())
    {
        bRglShowLidarPoints = ShowAttr->Value.ToBool();
    }

    auto PointRateAttr = GetAttribute(TEXT("rgl_lidar_draw_point_rate"));
    if (PointRateAttr.has_value())
    {
        RglLidarDrawPointRate = FMath::Clamp(FCString::Atof(*PointRateAttr->Value), 0.0f, 1.0f);
    }

    auto LifeTimeAttr = GetAttribute(TEXT("rgl_lidar_draw_life_time"));
    if (LifeTimeAttr.has_value())
    {
        RglLidarDrawLifeTime = FMath::Max(0.0f, FCString::Atof(*LifeTimeAttr->Value));
    }

    UE_LOG(LogCarla, Log,
        TEXT("ARGLLidar configured: Channels=%d Range=%.0f cm PPS=%d Freq=%.1f Hz FOV=[%.1f,%.1f] HorizFOV=%.1f ShowLidar=%s PointRate=%.2f LifeTime=%.2f"),
        Description.Channels, Description.Range,
        Description.PointsPerSecond, Description.RotationFrequency,
        Description.LowerFovLimit, Description.UpperFovLimit,
        Description.HorizontalFov,
        bRglShowLidarPoints ? TEXT("ON") : TEXT("OFF"),
        RglLidarDrawPointRate, RglLidarDrawLifeTime);
}

// ============================================================================
// BeginPlay / EndPlay
// ============================================================================

void ARGLLidar::BeginPlay()
{
    Super::BeginPlay();

    LidarMeasurement = carla::sensor::data::LidarData(Description.Channels);

    CreateRGLGraph();
    if (!bGraphCreated)
    {
        UE_LOG(LogCarla, Warning,
            TEXT("ARGLLidar: RGL graph not created. Sensor will produce no output."));
    }
}

void ARGLLidar::EndPlay(EEndPlayReason::Type EndPlayReason)
{
    DestroyRGLGraph();
    Super::EndPlay(EndPlayReason);
}

// ============================================================================
// PostPhysTick: main entry point called each physics tick
// ============================================================================

void ARGLLidar::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ARGLLidar::PostPhysTick);

    // Use the sensor tick interval instead of the physics step delta.
    // PostPhysTick receives the physics step (e.g., 0.01s at 100Hz), but
    // the LiDAR should simulate the full sensor_tick period (e.g., 0.1s at 10Hz)
    // to produce a complete rotational scan per publish event.
    const float SensorTickInterval = GetActorTickInterval();
    const float EffectiveDelta = (SensorTickInterval > 0.0f) ? SensorTickInterval : DeltaSeconds;
    SimulateLidar(EffectiveDelta);

    // Draw debug points in UE5 viewport (game thread, safe)
    if (bRglShowLidarPoints)
    {
        RglDrawLidarPoints(World);
    }

    // Send data to clients via the CARLA data pipeline (skip if no listeners)
    if (AreClientsListening())
    {
        auto DataStream = GetDataStream(*this);
        DataStream.SerializeAndSend(*this, LidarMeasurement, DataStream.PopBufferFromPool());
    }

#if defined(WITH_ROS2)
    // Only publish via CARLA's built-in ROS2 if enable_for_ros() was called.
    // This checks the per-sensor flag set by the Python API, not the global --ros2 flag.
    auto ROS2 = carla::ros2::ROS2::GetInstance();
    if (ROS2->IsEnabled())
    {
        auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
        auto *GameInstance = UCarlaStatics::GetGameInstance(GetWorld());
        const bool bSensorEnabledForRos = GameInstance &&
            GameInstance->GetServer().GetStreamingServer().IsEnabledForROS(StreamId);

        if (bSensorEnabledForRos)
        {
            TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send RGL LiDAR");
            auto DataStream = GetDataStream(*this);
            AActor* ParentActor = GetAttachParentActor();
            if (IsValid(ParentActor))
            {
                FTransform LocalTransformRelativeToParent =
                    GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
                ROS2->ProcessDataFromLidar(
                    DataStream.GetSensorType(), StreamId,
                    LocalTransformRelativeToParent,
                    Description.Channels, Description.UpperFovLimit, Description.LowerFovLimit,
                    LidarMeasurement, this);
            }
            else
            {
                ROS2->ProcessDataFromLidar(
                    DataStream.GetSensorType(), StreamId,
                    DataStream.GetSensorTransform(),
                    Description.Channels, Description.UpperFovLimit, Description.LowerFovLimit,
                    LidarMeasurement, this);
            }
        }
    }
#endif
}

// ============================================================================
// RGL backend delegation
// ============================================================================

void ARGLLidar::CreateRGLGraph()
{
    if (bGraphCreated) return;

    IRGLBackend* Backend = FRGLBackendRegistry::Get();
    if (!Backend)
    {
        UE_LOG(LogCarla, Warning,
            TEXT("ARGLLidar: No RGL backend registered (CarlaRGL module not loaded or ENABLE_RGL=OFF)."));
        return;
    }

    FRGLSessionConfig Config;
    Config.LidarDesc = Description;

    auto TopicAttr = GetAttribute(TEXT("rgl_ros2_topic"));
    if (TopicAttr.has_value()) Config.Ros2Topic = TopicAttr->Value;

    auto FrameAttr = GetAttribute(TEXT("rgl_ros2_frame_id"));
    Config.Ros2FrameId = FrameAttr.has_value() ? FrameAttr->Value : TEXT("lidar");

    auto RelAttr = GetAttribute(TEXT("rgl_ros2_reliability"));
    Config.Ros2Reliability = RelAttr.has_value() ? RelAttr->Value : TEXT("best_effort");

    auto DurAttr = GetAttribute(TEXT("rgl_ros2_durability"));
    Config.Ros2Durability = DurAttr.has_value() ? DurAttr->Value : TEXT("volatile");

    auto HistAttr = GetAttribute(TEXT("rgl_ros2_history"));
    Config.Ros2History = HistAttr.has_value() ? HistAttr->Value : TEXT("keep_last");

    auto DepthAttr = GetAttribute(TEXT("rgl_ros2_history_depth"));
    Config.Ros2HistoryDepth = DepthAttr.has_value() ? FCString::Atoi(*DepthAttr->Value) : 5;

    auto FmtAttr = GetAttribute(TEXT("rgl_ros2_format"));
    Config.Ros2Format = FmtAttr.has_value() ? FmtAttr->Value : TEXT("PointXYZIRCAEDT");

    RGLSessionHandle = Backend->CreateSession(Config, GetWorld());
    if (!RGLSessionHandle)
    {
        UE_LOG(LogCarla, Error, TEXT("ARGLLidar: Backend failed to create session."));
        return;
    }

    bGraphCreated = true;
    bRglRos2Active = !Config.Ros2Topic.IsEmpty();
    UE_LOG(LogCarla, Log, TEXT("ARGLLidar: Session created via CarlaRGL backend"));
}

void ARGLLidar::DestroyRGLGraph()
{
    if (!RGLSessionHandle)
    {
        bGraphCreated = false;
        return;
    }

    IRGLBackend* Backend = FRGLBackendRegistry::Get();
    if (Backend)
    {
        Backend->DestroySession(RGLSessionHandle);
    }
    RGLSessionHandle = nullptr;
    bGraphCreated = false;
}

void ARGLLidar::SimulateLidar(float DeltaSeconds)
{
    if (!bGraphCreated || !RGLSessionHandle) return;

    IRGLBackend* Backend = FRGLBackendRegistry::Get();
    if (!Backend) return;

    const bool bNeedResults = AreClientsListening() || bRglShowLidarPoints;

    FRGLTickResult Result = Backend->Tick(
        RGLSessionHandle,
        GetActorTransform(),
        DeltaSeconds,
        CurrentHorizontalAngle,
        bNeedResults,
        bRglShowLidarPoints);

    if (!bNeedResults || Result.TotalHits <= 0) return;

    // Populate LidarMeasurement from backend result
    std::vector<uint32_t> PointsPerChannel(Description.Channels, 0);
    const uint32_t Base = static_cast<uint32_t>(Result.TotalHits) / Description.Channels;
    const uint32_t Rem = static_cast<uint32_t>(Result.TotalHits) % Description.Channels;
    for (uint32_t ch = 0; ch < Description.Channels; ++ch)
        PointsPerChannel[ch] = Base + (ch < Rem ? 1 : 0);

    LidarMeasurement.ResetMemory(PointsPerChannel);
    LidarMeasurement.SetHorizontalAngle(
        carla::geom::Math::ToRadians(CurrentHorizontalAngle));

    for (int32 i = 0; i < Result.TotalHits; ++i)
    {
        float Intensity = Result.Intensities[i];
        if (Description.AtmospAttenRate > 0.0f)
            Intensity *= std::exp(-Description.AtmospAttenRate * Result.Distances[i]);

        carla::sensor::data::LidarDetection Det(
            Result.HitPoints[i].X, Result.HitPoints[i].Y,
            Result.HitPoints[i].Z, Intensity);
        LidarMeasurement.WritePointSync(Det);
    }
    LidarMeasurement.WriteChannelCount(PointsPerChannel);

    // Copy debug draw buffers
    if (bRglShowLidarPoints)
    {
        RglLidarDrawWorldPoints = MoveTemp(Result.WorldPoints);
        RglLidarDrawIntensities = MoveTemp(Result.WorldIntensities);
    }
}

// ============================================================================
// Debug visualization (always compiled, uses LidarMeasurement data)
// ============================================================================

void ARGLLidar::RglDrawLidarPoints(UWorld* World)
{
    static constexpr float POINT_SIZE = 5.0f;
    static constexpr uint8 DEPTH_PRIORITY = SDPG_World;

    if (!World || !World->PersistentLineBatcher)
    {
        return;
    }

    const int32 PointCount = RglLidarDrawWorldPoints.Num();
    if (PointCount <= 0)
    {
        return;
    }

    ULineBatchComponent* LineBatcher = World->PersistentLineBatcher;
    const int32 MaxPoints = FMath::Max(1, FMath::RoundToInt(PointCount * RglLidarDrawPointRate));
    const int32 Step = FMath::Max(1, PointCount / MaxPoints);

    for (int32 i = 0; i < PointCount; i += Step)
    {
        const float Intensity = RglLidarDrawIntensities[i];
        const float R = FMath::Clamp(Intensity, 0.0f, 1.0f);
        const float G = FMath::Clamp(1.0f - Intensity, 0.0f, 1.0f);
        const FLinearColor Color(R, G, 0.0f);

        LineBatcher->DrawPoint(RglLidarDrawWorldPoints[i], Color, POINT_SIZE, DEPTH_PRIORITY, RglLidarDrawLifeTime);
    }
}
