// Copyright (c) 2026 RGL Integration for CARLA.
// GPU-accelerated LiDAR sensor using RobotecGPULidar (RGL).
//
// This sensor replaces the CPU-based ARayCastLidar with GPU ray tracing
// via the RGL library, while maintaining the same CARLA data pipeline
// (LidarData -> LidarSerializer -> network stream).
//
// Key differences from ARayCastLidar:
//   - Inherits ASensor directly (not ARayCastSemanticLidar)
//   - Ray casting is done on GPU via RGL's OptiX backend
//   - Scene geometry is managed by FRGLSceneManager
//   - Same Python API: sensor.lidar.rgl
//
// NOTE: The UCLASS declaration must NOT be inside #ifdef blocks (UHT limitation).
// All RGL-specific logic is delegated to the IRGLBackend interface,
// which is implemented by the CarlaRGL module.

#pragma once

#include "Carla/Sensor/Sensor.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/RGL/IRGLBackend.h"

#include <util/disable-ue4-macros.h>
#include <carla/sensor/data/LidarData.h>
#include <util/enable-ue4-macros.h>

#include "RGLLidar.generated.h"

/// GPU-accelerated LiDAR sensor using RobotecGPULidar.
/// Registered as sensor.lidar.rgl in the CARLA sensor registry.
/// When no RGL backend is registered (CarlaRGL module not loaded), the sensor produces no output.
UCLASS()
class CARLA_API ARGLLidar : public ASensor
{
    GENERATED_BODY()

public:
    ARGLLidar(const FObjectInitializer& ObjectInitializer);
    ~ARGLLidar();

    /// Returns the sensor definition for the factory/registry.
    static FActorDefinition GetSensorDefinition();

    /// Configure sensor from actor description (called at spawn time).
    void Set(const FActorDescription& Description) override;

    /// Main simulation entry point, called each physics tick.
    void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds) override;

    void BeginPlay() override;

protected:
    void EndPlay(EEndPlayReason::Type EndPlayReason) override;

private:
    /// Build the RGL computation graph for LiDAR simulation.
    void CreateRGLGraph();

    /// Destroy the RGL computation graph.
    void DestroyRGLGraph();

    /// Simulate LiDAR for the current tick: delegates to IRGLBackend.
    void SimulateLidar(float DeltaSeconds);

    // ---- Configuration ----

    /// LiDAR parameters (channels, range, FOV, etc.) - same struct as ARayCastLidar.
    FLidarDescription Description;

    // ---- RGL session (managed by the CarlaRGL backend) ----

    /// Opaque handle returned by IRGLBackend::CreateSession().
    FRGLSessionHandle RGLSessionHandle = nullptr;

    // ---- Simulation State ----

    /// Current horizontal scan angle (degrees), advances each tick.
    float CurrentHorizontalAngle = 0.0f;

    /// Output data in CARLA format.
    carla::sensor::data::LidarData LidarMeasurement;

    /// Whether the RGL graph has been created.
    bool bGraphCreated = false;

    /// Draw point cloud in UE5 viewport (controlled via "rgl_lidar_show_points" attribute).
    UPROPERTY()
    bool bRglShowLidarPoints = false;

    /// Fraction of points to draw (0.0-1.0), controlled via "rgl_lidar_draw_point_rate" attribute.
    float RglLidarDrawPointRate = 1.0f;

    /// How long each drawn point persists (seconds). Controlled via "rgl_lidar_draw_life_time".
    /// 0 = single frame only, >0 = persists for that duration. Default 0.2s.
    float RglLidarDrawLifeTime = 0.2f;

    /// If true, RGL handles ROS2 publishing; skip CARLA's built-in ROS2 publish.
    bool bRglRos2Active = false;

    /// Draw LiDAR points in the UE5 viewport. Called from PostPhysTick (game thread).
    void RglDrawLidarPoints(UWorld* World);

    /// Buffered world-space points for viewport drawing (populated by CollectResults).
    TArray<FVector> RglLidarDrawWorldPoints;
    TArray<float> RglLidarDrawIntensities;
};
