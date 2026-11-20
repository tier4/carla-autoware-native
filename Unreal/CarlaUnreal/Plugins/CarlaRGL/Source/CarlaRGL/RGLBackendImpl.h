// Copyright (c) 2026 RGL Integration for CARLA.
// Implementation of IRGLBackend that wraps all rgl_* API calls.
// This file lives in the CarlaRGL module so that RGL library linkage
// is fully isolated from the Carla module.

#pragma once

#include "Carla/RGL/IRGLBackend.h"

#ifdef WITH_RGL

#include <util/disable-ue4-macros.h>
#include <rgl/api/core.h>
#include <rgl/api/extensions/ros2.h>
#include <util/enable-ue4-macros.h>

/// Point cloud format definitions matching AWSIM's PointCloudFormatLibrary.
/// See: AWSIM/Assets/Awsim/Scripts/Entity/Sensor/Lidar/PointCloudFormatLibrary.cs
struct FRGLPointCloudFormat
{
    const char* Name;
    const rgl_field_t* Fields;
    int32 FieldCount;
};

/// Session state holding all RGL graph nodes and buffers for one LiDAR sensor.
/// Replaces ARGLLidar::FRGLLidarImpl from the monolithic implementation.
struct FRGLSession
{
    // ---- Graph nodes ----
    rgl_node_t UseRaysNode = nullptr;
    rgl_node_t SetRangeNode = nullptr;
    rgl_node_t SetRingIdsNode = nullptr;
    rgl_node_t RaysTransformNode = nullptr;
    rgl_node_t RaytraceNode = nullptr;
    rgl_node_t CompactNode = nullptr;
    rgl_node_t ToSensorNode = nullptr;
    rgl_node_t YieldNode = nullptr;
    rgl_node_t Ue5ToRos2Node = nullptr;
    rgl_node_t FormatNode = nullptr;
    rgl_node_t Ros2PublishNode = nullptr;
    rgl_node_t UdpPublishNode = nullptr;  // UDP raw packet publish (optional)
    rgl_node_t AngularNoiseRayNode = nullptr;
    rgl_node_t AngularNoiseHitpointNode = nullptr;
    rgl_node_t DistanceNoiseNode = nullptr;

    // ---- Ray pattern buffers ----
    TArray<rgl_mat3x4f> RayTransforms;
    TArray<int32> RingIds;
    TArray<rgl_vec2f> RayRanges;  // per-ray range (min,max) in meters; empty = uniform
    TArray<int8> RayMask;  // per-ray mask (1=enabled, 0=masked); empty = no mask

    // ---- Cache state ----
    bool bRayPatternCached = false;
    int32 CachedTotalRays = 0;

    // ---- Configuration (copied from CreateSession) ----
    FRGLSessionConfig Config;

    // ---- World reference (for scene manager updates during Tick) ----
    TWeakObjectPtr<UWorld> World;

    // ---- ROS2 state ----
    bool bRos2Active = false;
    bool bUdpActive = false;

    // ---- Self cutout (rgl_self_cutout_radius; see IRGLBackend::SetSelfCutout) ----
    TWeakObjectPtr<AActor> SelfCutoutVehicle;
    float SelfCutoutRadiusCm = 0.0f;           // <= 0 = off
    float SelfCutoutMinElevationDeg = 0.0f;    // vertical field of view of the ray pattern
    float SelfCutoutMaxElevationDeg = 0.0f;

    // ---- The sensor's carrier (see IRGLBackend::SetAuditParent; the audit and the scene sync use it) ----
    TWeakObjectPtr<AActor> AuditParent;        // root of the sensor's attach chain; null = not attached

    // ---- Debug hit dump (rgl.Debug.HitDumpDir) ----
    uint64 HitDumpScans = 0;                   // raytraces seen while the dump is on
    bool bHitDumpReported = false;             // first written file announced
    bool bHitDumpFailureReported = false;      // first failure announced

    bool AllNodesValid() const
    {
        return UseRaysNode && SetRangeNode && RaysTransformNode && RaytraceNode &&
               CompactNode && ToSensorNode && YieldNode;
        // Optional nodes (not validated here):
        //   FormatNode, Ros2PublishNode (only when ROS2 topic is set)
        //   UdpPublishNode               (only when UDP is enabled and the
        //                                 RGL UDP extension is present)
    }
};

/// Concrete IRGLBackend implementation using the RGL library.
class FRGLBackendImpl : public IRGLBackend
{
public:
    virtual ~FRGLBackendImpl() = default;

    virtual FRGLSessionHandle CreateSession(const FRGLSessionConfig& Config, UWorld* World) override;
    virtual void DestroySession(FRGLSessionHandle Handle) override;
    virtual void SetSimulationTime(double InSimulationTime) override { SimulationTime = InSimulationTime; }
    virtual FRGLTickResult Tick(
        FRGLSessionHandle Handle,
        const FTransform& SensorWorldTransform,
        float EffectiveDelta,
        float& InOutHorizontalAngle,
        bool bNeedResults,
        bool bNeedWorldPoints) override;
    virtual void SetSelfCutout(FRGLSessionHandle Handle, AActor* Vehicle, float RadiusCm) override;
    virtual void SetAuditParent(FRGLSessionHandle Handle, AActor* RootParent) override;

private:
    double SimulationTime = -1.0;  // Elapsed since episode start; -1 = not set

    /// Generate ray pattern transforms for the given session.
    /// Returns the total number of rays generated.
    int32 GenerateRayPattern(FRGLSession* Session, float DeltaSeconds, float& InOutHorizontalAngle);

    /// Collect results from the RGL graph into an FRGLTickResult.
    void CollectResults(FRGLSession* Session, const FTransform& SensorWorldTransform,
                        bool bNeedWorldPoints, FRGLTickResult& OutResult);
};

#else // !WITH_RGL

/// Stub implementation when RGL is not available.
class FRGLBackendImpl : public IRGLBackend
{
public:
    virtual ~FRGLBackendImpl() = default;

    virtual FRGLSessionHandle CreateSession(const FRGLSessionConfig& /*Config*/, UWorld* /*World*/) override
    {
        return nullptr;
    }

    virtual void DestroySession(FRGLSessionHandle /*Handle*/) override {}
    virtual void SetSimulationTime(double /*InSimulationTime*/) override {}

    virtual FRGLTickResult Tick(
        FRGLSessionHandle /*Handle*/,
        const FTransform& /*SensorWorldTransform*/,
        float /*EffectiveDelta*/,
        float& /*InOutHorizontalAngle*/,
        bool /*bNeedResults*/,
        bool /*bNeedWorldPoints*/) override
    {
        return FRGLTickResult();
    }
};

#endif // WITH_RGL
