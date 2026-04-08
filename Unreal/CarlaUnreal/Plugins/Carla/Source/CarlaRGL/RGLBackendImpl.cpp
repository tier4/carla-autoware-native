// Copyright (c) 2026 RGL Integration for CARLA.
// Implementation of FRGLBackendImpl: all rgl_* API calls for LiDAR simulation.
// Ported from ARGLLidar::CreateRGLGraph / SimulateLidar / CollectResults.

#ifdef WITH_RGL

#include "RGLBackendImpl.h"
#include "RGLSceneManager.h"
#include "RGLCoordinateUtils.h"

#include <util/disable-ue4-macros.h>
#include <rgl/api/core.h>
#include <rgl/api/extensions/ros2.h>
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "Engine/World.h"
#include "Math/UnrealMathUtility.h"
#include <util/ue-header-guard-end.h>

#include <cmath>

// ============================================================================
// Helper macros
// ============================================================================

// RGL_CHECK is defined in RGLCoordinateUtils.h

// Check RGL call and return a value on failure.
#ifndef RGL_CHECK_RETURN
#define RGL_CHECK_RETURN(call, retval)                                        \
    do {                                                                      \
        rgl_status_t _s = (call);                                             \
        if (_s != RGL_SUCCESS) {                                              \
            const char* _err = nullptr;                                       \
            rgl_get_last_error_string(&_err);                                 \
            UE_LOG(LogTemp, Error,                                            \
                   TEXT("RGL error [RGLBackendImpl] %s: %s"),                 \
                   TEXT(#call),                                               \
                   _err ? *FString(UTF8_TO_TCHAR(_err)) : TEXT("unknown"));   \
            return retval;                                                    \
        }                                                                     \
    } while (0)
#endif

// ============================================================================
// Point cloud format definitions
// ============================================================================

// Minimal: xyz + distance + intensity (20 bytes)
static const rgl_field_t GFormatMinimal[] = {
    RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_DISTANCE_F32, RGL_FIELD_INTENSITY_F32
};

// Pcl24: Autoware 24-byte format
static const rgl_field_t GFormatPcl24[] = {
    RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_PADDING_32,
    RGL_FIELD_INTENSITY_F32, RGL_FIELD_RING_ID_U16, RGL_FIELD_PADDING_16
};

// Pcl48: Autoware 48-byte format
static const rgl_field_t GFormatPcl48[] = {
    RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_PADDING_32,
    RGL_FIELD_INTENSITY_F32, RGL_FIELD_RING_ID_U16, RGL_FIELD_PADDING_16,
    RGL_FIELD_AZIMUTH_F32, RGL_FIELD_DISTANCE_F32,
    RGL_FIELD_RETURN_TYPE_U8, RGL_FIELD_PADDING_8, RGL_FIELD_PADDING_16, RGL_FIELD_PADDING_32,
    RGL_FIELD_TIME_STAMP_F64
};

// PointXYZIRCAEDT: Autoware 32-byte format (matches CARLA's pointcloud_raw_ex)
static const rgl_field_t GFormatXYZIRCAEDT[] = {
    RGL_FIELD_XYZ_VEC3_F32,
    RGL_FIELD_INTENSITY_U8, RGL_FIELD_RETURN_TYPE_U8, RGL_FIELD_RING_ID_U16,
    RGL_FIELD_AZIMUTH_F32, RGL_FIELD_ELEVATION_F32, RGL_FIELD_DISTANCE_F32,
    RGL_FIELD_TIME_STAMP_U32
};

static const FRGLPointCloudFormat GPointCloudFormats[] = {
    {"minimal",         GFormatMinimal,     3},
    {"pcl24",           GFormatPcl24,       5},
    {"pcl48",           GFormatPcl48,      12},
    {"PointXYZIRCAEDT", GFormatXYZIRCAEDT,  8},
};

static const FRGLPointCloudFormat* FindFormat(const char* Name)
{
    for (const auto& Fmt : GPointCloudFormats)
    {
        if (FCStringAnsi::Stricmp(Fmt.Name, Name) == 0)
        {
            return &Fmt;
        }
    }
    return nullptr;
}

// ============================================================================
// CreateSession
// ============================================================================

FRGLSessionHandle FRGLBackendImpl::CreateSession(const FRGLSessionConfig& Config, UWorld* World)
{
    FRGLSession* Session = new FRGLSession();
    Session->Config = Config;

    const FLidarDescription& Desc = Config.LidarDesc;

    // Pre-initialize scene manager (triggers world scan)
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("RGLBackendImpl: No world available during session creation"));
        delete Session;
        return nullptr;
    }

    FRGLSceneManager& SceneMgr = FRGLSceneManager::GetInstance(World);
    SceneMgr.Update(World);
    rgl_scene_t Scene = SceneMgr.GetScene();

    // 1. UseRays node
    rgl_mat3x4f DummyRay = RGLCoord::Identity();
    RGL_CHECK_RETURN(rgl_node_rays_from_mat3x4f(&Session->UseRaysNode, &DummyRay, 1), nullptr);

    // 1.5. Set ray range (min=0, max=Description.Range in meters)
    const float RangeM = Desc.Range * RGLCoord::UE_TO_RGL;
    rgl_vec2f RayRange = {{ 0.0f, RangeM }};
    RGL_CHECK_RETURN(rgl_node_rays_set_range(&Session->SetRangeNode, &RayRange, 1), nullptr);

    // 1.6. Set ring IDs (channel assignment for each ray)
    int32_t DummyRingId = 0;
    RGL_CHECK_RETURN(rgl_node_rays_set_ring_ids(&Session->SetRingIdsNode, &DummyRingId, 1), nullptr);

    // 2. RaysTransform node
    rgl_mat3x4f SensorTf = RGLCoord::Identity();
    RGL_CHECK_RETURN(rgl_node_rays_transform(&Session->RaysTransformNode, &SensorTf), nullptr);

    // 3. Raytrace node
    RGL_CHECK_RETURN(rgl_node_raytrace(&Session->RaytraceNode, Scene), nullptr);
    RGL_CHECK(rgl_node_raytrace_configure_non_hits(Session->RaytraceNode, 0.0f, RangeM));

    // 4. Compact node
    RGL_CHECK_RETURN(rgl_node_points_compact_by_field(&Session->CompactNode, RGL_FIELD_IS_HIT_I32), nullptr);

    // 5. ToSensor node
    rgl_mat3x4f InvTf = RGLCoord::Identity();
    RGL_CHECK_RETURN(rgl_node_points_transform(&Session->ToSensorNode, &InvTf), nullptr);

    // 6. Yield node
    std::array<rgl_field_t, 3> OutputFields = {
        RGL_FIELD_XYZ_VEC3_F32,
        RGL_FIELD_DISTANCE_F32,
        RGL_FIELD_INTENSITY_F32
    };
    RGL_CHECK_RETURN(rgl_node_points_yield(
        &Session->YieldNode,
        OutputFields.data(),
        static_cast<int32_t>(OutputFields.size())), nullptr);

    // Validate all nodes were created
    if (!Session->AllNodesValid())
    {
        UE_LOG(LogTemp, Error,
               TEXT("RGLBackendImpl: RGL graph creation failed - some nodes are null."));
        // Destroy partially created graph
        if (Session->UseRaysNode)
        {
            rgl_graph_destroy(Session->UseRaysNode);
        }
        delete Session;
        return nullptr;
    }

    // 7. (Optional) RGL-native ROS2 publish: FormatNode -> Ros2PublishNode
    const bool bEnableRglRos2 = !Config.Ros2Topic.IsEmpty();

    if (bEnableRglRos2)
    {
        std::string TopicName = TCHAR_TO_UTF8(*Config.Ros2Topic);
        std::string FrameId = Config.Ros2FrameId.IsEmpty()
            ? "lidar"
            : TCHAR_TO_UTF8(*Config.Ros2FrameId);

        // Parse QoS settings from config strings
        rgl_qos_policy_reliability_t Reliability = QOS_POLICY_RELIABILITY_BEST_EFFORT;
        if (Config.Ros2Reliability == TEXT("reliable"))
        {
            Reliability = QOS_POLICY_RELIABILITY_RELIABLE;
        }

        rgl_qos_policy_durability_t Durability = QOS_POLICY_DURABILITY_VOLATILE;
        if (Config.Ros2Durability == TEXT("transient_local"))
        {
            Durability = QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        }

        rgl_qos_policy_history_t History = QOS_POLICY_HISTORY_KEEP_LAST;
        if (Config.Ros2History == TEXT("keep_all"))
        {
            History = QOS_POLICY_HISTORY_KEEP_ALL;
        }
        else if (Config.Ros2History == TEXT("system_default"))
        {
            History = QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
        }

        int32 HistoryDepth = Config.Ros2HistoryDepth;
        if (HistoryDepth <= 0)
        {
            HistoryDepth = 5;
        }

        // FormatNode: select binary layout from config
        std::string FormatName = Config.Ros2Format.IsEmpty()
            ? "PointXYZIRCAEDT"
            : TCHAR_TO_UTF8(*Config.Ros2Format);
        const FRGLPointCloudFormat* Format = FindFormat(FormatName.c_str());
        if (!Format)
        {
            RGLLog::Info("RGLBackendImpl: Unknown format '", FormatName.c_str(),
                         "', falling back to PointXYZIRCAEDT");
            Format = FindFormat("PointXYZIRCAEDT");
        }
        RGL_CHECK(rgl_node_points_format(
            &Session->FormatNode,
            Format->Fields,
            Format->FieldCount));
        RGLLog::Info("RGLBackendImpl: ROS2 format:", Format->Name, "(", Format->FieldCount, "fields)");

        // Ros2PublishNode: publish PointCloud2 with specified QoS
        RGL_CHECK(rgl_node_points_ros2_publish_with_qos(
            &Session->Ros2PublishNode,
            TopicName.c_str(),
            FrameId.c_str(),
            Reliability,
            Durability,
            History,
            HistoryDepth));

        const char* HistoryStr = (History == QOS_POLICY_HISTORY_KEEP_ALL) ? "keep_all" :
                                 (History == QOS_POLICY_HISTORY_SYSTEM_DEFAULT) ? "system_default" : "keep_last";
        RGLLog::Info("RGLBackendImpl: RGL ROS2 publish enabled, topic:", TopicName.c_str(),
                     "frame:", FrameId.c_str(),
                     "reliability:", Reliability == QOS_POLICY_RELIABILITY_RELIABLE ? "reliable" : "best_effort",
                     "durability:", Durability == QOS_POLICY_DURABILITY_VOLATILE ? "volatile" : "transient_local",
                     "history:", HistoryStr, "depth:", HistoryDepth);
    }

    // Connect graph: UseRays -> SetRange -> SetRingIds -> RaysTransform -> Raytrace -> Compact -> ToSensor -> Yield
    RGL_CHECK(rgl_graph_node_add_child(Session->UseRaysNode, Session->SetRangeNode));
    RGL_CHECK(rgl_graph_node_add_child(Session->SetRangeNode, Session->SetRingIdsNode));
    RGL_CHECK(rgl_graph_node_add_child(Session->SetRingIdsNode, Session->RaysTransformNode));
    RGL_CHECK(rgl_graph_node_add_child(Session->RaysTransformNode, Session->RaytraceNode));
    RGL_CHECK(rgl_graph_node_add_child(Session->RaytraceNode, Session->CompactNode));
    RGL_CHECK(rgl_graph_node_add_child(Session->CompactNode, Session->ToSensorNode));
    RGL_CHECK(rgl_graph_node_add_child(Session->ToSensorNode, Session->YieldNode));

    // Additional branch: ToSensor -> Ue5ToRos2 -> FormatNode -> Ros2PublishNode (parallel to Yield)
    // UE5 uses left-handed coords (Y=right), ROS2 uses right-handed (Y=left).
    if (bEnableRglRos2 && Session->FormatNode && Session->Ros2PublishNode)
    {
        // Y-axis flip matrix: scale Y by -1
        rgl_mat3x4f Ue5ToRos2Tf = RGLCoord::Identity();
        Ue5ToRos2Tf.value[1][1] = -1.0f;
        RGL_CHECK(rgl_node_points_transform(&Session->Ue5ToRos2Node, &Ue5ToRos2Tf));

        RGL_CHECK(rgl_graph_node_add_child(Session->ToSensorNode, Session->Ue5ToRos2Node));
        RGL_CHECK(rgl_graph_node_add_child(Session->Ue5ToRos2Node, Session->FormatNode));
        RGL_CHECK(rgl_graph_node_add_child(Session->FormatNode, Session->Ros2PublishNode));
        Session->bRos2Active = true;
    }

    // Store world reference for scene manager updates during Tick
    Session->World = World;

    RGLLog::Info("RGLBackendImpl: RGL computation graph created successfully");
    return static_cast<FRGLSessionHandle>(Session);
}

// ============================================================================
// DestroySession
// ============================================================================

void FRGLBackendImpl::DestroySession(FRGLSessionHandle Handle)
{
    if (!Handle)
    {
        return;
    }

    FRGLSession* Session = static_cast<FRGLSession*>(Handle);

    // Destroy the entire connected graph via the root node.
    // rgl_graph_destroy destroys all connected nodes in the graph.
    if (Session->UseRaysNode)
    {
        rgl_graph_destroy(Session->UseRaysNode);
    }

    // Clear all handles (they're invalid after graph_destroy)
    Session->UseRaysNode = nullptr;
    Session->RaysTransformNode = nullptr;
    Session->RaytraceNode = nullptr;
    Session->CompactNode = nullptr;
    Session->ToSensorNode = nullptr;
    Session->YieldNode = nullptr;

    delete Session;
}

// ============================================================================
// GenerateRayPattern
// ============================================================================

int32 FRGLBackendImpl::GenerateRayPattern(FRGLSession* Session, float DeltaSeconds,
                                           float& InOutHorizontalAngle)
{
    if (!Session)
    {
        return 0;
    }

    const FLidarDescription& Desc = Session->Config.LidarDesc;
    const uint32 ChannelCount = Desc.Channels;
    if (ChannelCount == 0)
    {
        return 0;
    }

    // Calculate the total angle swept in this tick.
    // Cap at one full rotation (HorizontalFov) to avoid redundant multi-rotation scans.
    float AngleDistanceOfTick = Desc.RotationFrequency
        * Desc.HorizontalFov * DeltaSeconds;
    AngleDistanceOfTick = FMath::Min(AngleDistanceOfTick, Desc.HorizontalFov);

    // Calculate points per channel proportional to the (capped) angle coverage.
    const float UnclampedAngle = Desc.RotationFrequency
        * Desc.HorizontalFov * DeltaSeconds;
    const float AngleRatio = (UnclampedAngle > 0.0f)
        ? (AngleDistanceOfTick / UnclampedAngle) : 1.0f;
    const uint32 PointsToScanWithOneLaser = FMath::RoundHalfFromZero(
        Desc.PointsPerSecond * DeltaSeconds / static_cast<float>(ChannelCount) * AngleRatio);

    if (PointsToScanWithOneLaser <= 0)
    {
        return 0;
    }

    const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

    // Pre-compute vertical angles per channel
    TArray<float> VerticalAngles;
    VerticalAngles.SetNum(ChannelCount);
    if (ChannelCount == 1)
    {
        VerticalAngles[0] = (Desc.UpperFovLimit + Desc.LowerFovLimit) / 2.0f;
    }
    else
    {
        for (uint32 ch = 0; ch < ChannelCount; ++ch)
        {
            VerticalAngles[ch] = Desc.UpperFovLimit -
                static_cast<float>(ch) * (Desc.UpperFovLimit - Desc.LowerFovLimit)
                / static_cast<float>(ChannelCount - 1);
        }
    }

    const int32 TotalRays = static_cast<int32>(ChannelCount * PointsToScanWithOneLaser);
    Session->RayTransforms.SetNum(TotalRays);
    Session->RingIds.SetNum(TotalRays);

    int32 RayIndex = 0;
    for (uint32 ch = 0; ch < ChannelCount; ++ch)
    {
        for (uint32 pt = 0; pt < PointsToScanWithOneLaser; ++pt)
        {
            // Center angle range around 0 (same as CPU ray_cast: -HorizontalFov/2 offset)
            const float HorizAngle = std::fmod(
                InOutHorizontalAngle + static_cast<float>(pt) * AngleDistanceOfLaserMeasure,
                Desc.HorizontalFov) - Desc.HorizontalFov / 2.0f;

            Session->RayTransforms[RayIndex] = RGLCoord::FromPitchYaw(
                VerticalAngles[ch],
                HorizAngle
            );
            Session->RingIds[RayIndex] = static_cast<int32>(ch);
            ++RayIndex;
        }
    }

    // Advance starting angle. With capped angle, this properly wraps around.
    InOutHorizontalAngle = std::fmod(
        InOutHorizontalAngle + AngleDistanceOfTick,
        Desc.HorizontalFov);

    return TotalRays;
}

// ============================================================================
// Tick
// ============================================================================

FRGLTickResult FRGLBackendImpl::Tick(
    FRGLSessionHandle Handle,
    const FTransform& SensorWorldTransform,
    float EffectiveDelta,
    float& InOutHorizontalAngle,
    bool bNeedResults,
    bool bNeedWorldPoints)
{
    FRGLTickResult Result;

    if (!Handle)
    {
        return Result;
    }

    FRGLSession* Session = static_cast<FRGLSession*>(Handle);
    if (!Session->AllNodesValid())
    {
        return Result;
    }

    // 1. Update scene geometry (only dynamic entities, frame-cached)
    UWorld* World = Session->World.Get();
    if (World)
    {
        FRGLSceneManager& SceneMgr = FRGLSceneManager::GetInstance(World);
        SceneMgr.Update(World);
    }

    // 2. Ray pattern: generate once, then cache (same DeltaSeconds = same pattern)
    if (!Session->bRayPatternCached)
    {
        const int32 TotalRays = GenerateRayPattern(Session, EffectiveDelta, InOutHorizontalAngle);
        if (TotalRays <= 0)
        {
            return Result;
        }

        RGL_CHECK(rgl_node_rays_from_mat3x4f(
            &Session->UseRaysNode,
            Session->RayTransforms.GetData(),
            TotalRays));

        RGL_CHECK(rgl_node_rays_set_ring_ids(
            &Session->SetRingIdsNode,
            Session->RingIds.GetData(),
            TotalRays));

        Session->CachedTotalRays = TotalRays;
        Session->bRayPatternCached = true;
    }

    if (Session->CachedTotalRays <= 0)
    {
        return Result;
    }

    // 3. Update sensor world transform (changes every tick)
    const FVector Scale = SensorWorldTransform.GetScale3D();
    if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
    {
        return Result;
    }

    rgl_mat3x4f SensorTf = RGLCoord::ToRGL(SensorWorldTransform);
    RGL_CHECK(rgl_node_rays_transform(&Session->RaysTransformNode, &SensorTf));

    // 4. Update inverse transform for world->sensor conversion
    rgl_mat3x4f InvTf = RGLCoord::InverseTransform(SensorWorldTransform);
    RGL_CHECK(rgl_node_points_transform(&Session->ToSensorNode, &InvTf));

    // 5. Run RGL graph (GPU raytrace + optional ROS2 publish)
    const double T0 = FPlatformTime::Seconds();
    rgl_status_t RunStatus = rgl_graph_run(Session->RaytraceNode);
    const double T1 = FPlatformTime::Seconds();
    if (RunStatus != RGL_SUCCESS)
    {
        const char* _err = nullptr;
        rgl_get_last_error_string(&_err);
        RGLLog::Info("[RGL] rgl_graph_run failed:",
                     _err ? _err : "unknown");
        return Result;
    }

    // 6. Collect results only if caller needs them
    double T2 = T1;
    if (bNeedResults)
    {
        CollectResults(Session, SensorWorldTransform, bNeedWorldPoints, Result);
        T2 = FPlatformTime::Seconds();
    }

    // [DEBUG] Log first 10 ticks with timing breakdown
    {
        static int32 sTickLog = 0;
        if (sTickLog < 10)
        {
            const double GraphRunMs = (T1 - T0) * 1000.0;
            const double CollectMs = (T2 - T1) * 1000.0;
            RGLLog::Info("[DEBUG] RGLBackendImpl tick", sTickLog,
                         "graphRun:", GraphRunMs, "ms",
                         "collect:", CollectMs, "ms",
                         "NeedResults:", bNeedResults ? "yes" : "no");
            ++sTickLog;
        }
    }

    return Result;
}

// ============================================================================
// CollectResults
// ============================================================================

void FRGLBackendImpl::CollectResults(FRGLSession* Session, const FTransform& SensorWorldTransform,
                                      bool bNeedWorldPoints, FRGLTickResult& OutResult)
{
    if (!Session || !Session->YieldNode)
    {
        return;
    }

    int32_t PointCount = 0;
    int32_t PointSize = 0;
    rgl_status_t Status = rgl_graph_get_result_size(
        Session->YieldNode, RGL_FIELD_XYZ_VEC3_F32, &PointCount, &PointSize);

    if (Status != RGL_SUCCESS || PointCount <= 0)
    {
        OutResult.TotalHits = 0;
        return;
    }

    // Sanity check: cap at 10M points to prevent allocation bombs
    if (PointCount > 10000000)
    {
        UE_LOG(LogTemp, Error,
               TEXT("RGLBackendImpl: Unreasonable point count %d from RGL, capping"),
               PointCount);
        PointCount = 10000000;
    }

    TArray<rgl_vec3f> RglHitPoints;
    RglHitPoints.SetNum(PointCount);
    TArray<float> Distances;
    Distances.SetNum(PointCount);
    TArray<float> Intensities;
    Intensities.SetNum(PointCount);

    rgl_status_t S1 = rgl_graph_get_result_data(
        Session->YieldNode, RGL_FIELD_XYZ_VEC3_F32, RglHitPoints.GetData());
    rgl_status_t S2 = rgl_graph_get_result_data(
        Session->YieldNode, RGL_FIELD_DISTANCE_F32, Distances.GetData());
    rgl_status_t S3 = rgl_graph_get_result_data(
        Session->YieldNode, RGL_FIELD_INTENSITY_F32, Intensities.GetData());

    if (S1 != RGL_SUCCESS || S2 != RGL_SUCCESS || S3 != RGL_SUCCESS)
    {
        UE_LOG(LogTemp, Warning, TEXT("RGLBackendImpl: Failed to read RGL results"));
        OutResult.TotalHits = 0;
        return;
    }

    // [DEBUG] Print first 5 collects
    {
        static int32 sHitLog = 0;
        if (sHitLog < 5)
        {
            float minDist = 1e9f, maxDist = 0.0f;
            for (int32_t d = 0; d < PointCount; ++d)
            {
                minDist = FMath::Min(minDist, Distances[d]);
                maxDist = FMath::Max(maxDist, Distances[d]);
            }
            RGLLog::Info("[DEBUG] RGLBackendImpl collect", sHitLog, "PointCount:", PointCount,
                         "minDist(m):", minDist, "maxDist(m):", maxDist);
            if (PointCount > 0)
            {
                RGLLog::Info("[DEBUG] Hit0 XYZ(m):", RglHitPoints[0].value[0],
                             RglHitPoints[0].value[1], RglHitPoints[0].value[2]);
            }
            ++sHitLog;
        }
    }

    // Populate the result struct
    OutResult.TotalHits = PointCount;
    OutResult.HitPoints.SetNum(PointCount);
    OutResult.Distances = MoveTemp(Distances);
    OutResult.Intensities = MoveTemp(Intensities);

    if (bNeedWorldPoints)
    {
        OutResult.WorldPoints.SetNum(PointCount);
        OutResult.WorldIntensities.SetNum(PointCount);
    }

    for (int32_t i = 0; i < PointCount; ++i)
    {
        // HitPoints: sensor-local, in meters (as RGL outputs)
        const float X = RglHitPoints[i].value[0];
        const float Y = RglHitPoints[i].value[1];
        const float Z = RglHitPoints[i].value[2];
        OutResult.HitPoints[i] = FVector(X, Y, Z);

        if (bNeedWorldPoints)
        {
            // Convert m -> cm for UE5 world space, then transform to world
            OutResult.WorldPoints[i] = SensorWorldTransform.TransformPosition(
                FVector(X * RGLCoord::RGL_TO_UE, Y * RGLCoord::RGL_TO_UE, Z * RGLCoord::RGL_TO_UE));
            OutResult.WorldIntensities[i] = OutResult.Intensities[i];
        }
    }
}

#endif // WITH_RGL
