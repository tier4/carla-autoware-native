// Copyright (c) 2026 RGL Integration for CARLA.
// Implementation of ARGLLidar: GPU-accelerated LiDAR sensor using RGL.
//
// When WITH_RGL is not defined, the sensor is still spawnable but produces
// no output (logs a warning on BeginPlay).

#include "Carla/Sensor/RGLLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/RGL/RGLCoordinateUtils.h"  // for RGLLog::Info

#include <util/ue-header-guard-begin.h>
#include "Engine/World.h"
#include "Math/UnrealMathUtility.h"
#include "Components/LineBatchComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "PhysicsEngine/PhysicsObjectExternalInterface.h"
#include <util/ue-header-guard-end.h>

#include <util/disable-ue4-macros.h>
#include <carla/Logging.h>
#include <carla/geom/Math.h>
#include <carla/sensor/data/LidarData.h>
#include <util/enable-ue4-macros.h>

#include <cmath>

// ============================================================================
// WITH_RGL: include RGL headers and define the PIMPL struct
// ============================================================================

#ifdef WITH_RGL

#include "Carla/RGL/RGLSceneManager.h"
#include "Carla/RGL/RGLCoordinateUtils.h"

#include <util/disable-ue4-macros.h>
#include <rgl/api/core.h>
#include <rgl/api/extensions/ros2.h>
#include <util/enable-ue4-macros.h>

// RGL_CHECK macro is defined in RGLCoordinateUtils.h

// Helper: check RGL call and return false on failure
#ifndef RGL_CHECK_RETURN
#define RGL_CHECK_RETURN(call, retval)                                        \
    do {                                                                      \
        rgl_status_t _s = (call);                                             \
        if (_s != RGL_SUCCESS) {                                              \
            const char* _err = nullptr;                                       \
            rgl_get_last_error_string(&_err);                                 \
            UE_LOG(LogCarla, Error,                                           \
                   TEXT("RGL error [ARGLLidar] %s: %s"),                      \
                   TEXT(#call),                                               \
                   _err ? *FString(UTF8_TO_TCHAR(_err)) : TEXT("unknown"));   \
            return retval;                                                    \
        }                                                                     \
    } while (0)
#endif

/// Point cloud format definitions matching AWSIM's PointCloudFormatLibrary.
/// See: AWSIM/Assets/Awsim/Scripts/Entity/Sensor/Lidar/PointCloudFormatLibrary.cs
struct FRGLPointCloudFormat
{
    const char* Name;
    const rgl_field_t* Fields;
    int32 FieldCount;
};

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
    {"minimal",        GFormatMinimal,    3},
    {"pcl24",          GFormatPcl24,      5},
    {"pcl48",          GFormatPcl48,     12},
    {"PointXYZIRCAEDT", GFormatXYZIRCAEDT, 8},
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

/// Opaque PIMPL struct holding all RGL-specific handles and buffers.
struct ARGLLidar::FRGLLidarImpl
{
    rgl_node_t UseRaysNode = nullptr;
    rgl_node_t SetRangeNode = nullptr;
    rgl_node_t SetRingIdsNode = nullptr;  // Assigns channel/ring IDs to rays
    rgl_node_t RaysTransformNode = nullptr;
    rgl_node_t RaytraceNode = nullptr;
    rgl_node_t CompactNode = nullptr;
    rgl_node_t ToSensorNode = nullptr;
    rgl_node_t YieldNode = nullptr;
    rgl_node_t Ue5ToRos2Node = nullptr;  // Y-axis flip (UE5 left-hand -> ROS2 right-hand)
    rgl_node_t FormatNode = nullptr;
    rgl_node_t Ros2PublishNode = nullptr;

    TArray<rgl_mat3x4f> RayTransforms;
    TArray<int32> RingIds;  // Per-ray channel/ring IDs

    // Cache: ray pattern and ring IDs only need uploading once if DeltaSeconds is constant
    bool bRayPatternCached = false;
    int32 CachedTotalRays = 0;

    bool AllNodesValid() const
    {
        return UseRaysNode && SetRangeNode && RaysTransformNode && RaytraceNode &&
               CompactNode && ToSensorNode && YieldNode;
        // FormatNode and Ros2PublishNode are optional (only when ROS2 attribute is set)
    }
};

#endif // WITH_RGL

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
#ifdef WITH_RGL
    DestroyRGLGraph();
#endif
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

    RGLLog::Info("ARGLLidar configured: Channels=", Description.Channels,
                 "Range=", Description.Range, "cm PPS=", Description.PointsPerSecond,
                 "Freq=", Description.RotationFrequency, "Hz FOV=[",
                 Description.LowerFovLimit, ",", Description.UpperFovLimit,
                 "] HorizFOV=", Description.HorizontalFov,
                 "ShowLidar=", bRglShowLidarPoints ? "ON" : "OFF",
                 "PointRate=", RglLidarDrawPointRate,
                 "LifeTime=", RglLidarDrawLifeTime);
}

// ============================================================================
// BeginPlay / EndPlay
// ============================================================================

void ARGLLidar::BeginPlay()
{
    Super::BeginPlay();

    LidarMeasurement = carla::sensor::data::LidarData(Description.Channels);

#ifdef WITH_RGL
    CreateRGLGraph();
#else
    UE_LOG(LogCarla, Warning,
           TEXT("ARGLLidar: WITH_RGL not defined at compile time. "
                "This sensor will produce no output. "
                "Rebuild with -DENABLE_RGL=ON to enable GPU LiDAR."));
#endif
}

void ARGLLidar::EndPlay(EEndPlayReason::Type EndPlayReason)
{
#ifdef WITH_RGL
    DestroyRGLGraph();
#endif
    Super::EndPlay(EndPlayReason);
}

// ============================================================================
// PostPhysTick: main entry point called each physics tick
// ============================================================================

void ARGLLidar::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ARGLLidar::PostPhysTick);

#ifdef WITH_RGL
    // Use the sensor tick interval instead of the physics step delta.
    // PostPhysTick receives the physics step (e.g., 0.01s at 100Hz), but
    // the LiDAR should simulate the full sensor_tick period (e.g., 0.1s at 10Hz)
    // to produce a complete rotational scan per publish event.
    const float SensorTickInterval = GetActorTickInterval();
    const float EffectiveDelta = (SensorTickInterval > 0.0f) ? SensorTickInterval : DeltaSeconds;
    SimulateLidar(EffectiveDelta);
#endif

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
// RGL-specific implementation (compiled only when WITH_RGL is defined)
// ============================================================================

#ifdef WITH_RGL

void ARGLLidar::CreateRGLGraph()
{
    if (bGraphCreated)
    {
        return;
    }

    RGLImpl = new FRGLLidarImpl();

    // Pre-initialize scene manager (triggers world scan)
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogCarla, Error, TEXT("ARGLLidar: No world available during graph creation"));
        delete RGLImpl;
        RGLImpl = nullptr;
        return;
    }

    FRGLSceneManager& SceneMgr = FRGLSceneManager::GetInstance(World);
    SceneMgr.Update(World); // Force initial scene sync before first raytrace
    rgl_scene_t Scene = SceneMgr.GetScene();

    // Build graph nodes with validation.
    // If any node creation fails, abort and disable the sensor.

    // 1. UseRays node
    rgl_mat3x4f DummyRay = RGLCoord::Identity();
    RGL_CHECK_RETURN(rgl_node_rays_from_mat3x4f(&RGLImpl->UseRaysNode, &DummyRay, 1), );

    // 1.5. Set ray range (min=0, max=Description.Range in meters)
    const float RangeM = Description.Range * RGLCoord::UE_TO_RGL;
    rgl_vec2f RayRange = {{ 0.0f, RangeM }};
    RGL_CHECK_RETURN(rgl_node_rays_set_range(&RGLImpl->SetRangeNode, &RayRange, 1), );

    // 1.6. Set ring IDs (channel assignment for each ray)
    //      Initialized with a single dummy ID; updated each tick in SimulateLidar.
    int32_t DummyRingId = 0;
    RGL_CHECK_RETURN(rgl_node_rays_set_ring_ids(&RGLImpl->SetRingIdsNode, &DummyRingId, 1), );

    // 2. RaysTransform node
    rgl_mat3x4f SensorTf = RGLCoord::Identity();
    RGL_CHECK_RETURN(rgl_node_rays_transform(&RGLImpl->RaysTransformNode, &SensorTf), );

    // 3. Raytrace node
    RGL_CHECK_RETURN(rgl_node_raytrace(&RGLImpl->RaytraceNode, Scene), );

    RGL_CHECK(rgl_node_raytrace_configure_non_hits(RGLImpl->RaytraceNode, 0.0f, RangeM));

    // 4. Compact node
    RGL_CHECK_RETURN(rgl_node_points_compact_by_field(&RGLImpl->CompactNode, RGL_FIELD_IS_HIT_I32), );

    // 5. ToSensor node
    rgl_mat3x4f InvTf = RGLCoord::Identity();
    RGL_CHECK_RETURN(rgl_node_points_transform(&RGLImpl->ToSensorNode, &InvTf), );

    // 6. Yield node
    std::array<rgl_field_t, 3> OutputFields = {
        RGL_FIELD_XYZ_VEC3_F32,
        RGL_FIELD_DISTANCE_F32,
        RGL_FIELD_INTENSITY_F32
    };
    RGL_CHECK_RETURN(rgl_node_points_yield(
        &RGLImpl->YieldNode,
        OutputFields.data(),
        static_cast<int32_t>(OutputFields.size())), );

    // Validate all nodes were created
    if (!RGLImpl->AllNodesValid())
    {
        UE_LOG(LogCarla, Error,
               TEXT("ARGLLidar: RGL graph creation failed - some nodes are null. Disabling sensor."));
        DestroyRGLGraph();
        return;
    }

    // 7. (Optional) RGL-native ROS2 publish: FormatNode -> Ros2PublishNode
    //    Controlled by rgl_ros2_topic attribute (empty = disabled).
    auto RglTopicAttr = GetAttribute(TEXT("rgl_ros2_topic"));
    const bool bEnableRglRos2 = RglTopicAttr.has_value() && !RglTopicAttr->Value.IsEmpty();

    if (bEnableRglRos2)
    {
        std::string TopicName = TCHAR_TO_UTF8(*RglTopicAttr->Value);

        auto FrameAttr = GetAttribute(TEXT("rgl_ros2_frame_id"));
        std::string FrameId = FrameAttr.has_value() ? TCHAR_TO_UTF8(*FrameAttr->Value) : "lidar";

        // Parse QoS settings from attributes
        rgl_qos_policy_reliability_t Reliability = QOS_POLICY_RELIABILITY_BEST_EFFORT;
        auto ReliabilityAttr = GetAttribute(TEXT("rgl_ros2_reliability"));
        if (ReliabilityAttr.has_value() && ReliabilityAttr->Value == TEXT("reliable"))
        {
            Reliability = QOS_POLICY_RELIABILITY_RELIABLE;
        }

        rgl_qos_policy_durability_t Durability = QOS_POLICY_DURABILITY_VOLATILE;
        auto DurabilityAttr = GetAttribute(TEXT("rgl_ros2_durability"));
        if (DurabilityAttr.has_value() && DurabilityAttr->Value == TEXT("transient_local"))
        {
            Durability = QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        }

        rgl_qos_policy_history_t History = QOS_POLICY_HISTORY_KEEP_LAST;
        auto HistoryAttr = GetAttribute(TEXT("rgl_ros2_history"));
        if (HistoryAttr.has_value())
        {
            if (HistoryAttr->Value == TEXT("keep_all"))
            {
                History = QOS_POLICY_HISTORY_KEEP_ALL;
            }
            else if (HistoryAttr->Value == TEXT("system_default"))
            {
                History = QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
            }
        }

        int32 HistoryDepth = 5;  // AWSIM default
        auto DepthAttr = GetAttribute(TEXT("rgl_ros2_history_depth"));
        if (DepthAttr.has_value())
        {
            HistoryDepth = FCString::Atoi(*DepthAttr->Value);
            if (HistoryDepth <= 0) HistoryDepth = 5;
        }

        // FormatNode: select binary layout from rgl_ros2_format attribute
        auto FormatAttr = GetAttribute(TEXT("rgl_ros2_format"));
        std::string FormatName = FormatAttr.has_value() ? TCHAR_TO_UTF8(*FormatAttr->Value) : "PointXYZIRCAEDT";
        const FRGLPointCloudFormat* Format = FindFormat(FormatName.c_str());
        if (!Format)
        {
            RGLLog::Info("ARGLLidar: Unknown format '", FormatName.c_str(), "', falling back to PointXYZIRCAEDT");
            Format = FindFormat("PointXYZIRCAEDT");
        }
        RGL_CHECK(rgl_node_points_format(
            &RGLImpl->FormatNode,
            Format->Fields,
            Format->FieldCount));
        RGLLog::Info("ARGLLidar: ROS2 format:", Format->Name, "(", Format->FieldCount, "fields)");

        // Ros2PublishNode: publish PointCloud2 with specified QoS
        RGL_CHECK(rgl_node_points_ros2_publish_with_qos(
            &RGLImpl->Ros2PublishNode,
            TopicName.c_str(),
            FrameId.c_str(),
            Reliability,
            Durability,
            History,
            HistoryDepth));

        const char* HistoryStr = (History == QOS_POLICY_HISTORY_KEEP_ALL) ? "keep_all" :
                                 (History == QOS_POLICY_HISTORY_SYSTEM_DEFAULT) ? "system_default" : "keep_last";
        RGLLog::Info("ARGLLidar: RGL ROS2 publish enabled, topic:", TopicName.c_str(),
                     "frame:", FrameId.c_str(),
                     "reliability:", Reliability == QOS_POLICY_RELIABILITY_RELIABLE ? "reliable" : "best_effort",
                     "durability:", Durability == QOS_POLICY_DURABILITY_VOLATILE ? "volatile" : "transient_local",
                     "history:", HistoryStr, "depth:", HistoryDepth);
    }

    // Connect graph: UseRays -> SetRange -> SetRingIds -> RaysTransform -> Raytrace -> Compact -> ToSensor -> Yield
    RGL_CHECK(rgl_graph_node_add_child(RGLImpl->UseRaysNode, RGLImpl->SetRangeNode));
    RGL_CHECK(rgl_graph_node_add_child(RGLImpl->SetRangeNode, RGLImpl->SetRingIdsNode));
    RGL_CHECK(rgl_graph_node_add_child(RGLImpl->SetRingIdsNode, RGLImpl->RaysTransformNode));
    RGL_CHECK(rgl_graph_node_add_child(RGLImpl->RaysTransformNode, RGLImpl->RaytraceNode));
    RGL_CHECK(rgl_graph_node_add_child(RGLImpl->RaytraceNode, RGLImpl->CompactNode));
    RGL_CHECK(rgl_graph_node_add_child(RGLImpl->CompactNode, RGLImpl->ToSensorNode));
    RGL_CHECK(rgl_graph_node_add_child(RGLImpl->ToSensorNode, RGLImpl->YieldNode));

    // Additional branch: ToSensor -> Ue5ToRos2 -> FormatNode -> Ros2PublishNode (parallel to Yield)
    // UE5 uses left-handed coords (Y=right), ROS2 uses right-handed (Y=left).
    // CARLA's built-in publisher applies Y-axis inversion in ConvertToRosFormat().
    // For RGL's direct publish, we apply the same inversion via a transform node.
    if (bEnableRglRos2 && RGLImpl->FormatNode && RGLImpl->Ros2PublishNode)
    {
        // Y-axis flip matrix: scale Y by -1
        rgl_mat3x4f Ue5ToRos2Tf = RGLCoord::Identity();
        Ue5ToRos2Tf.value[1][1] = -1.0f;  // Flip Y axis
        RGL_CHECK(rgl_node_points_transform(&RGLImpl->Ue5ToRos2Node, &Ue5ToRos2Tf));

        RGL_CHECK(rgl_graph_node_add_child(RGLImpl->ToSensorNode, RGLImpl->Ue5ToRos2Node));
        RGL_CHECK(rgl_graph_node_add_child(RGLImpl->Ue5ToRos2Node, RGLImpl->FormatNode));
        RGL_CHECK(rgl_graph_node_add_child(RGLImpl->FormatNode, RGLImpl->Ros2PublishNode));
        bRglRos2Active = true;
    }

    bGraphCreated = true;
    RGLLog::Info("ARGLLidar: RGL computation graph created successfully");
}

void ARGLLidar::DestroyRGLGraph()
{
    if (!RGLImpl)
    {
        bGraphCreated = false;
        return;
    }

    // Destroy the entire connected graph via the root node.
    // rgl_graph_destroy destroys all connected nodes in the graph.
    if (RGLImpl->UseRaysNode)
    {
        rgl_graph_destroy(RGLImpl->UseRaysNode);
    }

    // Clear all handles (they're invalid after graph_destroy)
    RGLImpl->UseRaysNode = nullptr;
    RGLImpl->RaysTransformNode = nullptr;
    RGLImpl->RaytraceNode = nullptr;
    RGLImpl->CompactNode = nullptr;
    RGLImpl->ToSensorNode = nullptr;
    RGLImpl->YieldNode = nullptr;

    delete RGLImpl;
    RGLImpl = nullptr;
    bGraphCreated = false;
}

int32 ARGLLidar::GenerateRayPattern(float DeltaSeconds)
{
    if (!RGLImpl)
    {
        return 0;
    }

    const uint32 ChannelCount = Description.Channels;
    if (ChannelCount == 0)
    {
        return 0;
    }

    // Calculate the total angle swept in this tick.
    // Cap at one full rotation (HorizontalFov) to avoid redundant multi-rotation scans
    // which waste GPU resources without adding new information.
    float AngleDistanceOfTick = Description.RotationFrequency
        * Description.HorizontalFov * DeltaSeconds;
    AngleDistanceOfTick = FMath::Min(AngleDistanceOfTick, Description.HorizontalFov);

    // Calculate points per channel proportional to the (capped) angle coverage.
    // Original formula: PPS * DeltaSeconds / Channels (may produce too many for multi-rotation).
    // Adjusted: scale by capped angle / uncapped angle ratio.
    const float UnclampedAngle = Description.RotationFrequency
        * Description.HorizontalFov * DeltaSeconds;
    const float AngleRatio = (UnclampedAngle > 0.0f)
        ? (AngleDistanceOfTick / UnclampedAngle) : 1.0f;
    const uint32 PointsToScanWithOneLaser = FMath::RoundHalfFromZero(
        Description.PointsPerSecond * DeltaSeconds / static_cast<float>(ChannelCount) * AngleRatio);

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
        VerticalAngles[0] = (Description.UpperFovLimit + Description.LowerFovLimit) / 2.0f;
    }
    else
    {
        for (uint32 ch = 0; ch < ChannelCount; ++ch)
        {
            VerticalAngles[ch] = Description.UpperFovLimit -
                static_cast<float>(ch) * (Description.UpperFovLimit - Description.LowerFovLimit)
                / static_cast<float>(ChannelCount - 1);
        }
    }

    const int32 TotalRays = static_cast<int32>(ChannelCount * PointsToScanWithOneLaser);
    RGLImpl->RayTransforms.SetNum(TotalRays);
    RGLImpl->RingIds.SetNum(TotalRays);

    int32 RayIndex = 0;
    for (uint32 ch = 0; ch < ChannelCount; ++ch)
    {
        for (uint32 pt = 0; pt < PointsToScanWithOneLaser; ++pt)
        {
            // Center angle range around 0 (same as CPU ray_cast: -HorizontalFov/2 offset)
            const float HorizAngle = std::fmod(
                CurrentHorizontalAngle + static_cast<float>(pt) * AngleDistanceOfLaserMeasure,
                Description.HorizontalFov) - Description.HorizontalFov / 2.0f;

            RGLImpl->RayTransforms[RayIndex] = RGLCoord::FromPitchYaw(
                VerticalAngles[ch],
                HorizAngle
            );
            RGLImpl->RingIds[RayIndex] = static_cast<int32>(ch);
            ++RayIndex;
        }
    }

    // Advance starting angle. With capped angle, this properly wraps around.
    CurrentHorizontalAngle = std::fmod(
        CurrentHorizontalAngle + AngleDistanceOfTick,
        Description.HorizontalFov);

    return TotalRays;
}

void ARGLLidar::SimulateLidar(float DeltaSeconds)
{
    if (!bGraphCreated || !RGLImpl || !RGLImpl->AllNodesValid())
    {
        return;
    }

    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    // ========================================================================
    // Optimized RGL GPU raytrace pipeline
    // ========================================================================

    // 1. Update scene geometry (only dynamic entities, frame-cached)
    FRGLSceneManager& SceneMgr = FRGLSceneManager::GetInstance(World);
    SceneMgr.Update(World);

    // 2. Ray pattern: generate once, then cache (same DeltaSeconds = same pattern)
    //    Only the sensor transform changes each tick, not the ray directions.
    if (!RGLImpl->bRayPatternCached)
    {
        const int32 TotalRays = GenerateRayPattern(DeltaSeconds);
        if (TotalRays <= 0)
        {
            return;
        }

        RGL_CHECK(rgl_node_rays_from_mat3x4f(
            &RGLImpl->UseRaysNode,
            RGLImpl->RayTransforms.GetData(),
            TotalRays));

        RGL_CHECK(rgl_node_rays_set_ring_ids(
            &RGLImpl->SetRingIdsNode,
            RGLImpl->RingIds.GetData(),
            TotalRays));

        RGLImpl->CachedTotalRays = TotalRays;
        RGLImpl->bRayPatternCached = true;
    }

    if (RGLImpl->CachedTotalRays <= 0)
    {
        return;
    }

    // 3. Update sensor world transform (changes every tick)
    const FTransform SensorWorldTransform = GetActorTransform();
    const FVector Scale = SensorWorldTransform.GetScale3D();
    if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
    {
        return;
    }

    rgl_mat3x4f SensorTf = RGLCoord::ToRGL(SensorWorldTransform);
    RGL_CHECK(rgl_node_rays_transform(&RGLImpl->RaysTransformNode, &SensorTf));

    // 4. Update inverse transform for world->sensor conversion
    rgl_mat3x4f InvTf = RGLCoord::InverseTransform(SensorWorldTransform);
    RGL_CHECK(rgl_node_points_transform(&RGLImpl->ToSensorNode, &InvTf));

    // 5. Run RGL graph (GPU raytrace + optional ROS2 publish)
    const double T0 = FPlatformTime::Seconds();
    rgl_status_t RunStatus = rgl_graph_run(RGLImpl->RaytraceNode);
    const double T1 = FPlatformTime::Seconds();
    if (RunStatus != RGL_SUCCESS)
    {
        const char* _err = nullptr;
        rgl_get_last_error_string(&_err);
        RGLLog::Info("[RGL] rgl_graph_run failed:",
                     _err ? _err : "unknown");
        return;
    }

    // 6. Collect results only if CARLA data pipeline or debug draw needs them.
    const bool bNeedResults = AreClientsListening() || bRglShowLidarPoints;
    double T2 = T1;
    if (bNeedResults)
    {
        CollectResults();
        T2 = FPlatformTime::Seconds();
    }

    // [DEBUG] Log first 10 ticks with timing breakdown
    {
        static int32 sTickLog = 0;
        if (sTickLog < 10)
        {
            const double GraphRunMs = (T1 - T0) * 1000.0;
            const double CollectMs = (T2 - T1) * 1000.0;
            RGLLog::Info("[DEBUG] tick", sTickLog,
                         "graphRun:", GraphRunMs, "ms",
                         "collect:", CollectMs, "ms",
                         "NeedResults:", bNeedResults ? "yes" : "no");
            ++sTickLog;
        }
    }
}

void ARGLLidar::CollectResults()
{
    if (!RGLImpl || !RGLImpl->YieldNode)
    {
        return;
    }

    int32_t PointCount = 0;
    int32_t PointSize = 0;
    rgl_status_t Status = rgl_graph_get_result_size(
        RGLImpl->YieldNode, RGL_FIELD_XYZ_VEC3_F32, &PointCount, &PointSize);

    if (Status != RGL_SUCCESS || PointCount <= 0)
    {
        std::vector<uint32_t> ZeroCounts(Description.Channels, 0);
        LidarMeasurement.ResetMemory(ZeroCounts);
        return;
    }

    // Sanity check: cap at 10M points to prevent allocation bombs
    if (PointCount > 10000000)
    {
        UE_LOG(LogCarla, Error,
               TEXT("ARGLLidar: Unreasonable point count %d from RGL, capping"),
               PointCount);
        PointCount = 10000000;
    }

    TArray<rgl_vec3f> HitPoints;
    HitPoints.SetNum(PointCount);
    TArray<float> Distances;
    Distances.SetNum(PointCount);
    TArray<float> Intensities;
    Intensities.SetNum(PointCount);

    rgl_status_t S1 = rgl_graph_get_result_data(
        RGLImpl->YieldNode, RGL_FIELD_XYZ_VEC3_F32, HitPoints.GetData());
    rgl_status_t S2 = rgl_graph_get_result_data(
        RGLImpl->YieldNode, RGL_FIELD_DISTANCE_F32, Distances.GetData());
    rgl_status_t S3 = rgl_graph_get_result_data(
        RGLImpl->YieldNode, RGL_FIELD_INTENSITY_F32, Intensities.GetData());

    if (S1 != RGL_SUCCESS || S2 != RGL_SUCCESS || S3 != RGL_SUCCESS)
    {
        UE_LOG(LogCarla, Warning, TEXT("ARGLLidar: Failed to read RGL results"));
        std::vector<uint32_t> ZeroCounts(Description.Channels, 0);
        LidarMeasurement.ResetMemory(ZeroCounts);
        return;
    }

    // [DEBUG] Print first 5 collects: hit count, nearest/farthest distances
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
            RGLLog::Info("[DEBUG] collect", sHitLog, "PointCount:", PointCount,
                         "minDist(m):", minDist, "maxDist(m):", maxDist);
            if (PointCount > 0)
            {
                RGLLog::Info("[DEBUG] Hit0 XYZ(m):", HitPoints[0].value[0], HitPoints[0].value[1], HitPoints[0].value[2]);
            }
            ++sHitLog;
        }
    }

    // Per-channel point distribution (simplified: even distribution)
    // TODO: Use RGL_FIELD_RING_ID_U16 for accurate per-channel tracking
    std::vector<uint32_t> PointsPerChannel(Description.Channels, 0);
    const uint32_t BaseCount = static_cast<uint32_t>(PointCount) / Description.Channels;
    const uint32_t Remainder = static_cast<uint32_t>(PointCount) % Description.Channels;
    for (uint32_t ch = 0; ch < Description.Channels; ++ch)
    {
        PointsPerChannel[ch] = BaseCount + (ch < Remainder ? 1 : 0);
    }

    LidarMeasurement.ResetMemory(PointsPerChannel);
    LidarMeasurement.SetHorizontalAngle(
        carla::geom::Math::ToRadians(CurrentHorizontalAngle));

    // Also populate debug buffers if debug draw is enabled
    if (bRglShowLidarPoints)
    {
        RglLidarDrawWorldPoints.SetNum(PointCount);
        RglLidarDrawIntensities.SetNum(PointCount);
    }

    const FTransform SensorTf = GetActorTransform();

    for (int32_t i = 0; i < PointCount; ++i)
    {
        // Store in meters (ROS2/Autoware standard) — RGL already outputs in meters
        const float X = HitPoints[i].value[0];
        const float Y = HitPoints[i].value[1];
        const float Z = HitPoints[i].value[2];

        const float DistanceM = Distances[i];
        float Intensity = Intensities[i];

        if (Description.AtmospAttenRate > 0.0f)
        {
            Intensity *= std::exp(-Description.AtmospAttenRate * DistanceM);
        }

        carla::sensor::data::LidarDetection Detection(X, Y, Z, Intensity);
        LidarMeasurement.WritePointSync(Detection);

        if (bRglShowLidarPoints)
        {
            // Convert m back to cm for UE5 debug drawing
            RglLidarDrawWorldPoints[i] = SensorTf.TransformPosition(
                FVector(X * RGLCoord::RGL_TO_UE, Y * RGLCoord::RGL_TO_UE, Z * RGLCoord::RGL_TO_UE));
            RglLidarDrawIntensities[i] = Intensity;
        }
    }

    LidarMeasurement.WriteChannelCount(PointsPerChannel);
}

#else // !WITH_RGL

// Stub implementations when RGL is not available
void ARGLLidar::CreateRGLGraph() {}
void ARGLLidar::DestroyRGLGraph() {}
int32 ARGLLidar::GenerateRayPattern(float) { return 0; }
void ARGLLidar::SimulateLidar(float) {}
void ARGLLidar::CollectResults() {}

#endif // WITH_RGL

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
