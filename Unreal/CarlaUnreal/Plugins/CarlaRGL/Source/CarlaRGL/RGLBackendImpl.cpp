// Copyright (c) 2026 RGL Integration for CARLA.
// Implementation of FRGLBackendImpl: all rgl_* API calls for LiDAR simulation.
// Ported from ARGLLidar::CreateRGLGraph / SimulateLidar / CollectResults.

#ifdef WITH_RGL

#include "RGLBackendImpl.h"
#include "RGLSceneManager.h"
#include "RGLCoordinateUtils.h"
#include "RGLUdpExtensionShim.h"
#include "RGLScanTiming.h"
#include "RGLScanTimingDiagnostics.h"
#include "RGLVelocityDistortion.h"
#include "RGLVehicleIgnore.h"
#include "RGLDynLoader.h"

#include <util/disable-ue4-macros.h>
#include <rgl/api/core.h>
#include <rgl/api/extensions/ros2.h>
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Math/UnrealMathUtility.h"
#include <util/ue-header-guard-end.h>

#include <cmath>
#include <cstdio>
#include <string>

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

namespace
{
// Shifts the scene time to the start of the scan for one graph run.
// rgl_scene_set_time synchronizes graph execution, so the shifted time remains
// visible to asynchronous UDP publishing until the graph has finished.
// Preserve the previous/frame pair when available. The first frame has no
// previous timestamp, so it only sets scan start and then frame time.
class FScopedRGLSceneTime
{
public:
    FScopedRGLSceneTime(bool bEnable, rgl_scene_t InScene, uint64 PrevFrameTimeNs, uint64 ScanTimeNs,
                       uint64 FrameTimeNs, bool bInHasPreviousFrame = true,
                       bool* OutRestoreFailed = nullptr)
        : bEnabled(bEnable), bHasPreviousFrame(bInHasPreviousFrame), Scene(InScene),
          PrevTimeNs(PrevFrameTimeNs), RestoreTimeNs(FrameTimeNs), RestoreFailure(OutRestoreFailed)
    {
        if (RestoreFailure) *RestoreFailure = false;
        if (bEnabled)
        {
            if (bHasPreviousFrame) Status = rgl_scene_set_time(Scene, PrevTimeNs);
            if (Status == RGL_SUCCESS) Status = rgl_scene_set_time(Scene, ScanTimeNs);
            RGL_CHECK(Status);
        }
    }
    ~FScopedRGLSceneTime() { Restore(); }

    // Idempotent: callers can inspect the result while the scope is still alive.
    void Restore()
    {
        if (!bEnabled || bRestoreAttempted) return;
        bRestoreAttempted = true;
        constexpr int MaxRestoreAttempts = 4;
        for (int Attempt = 0; Attempt < MaxRestoreAttempts; ++Attempt)
        {
            rgl_status_t RestoreStatus = RGL_SUCCESS;
            if (bHasPreviousFrame) RestoreStatus = rgl_scene_set_time(Scene, PrevTimeNs);
            if (RestoreStatus == RGL_SUCCESS)
                RestoreStatus = rgl_scene_set_time(Scene, RestoreTimeNs);
            if (RestoreStatus == RGL_SUCCESS) return;
            RGL_CHECK(RestoreStatus);
            // Synchronization errors may be consumed one at a time. Retry the
            // whole pair, including when only the final frame set failed.
        }
        bRestoreFailed = true;
        if (RestoreFailure) *RestoreFailure = true;
    }
    bool IsValid() const { return Status == RGL_SUCCESS; }
    bool RestoreFailed() const { return bRestoreFailed; }
    FScopedRGLSceneTime(const FScopedRGLSceneTime&) = delete;
    FScopedRGLSceneTime& operator=(const FScopedRGLSceneTime&) = delete;

private:
    bool bEnabled;
    bool bHasPreviousFrame;
    bool bRestoreAttempted = false;
    bool bRestoreFailed = false;
    rgl_scene_t Scene;
    uint64 PrevTimeNs;
    uint64 RestoreTimeNs;
    bool* RestoreFailure;
    rgl_status_t Status = RGL_SUCCESS;
};
}

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

// ============================================================================
// UDP Raw Packet support: model name -> RGL enum mapping + return mode whitelist.
// Mirrors AWSIM's UnityToRGLLidarModelsMapping and SupportedLidarsAndReturnModes
// from Assets/RGLUnityPlugin/Scripts/LidarUdpPublisher.cs.
// ============================================================================

static const TMap<FString, rgl_lidar_model_t> GLidarModelMap = {
    {TEXT("VelodyneVLP16"),     RGL_VELODYNE_VLP16},
    {TEXT("VelodyneVLP32C"),    RGL_VELODYNE_VLP32C},
    {TEXT("VelodyneVLS128"),    RGL_VELODYNE_VLS128},
    {TEXT("HesaiPandar40P"),    RGL_HESAI_PANDAR_40P},
    {TEXT("HesaiPandarQT"),     RGL_HESAI_PANDAR_QT64},
    {TEXT("HesaiPandarXT32"),   RGL_HESAI_PANDAR_XT32},
    {TEXT("HesaiQT128C2X"),     RGL_HESAI_QT128C2X},
    {TEXT("HesaiPandar128E4X"), RGL_HESAI_PANDAR_128E4X},
    {TEXT("HesaiPandar128E4XHighRes"), RGL_HESAI_PANDAR_128E4X},
};

// Return mode whitelist per LiDAR model. Names match the strings accepted
// by FLidarDescription::ReturnMode (lower-case).
static bool IsReturnModeSupportedForModel(const FString& Model, const FString& Mode)
{
    static const TMap<FString, TSet<FString>> Whitelist = {
        {TEXT("VelodyneVLP16"),
         {TEXT("strongest"), TEXT("last"), TEXT("last_strongest")}},
        {TEXT("VelodyneVLP32C"),
         {TEXT("strongest"), TEXT("last"), TEXT("last_strongest")}},
        {TEXT("VelodyneVLS128"),
         {TEXT("strongest"), TEXT("last"), TEXT("last_strongest")}},
        {TEXT("HesaiPandar40P"),
         {TEXT("strongest"), TEXT("last"), TEXT("last_strongest")}},
        {TEXT("HesaiPandarQT"),
         {TEXT("first"), TEXT("last"), TEXT("first_last")}},
        {TEXT("HesaiQT128C2X"),
         {TEXT("first"), TEXT("second"), TEXT("strongest"), TEXT("last"),
          TEXT("last_strongest"), TEXT("first_last"), TEXT("first_strongest"),
          TEXT("strongest_second_strongest"), TEXT("first_second")}},
        {TEXT("HesaiPandar128E4X"),
         {TEXT("first"), TEXT("strongest"), TEXT("last"),
          TEXT("last_strongest"), TEXT("first_last"), TEXT("first_strongest")}},
        {TEXT("HesaiPandar128E4XHighRes"),
         {TEXT("first"), TEXT("strongest"), TEXT("last"),
          TEXT("last_strongest"), TEXT("first_last"), TEXT("first_strongest")}},
        {TEXT("HesaiPandarXT32"),
         {TEXT("strongest"), TEXT("last"), TEXT("last_strongest")}},
    };
    // Empty ReturnMode means "first" by historical default — also acceptable
    // when whitelist contains "first".
    const FString Effective = Mode.IsEmpty() ? TEXT("first") : Mode.ToLower();
    const TSet<FString>* Set = Whitelist.Find(Model);
    return Set && Set->Contains(Effective);
}

static bool IsVelodyneModel(rgl_lidar_model_t M)
{
    return M == RGL_VELODYNE_VLP16
        || M == RGL_VELODYNE_VLP32C
        || M == RGL_VELODYNE_VLS128;
}

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
    const bool bVelocityDistortion = Desc.RglScanTiming == RGLScanTiming::VelocityDistortion();
    RGLScanTiming::SessionDiagnostics TimingDiagnostics(bVelocityDistortion, Session,
        TCHAR_TO_UTF8(*Config.SensorName), TCHAR_TO_UTF8(*Desc.RglLidarModelName));
    Session->Capture = RGLScanTiming::CaptureDiagnostics(Session,
        TCHAR_TO_UTF8(*Config.SensorName), TCHAR_TO_UTF8(*Desc.RglLidarModelName), bVelocityDistortion);
    const auto RejectTiming = [&TimingDiagnostics](const char* Reason) { TimingDiagnostics.Reject(Reason); };
    if (bVelocityDistortion &&
        (!RGLDynLoader::IsVelocityDistortionApiAvailable() || !RGLDynLoader::IsSensorIgnoreApiAvailable()))
    {
        const char* Reason = !RGLDynLoader::IsVelocityDistortionApiAvailable()
            ? "missing_velocity_distortion_api" : "missing_sensor_ignore_api";
        RejectTiming(Reason);
        UE_LOG(LogTemp, Error, TEXT("RGL: session rejected: %s"), UTF8_TO_TCHAR(Reason));
        delete Session;
        return nullptr;
    }
    if (bVelocityDistortion)
    {
        // Rays from inside the vehicle cross many of its ignored faces; libraries that continue
        // with nested traces do not export the limit and cannot pass a whole vehicle.
        using RGLVehicleIgnore::FacesLimitQuery;
        int32_t FacesLimit = 0;
        const FacesLimitQuery Query = !RGLDynLoader::HasIgnoredFacesLimit() ? FacesLimitQuery::Missing
            : RGLDynLoader::GetIgnoredFacesLimit(FacesLimit) ? FacesLimitQuery::Available : FacesLimitQuery::Error;
        if (!RGLVehicleIgnore::AcceptsIgnoredFacesLimit(Query, FacesLimit))
        {
            const std::string Field = RGLVehicleIgnore::IgnoredFacesLimitField(Query, FacesLimit);
            TimingDiagnostics.Reject(RGLVehicleIgnore::IgnoredFacesLimitReason, Field.c_str());
            UE_LOG(LogTemp, Error, TEXT("RGL: session rejected: the loaded RGL library does not let a ray pass enough faces of ignored entities (%s, at least %d required)."),
                UTF8_TO_TCHAR(Field.c_str()), RGLVehicleIgnore::MinIgnoredFacesLimit);
            delete Session;
            return nullptr;
        }
    }
    if (bVelocityDistortion &&
        (!std::isfinite(Desc.RotationFrequency) || Desc.RotationFrequency <= 0 ||
         !std::isfinite(Desc.HorizontalFov) || Desc.HorizontalFov <= 0 || Desc.HorizontalFov > 360))
    {
        RejectTiming("invalid_rotation_frequency_or_fov");
        UE_LOG(LogTemp, Error, TEXT("RGL: invalid velocity_distortion rotation frequency or FoV"));
        delete Session;
        return nullptr;
    }

    if (!World)
    {
        if (bVelocityDistortion) RejectTiming("missing_world");
        UE_LOG(LogTemp, Error, TEXT("RGLBackendImpl: No world available during session creation"));
        delete Session;
        return nullptr;
    }

    if (bVelocityDistortion)
    {
        const double ScanDelta = Config.ScanDeltaSeconds > 0
            ? Config.ScanDeltaSeconds : World->GetDeltaSeconds();
        const auto Steps = RGLVelocityDistortion::ScanSteps(
            Desc.PointsPerSecond, Desc.Channels, ScanDelta, Desc.RotationFrequency);
        const char* Reason = RGLVelocityDistortion::ValidateOutput(
            Desc.UdpEnabled, !Config.Ros2Topic.IsEmpty(), TCHAR_TO_UTF8(*Desc.RglLidarModelName),
            Steps, Desc.RotationFrequency,
            std::fmin(static_cast<double>(Desc.HorizontalFov),
                Desc.RotationFrequency * static_cast<double>(Desc.HorizontalFov) * ScanDelta), Desc.Channels);
        if (Reason)
        {
            RejectTiming(Reason);
            UE_LOG(LogTemp, Error, TEXT("RGL: session rejected: %s [%s] (model=%s, steps=%llu, rotation_hz=%.6f)."),
                UTF8_TO_TCHAR(RGLVelocityDistortion::Describe(Reason)), UTF8_TO_TCHAR(Reason), *Desc.RglLidarModelName,
                static_cast<unsigned long long>(Steps), Desc.RotationFrequency);
            delete Session;
            return nullptr;
        }
    }

    // Do not scan the world here: sensor position is not available yet, and
    // uploading every static mesh at once can exhaust OptiX GPU memory.
    FRGLSceneManager& SceneMgr = FRGLSceneManager::GetInstance(World);
    rgl_scene_t Scene = SceneMgr.GetScene();

    // 1. UseRays node
    rgl_mat3x4f DummyRay = RGLCoord::Identity();
    RGL_CHECK_RETURN(rgl_node_rays_from_mat3x4f(&Session->UseRaysNode, &DummyRay, 1), nullptr);

    if (bVelocityDistortion)
    {
        const auto Offsets = RGLVelocityDistortion::TimeOffsetsMs(1, 1, Desc.HorizontalFov, Desc.RotationFrequency);
        RGL_CHECK_RETURN(rgl_node_rays_set_time_offsets(&Session->SetTimeOffsetsNode, Offsets.data(), 1), nullptr);
    }

    // 1.5. Set ray range (min/max in meters)
    const float MinRangeM = Desc.MinRange * RGLCoord::UE_TO_RGL;
    const float MaxRangeM = Desc.Range * RGLCoord::UE_TO_RGL;
    rgl_vec2f RayRange = {{ MinRangeM, MaxRangeM }};
    RGL_CHECK_RETURN(rgl_node_rays_set_range(&Session->SetRangeNode, &RayRange, 1), nullptr);

    // 1.6. Set ring IDs (channel assignment for each ray)
    int32_t DummyRingId = 0;
    RGL_CHECK_RETURN(rgl_node_rays_set_ring_ids(&Session->SetRingIdsNode, &DummyRingId, 1), nullptr);

    // 2. RaysTransform node
    rgl_mat3x4f SensorTf = RGLCoord::Identity();
    RGL_CHECK_RETURN(rgl_node_rays_transform(&Session->RaysTransformNode, &SensorTf), nullptr);

    // 3. Raytrace node
    RGL_CHECK_RETURN(rgl_node_raytrace(&Session->RaytraceNode, Scene), nullptr);
    if (RGLDynLoader::IsVelocityDistortionApiAvailable())
    {
        RGL_CHECK_RETURN(rgl_node_raytrace_configure_distortion(Session->RaytraceNode, bVelocityDistortion), nullptr);
    }
    RGL_CHECK(rgl_node_raytrace_configure_non_hits(Session->RaytraceNode, 0.0f, MaxRangeM));

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

    // ============================================================================
    // (Optional) UDP Raw Packet publishing branch — AWSIM LidarUdpPublisher port
    // ============================================================================
    const bool bUdpRequested = Desc.UdpEnabled && !Desc.UdpDestIp.IsEmpty();

    if (bUdpRequested)
    {
        // 1. Extension availability (graceful fallback when missing in .so).
        int32_t UdpAvailable = 0;
        rgl_get_extension_info(RGL_EXTENSION_UDP, &UdpAvailable);
        if (!UdpAvailable)
        {
            UE_LOG(LogTemp, Warning,
                TEXT("RGLBackendImpl: UDP publishing requested but RGL_EXTENSION_UDP "
                     "not present in libRobotecGPULidar.so. Rebuild RGL with "
                     "RGL_BUILD_UDP_EXTENSION=ON. Skipping UDP node."));
        }
        else if (Desc.RglLidarModelName.IsEmpty())
        {
            UE_LOG(LogTemp, Warning,
                TEXT("RGLBackendImpl: UDP publishing requested but "
                     "rgl_lidar_model_name is empty. Skipping UDP node."));
        }
        else
        {
            const rgl_lidar_model_t* FoundModel = GLidarModelMap.Find(Desc.RglLidarModelName);
            if (!FoundModel)
            {
                UE_LOG(LogTemp, Warning,
                    TEXT("RGLBackendImpl: Unknown rgl_lidar_model_name '%s'. "
                         "UDP disabled."), *Desc.RglLidarModelName);
            }
            else if (FMath::Abs(Desc.HorizontalFov - 360.0f) > 0.01f)
            {
                UE_LOG(LogTemp, Warning,
                    TEXT("RGLBackendImpl: UDP requires HorizontalFov=360.0 but "
                         "is %.2f. UDP disabled."), Desc.HorizontalFov);
            }
            else if (!IsReturnModeSupportedForModel(Desc.RglLidarModelName, Desc.ReturnMode))
            {
                UE_LOG(LogTemp, Warning,
                    TEXT("RGLBackendImpl: Return mode '%s' not supported by "
                         "model '%s' for UDP. UDP disabled."),
                    *Desc.ReturnMode, *Desc.RglLidarModelName);
            }
            else
            {
                // Velodyne legacy packet format saturates at 262.14 m. Continue with warning.
                if (IsVelodyneModel(*FoundModel)
                    && Desc.Range * RGLCoord::UE_TO_RGL > 262.14f)
                {
                    UE_LOG(LogTemp, Warning,
                        TEXT("RGLBackendImpl: Range %.2fm exceeds Velodyne Legacy "
                             "Format limit 262.14m. Encoded distance may saturate."),
                        Desc.Range * RGLCoord::UE_TO_RGL);
                }

                // Compute udp_options bitmask with AWSIM-style model-specific forcing.
                uint32_t UdpOptions = RGL_UDP_NO_ADDITIONAL_OPTIONS;
                bool EffEnableSeq = Desc.UdpHesaiEnableUdpSequence;
                bool EffBlockage  = Desc.UdpHesaiBlockageDetection;
                bool EffPandarDrv = Desc.UdpHesaiPandarDriverCompat;

                const FString& M = Desc.RglLidarModelName;
                if (M == TEXT("HesaiQT128C2X")
                    || M == TEXT("HesaiPandar128E4X")
                    || M == TEXT("HesaiPandar128E4XHighRes")
                    || M == TEXT("HesaiPandarXT32"))
                {
                    if (!EffEnableSeq)
                    {
                        UE_LOG(LogTemp, Log,
                            TEXT("RGLBackendImpl: enable_hesai_udp_sequence forced ON for '%s'"),
                            *M);
                        EffEnableSeq = true;
                    }
                }
                if (M != TEXT("HesaiQT128C2X") && EffBlockage)
                {
                    UE_LOG(LogTemp, Warning,
                        TEXT("RGLBackendImpl: blockage_detection valid only for "
                             "HesaiQT128C2X. Disabled for '%s'."), *M);
                    EffBlockage = false;
                }
                const bool EffPandarDrvActive = (M == TEXT("HesaiPandarQT")) && EffPandarDrv;

                if (EffEnableSeq)      UdpOptions |= RGL_UDP_ENABLE_HESAI_UDP_SEQUENCE;
                if (EffBlockage)       UdpOptions |= RGL_UDP_UP_CLOSE_BLOCKAGE_DETECTION;
                if (EffPandarDrvActive) UdpOptions |= RGL_UDP_FIT_QT64_TO_HESAI_PANDAR_DRIVER;

                // HesaiPandar128E4XHighRes is the same physical sensor as
                // HesaiPandar128E4X but emits a two-firing-sequence packet that
                // doubles azimuth density (0.2 -> 0.1 deg). AWSIM's
                // LidarUdpPublisher sets this flag purely from the model, so we
                // mirror that — no user-facing attribute is involved.
                if (M == TEXT("HesaiPandar128E4XHighRes"))
                {
                    UdpOptions |= RGL_UDP_HIGH_RESOLUTION_MODE;
                    UE_LOG(LogTemp, Log,
                        TEXT("RGLBackendImpl: high-resolution mode enabled for '%s'"),
                        *M);
                }

                // Both scan timings retain the model's advancing packet times.
                UdpOptions |= RGLScanTiming::UdpOptions(Desc.RglScanTiming);

                const FString EffSrcIp = Desc.UdpSourceIp.IsEmpty()
                    ? TEXT("0.0.0.0") : Desc.UdpSourceIp;

                rgl_status_t S = rgl_node_points_udp_publish(
                    &Session->UdpPublishNode,
                    *FoundModel,
                    static_cast<rgl_udp_options_t>(UdpOptions),
                    TCHAR_TO_UTF8(*EffSrcIp),
                    TCHAR_TO_UTF8(*Desc.UdpDestIp),
                    Desc.UdpDestPort);

                if (S != RGL_SUCCESS)
                {
                    const char* Err = nullptr;
                    rgl_get_last_error_string(&Err);
                    UE_LOG(LogTemp, Warning,
                        TEXT("RGLBackendImpl: rgl_node_points_udp_publish failed: %s"),
                        Err ? *FString(UTF8_TO_TCHAR(Err)) : TEXT("unknown"));
                    Session->UdpPublishNode = nullptr;
                }
                else
                {
                    Session->bUdpActive = true;
                    UE_LOG(LogTemp, Log,
                        TEXT("RGLBackendImpl: UDP publish enabled: model=%s dest=%s:%d opts=0x%08X scan_timing=%s"),
                        *M, *Desc.UdpDestIp, Desc.UdpDestPort, UdpOptions, *Desc.RglScanTiming);
                }
            }
        }
    }

    // --- Noise nodes (conditional) ---
    const FString& AngularType = Desc.NoiseAngularType;
    const float AngStdDevRad = FMath::DegreesToRadians(Desc.NoiseAngularStdDev);
    const float AngMeanRad = FMath::DegreesToRadians(Desc.NoiseAngularMean);

    // Parse rotation axis
    rgl_axis_t RotAxis = RGL_AXIS_Y;  // default
    if (Desc.NoiseAngularAxis.Equals(TEXT("X"), ESearchCase::IgnoreCase))
        RotAxis = RGL_AXIS_X;
    else if (Desc.NoiseAngularAxis.Equals(TEXT("Z"), ESearchCase::IgnoreCase))
        RotAxis = RGL_AXIS_Z;

    if (AngularType.Equals(TEXT("ray"), ESearchCase::IgnoreCase) && AngStdDevRad > 0.0f)
    {
        RGL_CHECK(rgl_node_gaussian_noise_angular_ray(
            &Session->AngularNoiseRayNode, AngMeanRad, AngStdDevRad, RotAxis));
    }

    if (AngularType.Equals(TEXT("hitpoint"), ESearchCase::IgnoreCase) && AngStdDevRad > 0.0f)
    {
        RGL_CHECK(rgl_node_gaussian_noise_angular_hitpoint(
            &Session->AngularNoiseHitpointNode, AngMeanRad, AngStdDevRad, RotAxis));
    }

    if (Desc.NoiseDistanceStdDevBase > 0.0f || Desc.NoiseDistanceStdDevRise > 0.0f)
    {
        RGL_CHECK(rgl_node_gaussian_noise_distance(
            &Session->DistanceNoiseNode,
            Desc.NoiseDistanceMean,
            Desc.NoiseDistanceStdDevBase,
            Desc.NoiseDistanceStdDevRise));
    }

    // --- Beam divergence (configure existing RaytraceNode) ---
    const float HDiv = FMath::DegreesToRadians(Desc.BeamDivergenceH);
    const float VDiv = FMath::DegreesToRadians(Desc.BeamDivergenceV);
    if (HDiv > 0.0f && VDiv > 0.0f)
    {
        RGL_CHECK(rgl_node_raytrace_configure_beam_divergence(
            Session->RaytraceNode, HDiv, VDiv));
        RGLLog::Info("RGLBackendImpl: Beam divergence enabled: H=",
            Desc.BeamDivergenceH, "deg V=", Desc.BeamDivergenceV, "deg");
    }
    else if (!(HDiv == 0.0f && VDiv == 0.0f))
    {
        UE_LOG(LogTemp, Warning,
            TEXT("RGL: beam_divergence_h and beam_divergence_v must both be >0 or both be 0. Ignoring beam divergence."));
    }

    // --- Return mode (configure existing RaytraceNode) ---
    static const TMap<FString, rgl_return_mode_t> ReturnModeMap = {
        {TEXT("first"), RGL_RETURN_FIRST},
        {TEXT("second"), RGL_RETURN_SECOND},
        {TEXT("last"), RGL_RETURN_LAST},
        {TEXT("strongest"), RGL_RETURN_STRONGEST},
        {TEXT("first_last"), RGL_RETURN_FIRST_LAST},
        {TEXT("first_strongest"), RGL_RETURN_FIRST_STRONGEST},
        {TEXT("first_second"), RGL_RETURN_FIRST_SECOND},
        {TEXT("last_strongest"), RGL_RETURN_LAST_STRONGEST},
        {TEXT("strongest_second_strongest"), RGL_RETURN_STRONGEST_SECOND_STRONGEST},
    };

    if (!Desc.ReturnMode.IsEmpty())
    {
        const FString ModeLower = Desc.ReturnMode.ToLower();
        const rgl_return_mode_t* FoundMode = ReturnModeMap.Find(ModeLower);
        if (FoundMode)
        {
            rgl_return_mode_t Mode = *FoundMode;
            // Dual mode requires beam divergence > 0
            const bool bIsDual = (static_cast<int32>(Mode) >> 24) == 2;
            if (bIsDual && !(HDiv > 0.0f && VDiv > 0.0f))
            {
                // Extract first return type and fall back to single
                const rgl_return_type_t FirstType = static_cast<rgl_return_type_t>(
                    static_cast<int32>(Mode) & 0xFF);
                Mode = static_cast<rgl_return_mode_t>(
                    (1 << 24) | static_cast<int32>(FirstType));
                UE_LOG(LogTemp, Warning,
                    TEXT("RGL: Dual return mode '%s' requires beam_divergence > 0. "
                         "Falling back to single return."), *Desc.ReturnMode);
            }
            RGL_CHECK(rgl_node_raytrace_configure_return_mode(
                Session->RaytraceNode, Mode));
            if (Mode != RGL_RETURN_FIRST)
            {
                RGLLog::Info("RGLBackendImpl: Return mode set to:", TCHAR_TO_ANSI(*Desc.ReturnMode));
            }
        }
        else
        {
            UE_LOG(LogTemp, Warning,
                TEXT("RGL: Unknown return_mode '%s'. Using default 'first'."), *Desc.ReturnMode);
        }
    }

    // Connect graph with optional noise nodes:
    // UseRays -> SetRange -> SetRingIds -> [SetTimeOffsets] -> RaysTransform
    //   -> [AngularNoiseRay] -> Raytrace -> [AngularNoiseHitpoint] -> [DistanceNoise]
    //   -> Compact -> ToSensor -> Yield
    RGL_CHECK(rgl_graph_node_add_child(Session->UseRaysNode, Session->SetRangeNode));
    RGL_CHECK(rgl_graph_node_add_child(Session->SetRangeNode, Session->SetRingIdsNode));
    if (Session->SetTimeOffsetsNode)
    {
        RGL_CHECK(rgl_graph_node_add_child(Session->SetRingIdsNode, Session->SetTimeOffsetsNode));
        RGL_CHECK(rgl_graph_node_add_child(Session->SetTimeOffsetsNode, Session->RaysTransformNode));
    }
    else
    {
        RGL_CHECK(rgl_graph_node_add_child(Session->SetRingIdsNode, Session->RaysTransformNode));
    }

    // RaysTransform -> [AngularNoiseRay] -> Raytrace
    if (Session->AngularNoiseRayNode)
    {
        RGL_CHECK(rgl_graph_node_add_child(Session->RaysTransformNode, Session->AngularNoiseRayNode));
        RGL_CHECK(rgl_graph_node_add_child(Session->AngularNoiseRayNode, Session->RaytraceNode));
    }
    else
    {
        RGL_CHECK(rgl_graph_node_add_child(Session->RaysTransformNode, Session->RaytraceNode));
    }

    // Raytrace -> [AngularNoiseHitpoint] -> [DistanceNoise] -> Compact
    rgl_node_t PostRaytraceNode = Session->RaytraceNode;
    if (Session->AngularNoiseHitpointNode)
    {
        RGL_CHECK(rgl_graph_node_add_child(PostRaytraceNode, Session->AngularNoiseHitpointNode));
        PostRaytraceNode = Session->AngularNoiseHitpointNode;
    }
    if (Session->DistanceNoiseNode)
    {
        RGL_CHECK(rgl_graph_node_add_child(PostRaytraceNode, Session->DistanceNoiseNode));
        PostRaytraceNode = Session->DistanceNoiseNode;
    }
    // UDP branch consumes the non-compacted, ray-ordered output (slot order
    // matches firing sequence). DISTANCE is computed at Raytrace and is
    // invariant under transforms, so frame choice doesn't matter for payload.
    if (Session->bUdpActive && Session->UdpPublishNode)
    {
        RGL_CHECK(rgl_graph_node_add_child(PostRaytraceNode, Session->UdpPublishNode));
    }
    RGL_CHECK(rgl_graph_node_add_child(PostRaytraceNode, Session->CompactNode));

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

    // The accepted line follows the first successful capture (Tick), once the vehicle to ignore is
    // known and RGL has run the graph: CARLA attaches a sensor after spawning it, and RGL checks node
    // inputs when the graph runs.
    TimingDiagnostics.Accept();
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

    // Remove this sensor from distance-culling registration so its surrounding
    // geometry can be released. World may already be gone (instance auto-cleaned).
    if (UWorld* World = Session->World.Get())
    {
        FRGLSceneManager::GetInstance(World).UnregisterSensor(static_cast<const void*>(Session));
    }

    // Destroy the entire connected graph via the root node.
    // rgl_graph_destroy destroys all connected nodes in the graph.
    if (Session->UseRaysNode)
    {
        rgl_graph_destroy(Session->UseRaysNode);
    }

    // Clear all handles (they're invalid after graph_destroy)
    Session->UseRaysNode = nullptr;
    Session->SetTimeOffsetsNode = nullptr;
    Session->RaysTransformNode = nullptr;
    Session->RaytraceNode = nullptr;
    Session->CompactNode = nullptr;
    Session->ToSensorNode = nullptr;
    Session->YieldNode = nullptr;
    Session->UdpPublishNode = nullptr;
    Session->bUdpActive = false;

    delete Session;
}

// ============================================================================
// SetSelfCutout
// ============================================================================

void FRGLBackendImpl::SetSelfCutout(FRGLSessionHandle Handle, AActor* Vehicle, float RadiusCm)
{
    if (!Handle)
    {
        return;
    }
    FRGLSession* Session = static_cast<FRGLSession*>(Handle);
    Session->SelfCutoutVehicle = Vehicle;
    Session->SelfCutoutRadiusCm = FMath::Max(0.0f, RadiusCm);
    if (Session->SelfCutoutRadiusCm <= 0.0f)
    {
        return;
    }

    // Vertical field of view of the ray pattern (see GenerateRayPattern): the per-channel angles when
    // given (channels beyond that list fire at 0 degrees), else the uniform [lower_fov, upper_fov].
    const FLidarDescription& Desc = Session->Config.LidarDesc;
    float MinElevation = FMath::Min(Desc.LowerFovLimit, Desc.UpperFovLimit);
    float MaxElevation = FMath::Max(Desc.LowerFovLimit, Desc.UpperFovLimit);
    if (Desc.VerticalAngles.Num() > 0)
    {
        MinElevation = MaxElevation = Desc.VerticalAngles[0];
        for (const float Angle : Desc.VerticalAngles)
        {
            MinElevation = FMath::Min(MinElevation, Angle);
            MaxElevation = FMath::Max(MaxElevation, Angle);
        }
        if (Desc.Channels > static_cast<uint32>(Desc.VerticalAngles.Num()))
        {
            MinElevation = FMath::Min(MinElevation, 0.0f);
            MaxElevation = FMath::Max(MaxElevation, 0.0f);
        }
    }
    Session->SelfCutoutMinElevationDeg = MinElevation;
    Session->SelfCutoutMaxElevationDeg = MaxElevation;

    // One grep-friendly line per change (Shipping compiles RGLLog::Info out, hence stderr there). The
    // sensor key is the session handle, as in the scene manager's rgl_self_cutout line.
    const FString Line = FString::Printf(
        TEXT("rgl_self_cutout_sensor sensor=0x%llx vehicle=%s radius_m=%.3f elevation_deg=%.2f..%.2f"),
        static_cast<unsigned long long>(reinterpret_cast<UPTRINT>(Session)), *GetNameSafe(Vehicle),
        Session->SelfCutoutRadiusCm * RGLCoord::UE_TO_RGL, MinElevation, MaxElevation);
#if UE_BUILD_SHIPPING
    fprintf(stderr, "%s\n", TCHAR_TO_UTF8(*Line));
#else
    RGLLog::Info(TCHAR_TO_UTF8(*Line));
#endif
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

    // Keep velocity-distortion steps identical to session validation while
    // preserving the existing float calculation for firing mode.
    const float UnclampedAngle = Desc.RotationFrequency
        * Desc.HorizontalFov * DeltaSeconds;
    const float AngleRatio = (UnclampedAngle > 0.0f)
        ? (AngleDistanceOfTick / UnclampedAngle) : 1.0f;
    const uint32 PointsToScanWithOneLaser = Desc.RglScanTiming == RGLScanTiming::VelocityDistortion()
        ? static_cast<uint32>(RGLVelocityDistortion::ScanSteps(
            Desc.PointsPerSecond, ChannelCount, DeltaSeconds, Desc.RotationFrequency))
        : FMath::RoundHalfFromZero(
            Desc.PointsPerSecond * DeltaSeconds / static_cast<float>(ChannelCount) * AngleRatio);

    if (PointsToScanWithOneLaser <= 0)
    {
        return 0;
    }

    const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

    // Pre-compute vertical angles and horizontal offsets per channel
    TArray<float> VerticalAngles;
    TArray<float> HorizOffsets;

    if (Desc.VerticalAngles.Num() > 0)
    {
        // Per-channel preset mode: use explicit angles
        VerticalAngles = Desc.VerticalAngles;
        HorizOffsets = Desc.HorizontalAngleOffsets;
    }
    else
    {
        // Legacy uniform mode: distribute evenly between upper/lower FOV
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
    }

    const int32 TotalRays = static_cast<int32>(ChannelCount * PointsToScanWithOneLaser);
    Session->RayTransforms.SetNum(TotalRays);
    Session->RingIds.SetNum(TotalRays);

    // ================================================================
    // Parse ray mask conditions (all ANDed together)
    // ================================================================
    const bool bHasAzMask = !Desc.RayMaskAzimuth.IsEmpty();
    const bool bHasRingMask = !Desc.RayMaskRings.IsEmpty();
    const bool bHasRectMask = !Desc.RayMaskRects.IsEmpty();
    const bool bHasRawMask = Desc.RayMaskRaw.Num() > 0;
    const bool bHasAnyMask = bHasAzMask || bHasRingMask || bHasRectMask || bHasRawMask;

    // Azimuth sections (whitelist)
    struct FAzSec { float S; float E; };
    TArray<FAzSec> AzSections;
    if (bHasAzMask)
    {
        TArray<FString> SecStrs;
        Desc.RayMaskAzimuth.ParseIntoArray(SecStrs, TEXT(";"), true);
        for (const FString& Str : SecStrs)
        {
            TArray<FString> P;
            Str.ParseIntoArray(P, TEXT(","), true);
            if (P.Num() == 2 && AzSections.Num() < 5)
            {
                float S = FCString::Atof(*P[0]), E = FCString::Atof(*P[1]);
                if (S >= 0.f && S <= 360.f && E >= 0.f && E <= 360.f)
                    AzSections.Add({S, E});
                else
                    UE_LOG(LogTemp, Warning, TEXT("RGL mask: invalid azimuth range: %s"), *Str);
            }
            else
            {
                UE_LOG(LogTemp, Warning, TEXT("RGL mask: malformed azimuth section: %s"), *Str);
            }
        }
    }

    // Disabled rings (blacklist)
    TSet<int32> DisabledRings;
    if (bHasRingMask)
    {
        TArray<FString> RStrs;
        Desc.RayMaskRings.ParseIntoArray(RStrs, TEXT(","), true);
        for (const FString& R : RStrs)
        {
            int32 Id = FCString::Atoi(*R.TrimStartAndEnd());
            if (Id >= 0) DisabledRings.Add(Id);
            else UE_LOG(LogTemp, Warning, TEXT("RGL mask: invalid ring ID: %s"), *R);
        }
    }

    // Rectangular masks (blacklist)
    struct FMaskRect { float AzS; float AzE; float ElS; float ElE; };
    TArray<FMaskRect> MaskRects;
    if (bHasRectMask)
    {
        TArray<FString> RStrs;
        Desc.RayMaskRects.ParseIntoArray(RStrs, TEXT(";"), true);
        for (const FString& Str : RStrs)
        {
            TArray<FString> P;
            Str.ParseIntoArray(P, TEXT(","), true);
            if (P.Num() == 4)
                MaskRects.Add({FCString::Atof(*P[0]), FCString::Atof(*P[1]),
                               FCString::Atof(*P[2]), FCString::Atof(*P[3])});
            else
                UE_LOG(LogTemp, Warning, TEXT("RGL mask: malformed rect (need 4 values): %s"), *Str);
        }
    }

    if (bHasAnyMask)
    {
        Session->RayMask.SetNum(TotalRays);
        FMemory::Memset(Session->RayMask.GetData(), 1, TotalRays);
    }

    // Compute sweep window offset. When HorizontalStartAngle is non-zero,
    // shift the centred [-HFOV/2, +HFOV/2] sweep so that it spans
    // [HorizontalStartAngle, HorizontalStartAngle + HFOV] instead.
    const float SweepCenterOffset =
        (Desc.HorizontalStartAngle != 0.0f)
            ? (Desc.HorizontalStartAngle + Desc.HorizontalFov / 2.0f)
            : 0.0f;

    // Ray array layout MUST be azimuth-major: all channels of one horizontal
    // step are contiguous, then the step advances. This matches AWSIM
    // RGLUnityPlugin (LidarConfiguration.GetRayPoses: idx = laser + hStep*numLasers)
    // and the RGL UDP extension, which derives each packet point's azimuth+channel
    // purely from the array index (firing sequence). The loop nests channel-inside
    // but writes to RayIndex = pt*ChannelCount + ch so the stored layout is
    // azimuth-major. The ROS2-direct path carries per-point azimuth and ring_id, so
    // it is order-independent and unaffected by this layout.
    for (uint32 ch = 0; ch < ChannelCount; ++ch)
    {
        const float ChVertAngle = (ch < static_cast<uint32>(VerticalAngles.Num()))
            ? VerticalAngles[ch] : 0.0f;
        const float ChHorizOffset = (ch < static_cast<uint32>(HorizOffsets.Num()))
            ? HorizOffsets[ch] : 0.0f;
        const float ChStepOffset = (ch < static_cast<uint32>(Desc.HorizontalStepOffsets.Num()))
            ? Desc.HorizontalStepOffsets[ch] : 0.0f;
        const int32 ChRingId = (ch < static_cast<uint32>(Desc.RingIds.Num()))
            ? Desc.RingIds[ch] : static_cast<int32>(ch);

        for (uint32 pt = 0; pt < PointsToScanWithOneLaser; ++pt)
        {
            // Azimuth-major position: horizontal step pt holds all channels contiguously.
            const int32 RayIndex = static_cast<int32>(pt) * static_cast<int32>(ChannelCount)
                                 + static_cast<int32>(ch);

            const float HorizAngle =
                std::fmod(
                    InOutHorizontalAngle + static_cast<float>(pt) * AngleDistanceOfLaserMeasure,
                    Desc.HorizontalFov)
                - Desc.HorizontalFov / 2.0f
                + SweepCenterOffset
                + ChHorizOffset + ChStepOffset;

            Session->RayTransforms[RayIndex] = RGLCoord::FromPitchYaw(
                ChVertAngle,
                HorizAngle
            );
            Session->RingIds[RayIndex] = ChRingId;

            // Apply ray mask conditions (AND logic)
            if (bHasAnyMask)
            {
                int8& Mask = Session->RayMask[RayIndex];
                const float NormAz = FMath::Fmod(HorizAngle + 360.0f, 360.0f);

                // 1. Azimuth whitelist
                if (Mask && AzSections.Num() > 0)
                {
                    bool bIn = false;
                    for (const auto& Sec : AzSections)
                    {
                        if (Sec.S <= Sec.E ? (NormAz >= Sec.S && NormAz <= Sec.E)
                                           : (NormAz >= Sec.S || NormAz <= Sec.E))
                        { bIn = true; break; }
                    }
                    if (!bIn) Mask = 0;
                }
                // 2. Ring blacklist
                if (Mask && DisabledRings.Num() > 0 && DisabledRings.Contains(ChRingId))
                    Mask = 0;
                // 3. Rectangular blacklist
                if (Mask && MaskRects.Num() > 0)
                {
                    for (const auto& R : MaskRects)
                    {
                        bool bAz = R.AzS <= R.AzE
                            ? (NormAz >= R.AzS && NormAz <= R.AzE)
                            : (NormAz >= R.AzS || NormAz <= R.AzE);
                        if (bAz && ChVertAngle >= R.ElS && ChVertAngle <= R.ElE)
                        { Mask = 0; break; }
                    }
                }
                // 4. Raw per-channel mask
                if (Mask && bHasRawMask && ch < static_cast<uint32>(Desc.RayMaskRaw.Num())
                    && Desc.RayMaskRaw[ch] == 0)
                    Mask = 0;
            }
        }
    }

    // Clear mask if no mask conditions were active
    if (!bHasAnyMask)
    {
        Session->RayMask.Empty();
    }

    // Generate per-ray range array if periodic range pattern is defined.
    // Each channel cycles through (RangePatternPeriod) range pairs across horizontal steps.
    // This is data-driven: model-specific knowledge lives in Python presets, not here.
    if (Desc.PerChannelMinRanges.Num() > 0 && Desc.RangePatternPeriod > 0)
    {
        Session->RayRanges.SetNum(TotalRays);
        const int32 Period = Desc.RangePatternPeriod;
        const int32 MaxPatIdx = Desc.PerChannelMinRanges.Num() - 1;
        for (uint32 ch = 0; ch < ChannelCount; ++ch)
        {
            for (uint32 pt = 0; pt < PointsToScanWithOneLaser; ++pt)
            {
                // Azimuth-major layout — must match the ray-pattern loop above.
                const int32 Idx = static_cast<int32>(pt) * static_cast<int32>(ChannelCount)
                                + static_cast<int32>(ch);
                const int32 PatIdx = FMath::Min(
                    static_cast<int32>(ch) * Period + static_cast<int32>(pt % Period),
                    MaxPatIdx);
                Session->RayRanges[Idx] = {{
                    Desc.PerChannelMinRanges[PatIdx],
                    Desc.PerChannelMaxRanges[PatIdx]
                }};
            }
        }
    }
    else
    {
        Session->RayRanges.Empty();
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
    FRGLSceneManager* SceneManager = nullptr;
    const FLidarDescription& Desc = Session->Config.LidarDesc;
    const bool bVelocityDistortion = Desc.RglScanTiming == RGLScanTiming::VelocityDistortion();
    if (World)
    {
        FRGLSceneManager& SceneMgr = FRGLSceneManager::GetInstance(World);
        SceneManager = &SceneMgr;
        if (!SceneMgr.IsSceneTimeUsable())
        {
            Session->Capture.Stopped("scene_time_disabled");
            return Result;
        }
        // Keep the static scene registration close to the actual LiDAR range.
        // Large city maps contain thousands of static meshes; forcing a 300 m minimum
        // can exhaust OptiX memory before the first scan.
        const float RegistrationDistanceCm = FMath::Max(5000.0f, Desc.Range + 2000.0f);
        // SensorId = session handle (stable for the sensor's lifetime). Per-sensor
        // registration lets spatially separated sensors share one scene via union.
        SceneMgr.RegisterSensor(
            static_cast<const void*>(Session),
            SensorWorldTransform.GetLocation(),
            RegistrationDistanceCm,
            SimulationTime);
        if (Session->SelfCutoutRadiusCm > 0.0f)
        {
            SceneMgr.SetSensorSelfCutout(
                static_cast<const void*>(Session),
                Session->SelfCutoutVehicle.Get(),
                Session->SelfCutoutRadiusCm,
                SensorWorldTransform,
                Session->SelfCutoutMinElevationDeg,
                Session->SelfCutoutMaxElevationDeg);
        }
        if (bVelocityDistortion)
        {
            AActor* Vehicle = Session->SelfCutoutVehicle.Get();
            const int32 Id = SceneMgr.SetVelocityDistortionVehicle(Session, Vehicle);
            if (IsValid(Vehicle) && Id == 0)
            {
                Session->Capture.Stopped("sensor_id_exhausted");
                return Result;
            }
            // Zero explicitly restores default visibility after detachment.
            if (Session->RaytraceSensorId != Id)
            {
                const rgl_status_t IdStatus = rgl_node_raytrace_configure_id(Session->RaytraceNode, Id);
                if (IdStatus != RGL_SUCCESS)
                {
                    RGL_CHECK(IdStatus);
                    Session->Capture.Stopped("raytrace_id_failed");
                    return Result;
                }
                Session->RaytraceSensorId = Id;
            }
        }
        SceneMgr.Update(World, SimulationTime);
        // Only velocity-distortion captures depend on the vehicle ignore ids; a failed
        // refresh is retried in the next frame.
        const bool bIgnoresApplied = SceneMgr.RefreshVehicleIgnores();
        if (bVelocityDistortion && !bIgnoresApplied)
        {
            Session->Capture.Stopped("vehicle_ignore_refresh_failed");
            return Result;
        }
    }
    if (bVelocityDistortion && !SceneManager) return Result;

    // From here an RGL failure ends the capture and writes its reason to stdout, once per reason
    // (also in Shipping builds).
    const auto Stop = [Session, &Result](const char* Reason) { Session->Capture.Stopped(Reason); return Result; };

    // 2. Ray pattern: generate once, then cache (same DeltaSeconds = same pattern)
    if (!Session->bRayPatternCached)
    {
        const int32 TotalRays = GenerateRayPattern(Session, EffectiveDelta, InOutHorizontalAngle);
        if (TotalRays <= 0)
        {
            return Result;
        }

        if (bVelocityDistortion)
        {
            // EffectiveDelta can differ from the configured interval (sensor_tick=0).
            // Reject before publishing if the actual pattern is incompatible.
            const char* Reason = RGLVelocityDistortion::ValidateOutput(
                Desc.UdpEnabled, !Session->Config.Ros2Topic.IsEmpty(), TCHAR_TO_UTF8(*Desc.RglLidarModelName),
                static_cast<std::size_t>(TotalRays) / Desc.Channels,
                Desc.RotationFrequency,
                std::fmin(static_cast<double>(Desc.HorizontalFov),
                    Desc.RotationFrequency * static_cast<double>(Desc.HorizontalFov) * EffectiveDelta), Desc.Channels);
            if (Reason)
            {
                // Re-checked every tick (the frame delta may change), reported once.
                if (Session->Capture.Rejected(Reason))
                {
                    UE_LOG(LogTemp, Error, TEXT("RGL: capture rejected: %s [%s]"),
                        UTF8_TO_TCHAR(RGLVelocityDistortion::Describe(Reason)), UTF8_TO_TCHAR(Reason));
                }
                return Result;
            }
        }

        RGL_CHECK_RETURN(rgl_node_rays_from_mat3x4f(
            &Session->UseRaysNode,
            Session->RayTransforms.GetData(),
            TotalRays), Stop("ray_pattern_failed"));

        if (bVelocityDistortion)
        {
            const auto Offsets = RGLVelocityDistortion::TimeOffsetsMs(
                static_cast<std::size_t>(TotalRays) / Desc.Channels, Desc.Channels,
                Desc.HorizontalFov, Desc.RotationFrequency,
                Desc.UdpEnabled ? TCHAR_TO_UTF8(*Desc.RglLidarModelName) : nullptr);
            RGL_CHECK_RETURN(rgl_node_rays_set_time_offsets(
                &Session->SetTimeOffsetsNode, Offsets.data(), TotalRays), Stop("ray_time_offsets_failed"));
        }

        RGL_CHECK(rgl_node_rays_set_ring_ids(
            &Session->SetRingIdsNode,
            Session->RingIds.GetData(),
            TotalRays));

        // Update per-ray ranges if periodic range pattern is active
        if (Session->RayRanges.Num() == TotalRays)
        {
            RGL_CHECK(rgl_node_rays_set_range(
                &Session->SetRangeNode,
                Session->RayRanges.GetData(),
                TotalRays));
        }

        // Apply ray mask
        if (Session->RayMask.Num() == TotalRays)
        {
            RGL_CHECK(rgl_node_raytrace_configure_mask(
                Session->RaytraceNode,
                Session->RayMask.GetData(),
                TotalRays));
        }

        Session->CachedTotalRays = TotalRays;
        Session->bRayPatternCached = true;
        if (bVelocityDistortion)
        {
            // For the accepted line: step_us is the model interval with UDP.
            const std::size_t Steps = Desc.Channels ? static_cast<std::size_t>(TotalRays) / Desc.Channels : 0;
            const double StepUs = Desc.UdpEnabled
                ? RGLVelocityDistortion::ModelStepSeconds(TCHAR_TO_UTF8(*Desc.RglLidarModelName)) * 1e6
                : (Steps ? 1e6 / Desc.RotationFrequency / Steps * Desc.HorizontalFov / 360.0 : 0.0);
            Session->Capture.SetRayPattern(Steps, StepUs);
        }
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

    const uint64 FrameTimeNs = SceneManager ? SceneManager->GetSceneTimeNs() : 0;
    uint64 PrevFrameTimeNs = 0;
    const bool bHasPreviousFrame = SceneManager && SceneManager->GetPrevSceneTimeNs(PrevFrameTimeNs);
    const bool bShiftScanTime = bVelocityDistortion && SceneManager;
    uint64 ScanTimeNs = FrameTimeNs;
    if (bVelocityDistortion)
    {
        const double PeriodSeconds = 1.0 / Desc.RotationFrequency;
        const double DeltaSeconds = FrameTimeNs >= Session->PreviousCaptureTimeNs
            ? static_cast<double>(FrameTimeNs - Session->PreviousCaptureTimeNs) * 1e-9
            : -static_cast<double>(Session->PreviousCaptureTimeNs - FrameTimeNs) * 1e-9;
        const auto Timing = RGLVelocityDistortion::CaptureTiming(FrameTimeNs, Desc.RotationFrequency);
        auto Velocity = RGLVelocityDistortion::EstimateVelocity(
            SensorTf.value, Session->bHasPreviousCapture ? Session->PreviousSensorTf.value : nullptr,
            DeltaSeconds, PeriodSeconds);
        // Before one full rotation, keep all points at the current pose while
        // the packet epoch remains clamped to zero.
        if (Timing.ZeroVelocity) Velocity = RGLVelocityDistortion::Velocity{};
        if (Velocity.State != RGLVelocityDistortion::Status::Valid && !Session->bVelocityFallbackLogged)
        {
            UE_LOG(LogTemp, Log, TEXT("RGL: velocity_distortion uses zero velocity for missing or invalid capture history (reason=%d, dt=%.6f s); logged once per sensor."),
                static_cast<int>(Velocity.State), DeltaSeconds);
            Session->bVelocityFallbackLogged = true;
        }
        const rgl_vec3f Linear = {{Velocity.Linear[0], Velocity.Linear[1], Velocity.Linear[2]}};
        const rgl_vec3f Angular = {{Velocity.Angular[0], Velocity.Angular[1], Velocity.Angular[2]}};
        RGL_CHECK_RETURN(rgl_node_raytrace_configure_velocity(Session->RaytraceNode, &Linear, &Angular),
            Stop("velocity_failed"));
        ScanTimeNs = Timing.ScanTimeNs;
    }

    // 5. Run RGL graph (GPU raytrace + optional ROS2 publish)
    const double T0 = FPlatformTime::Seconds();
    bool bRestoreFailed = false;
    bool bScanTimeSet = false;
    bool bGraphSucceeded = false;
    {
        // Restore frame time on all exits, including graph errors. Do not update
        // entity transforms or skeletal poses while the packet epoch is shifted.
        FScopedRGLSceneTime ScanTime(bShiftScanTime,
            SceneManager ? SceneManager->GetScene() : nullptr, PrevFrameTimeNs, ScanTimeNs, FrameTimeNs,
            bHasPreviousFrame, &bRestoreFailed);
        bScanTimeSet = ScanTime.IsValid();
        if (bScanTimeSet)
        {
            const rgl_status_t RunStatus = rgl_graph_run(Session->RaytraceNode);
            bGraphSucceeded = RunStatus == RGL_SUCCESS;
            if (!bGraphSucceeded)
            {
                const char* Error = nullptr;
                rgl_get_last_error_string(&Error);
                RGLLog::Info("[RGL] rgl_graph_run failed:", Error ? Error : "unknown");
            }
        }
    }
    if (bRestoreFailed)
    {
        // Stop other sensors from using the damaged shared time state too.
        if (SceneManager) SceneManager->InvalidateSceneTime();
        Session->Capture.Stopped("scene_time_restore_failed");
        UE_LOG(LogTemp, Error, TEXT("RGL: scene time restoration failed after four attempts; capture history is unchanged and this scene is disabled until recreated."));
        return Result;
    }
    if (!bScanTimeSet) return Stop("scan_time_failed");
    if (!bGraphSucceeded) return Stop("graph_run_failed");
    // The accepted line (velocity_distortion) follows the first successful run: RGL checks node
    // inputs when the graph runs, and the vehicle is known once the sensor ticks attached.
    const double T1 = FPlatformTime::Seconds();
    Session->Capture.Captured(Session->RaytraceSensorId);
    if (bVelocityDistortion)
    {
        Session->PreviousSensorTf = SensorTf;
        Session->PreviousCaptureTimeNs = FrameTimeNs;
        Session->bHasPreviousCapture = true;
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
