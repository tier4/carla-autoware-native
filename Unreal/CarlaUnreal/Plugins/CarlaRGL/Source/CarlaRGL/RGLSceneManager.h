// Copyright (c) 2026 RGL Integration for CARLA.
// Scene synchronization manager: extracts UE5 mesh geometry and
// uploads it to the RobotecGPULidar (RGL) scene for GPU ray tracing.
//
// Design:
//   - Singleton per UWorld (accessed via GetInstance)
//   - Caches static mesh asset -> rgl_mesh_t to avoid re-uploading identical meshes, keyed by the asset's identity
//     (FObjectKey: after garbage collection, another asset at the same address has another key)
//   - Creates rgl_entity_t for each UStaticMeshComponent instance
//   - Updates entity transforms each tick; drops entities whose component left the world (unregistered) at once
//   - Handles actor creation/destruction via periodic world scans (every rgl.Scene.SyncIntervalSeconds of simulation time
//     by default; rgl.Scene.SyncClock = 0 counts the scene Updates' world dt instead), plus a scan when a level with static
//     geometry in some sensor's sphere is added (World Partition cells, level streaming). One actor scan per sync
//     serves the static and the skeletal paths; the sky-name verdict is remembered per mesh
//     (the rgl_scene_sync line splits the sync's time: scan_ms, reg_ms, remove_ms, skel_ms)
//   - Amortizes new-mesh creation across scans (rgl.Scene.* CVars): a budgeted, nearest-first
//     create queue, an unload radius wider than the load radius, and an LRU of idle meshes
//   - Re-evaluates the instances of Static instanced static meshes against the sensor spheres after each scan
//     (on the following Updates, capped per Update), instead of choosing them once at registration
//   - On World Partition maps, warns once per sensor whose range stays not streamed in (the scene follows the
//     loaded world; what is not loaded is not in it)
//   - On World Partition maps only (rgl.Scene.WpMovableCulling), non-Static static mesh parts that are not vehicle,
//     walker, physics or sensor-carrier parts and have not been seen moving are chosen against the union of the sensor
//     spheres with a speed-dependent margin, per ISM instance, and created through a budgeted queue with a deadline.
//     Other maps keep the original rule (every non-Static part is registered at any distance).

#pragma once

#ifdef WITH_RGL

#include <util/disable-ue4-macros.h>
#include <rgl/api/core.h>
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "UObject/ObjectKey.h"                 // FObjectKey: generation-aware object identity
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SkinnedMeshComponent.h"   // EVisibilityBasedAnimTickOption
#include "Containers/BitArray.h"
#include "Math/Sphere.h"
#include "HAL/IConsoleManager.h"               // EConsoleVariableFlags (ApplySceneSwitches)
#include "Misc/Optional.h"
#include <util/ue-header-guard-end.h>

#include "RGLSelfCutout.h"

// Forward declarations
class UWorld;
class ULevel;
class AActor;
class USkeletalMeshComponent;
class USkeletalMesh;

/// Result of FRGLSceneManager::AuditScene: how the RGL static scene differs from the world UE has loaded.
/// Counts are RGL entities (an ISM instance is one entity) unless noted. A "visible level" is
/// bIsVisible && !bIsBeingRemoved; components of a level being added or removed are counted nowhere.
struct FRGLSceneAuditResult
{
    int32 Ghost = 0;             // registered, but the component is gone, unregistered, its level hidden, or the ISM instance removed
    int32 GhostInRange = 0;      // ghosts inside some sensor's range sphere (current position)
    int32 MissingSync = 0;       // eligible, inside R around a sensor's position at the last sync, not registered
    int32 MissingInRange = 0;    // eligible, inside some sensor's range around its current position, not registered
    int32 MeshMismatch = 0;      // the component's static mesh is not the asset the entity was built from
    int32 StaleInstance = 0;     // ISM instance whose world transform changed since registration
    int32 ExcludedByName = 0;    // left out by the name rule (sky domes; see GateStaticComponent), inside the spheres above
    int32 Unextractable = 0;     // left out while a failed extraction/upload waits for its retry (inside the spheres)
    int32 NonSmCollision = 0;    // components (not static mesh, skeletal mesh or Landscape) that block ray_cast's channel, inside the spheres
    int32 HlodRegistered = 0;    // registered entities owned by a World Partition HLOD actor
    int32 Registered = 0;        // static entities in the RGL scene
    int32 Pending = 0;           // components waiting in the create queue
    int32 MovableRegistered = 0; // registered entities whose component is not Static (always registered, any distance)
    int32 LevelsAdded = 0;       // FWorldDelegates::LevelAddedToWorld events since the previous audit line
    double SecondsSinceSync = 0.0; // simulation seconds since the last static sync (-1 = none yet)
    double FrameMs = 0.0;        // wall time between the last two scene Updates
    double AuditMs = 0.0;        // wall time of this audit
    int32 StreamCovered = -1;    // -1 = not a World Partition world; 1 = every sensor's range sphere is streamed in (Activated)
    float ParentStreamRadiusM = -1.0f; // smallest streaming-source radius on the root actor a sensor is attached to; -1 = none
    int32 MissingIsmSync = 0;    // the ISM-instance part of MissingSync (the rest are plain static mesh components)
    int32 MissingIsmInRange = 0; // the ISM-instance part of MissingInRange
    int32 GhostIsm = 0;          // the ISM-instance part of Ghost
    int32 GhostIsmInRange = 0;   // the ISM-instance part of GhostInRange
    int32 MissingSteady = 0;     // eligible, not registered, inside R around a sensor's position at the last sync AND inside
                                 // R around a sensor's current position (ISM: per instance): what the last sync should
                                 // have registered and is still needed, so not explained by sync timing or movement
    int32 SplineRegistered = 0;  // registered entities whose component is a USplineMeshComponent (uploaded undeformed)
    int32 CacheStale = 0;        // MeshCache entries whose key no longer resolves (the asset was collected or is garbage)
    int32 PendingIsm = 0;        // Static ISM groups waiting for their instances to be re-evaluated against the sensor spheres
    int32 MissingWpCulledInRange = 0; // World Partition culling of non-Static parts: a part (ISM: instance) that an earlier
                                 // sync already looked at with the distance rule, inside some sensor's range now, without
                                 // an entity (pending, partial or being promoted counts too); part of MissingInRange
    int32 WpPending = 0;         // jobs in the culling's queue (current); a line with jobs waiting is not steady (D13)
};

/// Optional output of FRGLSceneManager::AuditScene: the name rule over the whole loaded world (every registered static
/// mesh component of a visible level that the rule looks at, whatever its distance to the sensors). Counts are
/// components (an ISM component counts once). Each list holds distinct mesh names, at most MaxListed of them.
struct FRGLSceneAuditExtras
{
    static constexpr int32 MaxListed = 100;
    TArray<FString> ExcludedMeshes;    // meshes the name rule leaves out
    TArray<FString> LegacyOnlyMeshes;  // meshes only the old rule ("sky" in the owner, component or mesh name) left out
    TArray<FString> HugeMeshes;        // meshes of components whose bounds radius exceeds 2 km, whatever their names
    int32 ExcludedWorld = 0;           // components the name rule leaves out
    int32 ExcludedLegacyWorld = 0;     // components the old rule left out
};

/// Manages synchronization between the UE5 scene and the RGL scene.
/// Call Update() each tick before running the RGL raytrace graph.
class FRGLSceneManager
{
public:
    ~FRGLSceneManager();

    /// Get the singleton instance for the given world.
    /// Creates the instance on first call. Cleans up stale instances.
    static FRGLSceneManager& GetInstance(UWorld* World);

    /// Destroy the singleton instance (call on world teardown).
    static void DestroyInstance(UWorld* World);

    /// Destroy all singleton instances across all worlds (call on module shutdown).
    static void DestroyAllInstances();

    /// Get the RGL scene handle. Pass to rgl_node_raytrace().
    /// Returns nullptr which RGL treats as the default scene.
    rgl_scene_t GetScene() const { return Scene; }

    /// Synchronize UE5 world geometry with RGL scene.
    /// Should be called each frame before raytrace.
    /// @param SimulationTime Elapsed simulation time in seconds (from episode start).
    ///        Used for RGL scene time and ROS2 timestamp synchronization.
    ///        Pass -1.0 to fall back to World->GetTimeSeconds().
    void Update(UWorld* World, double SimulationTime = -1.0);

    /// Update ground plane position to follow the sensor.
    /// Uses a UE5 line trace to detect actual ground level.
    void UpdateGroundPlane(UWorld* World, const FTransform& SensorTransform);

    /// Register or refresh a sensor's culling sphere. Call each tick before Update().
    /// SensorId must be stable for the sensor's lifetime (use the session handle).
    /// Now = current simulation time (seconds), used for stale-sensor eviction.
    void RegisterSensor(const void* SensorId, const FVector& Position, float DistanceCm, double Now)
    {
        FSensorRegistration& S = RegisteredSensors.FindOrAdd(SensorId);
        // Pose history (read only by the World Partition culling of non-Static parts: refresh gap and speed).
        if (S.bPoseValid)
        {
            const double Gap = Now - S.LastUpdateTime;
            if (Gap > 0.0)
            {
                S.LastRefreshGapSeconds = Gap;
                S.ObservedSpeedCmps = static_cast<float>(FVector::Dist(Position, S.Position) / Gap);
                S.bRefreshed = true;
            }
            else if (Gap < 0.0)
            {
                S.bTimeRewound = true;
                S.ObservedSpeedCmps = 0.0f;
            }
        }
        S.bPoseValid = true;
        S.Position = Position;
        S.RegistrationDistanceCm = FMath::Max(5000.0f, DistanceCm);
        S.LastUpdateTime = Now;
    }

    /// Self cutout (rgl_self_cutout_radius) of a registered sensor: the triangles of Vehicle's skeletal
    /// meshes within RadiusCm of the sensor and inside [MinElevationDeg, MaxElevationDeg] (its vertical
    /// field of view, sensor frame) are left out of the RGL scene. No sensor sees them, including
    /// sensors on other vehicles, so the radius should stay about the size of the sensor's housing.
    /// Call each tick after RegisterSensor() and before Update(). Sensors that never call it, and
    /// RadiusCm <= 0, keep the default path.
    void SetSensorSelfCutout(const void* SensorId, AActor* Vehicle, float RadiusCm, const FTransform& SensorWorldTransform,
                             float MinElevationDeg, float MaxElevationDeg);

    /// Set the periodic sync interval: writes rgl.Scene.SyncIntervalSeconds with ECVF_SetByCode (seconds of the clock
    /// rgl.Scene.SyncClock chooses). Default: 1.0 s. Minimum: 0.1 s. The CVar is process-wide: every world's manager
    /// reads it at each Update.
    void SetSyncInterval(float Seconds);

    /// The effective values of the rgl.Scene.* knobs (after their range checks), as the rgl_scene_config line prints them.
    struct FSceneKnobs
    {
        int32 MaxMeshCreatesPerUpdate = 8;
        float CreateBudgetMs = 4.0f;
        float UnloadDistanceFactor = 1.0f;
        int32 IdleMeshCacheMax = 0;
        int32 MaxIsmInstanceChangesPerUpdate = 512;
        float UnloadMarginCm = 2000.0f;     // not a CVar: the manager's unload margin (LogSceneConfigIfChanged fills it in)
        int32 SyncClock = 1;                // rgl.Scene.SyncClock: 1 = simulation time, 0 = the scene Updates' world dt
        float SyncIntervalSeconds = 1.0f;   // rgl.Scene.SyncIntervalSeconds (>= 0.1)
        int32 WpMap = 0;                    // not a CVar: the culling applies to this world (LogSceneConfigIfChanged fills it in)
        int32 WpMovableCulling = 1;
        float MovableMinMarginM = 20.0f;
        float MovableSensorSpeedBoundMps = 50.0f;
        float MovableObjectSpeedBoundMps = 50.0f;
        float MovableMaxUpdateGapSeconds = 0.1f;
        float MovableCreateLatencySeconds = 1.0f;
        int32 MaxMovableEntityChangesPerUpdate = 512;
        float MovableEntityBudgetMs = 4.0f;
        bool operator==(const FSceneKnobs& O) const;
        bool operator!=(const FSceneKnobs& O) const { return !(*this == O); }
    };
    /// The rgl.Scene.* CVars through their range checks (an invalid value: its default, with one warning line).
    static FSceneKnobs ReadSceneKnobs();
    /// The rgl_scene_config line (one grep-friendly key=value line): the knobs at SimTime; sync_clock=sim|update.
    static FString FormatSceneConfigLine(double SimTime, const FSceneKnobs& K);

    /// The Shipping-safe -rgl-scene-<knob>= switches (and -rgl-debug-hit-dump=, -rgl-debug-hit-dump-every=), read from
    /// CommandLine: each switch present sets its rgl.* CVar with SetBy; an absent one leaves its CVar alone. The first
    /// manager calls it once with the process command line (ECVF_SetByCommandline, below the code and console).
    static void ApplySceneSwitches(const TCHAR* CommandLine, EConsoleVariableFlags SetBy = ECVF_SetByCommandline);

    /// Check if the scene manager has been initialized.
    bool IsInitialized() const { return bInitialized; }

    /// Remove a sensor when its session is destroyed (defined in .cpp; tears skeletal state down when none remain).
    void UnregisterSensor(const void* SensorId);

    // ---- World Partition culling of non-Static parts (rgl.Scene.WpMovableCulling; README_RGL "non-Static parts") ----

    /// The clock the periodic sync runs on (rgl.Scene.SyncClock): the Update clock adds the world dt once per scene Update
    /// (0, the original timer), the simulation clock follows the simulation time (1, the default).
    enum class EWpCullClock : uint8 { UpdateClock, SimClock };
    /// How long a part chosen out by distance may go without being looked at again and registered (seconds, sim).
    struct FWpCullHorizon
    {
        double SyncBoundSeconds = 0.0;   // T_sync_max: the latest next sync after a culling sync
        double HorizonSeconds = 0.0;     // H = T_sync_max + L_create + U
    };
    /// Update clock: T_sync_max = ceil(I / d) * max(U, d) + U; simulation clock: I + U. H = T_sync_max + L_create + U.
    /// I = the sync interval the trigger uses (rgl.Scene.SyncIntervalSeconds), d = the world dt,
    /// U = rgl.Scene.MovableMaxUpdateGapSeconds, L_create = rgl.Scene.MovableCreateLatencySeconds. A dt that is not
    /// positive and finite is taken as U. The culling sync passes the clock rgl.Scene.SyncClock chooses.
    static FWpCullHorizon ComputeWpCullHorizon(EWpCullClock Clock, double IntervalSeconds, double WorldDtSeconds,
                                               double MaxUpdateGapSeconds, double CreateLatencySeconds);
    /// M = max(MinMarginM, (V_sensor + V_object) * H), metres.
    static double ComputeWpMarginM(double HorizonSeconds, double SensorSpeedMps, double ObjectSpeedMps, double MinMarginM);

    /// The culling applies: rgl.Scene.WpMovableCulling = 1 and World has a World Partition (the test seam
    /// WpMapOverride_ForTest replaces the second part only).
    bool UsesWpNonStaticCulling(const UWorld* World) const;

    /// Why a non-Static static mesh component keeps its original registration (any distance) on a World Partition map.
    /// None = the distance rule chooses it. Looked up through the owner and attachment chain: a Pawn, an actor with a
    /// movement component or a skeletal mesh component, a part (or parent) simulating physics, a sensor's carrier.
    enum class EWpProtect : uint8 { None, Pawn, MovementComponent, Skeletal, Physics, SensorParent };
    /// bRemember = false (the read-only audit): a verdict is not added to the sticky set (a part once protected stays so).
    EWpProtect ClassifyWpProtection(const UStaticMeshComponent* Component, bool bRemember = true) const;
    static const TCHAR* WpProtectName(EWpProtect Reason);

    /// Wall time of the first RGL API call made between Arm and Disarm (wait_ms= / wait_api= of the sync line): the
    /// call that waits for the previous raytrace graph, plus its own work. Later calls are not timed.
    struct FRGLFirstCallProbe
    {
        double (*Clock)() = &FPlatformTime::Seconds;
        bool bArmed = false;
        double Ms = 0.0;
        const TCHAR* Api = TEXT("none");
        void Arm() { bArmed = true; Ms = 0.0; Api = TEXT("none"); }
        void Disarm() { bArmed = false; }
        template <class TFn>
        auto Call(const TCHAR* InApi, TFn&& Fn) -> decltype(Fn())
        {
            if (!bArmed)
            {
                return Fn();
            }
            bArmed = false;
            const double T0 = Clock();
            auto Result = Fn();
            Ms = (Clock() - T0) * 1000.0;
            Api = InApi;
            return Result;
        }
    };

    // ---- Scene audit (read-only) ----

    /// Compare the RGL static scene with the loaded world (see FRGLSceneAuditResult). Changes nothing: no
    /// registration, no cache maintenance (no idle purge). Scans every actor of World.
    /// OutExtras (optional) gets the name rule's world-wide counts and mesh lists (see FRGLSceneAuditExtras).
    FRGLSceneAuditResult AuditScene(UWorld* World, FRGLSceneAuditExtras* OutExtras = nullptr) const;

    // ---- The name rule (N1) ----

    /// Components whose bounds radius exceeds this (2 km) are left out when the old rule's names match (size guard).
    static constexpr float HugeSkyRadiusCm = 200000.0f;

    /// True when MeshName contains SkySphere, SkyDome, Sky_Sphere or Sky_Dome (case-insensitive): a sky dome or sphere.
    static bool IsSkyDomeMeshName(const FString& MeshName);

    /// The old rule: "sky" (case-insensitive) in the owner's, the component's or the static mesh's name.
    static bool IsLegacySkyName(const UStaticMeshComponent* Component);

    /// Range (cm) the sensor's rays reach, for missing_in_range / ghost_in_range. Call after RegisterSensor().
    /// Never set (or negative): RegistrationDistanceCm minus the 20 m unload margin.
    void SetSensorAuditRange(const void* SensorId, float RangeCm);

    /// Root actor of the sensor's attach chain (nullptr = not attached), for parent_stream_radius_m.
    /// Call after RegisterSensor().
    void SetSensorAuditParent(const void* SensorId, AActor* RootParent);

    /// The rgl_scene_audit line (one grep-friendly key=value line).
    static FString FormatAuditLine(uint64 Frame, double SimTime, const FRGLSceneAuditResult& R);

    /// The rgl_scene_stream_warning line (see CheckStreamCoverage). ParentName empty = no parent (printed as none);
    /// ParentStreamRadiusM < 0 = the parent has no streaming source radius (printed as -1).
    static FString FormatStreamWarningLine(double SimTime, const void* SensorId, float RangeM, const FString& ParentName,
                                           float ParentStreamRadiusM);

    /// rgl.Scene.Audit = N >= 1 (Shipping: -rgl-scene-audit=N): print one rgl_scene_audit line every N-th
    /// call with a new Frame. Call after Update() and before the raytrace; further calls with the same Frame
    /// (other sensors ticking in that frame) do nothing. 0 = off (default).
    void MaybeEmitAudit(UWorld* World, double SimTime, uint64 Frame);

    /// Debug hit dump (rgl.Debug.HitDumpDir / -rgl-debug-hit-dump=, rgl.Debug.HitDumpEvery /
    /// -rgl-debug-hit-dump-every=): the directory (empty = off) and the scan interval (>= 1).
    static FString GetHitDumpDir();
    static int32 GetHitDumpEvery();

#if !UE_BUILD_SHIPPING
    struct FSceneCounts
    {
        int32 Entities = 0;
        int32 IsmcGroups = 0;
        int32 CachedMeshes = 0;
        int32 IdleMeshes = 0;
    };
    /// One static sync now plus an unlimited drain of the create queue; sensors' LastSyncPosition and
    /// bLastSyncValid are updated as Update() does after a sync. Sets the scene time like Update().
    void SyncStaticNow_ForTest(UWorld* World, double Now);
    FSceneCounts GetSceneCounts_ForTest() const;
    /// rgl_scene_audit lines printed by this instance.
    int32 AuditLinesEmitted_ForTest = 0;

    /// Update() without its once-per-frame guard (triggers, sync, drain and transform step as in Update()).
    void Update_ForTest(UWorld* World, double SimTime);
    /// The FWorldDelegates::LevelAddedToWorld handler, called directly.
    void OnLevelAddedToWorld_ForTest(ULevel* Level, UWorld* World);
    /// Static syncs run by Update() / Update_ForTest(), and the trigger name of the last one.
    int32 SyncCount_ForTest = 0;
    FString LastSyncTrigger_ForTest;
    /// The last rgl_scene_sync line printed by Update() / Update_ForTest() (P7: the tests read its keys).
    FString LastSyncLine_ForTest;
    /// Entries of the per-mesh memo of the sky-name verdict (P7).
    int32 GetSkyVerdictCount_ForTest() const;
    /// Static meshes uploaded to RGL (rgl_mesh_create) by this instance.
    int32 MeshUploads_ForTest = 0;
    /// Puts an idle MeshCache entry for Asset (no RGL mesh, no entity), as if its last entity had left; nothing is
    /// uploaded. For the tests of how the cache treats an entry whose asset is gone. No-op if Asset has an entry.
    void AddIdleMeshEntry_ForTest(const UStaticMesh* Asset);
    /// RGL entities of Comp's ISM group (0 = no group).
    int32 GetIsmEntityCount_ForTest(const UInstancedStaticMeshComponent* Comp) const;
    /// Levels waiting in the level-added list (P5).
    int32 GetPendingAddedLevelCount_ForTest() const { return PendingAddedLevels.Num(); }
    /// rgl_scene_stream_warning lines printed by this instance (P4).
    int32 StreamWarnings_ForTest = 0;
    /// CheckStreamCoverage at SimTime with the World Partition answer given instead of asked: bIsWp = false is a map
    /// without World Partition (nothing is counted), bCovered = every sensor's range sphere is streamed in. The rest
    /// (the parent's ring, the 5 s, once per sensor) is the real logic. Sets the scene time like Update() does.
    void CheckStreamCoverage_ForTest(double SimTime, bool bIsWp, bool bCovered);

    // Test seams: bypass the per-frame guard and the wall-clock sync timer; set scene time like Update() does.
    void SyncSkeletalNow_ForTest(UWorld* World, double SimTime);
    void UpdateSkeletalPosesNow_ForTest(double SimTime);
    int32 GetSkeletalRegisteredCount_ForTest() const { return SkeletalEntityMap.Num(); }
    int32 GetSkeletalLiveEntityCount_ForTest() const;
    int32 GetSkeletalMeshCacheCount_ForTest() const { return SkeletalMeshCache.Num(); }
    bool  IsSkeletalRegistered_ForTest(const USkeletalMeshComponent* Comp) const { return SkeletalEntityMap.Contains(const_cast<USkeletalMeshComponent*>(Comp)); }
    bool  IsSkeletalCutout_ForTest(const USkeletalMeshComponent* Comp) const;
    /// Triangles of Comp kept in the scene / left out as near a cutout sensor; -1 = no cutout mesh built.
    int32 GetSkeletalCutoutKeptTriangles_ForTest(const USkeletalMeshComponent* Comp) const;
    int32 GetSkeletalCutoutRemovedTriangles_ForTest(const USkeletalMeshComponent* Comp) const;
    /// Number of cutout meshes built since this instance was created.
    int32 GetSkeletalCutoutBuildCount_ForTest() const { return CutoutBuildCount; }

    // ---- World Partition culling of non-Static parts ----
    /// Replaces "World has a World Partition" in UsesWpNonStaticCulling (unset = ask the world).
    TOptional<bool> WpMapOverride_ForTest;
    /// Replaces World->GetDeltaSeconds() in UpdateStep (the sync timer and the horizon); unset = the world's dt.
    TOptional<float> WorldDtOverride_ForTest;
    struct FWpScope_ForTest
    {
        FRGLSceneManager& SM; TOptional<bool> OldMap; TOptional<float> OldDt;
        FWpScope_ForTest(FRGLSceneManager& InSM, bool bWp, float Dt) : SM(InSM), OldMap(InSM.WpMapOverride_ForTest), OldDt(InSM.WorldDtOverride_ForTest)
        { SM.WpMapOverride_ForTest = bWp; SM.WorldDtOverride_ForTest = Dt; }
        ~FWpScope_ForTest() { SM.WpMapOverride_ForTest = OldMap; SM.WorldDtOverride_ForTest = OldDt; }
    };
    int32 GetWpCandidateCount_ForTest() const { return WpCandidates.Num(); }
    int32 GetWpPendingJobs_ForTest() const { return WpJobs.Num(); }
    double GetWpQueueAgeMax_ForTest() const;
    bool IsWpCandidate_ForTest(const UStaticMeshComponent* Comp) const { return WpCandidates.Contains(FObjectKey(Comp)); }
    bool IsWpPromoted_ForTest(const UStaticMeshComponent* Comp) const;
    bool IsEntityRegistered_ForTest(const UStaticMeshComponent* Comp) const { return EntityMap.Contains(const_cast<UStaticMeshComponent*>(Comp)); }
    /// The ISM group's entities, indices, mask and mesh references agree (no duplicate index, mask = indices).
    bool IsIsmGroupConsistent_ForTest(const UInstancedStaticMeshComponent* Comp) const;
    int32 GetMeshRefCount_ForTest(const UStaticMesh* Mesh) const { return MeshRefCounts.FindRef(FObjectKey(Mesh)); }
    /// rgl_scene_wp_cull_violation lines by kind (update_gap, pose_gap, sensor_speed, object_speed, queue_age, late_create).
    int32 GetWpViolations_ForTest(const TCHAR* Kind) const { return WpViolationCounts.FindRef(FString(Kind)); }
    int32 WpDeadlineSyncs_ForTest() const { return WpDeadlineSyncs; }
    /// Forget the previous Update's sim time (the tests' clocks do not continue one another).
    void ResetWpUpdateClock_ForTest() { WpPrevUpdateSimTime = -1.0; bWpStartupSceneIntervalPending = false; }
    /// The load radius (cm) the culling uses for Sensor at the last culling sync (-1 = no such sensor or no sync yet).
    float GetWpLoadRadiusCm_ForTest(const void* SensorId) const;
    /// The last rgl_scene_config line printed by this instance.
    FString LastConfigLine_ForTest;
    /// The rgl_scene_sensor lines of the last Update that synced (one per sensor synced for the first time; empty if none).
    TArray<FString> LastSensorLines_ForTest;
    /// Entities added / removed by the WP queue in the last Update.
    int32 WpAddedLastUpdate_ForTest = 0;
    int32 WpRemovedLastUpdate_ForTest = 0;
    /// The wait probe of this instance (its clock can be replaced).
    FRGLFirstCallProbe& GetWaitProbe_ForTest() { return WaitProbe; }
#endif

private:
    FRGLSceneManager();

    // Non-copyable
    FRGLSceneManager(const FRGLSceneManager&) = delete;
    FRGLSceneManager& operator=(const FRGLSceneManager&) = delete;

    /// Update() after its once-per-frame guard.
    void UpdateStep(UWorld* World, double SimulationTime);

    /// Initial scan of the world to upload all static meshes.
    void InitializeFromWorld(UWorld* World);

    /// Upload a UStaticMesh to RGL (vertex + index data).
    /// Returns nullptr on failure (mesh skipped silently).
    rgl_mesh_t UploadMesh(UStaticMesh* StaticMesh);

    /// Create an RGL entity for a static mesh component. Returns true on success.
    bool RegisterComponent(UStaticMeshComponent* Component);

    /// Filter components before uploading them to RGL.
    bool ShouldRegisterComponent(UStaticMeshComponent* Component, bool& bOutOfRange) const;

    /// The distance-independent part of ShouldRegisterComponent (validity, registration, mesh, visibility, scale,
    /// the name rule), shared with AuditScene so the audit judges with the same predicate. A component that
    /// is not registered (its level is being streamed out, or its owner unregistered it) is no candidate.
    /// The name rule leaves out a component whose mesh name is a sky dome's (IsSkyDomeMeshName), or whose bounds
    /// radius exceeds HugeSkyRadiusCm and whose names the old rule matches (IsLegacySkyName).
    /// P7: the dome-word verdict belongs to the mesh and is remembered per mesh (SkyVerdictByMesh), so a sync builds no
    /// name string per component; the size guard is still judged per component (its names are built only for a
    /// component wider than 2 km). bRememberVerdict = false (the read-only audit): a verdict not remembered yet is
    /// worked out without adding it.
    enum class EStaticGate : uint8 { Reject, ExcludedByName, Pass };
    EStaticGate GateStaticComponent(const UStaticMeshComponent* Component, bool bRememberVerdict = true) const;

    /// IsSkyDomeMeshName of Mesh's name, through the per-mesh memo (see GateStaticComponent).
    bool IsSkyDomeMesh(const UStaticMesh* Mesh, bool bRemember) const;

    /// Check whether an already registered component should stay in the active RGL scene.
    bool ShouldKeepRegisteredComponent(UStaticMeshComponent* Component) const;

    /// Union distance culling helpers (within ANY registered sensor's sphere).
    /// The tested radius per sensor is RegistrationDistanceCm * RadiusScale + BoundsRadiusCm + ExtraCm;
    /// RadiusScale > 1 is the unload hysteresis (rgl.Scene.UnloadDistanceFactor).
    bool IsWithinAnySensor(const FVector& CenterCm, float BoundsRadiusCm, float ExtraCm, float RadiusScale = 1.0f) const;
    bool IsComponentWithinAnySensor(UStaticMeshComponent* Component, float ExtraCm, float RadiusScale = 1.0f) const;
    /// An ISM instance's sphere: its location, and the mesh's bounds radius times its largest absolute scale.
    bool IsInstanceWithinAnySensor(
        const UInstancedStaticMeshComponent* Component,
        const UStaticMesh* StaticMesh,
        const FTransform& InstanceTransform,
        float ExtraCm,
        float RadiusScale = 1.0f) const;

    /// The instances of Component (not of zero scale) whose sphere reaches some sensor's registration sphere (R),
    /// in index order; reads the instance transforms. bStopAtFirst: stop at the first one (a yes/no question).
    /// Returns true when there is at least one.
    bool CollectInRangeInstances(const UInstancedStaticMeshComponent* Component, const UStaticMesh* StaticMesh,
                                 TArray<int32>& OutIndices, bool bStopAtFirst = false) const;

    /// Remove an RGL entity for a destroyed component.
    void UnregisterComponent(UStaticMeshComponent* Component);

    /// Register the instances of an ISMC as separate RGL entities: a Static ISM's instances inside R (and the sphere of
    /// every instance, for the re-evaluation), a Movable ISM's all. With no instance to register: false before any mesh
    /// upload (counted as ism_empty_skipped).
    bool RegisterISMComponent(UInstancedStaticMeshComponent* Component);

    /// Remove all RGL entities for an ISMC.
    void UnregisterISMComponent(UInstancedStaticMeshComponent* Component);

    /// Track active RGL entity references to cached meshes and free unused GPU meshes. MeshKey = the key the entity's
    /// mesh was cached under (FEntityInfo / FISMCEntityGroup keep it): never rebuilt from an address after GC.
    void AddMeshReference(const FObjectKey& MeshKey, int32 Count = 1);
    void ReleaseMeshReference(const FObjectKey& MeshKey, int32 Count = 1);

    /// Update all entity transforms to match current UE5 state. Entities and ISM groups whose component is gone
    /// or no longer registered leave the RGL scene here, every Update.
    void UpdateTransforms();

    /// Per-sync counters for the rgl_scene_sync telemetry line.
    struct FSceneSyncStats
    {
        int32 ScannedActors = 0;  // valid actors visited by the static-mesh scan
        int32 AddedCached = 0;    // registered during the scan (mesh already has live entities)
        int32 Queued = 0;         // components waiting for a new mesh or idle reactivation
        int32 Removed = 0;        // regular + ISMC components unregistered
        int32 RangeSkipped = 0;   // components outside every sensor's registration sphere
        // P7: where the sync's wall time goes (scan_ms, reg_ms, remove_ms, skel_ms on the line).
        double ScanMs = 0.0;      // the actor scan (static candidates, and the skeletal candidates of the same actors),
                                  // without the RGL calls it makes (RegMs)
        double RegMs = 0.0;       // RGL registrations during the scan (meshes with live entities, rebuilt ISM groups)
        double RemoveMs = 0.0;    // removal of the registered components the scan no longer keeps
        double SkelMs = 0.0;      // the skeletal sync on the scan's candidates
    };

    /// Per-Update counters of one drain step of the pending-create queue.
    struct FDrainStats
    {
        int32 Created = 0;          // new RGL meshes still resident after registration (GAS builds at raytrace)
        int32 Failed = 0;           // actual extraction/upload failures, excluding deferred retries
        int32 RegisteredCached = 0; // queued components registered on an already live mesh
        int32 Dropped = 0;          // queued components that became stale, ineligible or out of range
        double Ms = 0.0;            // wall time of the drain step
    };

    /// Scan world for new/removed components since last update. Components whose mesh has live entities
    /// are registered at once; new meshes and idle reactivations are (re)queued, nearest first.
    void SyncWorldComponents(UWorld* World, FSceneSyncStats& OutStats);

    /// Create queued meshes under the rgl.Scene.MaxMeshCreatesPerUpdate / rgl.Scene.CreateBudgetMs budget
    /// (bUnlimited ignores both). Removal is never deferred; only creation is.
    void DrainPendingCreates(bool bUnlimited, FDrainStats& OutStats);

    /// Per-Update counters of one step of the ISM re-evaluation (P3).
    struct FIsmReevalStats
    {
        int32 Added = 0;     // instance entities created
        int32 Removed = 0;   // instance entities destroyed
        double Ms = 0.0;     // wall time of the step
    };

    /// P3: work through PendingIsmReevals (nearest group first). Per group: add the entities of instances that reach R
    /// and have none, then remove those of instances beyond R x rgl.Scene.UnloadDistanceFactor + the unload margin; the
    /// group's InstanceSpheres decide, an instance's transform is read only to create its entity. At most MaxChanges
    /// additions plus removals (0 = unlimited); a group the cap stops inside stays at the head for the next call. A group
    /// left without entities leaves the scene. Update() calls it on the Updates without a sync (D7).
    void DrainIsmReevaluations(int32 MaxChanges, FIsmReevalStats& OutStats);

    /// Distance from the nearest registered sensor to a bounding sphere's surface (cm). Sort key of the queue.
    float NearestSensorGapCm(const FVector& CenterCm, float BoundsRadiusCm) const;

    /// Resident RGL mesh for StaticMesh (live or idle), or nullptr; looked up by FObjectKey(StaticMesh). The key
    /// carries the asset's GC serial, so an asset that took a collected one's address never gets that one's geometry.
    rgl_mesh_t FindCachedMesh(const UStaticMesh* StaticMesh) const;

    /// Failed extraction/upload retries are deferred across queue rebuilds using generation-aware keys.
    bool IsMeshRetryDeferred(UStaticMesh* StaticMesh) const;
    void RememberMeshFailure(UStaticMesh* StaticMesh);

    /// Idle mesh LRU maintenance (rgl.Scene.IdleMeshCacheMax). PurgeIdleMeshes destroys the idle entries whose key
    /// no longer resolves (no lookup can reach them again), then applies the cap.
    void DestroyIdleMeshAt(int32 LruIndex);
    void TrimIdleMeshes(int32 MaxIdle);
    void PurgeIdleMeshes();

    /// Print an rgl_scene_config line when any rgl.Scene.* knob differs from the last printed value.
    void LogSceneConfigIfChanged();

    // ---- Skeletal mesh path (spec 2026-09-10-rgl-skeletal-mesh-design.md) ----
    struct FSkeletalEntityInfo
    {
        rgl_entity_t                           Entity = nullptr;   // nullptr = pending first pose
        TWeakObjectPtr<USkeletalMeshComponent> Component;
        USkeletalMesh*                         SkeletalMesh = nullptr;
        int32                                  LODIndex = -1;
        int32                                  RawBoneNum = 0;
        TArray<rgl_mat3x4f>                    PoseScratch;
        EVisibilityBasedAnimTickOption         SavedTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones;
        bool                                   bSavedURO = true;
        bool                                   bTickPolicyApplied = false;
        // Self cutout path: the owner carries a sensor with rgl_self_cutout_radius > 0. Entity is built
        // on CutoutMesh (owned here, never in SkeletalMeshCache): the triangles not near any of those
        // sensors. CutoutMesh is rebuilt whenever CutoutConfig changes.
        bool                                   bCutout = false;
        bool                                   bCutoutBuilt = false;    // CutoutMesh matches CutoutConfig
        rgl_mesh_t                             CutoutMesh = nullptr;    // nullptr when no triangle is kept
        int32                                  CutoutKeptTriangles = 0;
        int32                                  CutoutRemovedTriangles = 0;
        TSharedPtr<FRGLSkeletalMeshData>       CutoutSource;            // extracted once at registration
        TArray<RGLSelfCutout::FSensorConfig>   CutoutConfig;
    };
    struct FSkeletalMeshKey
    {
        USkeletalMesh* Mesh = nullptr; int32 LOD = -1;
        bool operator==(const FSkeletalMeshKey& O) const { return Mesh == O.Mesh && LOD == O.LOD; }
        friend uint32 GetTypeHash(const FSkeletalMeshKey& K) { return HashCombine(PointerHash(K.Mesh), GetTypeHash(K.LOD)); }
    };
    TMap<USkeletalMeshComponent*, FSkeletalEntityInfo> SkeletalEntityMap;
    TMap<FSkeletalMeshKey, rgl_mesh_t>                 SkeletalMeshCache;
    TMap<FSkeletalMeshKey, int32>                      SkeletalMeshRefCounts;
    /// Log suppression key: (component, asset, hash(message)). FObjectKey is generation-aware,
    /// so a GC'd address reused by a new component never inherits an old suppression; including
    /// the message hash keeps a *different* later failure for the same pair loggable once too.
    using FSkeletalWarnKey = TTuple<FObjectKey, FObjectKey, uint32>;
    /// (component, asset) identity for the permanent-skip set.
    using FSkeletalSkipKey = TPair<FObjectKey, FObjectKey>;
    TSet<FSkeletalWarnKey>                             SkeletalWarned;
    /// (component, asset) pairs whose extraction/upload failed for a non-recoverable
    /// reason (spec §6.1: only §3.1 readiness failures are retried). Permanently skipped.
    TSet<FSkeletalSkipKey>                             SkeletalSkipped;
    bool                                               bSkeletalApiAvailable = false;

    static void RglDestroyChecked(rgl_status_t Status, const TCHAR* Api);
    bool  SkeletalEnabled() const;
    bool  ShouldRegisterSkeletalComponent(USkeletalMeshComponent* Comp, bool& bOutOfRange, bool& bRetryLater, FString& OutReason) const;
    bool  RegisterSkeletalComponent(USkeletalMeshComponent* Comp);
    void  UnregisterSkeletalComponent(USkeletalMeshComponent* Key);
    void  UpdateSkeletalPoses();
    /// The skeletal sync with its own actor scan (initialization, the test seam).
    void  SyncSkeletalComponents(UWorld* World);
    /// The skeletal sync on Candidates: every skeletal mesh component of the scanned actors, in scan order.
    /// SyncWorldComponents gathers them in its own actor scan (P7: one scan of the world per sync, not two).
    void  SyncSkeletalCandidates(const TArray<USkeletalMeshComponent*>& Candidates);
    void  TeardownSkeletal();
    void  ApplyTickPolicy(USkeletalMeshComponent* Comp, FSkeletalEntityInfo& Info, bool bEnable);
    void  WarnSkeletalOnce(const USkeletalMeshComponent* Comp, const USkeletalMesh* Mesh, const FString& Msg);
    rgl_mesh_t GetOrUploadSkeletalMesh(USkeletalMesh* Mesh, int32& OutLOD, int32& OutRawBoneNum, FString& OutReason);
    void  ReleaseSkeletalMeshReference(const FSkeletalMeshKey& Key);

    // ---- Self cutout (rgl_self_cutout_radius) ----
    int32 CutoutBuildCount = 0;
    bool  AnyCutoutSensor() const;
    bool  WantsSelfCutout(const USkeletalMeshComponent* Comp) const;
    void  GatherCutoutConfig(const AActor* Owner, TArray<RGLSelfCutout::FSensorConfig>& Out) const;
    /// Registration on the cutout path: own extraction, no shared mesh cache.
    bool  RegisterSkeletalCutoutComponent(USkeletalMeshComponent* Comp);
    /// Pose step of a cutout component (pose already in Info.PoseScratch): rebuild CutoutMesh if the
    /// configuration changed, then create/pose Entity on it. False = pending, retried next tick.
    bool  UpdateCutoutEntity(USkeletalMeshComponent* Comp, FSkeletalEntityInfo& Info);
    bool  BuildCutoutMesh(USkeletalMeshComponent* Comp, FSkeletalEntityInfo& Info, const TArray<RGLSelfCutout::FSensorConfig>& Config);
    /// Destroys Entity and CutoutMesh and forgets the configuration they were built for.
    void  DestroyCutout(FSkeletalEntityInfo& Info);

    /// Create a large ground plane in the RGL scene to represent the road surface.
    void CreateGroundPlane();

    // ---- Data ----

    /// RGL scene handle (nullptr = default scene).
    rgl_scene_t Scene = nullptr;

    /// Cache: static mesh asset -> rgl_mesh_t (avoids duplicate uploads), keyed by FObjectKey (P2): an asset that takes
    /// the address of a collected one has another key (GC serial), so it never gets that one's geometry. The mesh
    /// cache, the reference counts and the idle list all use the key; nothing rebuilds it from an address.
    /// Invariant: every entry is either referenced (MeshRefCounts > 0) or listed in IdleMeshLru.
    struct FCachedMesh
    {
        rgl_mesh_t Mesh = nullptr;
    };
    TMap<FObjectKey, FCachedMesh> MeshCache;

    /// Retry failed assets after 16 syncs: transient streaming/upload failures can recover without
    /// retrying every scan. FObjectKey includes the GC serial; dead keys are purged at each sync.
    TMap<FObjectKey, uint64> FailedMeshRetrySync;
    uint64 SceneSyncSerial = 0;
    int32 MeshFailuresSinceLog = 0; // actual failures since the last rgl_scene_sync, including non-sync drains

    /// Active entity references per cached mesh. At zero the mesh becomes idle (kept in IdleMeshLru)
    /// or, with rgl.Scene.IdleMeshCacheMax = 0, is destroyed at once.
    TMap<FObjectKey, int32> MeshRefCounts;

    /// Unreferenced but still resident meshes, least recently idled first.
    TArray<FObjectKey> IdleMeshLru;

    /// Idle meshes destroyed (LRU overflow or stale identity) since the last rgl_scene_sync line.
    int32 IdleDestroyedSinceLog = 0;

    /// ISM groups dropped by UpdateTransforms (component gone or unregistered) since the last rgl_scene_sync line.
    int32 DeadIsmcSinceLog = 0;

    /// Components in range needing a new mesh or idle reactivation, nearest to any sensor first. Rebuilt by every
    /// sync (so it never holds duplicates or components the last scan no longer saw), drained by Update().
    struct FPendingCreate
    {
        TWeakObjectPtr<UStaticMeshComponent> Component;
        float GapCm = 0.0f;
    };
    TArray<FPendingCreate> PendingCreates;

    /// One queue episode = from the queue becoming non-empty to it draining (rgl_scene_queue_drained line).
    struct FQueueEpisode
    {
        bool   bActive = false;
        double StartSimTime = 0.0;
        int32  Updates = 0;
        int32  Created = 0;
        int32  Failed = 0;
        int32  Dropped = 0;
    };
    FQueueEpisode QueueEpisode;

    /// Last printed rgl.Scene.* knob values (rgl_scene_config line); none printed yet while bSceneKnobsLogged is false.
    FSceneKnobs LoggedSceneKnobs;
    bool bSceneKnobsLogged = false;

    /// Mapping: UStaticMeshComponent* -> RGL entity + weak reference.
    struct FEntityInfo
    {
        rgl_entity_t Entity = nullptr;
        TWeakObjectPtr<UStaticMeshComponent> Component;
        FObjectKey MeshKey;                    // The asset Entity was built from (GC-generation aware): its cache key.
        bool bIsStatic = false;            // True if Mobility == Static (never moves)
        bool bTransformInitialized = false; // True after first transform set
        bool bInRange = true;              // Distance culling state
        FTransform LastTransform;           // Cached for change detection
    };
    TMap<UStaticMeshComponent*, FEntityInfo> EntityMap;

    /// ISMC entity group: one ISMC component -> N RGL entities (one per instance).
    struct FISMCEntityGroup
    {
        TArray<rgl_entity_t> Entities;
        TWeakObjectPtr<UInstancedStaticMeshComponent> Component;
        FObjectKey MeshKey;                // The asset Entities were built from (GC-generation aware): its cache key.
        int32 OriginalInstanceCount = 0;  // Instance count at registration time (may differ from Entities.Num() due to skipped instances)
        int32 MeshRefCount = 0;           // Number of live RGL entities referencing the mesh of MeshKey.
        TArray<int32> InstanceIndices;            // Per entity (same order as Entities): the ISM instance index
        TArray<FTransform> InstanceWorldTransforms; // Per entity: the instance's world transform when its entity was made
        /// Static groups only: one sphere per instance of the component (index = instance index), taken at registration:
        /// the instance's location and the mesh's bounds radius times its largest absolute scale. W < 0 = never gets an
        /// entity (zero scale, or no transform). The re-evaluation (P3) decides with these.
        TArray<FSphere3f> InstanceSpheres;
        /// Per instance of the component: true while the instance has an entity in Entities.
        TBitArray<> RegisteredMask;
    };
    TMap<UInstancedStaticMeshComponent*, FISMCEntityGroup> ISMCEntityMap;

    /// One group's pass of DrainIsmReevaluations. False = the cap was reached with a change left in this group.
    bool ReevaluateIsmGroup(UInstancedStaticMeshComponent* Component, FISMCEntityGroup& Group, int32 MaxChanges,
                            int32& InOutChanges, FIsmReevalStats& OutStats);

    /// P3: Static ISM groups waiting to be re-evaluated against the sensor spheres, nearest to any sensor first. Every
    /// sync queues all registered Static groups again (rebuilt, like PendingCreates); DrainIsmReevaluations empties it.
    struct FPendingIsmReeval
    {
        TWeakObjectPtr<UInstancedStaticMeshComponent> Component;
        float GapCm = 0.0f;
    };
    TArray<FPendingIsmReeval> PendingIsmReevals;
    TArray<int32> IsmScanIndices;   // CollectInRangeInstances' output in the sync's scan, reused

    // ---- P7: the sync's scan, kept between syncs so that a sync allocates nothing in steady state ----
    /// Name-rule verdict per static mesh (true = a sky dome's mesh name), worked out once per mesh. Entries whose key no
    /// longer resolves are dropped at each sync. Filled by the sync paths only (the audit reads it).
    mutable TMap<FObjectKey, bool> SkyVerdictByMesh;
    TArray<UStaticMeshComponent*> ScanComponents;              // one actor's static mesh components
    TArray<USkeletalMeshComponent*> ScanActorSkeletal;         // one actor's skeletal mesh components
    TArray<USkeletalMeshComponent*> ScanSkeletalCandidates;    // every scanned actor's skeletal mesh components
    TSet<UStaticMeshComponent*> ScanCurrentComponents;         // regular components the scan keeps
    TSet<UInstancedStaticMeshComponent*> ScanCurrentIsmComponents;   // ISM components the scan keeps
    TArray<FPendingCreate> ScanNewPending;                     // the next create queue (swapped with PendingCreates)
    TArray<UStaticMeshComponent*> ScanToRemove;
    TArray<UInstancedStaticMeshComponent*> ScanIsmToRemove;

    /// ISM re-evaluation telemetry since the last rgl_scene_sync line (ism_* keys).
    int32  IsmAddedSinceLog = 0;
    int32  IsmRemovedSinceLog = 0;
    int32  IsmEmptySkippedSinceLog = 0;   // Static ISMs reaching R with no instance inside: not queued, not uploaded
    double IsmMsSinceLog = 0.0;           // wall time of the re-evaluation steps, summed
    double IsmMaxMsSinceLog = 0.0;        // the longest single step
    int32  IsmPendingAtSync = 0;          // groups whose re-evaluation had not finished when the sync started

    bool bInitialized = false;

    /// Frame tracking — skip Update if already done this frame
    uint64 LastUpdateFrame = 0;

    /// Per-sensor culling registration. A static mesh is registered if it falls
    /// within ANY active sensor's sphere (union), so spatially separated sensors
    /// (multi-vehicle / roadside) are all served correctly by the single shared scene.
    struct FSensorRegistration
    {
        FVector Position = FVector::ZeroVector;
        float   RegistrationDistanceCm = 30000.0f; // per-sensor radius (UE5 cm)
        FVector LastSyncPosition = FVector::ZeroVector;
        bool    bLastSyncValid = false;
        double  LastUpdateTime = 0.0;              // sim time of last RegisterSensor()
        // Scene audit (SetSensorAuditRange / SetSensorAuditParent).
        float   AuditRangeCm = -1.0f;              // < 0 = RegistrationDistanceCm - UnregistrationHysteresisCm
        TWeakObjectPtr<AActor> AuditParent;        // root of the sensor's attach chain; null = not attached
        // Stream coverage warning (P4, CheckStreamCoverage).
        double  UncoveredSinceSimTime = -1.0;      // sim time of the first check of the current not-streamed-in run; -1 = none
        bool    bStreamWarned = false;             // the rgl_scene_stream_warning line was printed for this sensor
        // Self cutout (SetSensorSelfCutout); CutoutRadiusCm <= 0 = off.
        TWeakObjectPtr<AActor> CutoutVehicle;
        float      CutoutRadiusCm = 0.0f;
        float      CutoutMinElevationDeg = 0.0f;
        float      CutoutMaxElevationDeg = 0.0f;
        FTransform CutoutSensorToVehicle = FTransform::Identity;   // taken in the same tick as the vehicle pose
        // Pose history (RegisterSensor), read by the World Partition culling of non-Static parts only.
        bool    bPoseValid = false;                // a pose was registered before
        bool    bRefreshed = false;                // a refresh with a later time since the culling last looked
        bool    bTimeRewound = false;              // a refresh with an earlier time since the culling last looked
        // R7: armed exactly once by this sensor's first WP init sync. Later syncs never re-arm it.
        double  WpInitSyncSimTime = -1.0;
        double  WpInitPoseTime = -1.0;
        bool    bWpStartupPosePending = false;
        bool    bWpStartupUpdatePending = false;
        double  LastRefreshGapSeconds = 0.0;       // sim time between the last two refreshes
        float   ObservedSpeedCmps = 0.0f;          // distance / time between the last two refreshes
    };
    /// Key = sensor id (session handle). Pointer key uses pointer hashing.
    TMap<const void*, FSensorRegistration> RegisteredSensors;

    /// The range (cm) a sensor's rays reach (audit, stream check): SetSensorAuditRange's value, or, when it was never
    /// set, RegistrationDistanceCm minus UnregistrationHysteresisCm. A sensor that only calls RegisterSensor is served.
    float SensorRangeCm(const FSensorRegistration& S) const;

    /// The distance trigger (cm): a sensor that moved farther than this since the last sync syncs now. 0.2 x
    /// RegistrationDistanceCm. Printed as trigger_m= on the sensor's rgl_scene_sensor line.
    float SyncTriggerCm(const FSensorRegistration& S) const;

    /// Radius (cm) that RootParent's World Partition streaming source loads around the actor: the largest radius among
    /// its shapes that do not use the grid's loading range. -1 = no actor, no source, or no such shape. CARLA puts the
    /// source on spawned vehicles and walkers (UCarlaEpisode::SpawnActorWithInfo). Shared with AuditScene.
    static float StreamSourceRadiusCm(const AActor* RootParent);

    // ---- Stream coverage warning (P4) ----
    /// World Partition maps only (other maps: nothing). Runs after each static sync (Update). Per sensor not warned yet:
    /// is its range sphere (position, SensorRangeCm) streamed in (Activated)? The same query as AuditScene's
    /// stream_covered, one sensor at a time. A sensor whose sphere stays not streamed in for StreamWarningSeconds (sim,
    /// from the first such check to a later one, with no covered check between) gets one rgl_scene_stream_warning
    /// line, once per sensor (per registration). A check at which the parent's own streaming source sphere holds the
    /// whole range sphere counts as covered: the parent's ring loads that sphere, so it is streaming in progress (the
    /// default 3 km hero ring takes more than 5 s at start-up), not a configuration the hint can fix. RegisterSensor
    /// alone is enough: no range set = SensorRangeCm's fallback, no parent = none (every not-streamed-in check counts).
    /// The ring is looked at first: a sensor whose parent's ring holds its range is not queried.
    void CheckStreamCoverage(UWorld* World);
    /// One sensor's check at CurrentSimTime (bStreamedIn = its range sphere is Activated): the clock, then the line.
    /// A streamed-in check returns before the parent is looked at.
    void StepStreamCoverage(const void* SensorId, FSensorRegistration& S, bool bStreamedIn);
    /// The parent's own streaming source sphere holds S's whole range sphere (S.AuditParent, StreamSourceRadiusCm).
    bool ParentRingHoldsRange(const FSensorRegistration& S) const;
    static constexpr double StreamWarningSeconds = 5.0;

    /// Hysteresis for dynamic unloads to avoid add/remove churn around the boundary (UE5 cm).
    /// Static meshes unload beyond RegistrationDistanceCm * rgl.Scene.UnloadDistanceFactor + this margin
    /// (factor 1.0 = the original rule); skeletal components use this margin alone.
    float UnregistrationHysteresisCm = 2000.0f; // 20m

    /// Evict sensors that stopped ticking (paused but not destroyed) to free their VRAM.
    float SensorStaleTimeoutSeconds = 5.0f;

    /// Latest simulation time seen by Update(); used for stale-sensor eviction.
    double CurrentSimTime = 0.0;

    /// Virtual ground plane (not tied to any UE5 component).
    rgl_mesh_t GroundMesh = nullptr;
    rgl_entity_t GroundEntity = nullptr;

    /// The Update clock of the periodic sync (rgl.Scene.SyncClock = 0): the world dt of the scene Updates since the last
    /// sync. Counted on either clock (a live switch to 0 continues from it); the interval is rgl.Scene.SyncIntervalSeconds.
    float TimeSinceLastSync = 0.0f;

    // ---- Scene audit bookkeeping ----
    double LastSyncSimTime = -1.0;        // CurrentSimTime of the last static sync; -1 = none yet (also the simulation clock
                                          // of the periodic sync, rgl.Scene.SyncClock = 1)
    double LastUpdateWallSeconds = 0.0;   // FPlatformTime::Seconds() of the last Update that ran
    double LastFrameMs = 0.0;             // wall time between the last two Updates that ran
    int32  LevelsAddedSinceAudit = 0;     // LevelAddedToWorld events of this world since the last audit line (0 while off)

    // rgl.Scene.Audit line emission (MaybeEmitAudit)
    uint64 LastAuditFrame = 0;
    bool   bAuditFrameValid = false;
    bool   bExcludedLineLogged = false;   // the rgl_scene_audit_excluded line was printed (with the first audit line)
    uint64 AuditUpdateCount = 0;          // distinct frames seen while the audit is on

    // ---- Level-added sync (P5) ----
    /// Remember the instance's world and bind FWorldDelegates::LevelAddedToWorld for the instance's whole life
    /// (GetInstance calls it once, when it creates the instance; the destructor removes the binding).
    void BindWorld(UWorld* World);
    /// LevelAddedToWorld: a level of this world (OwnerWorldKey) waits in PendingAddedLevels for the next Update.
    void OnLevelAddedToWorld(ULevel* Level, UWorld* World);
    /// True when a level added since the last sync holds a registered static mesh component whose bounds reach some
    /// sensor's registration sphere (R) and that passes GateStaticComponent (what a sync would register); the list
    /// is emptied either way. Within LevelSyncMinIntervalSeconds (sim) of the last sync: false, and the list is kept
    /// for a later Update (D5). Looks at the levels' own actors only.
    bool ConsumeRelevantLevelAdds();
    static constexpr double LevelSyncMinIntervalSeconds = 0.3;
    FDelegateHandle LevelAddedHandle;     // FWorldDelegates::LevelAddedToWorld, bound for the instance's life
    FObjectKey OwnerWorldKey;             // the world this instance serves; other worlds' events are ignored
    /// Levels added since the last sync. Emptied by every sync (the full scan sees them) and by
    /// ConsumeRelevantLevelAdds once the interval allows a look.
    TArray<TWeakObjectPtr<ULevel>> PendingAddedLevels;
    TArray<UStaticMeshComponent*> LevelScanComponents;   // ConsumeRelevantLevelAdds' component array, reused

    // ---- World Partition culling of non-Static parts (7b) ----
    /// UsesWpNonStaticCulling of the Update's world, decided at the start of each Update (before the first scan).
    bool bWpActive = false;
    /// A non-Static part the distance rule chooses (not protected). Kept from one sync to the next (FObjectKey + weak
    /// pointer) so that an ISM with no entity stays known; dropped when a sync no longer sees it.
    struct FWpCandidate
    {
        TWeakObjectPtr<UStaticMeshComponent> Component;
        bool   bIsm = false;
        bool   bMoved = false;              // seen moving (transform or velocity): never chosen out by distance again
        uint32 Generation = 0;              // bumped when the mesh or the instance layout changes (old jobs drop out)
        FObjectKey MeshKey;                 // the mesh when the transforms below were recorded
        int32  InstanceCount = 0;           // ISM: instances when the transforms were recorded
        double SeenSimTime = 0.0;           // sim time of the sync that recorded the transforms
        uint64 SeenSyncSerial = 0;          // that sync
        FTransform Transform;               // the component transform at that sync
        double WantSince = -1.0;            // regular: since when it is wanted without an entity (-1 = not waiting)
        bool   bCulledAtSync = false;       // regular: outside every load sphere without an entity at that sync
        /// ISM, per instance: its local matrix at that sync (an instance whose matrix and component transform are
        /// unchanged has not moved; only changed ones are read again), its sphere (W < 0 = never gets an entity), and
        /// since when it is wanted without an entity.
        TArray<decltype(FInstancedStaticMeshInstanceData::Transform)> InstanceLocal;
        TArray<FSphere3f> InstanceSpheres;
        TArray<double> InstanceWantSince;
        TBitArray<> InstanceCulled;         // ISM, per instance: as bCulledAtSync
    };
    TMap<FObjectKey, FWpCandidate> WpCandidates;
    /// One entity to add or remove (a regular component, or one ISM instance). Rebuilt by every culling sync from
    /// WpCandidates (the wait is kept in the candidate, not reset by a sync); drained by every Update under
    /// rgl.Scene.MaxMovableEntityChangesPerUpdate / MovableEntityBudgetMs, except R6 raw-range discovery creates.
    struct FWpJob
    {
        TWeakObjectPtr<UStaticMeshComponent> Component;
        FObjectKey Key;
        int32  Instance = INDEX_NONE;       // ISM instance; INDEX_NONE = the regular component
        uint32 Generation = 0;
        bool   bAdd = true;
        bool   bInRangeAtDiscovery = false; // R6: raw-range creates finish this Update without either budget
        double SinceSimTime = 0.0;          // adds: wanted since (sim)
        double ReachSimTime = 0.0;          // adds: earliest time its bounds could reach some sensor's range (order)
        float  GapCm = 0.0f;                // distance of its bounds to the nearest sensor
        bool   bAgeReported = false;
        /// The contract covers it: the part was known at an earlier sync (same layout) and this queue was not built by
        /// a sync after a discontinuity (a new sensor, a pose jump, time going back). Only covered jobs can break
        /// the L_create and late-creation contracts (a part found in range for the first time is the audit's miss).
        bool   bCovered = false;
        /// The late-creation contract: the part was outside every load sphere without an entity at the previous sync
        /// (the margin protects exactly those); a part first seen inside a range (a cell streamed in) is the audit's miss.
        bool   bLateCovered = false;
    };
    TArray<FWpJob> WpJobs;
    TArray<FWpJob> ScanWpJobs;              // the next queue (swapped with WpJobs), allocation kept
    double WpVisitRegSeconds = 0.0;         // RGL calls inside the candidate visits (a rebuilt group), summed
    double WpSyncCheckMs = 0.0;             // this sync's candidate visits
    /// One-entry cache of the actor part of ClassifyWpProtection within one sync (components of an actor come together).
    mutable const AActor* WpCachedActor = nullptr;
    mutable uint64 WpCachedActorSerial = 0;
    mutable EWpProtect WpCachedActorVerdict = EWpProtect::None;
    /// Parts once protected stay protected (moved to the original path for good), with the first reason.
    mutable TMap<FObjectKey, EWpProtect> WpStickyProtected;
    /// ISM candidates seen moving: their registered instances follow the instance transforms every Update.
    TArray<FObjectKey> WpMovedIsm;
    // Contract bookkeeping
    bool   bWpSyncDiscontinuous = false;    // the running sync follows a new sensor, a pose jump or time going back
    bool   bWpSyncValid = false;            // a culling sync ran (the deadline below is armed)
    double WpLastSyncSimTime = 0.0;         // sim time of the last culling sync
    double WpAssumedSyncSeconds = 0.0;      // T_sync_max that sync assumed (the deadline)
    double WpHorizonSeconds = 0.0;          // H of that sync
    double WpMarginCm = 0.0;                // M of that sync
    double WpPrevUpdateSimTime = -1.0;      // sim time of the previous Update (U check)
    bool bWpStartupSceneIntervalPending = false; // R10: every sensor at the preceding interval start was newly initialized
    /// The inputs of the horizon and margin (the sync clock and interval included: rgl.Scene.SyncClock, SyncIntervalSeconds).
    struct FWpKnobs
    {
        int32 Enable = 1; float MinMarginM = 20.f; float SensorSpeedMps = 50.f; float ObjectSpeedMps = 50.f;
        float MaxUpdateGapS = 0.1f; float CreateLatencyS = 1.0f; float IntervalS = 1.0f; int32 Clock = 1;
        bool operator==(const FWpKnobs& O) const
        {
            return Enable == O.Enable && MinMarginM == O.MinMarginM && SensorSpeedMps == O.SensorSpeedMps &&
                   ObjectSpeedMps == O.ObjectSpeedMps && MaxUpdateGapS == O.MaxUpdateGapS &&
                   CreateLatencyS == O.CreateLatencyS && IntervalS == O.IntervalS && Clock == O.Clock;
        }
        bool operator!=(const FWpKnobs& O) const { return !(*this == O); }
    };
    FWpKnobs CurrentWpKnobs() const;
    FWpKnobs WpSyncKnobs;                   // the knobs of the last culling sync (a live change forces a sync)
    // Telemetry since the last rgl_scene_sync line (wp_* keys)
    int32  WpCheckN = 0;                    // candidates looked at by the syncs
    int32  WpCheckInstances = 0;            // ISM instances among them
    double WpCheckMs = 0.0;                 // their time, summed
    double WpCheckMaxMs = 0.0;              // the longest single sync's look
    int32  WpEntityAdded = 0;
    int32  WpEntityRemoved = 0;
    double WpEntityMs = 0.0;
    // R6: this Update only, including mesh upload/API wait; unlike wp_entity_* these are not interval sums.
    int32  WpInRangeAddedLastUpdate = 0;
    double WpInRangeMsLastUpdate = 0.0;
    int32  WpViolationsSinceLog = 0;
    int32  WpDeadlineSyncs = 0;             // syncs forced by the deadline (whole life of the instance)
    TMap<FString, int32> WpViolationCounts; // by kind (whole life)
    /// The wait probe of the sync's measured interval (wait_ms=).
    FRGLFirstCallProbe WaitProbe;

    /// World dt of this Update (the test seam may replace it).
    float WorldDeltaSeconds(const UWorld* World) const;
    /// Per Update (O(sensors)): the U, refresh and speed contracts, and whether a WP sync is due now (deadline, or a
    /// reset: time going back, a pose jump, a knob change). bOutJump: a pose jump or time going back (a discontinuity).
    void WpCheckContracts(const UWorld* World, bool& bOutDeadline, bool& bOutReset, bool& bOutJump);
    void WpViolation(const TCHAR* Kind, double Value, double Bound);
    /// Start of a culling sync: horizon, margin, deadline (Dt = this Update's world dt).
    void WpBeginSync(float Dt);
    /// The scan met a candidate: record, check for motion, choose its entities, queue the changes.
    void WpVisitCandidate(UStaticMeshComponent* Comp, UInstancedStaticMeshComponent* Ism);
    /// End of the culling sync's scan: candidates not seen are dropped, the queue is sorted.
    void WpEndSync();
    /// Drop every WP state (a live switch to the original rule, a world that no longer culls). Partial ISM groups leave
    /// the scene so that the original rule registers them whole.
    void WpTeardown();
    /// One budgeted step of the queue (bUnlimited: everything).
    void WpDrain(bool bUnlimited);
    /// Registered instances of moved ISM candidates follow their instance transforms (every Update, after the time).
    void WpUpdateMovedInstanceTransforms();
    /// The load / unload radius (cm) of a sensor for the culling.
    float WpLoadRadiusCm(const FSensorRegistration& S) const;
    bool WpInAnyLoadSphere(const FVector& CenterCm, float RadiusCm) const;
    bool WpInAnyUnloadSphere(const FVector& CenterCm, float RadiusCm) const;
    /// Earliest sim time the sphere could touch some sensor's range at the bounded closing speed.
    double WpReachSimTime(const FVector& CenterCm, float RadiusCm) const;
    /// The sphere of an ISM instance on the new path: the mesh bounds' origin through the instance transform, and the
    /// mesh's bounds radius times the instance's largest absolute scale.
    static FSphere WpInstanceSphere(const UStaticMesh* Mesh, const FTransform& InstanceTransform);
    /// Add / remove one entity of the queue. False = not done (already done, or could not be done now).
    bool WpAddRegular(UStaticMeshComponent* Comp);
    bool WpAddInstance(UInstancedStaticMeshComponent* Comp, int32 Instance, const FTransform& InstanceTransform);
    bool WpRemoveInstance(UInstancedStaticMeshComponent* Comp, int32 Instance);
    /// Totals for the sync line: entities of every kind (regular + ISM instances + live skeletal), non-Static ones.
    void CountSceneEntities(int32& OutTotal, int32& OutNonStatic) const;

    // ---- Static singleton storage ----
    static TMap<UWorld*, FRGLSceneManager*> Instances;
};

#endif // WITH_RGL
