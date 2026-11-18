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
//   - Handles actor creation/destruction via periodic world scans, plus a scan when a level with static
//     geometry in some sensor's sphere is added (World Partition cells, level streaming)
//   - Amortizes new-mesh creation across scans (rgl.Scene.* CVars): a budgeted, nearest-first
//     create queue, an unload radius wider than the load radius, and an LRU of idle meshes
//   - Re-evaluates the instances of Static instanced static meshes against the sensor spheres after each scan
//     (on the following Updates, capped per Update), instead of choosing them once at registration

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

    /// Set the world sync interval in simulation seconds. Default: 1.0s. Minimum: 0.1s.
    void SetSyncInterval(float Seconds) { SyncIntervalSeconds = FMath::Max(0.1f, Seconds); }

    /// Check if the scene manager has been initialized.
    bool IsInitialized() const { return bInitialized; }

    /// Remove a sensor when its session is destroyed (defined in .cpp; tears skeletal state down when none remain).
    void UnregisterSensor(const void* SensorId);

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
    /// Static meshes uploaded to RGL (rgl_mesh_create) by this instance.
    int32 MeshUploads_ForTest = 0;
    /// Puts an idle MeshCache entry for Asset (no RGL mesh, no entity), as if its last entity had left; nothing is
    /// uploaded. For the tests of how the cache treats an entry whose asset is gone. No-op if Asset has an entry.
    void AddIdleMeshEntry_ForTest(const UStaticMesh* Asset);
    /// RGL entities of Comp's ISM group (0 = no group).
    int32 GetIsmEntityCount_ForTest(const UInstancedStaticMeshComponent* Comp) const;
    /// Levels waiting in the level-added list (P5).
    int32 GetPendingAddedLevelCount_ForTest() const { return PendingAddedLevels.Num(); }

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
    enum class EStaticGate : uint8 { Reject, ExcludedByName, Pass };
    static EStaticGate GateStaticComponent(const UStaticMeshComponent* Component);

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
    void  SyncSkeletalComponents(UWorld* World);
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

    /// Last printed rgl.Scene.* knob values (rgl_scene_config line).
    int32 LoggedMaxCreates = -1;
    float LoggedCreateBudgetMs = -1.0f;
    float LoggedUnloadFactor = -1.0f;
    int32 LoggedIdleMeshCacheMax = -1;
    int32 LoggedMaxIsmChanges = -1;

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
        // Self cutout (SetSensorSelfCutout); CutoutRadiusCm <= 0 = off.
        TWeakObjectPtr<AActor> CutoutVehicle;
        float      CutoutRadiusCm = 0.0f;
        float      CutoutMinElevationDeg = 0.0f;
        float      CutoutMaxElevationDeg = 0.0f;
        FTransform CutoutSensorToVehicle = FTransform::Identity;   // taken in the same tick as the vehicle pose
    };
    /// Key = sensor id (session handle). Pointer key uses pointer hashing.
    TMap<const void*, FSensorRegistration> RegisteredSensors;

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

    /// Time-based periodic sync interval (simulation seconds).
    float SyncIntervalSeconds = 1.0f;
    float TimeSinceLastSync = 0.0f;

    // ---- Scene audit bookkeeping ----
    double LastSyncSimTime = -1.0;        // CurrentSimTime of the last static sync; -1 = none yet
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

    // ---- Static singleton storage ----
    static TMap<UWorld*, FRGLSceneManager*> Instances;
};

#endif // WITH_RGL
