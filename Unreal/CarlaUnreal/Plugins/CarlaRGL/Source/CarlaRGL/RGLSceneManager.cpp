// Copyright (c) 2026 RGL Integration for CARLA.
// Implementation of FRGLSceneManager: UE5 <-> RGL scene synchronization.

#ifdef WITH_RGL

#include "RGLSceneManager.h"
#include "RGLCoordinateUtils.h"
#include "CarlaRGLModule.h"
#include "RGLSkeletalMeshExtractor.h"
#include "RGLDynLoader.h"

#include <cstdio>   // Shipping breadcrumbs: NO_LOGGING compiles UE_LOG out (precedent: RGLDynLoader.cpp)

#include <util/ue-header-guard-begin.h>
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/SkeletalMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SplineMeshComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/MovementComponent.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Rendering/PositionVertexBuffer.h"
#include "StaticMeshResources.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "EngineUtils.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include "Engine/Level.h"
#include "Components/WorldPartitionStreamingSourceComponent.h"
#include "WorldPartition/WorldPartition.h"
#include "WorldPartition/WorldPartitionSubsystem.h"
#include "WorldPartition/WorldPartitionRuntimeCell.h"
#include "WorldPartition/WorldPartitionStreamingSource.h"
#include "WorldPartition/HLOD/HLODActor.h"
#include <util/ue-header-guard-end.h>

#include <util/disable-ue4-macros.h>
#include <carla/Logging.h>
#include <util/enable-ue4-macros.h>

// RGL_CHECK macro is defined in RGLCoordinateUtils.h

// [DEBUG] Global counters for mesh extraction path statistics (Development only)
#if !UE_BUILD_SHIPPING
int32 GRGLExtractRender = 0, GRGLExtractGPU = 0, GRGLExtractChaos = 0, GRGLExtractFailed = 0;
int64 GRGLTotalVertices = 0, GRGLTotalTriangles = 0;
#endif

// ============================================================================
// Static singleton storage
// ============================================================================

// ---- Skeletal mesh path CVars (spec §2.3) ----
static TAutoConsoleVariable<int32> CVarRGLSkeletalEnable(TEXT("rgl.SkeletalMesh.Enable"), 1,
    TEXT("Register pawn-owned USkeletalMeshComponents in the RGL scene. 0 = legacy static-mesh-only (live: tears down)."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalAlwaysTickPose(TEXT("rgl.SkeletalMesh.AlwaysTickPose"), 1,
    TEXT("Force AlwaysTickPoseAndRefreshBones and disable URO on registered skeletal components (live: re-applied/restored at next sync)."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalScope(TEXT("rgl.SkeletalMesh.Scope"), 0,
    TEXT("0 = APawn-owned components only. 1 = every USkeletalMeshComponent."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalMaxEntities(TEXT("rgl.SkeletalMesh.MaxEntities"), 64,
    TEXT("Budget of simultaneously registered skeletal components; ranked by distance to the nearest sensor. "
         "Device memory is roughly 24 B per vertex per registered entity (RGL SkeletonAnimator + GAS), "
         "so a 350k-vertex LOD0 vehicle costs ~8 MB; lower this (or raise MinLOD) on VRAM-tight setups."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalMinLOD(TEXT("rgl.SkeletalMesh.MinLOD"), 0,
    TEXT("Lowest LOD index the extractor may use; raise to trade fidelity for VRAM/skinning cost."), ECVF_Default);
#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarRGLSkeletalDebugForceApiUnavailable(TEXT("rgl.SkeletalMesh.Debug.ForceApiUnavailable"), 0,
    TEXT("Test only: pretend the RGL skeleton API did not resolve."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalDebugForcePoseFailure(TEXT("rgl.SkeletalMesh.Debug.ForcePoseFailure"), 0,
    TEXT("Test only: make every skeletal pose build fail (exercises destroy->pending->recover)."), ECVF_Default);
#endif

// Shipping-safe rebuild-free kill switch (spec §6.4). A packaged Shipping server cannot
// take `-ExecCmds=` (UnrealEngine.cpp: #if !UE_BUILD_SHIPPING) nor `-ini:` overrides
// (ALLOW_INI_OVERRIDE_FROM_COMMANDLINE = UE_SERVER || !UE_BUILD_SHIPPING), so the CVars
// above would be unreachable from the command line. FParse::Value is compiled in for every
// configuration, so these switches work everywhere; they only *set* the CVars, which remain
// the single source of truth. ECVF_SetByCommandline ranks below ECVF_SetByCode, so console
// commands and the Automation tests can still override them at runtime.
static void ApplySkeletalCommandLineOverrides()
{
    static bool bApplied = false; if (bApplied) return; bApplied = true;
    int32 V = 0;
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-enable="), V))
    {
        CVarRGLSkeletalEnable->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: Enable=%d"), V);
    }
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-always-tick-pose="), V))
    {
        CVarRGLSkeletalAlwaysTickPose->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: AlwaysTickPose=%d"), V);
    }
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-scope="), V))
    {
        CVarRGLSkeletalScope->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: Scope=%d"), V);
    }
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-max-entities="), V))
    {
        CVarRGLSkeletalMaxEntities->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: MaxEntities=%d"), V);
    }
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-min-lod="), V))
    {
        CVarRGLSkeletalMinLOD->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: MinLOD=%d"), V);
    }
}

// ---- Static scene sync amortization CVars ----
// A sync that brings many new meshes into range used to extract, upload and BVH-build all of them in one
// game-thread frame. These knobs spread that work over successive scans. Setting
// MaxMeshCreatesPerUpdate=0, CreateBudgetMs=0, UnloadDistanceFactor=1, IdleMeshCacheMax=0 restores the
// original single-frame behaviour (A/B).
static TAutoConsoleVariable<int32> CVarRGLSceneMaxMeshCreatesPerUpdate(TEXT("rgl.Scene.MaxMeshCreatesPerUpdate"), 8,
    TEXT("Most new RGL meshes or idle reactivations per scene Update; the rest wait in a nearest-first queue. "
         "Components whose mesh already has live entities are registered at once. 0 = unlimited (original behaviour)."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneCreateBudgetMs(TEXT("rgl.Scene.CreateBudgetMs"), 4.0f,
    TEXT("Wall-time budget (ms) for mesh creation/idle reactivation per scene Update, checked between creations; "
         "at least one creatable mesh progresses. Excludes raytrace-time GAS builds. 0 = unlimited (original behaviour)."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneUnloadDistanceFactor(TEXT("rgl.Scene.UnloadDistanceFactor"), 1.0f,
    TEXT("Static meshes unload only beyond RegistrationDistance * factor + 20 m (load radius = RegistrationDistance). "
         "Values below 1 are treated as 1. 1 = original behaviour. Larger values keep more geometry resident (VRAM)."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSceneIdleMeshCacheMax(TEXT("rgl.Scene.IdleMeshCacheMax"), 0,
    TEXT("RGL meshes kept resident after their last entity is removed, for a cheap re-entry; least recently idled "
         "is destroyed first. 0 = destroy at once (original behaviour)."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSceneMaxIsmInstanceChangesPerUpdate(TEXT("rgl.Scene.MaxIsmInstanceChangesPerUpdate"), 512,
    TEXT("Most Static ISM instances added to or removed from the RGL scene per scene Update when a sync re-evaluates "
         "the registered ISM groups against the sensor spheres (on the Updates without a sync); the rest wait for the "
         "next Update. 0 = unlimited."), ECVF_Default);

// ---- The periodic static sync: its clock and interval ----
// The sync's time trigger. The original timer added the world dt once per scene Update, so the time between Updates that
// the dt does not cover (a 0.1 s LiDAR at a 0.025 s world dt: 0.075 s per Update) was not counted, and a 1 s interval took
// about 4 s of simulation time. rgl.Scene.SyncClock = 0 restores it (the A/B and the old-value runs).
static TAutoConsoleVariable<int32> CVarRGLSceneSyncClock(TEXT("rgl.Scene.SyncClock"), 1,
    TEXT("Clock of the periodic static sync: 1 = simulation time (a sync once rgl.Scene.SyncIntervalSeconds of simulation "
         "time have passed since the last sync), 0 = the world dt of the scene Updates only (the original timer; with Updates "
         "less often than every frame it runs slower than simulation time). Other values are taken as 1."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneSyncIntervalSeconds(TEXT("rgl.Scene.SyncIntervalSeconds"), 1.0f,
    TEXT("Seconds of the rgl.Scene.SyncClock clock between two periodic static syncs (at least 0.1). The distance, level "
         "and new-sensor triggers come on top of it."), ECVF_Default);

// ---- The range edge: the distance trigger and the queued parts already inside a range ----
// A sensor registers the parts within R = its range + 20 m. The distance trigger was 0.2 R (44 m for a 200 m lidar): a move
// of more than 20 m between syncs brought parts that were outside R at the last sync inside the range, unregistered, until
// the next sync (and then the create budget could hold them in the queue for a few more Updates). The cap keeps the trigger
// below that 20 m margin, and the in-range priority creates such parts in the Update that queues them. The legacy values
// (ratio 0.2, slack -1, in-range creates 0) restore the original behaviour (the A/B and the old-value runs).
static TAutoConsoleVariable<float> CVarRGLSceneSyncTriggerRatio(TEXT("rgl.Scene.SyncTriggerRatio"), 0.2f,
    TEXT("Distance trigger of the static sync: a sensor that moved more than this ratio x its registration radius R since the "
         "last sync syncs now (capped by rgl.Scene.SyncTriggerMarginSlackM). > 0."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneSyncTriggerMarginSlackM(TEXT("rgl.Scene.SyncTriggerMarginSlackM"), 5.0f,
    TEXT("Cap of the distance trigger: at most (R - the sensor's range) minus this (m), and at least 1 m, so that the sync "
         "comes before a part just outside R at the last sync can reach the range. Negative = no cap (ratio x R alone)."),
    ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSceneInRangeCreatesPerUpdate(TEXT("rgl.Scene.InRangeCreatesPerUpdate"), 16,
    TEXT("Queued static parts whose bounds reach some sensor's range (its rays reach them) are created first, outside "
         "rgl.Scene.MaxMeshCreatesPerUpdate / CreateBudgetMs: at most this many per scene Update; the rest wait for that "
         "budget like the other queued parts. >= 0, 0 = no priority (the original queue)."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneInRangeCreateBudgetMs(TEXT("rgl.Scene.InRangeCreateBudgetMs"), 8.0f,
    TEXT("Wall-time cap (ms) of those in-range creations per scene Update, checked between creations; at least one "
         "proceeds. >= 0, 0 = no ms cap."), ECVF_Default);

static int32 SceneMaxMeshCreatesPerUpdate() { return FMath::Max(0, CVarRGLSceneMaxMeshCreatesPerUpdate.GetValueOnGameThread()); }
static float SceneCreateBudgetMs()          { const float V = CVarRGLSceneCreateBudgetMs.GetValueOnGameThread(); return (V > 0.0f) ? V : 0.0f; }
static float SceneUnloadDistanceFactor()    { const float V = CVarRGLSceneUnloadDistanceFactor.GetValueOnGameThread(); return (V >= 1.0f) ? V : 1.0f; }
static int32 SceneIdleMeshCacheMax()        { return FMath::Max(0, CVarRGLSceneIdleMeshCacheMax.GetValueOnGameThread()); }
static int32 SceneMaxIsmInstanceChangesPerUpdate() { return FMath::Max(0, CVarRGLSceneMaxIsmInstanceChangesPerUpdate.GetValueOnGameThread()); }

// ---- Read-only scene audit (rgl_scene_audit line) ----
static TAutoConsoleVariable<int32> CVarRGLSceneAudit(TEXT("rgl.Scene.Audit"), 0,
    TEXT("Read-only scene audit: N >= 1 prints one rgl_scene_audit line every N-th scene Update (once per frame, "
         "however many RGL sensors tick), comparing the RGL static scene with the loaded world. Each line scans every "
         "actor, so leave it off (0, the default) when timing frames."), ECVF_Default);
static int32 SceneAuditEvery() { return FMath::Max(0, CVarRGLSceneAudit.GetValueOnGameThread()); }

// ---- Debug: RGL lidar hits to files (compared offline with ray_cast lidars) ----
static TAutoConsoleVariable<FString> CVarRGLDebugHitDumpDir(TEXT("rgl.Debug.HitDumpDir"), TEXT(""),
    TEXT("Debug: directory that receives one <sensor-id>_<frame>.bin per dumped RGL lidar scan (hit points in CARLA "
         "world coordinates, m). Empty = off. While set, hits are read back from the GPU even when no client listens."),
    ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLDebugHitDumpEvery(TEXT("rgl.Debug.HitDumpEvery"), 10,
    TEXT("Debug: with rgl.Debug.HitDumpDir set, dump each sensor's first scan and then every N-th one."), ECVF_Default);

// Scene telemetry: one grep-friendly key=value line. RGLLog::Info is compiled out in Shipping (RGLLog.h),
// and the packaged Shipping server is what the E2E runs, so Shipping prints the same line to stderr
// (precedent: the skeletal breadcrumbs below).
static void EmitSceneLine(const FString& Line)
{
#if UE_BUILD_SHIPPING
    fprintf(stderr, "%s\n", TCHAR_TO_UTF8(*Line));
#else
    RGLLog::Info(TCHAR_TO_UTF8(*Line));
#endif
}

// ---- World Partition culling of non-Static parts (7b; README_RGL "non-Static parts on World Partition maps") ----
// Only a map with a World Partition uses these; on other maps every non-Static part is registered at any distance.
static TAutoConsoleVariable<int32> CVarRGLSceneWpMovableCulling(TEXT("rgl.Scene.WpMovableCulling"), 1,
    TEXT("World Partition maps only: 1 = non-Static static mesh parts that are not vehicle/walker/physics/sensor-carrier "
         "parts and were not seen moving are chosen against the sensor spheres with a speed-dependent margin. 0 = the "
         "original rule (every non-Static part at any distance). Maps without World Partition always use the original rule."),
    ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneMovableMinMarginM(TEXT("rgl.Scene.MovableMinMarginM"), 20.0f,
    TEXT("World Partition culling: smallest registration margin beyond a sensor's range (m, >= 20)."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneMovableSensorSpeedBoundMps(TEXT("rgl.Scene.MovableSensorSpeedBoundMps"), 50.0f,
    TEXT("World Partition culling: the speed a sensor is assumed to stay under (m/s, > 0); faster = a pose jump (logged, "
         "forces a sync)."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneMovableObjectSpeedBoundMps(TEXT("rgl.Scene.MovableObjectSpeedBoundMps"), 50.0f,
    TEXT("World Partition culling: the speed a culled part that starts moving is assumed to stay under (m/s, > 0). Parts "
         "seen moving are kept at any distance afterwards."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneMovableMaxUpdateGapSeconds(TEXT("rgl.Scene.MovableMaxUpdateGapSeconds"), 0.1f,
    TEXT("World Partition culling: the longest sim time between two scene Updates and between two pose refreshes of a "
         "sensor (s, > 0); longer gaps are logged as violations."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneMovableCreateLatencySeconds(TEXT("rgl.Scene.MovableCreateLatencySeconds"), 1.0f,
    TEXT("World Partition culling: sim time within which a chosen part's entities must be created (s, > 0); a longer "
         "wait is logged as a violation."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSceneMaxMovableEntityChangesPerUpdate(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 512,
    TEXT("World Partition culling: most entities (a regular part or one ISM instance) added or removed per scene Update "
         "(>= 0, 0 = no cap)."), ECVF_Default);
static TAutoConsoleVariable<float> CVarRGLSceneMovableEntityBudgetMs(TEXT("rgl.Scene.MovableEntityBudgetMs"), 4.0f,
    TEXT("World Partition culling: wall time (ms) for those changes per scene Update, checked between changes after the "
         "first one (>= 0, 0 = no cap). At least one waiting change proceeds."), ECVF_Default);

// A value outside its range (or NaN / Inf) is not used: one warning line per knob and value, and the default instead.
static void WarnInvalidKnob(const TCHAR* Name, double Value, double Fallback)
{
    static TSet<FString> Warned;
    const FString Key = FString::Printf(TEXT("%s=%g"), Name, Value);
    if (Warned.Contains(Key))
    {
        return;
    }
    Warned.Add(Key);
    EmitSceneLine(FString::Printf(TEXT("rgl_scene_config_warning knob=%s value=%g fallback=%g"), Name, Value, Fallback));
}
static float ValidFloatKnob(TAutoConsoleVariable<float>& CVar, const TCHAR* Name, float Default, float Min, bool bExclusiveMin)
{
    const float V = CVar.GetValueOnGameThread();
    const bool bOk = FMath::IsFinite(V) && (bExclusiveMin ? V > Min : V >= Min);
    if (!bOk)
    {
        WarnInvalidKnob(Name, V, Default);
        return Default;
    }
    return V;
}
static int32 SceneWpMovableCulling()
{
    const int32 V = CVarRGLSceneWpMovableCulling.GetValueOnGameThread();
    if (V != 0 && V != 1)
    {
        WarnInvalidKnob(TEXT("rgl.Scene.WpMovableCulling"), V, 1);
        return 1;
    }
    return V;
}
static float SceneMovableMinMarginM()            { return ValidFloatKnob(CVarRGLSceneMovableMinMarginM, TEXT("rgl.Scene.MovableMinMarginM"), 20.0f, 20.0f, false); }
static float SceneMovableSensorSpeedBoundMps()   { return ValidFloatKnob(CVarRGLSceneMovableSensorSpeedBoundMps, TEXT("rgl.Scene.MovableSensorSpeedBoundMps"), 50.0f, 0.0f, true); }
static float SceneMovableObjectSpeedBoundMps()   { return ValidFloatKnob(CVarRGLSceneMovableObjectSpeedBoundMps, TEXT("rgl.Scene.MovableObjectSpeedBoundMps"), 50.0f, 0.0f, true); }
static float SceneMovableMaxUpdateGapSeconds()   { return ValidFloatKnob(CVarRGLSceneMovableMaxUpdateGapSeconds, TEXT("rgl.Scene.MovableMaxUpdateGapSeconds"), 0.1f, 0.0f, true); }
static float SceneMovableCreateLatencySeconds()  { return ValidFloatKnob(CVarRGLSceneMovableCreateLatencySeconds, TEXT("rgl.Scene.MovableCreateLatencySeconds"), 1.0f, 0.0f, true); }
static float SceneMovableEntityBudgetMs()        { return ValidFloatKnob(CVarRGLSceneMovableEntityBudgetMs, TEXT("rgl.Scene.MovableEntityBudgetMs"), 4.0f, 0.0f, false); }
static int32 SceneMaxMovableEntityChangesPerUpdate()
{
    const int32 V = CVarRGLSceneMaxMovableEntityChangesPerUpdate.GetValueOnGameThread();
    if (V < 0)
    {
        WarnInvalidKnob(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), V, 512);   // never "no cap"
        return 512;
    }
    return V;
}
static int32 SceneSyncClock()
{
    const int32 V = CVarRGLSceneSyncClock.GetValueOnGameThread();
    if (V != 0 && V != 1)
    {
        WarnInvalidKnob(TEXT("rgl.Scene.SyncClock"), V, 1);
        return 1;
    }
    return V;
}
static float SceneSyncIntervalSeconds()
{
    const float V = CVarRGLSceneSyncIntervalSeconds.GetValueOnGameThread();
    if (!FMath::IsFinite(V))
    {
        WarnInvalidKnob(TEXT("rgl.Scene.SyncIntervalSeconds"), V, 1.0);
        return 1.0f;
    }
    if (V < 0.1f)
    {
        WarnInvalidKnob(TEXT("rgl.Scene.SyncIntervalSeconds"), V, 0.1);   // the floor SetSyncInterval has always had
        return 0.1f;
    }
    return V;
}
static float SceneSyncTriggerRatio()       { return ValidFloatKnob(CVarRGLSceneSyncTriggerRatio, TEXT("rgl.Scene.SyncTriggerRatio"), 0.2f, 0.0f, true); }
static float SceneInRangeCreateBudgetMs()  { return ValidFloatKnob(CVarRGLSceneInRangeCreateBudgetMs, TEXT("rgl.Scene.InRangeCreateBudgetMs"), 8.0f, 0.0f, false); }
static float SceneSyncTriggerMarginSlackM()
{
    const float V = CVarRGLSceneSyncTriggerMarginSlackM.GetValueOnGameThread();
    if (!FMath::IsFinite(V))   // any finite value is valid (negative = no cap)
    {
        WarnInvalidKnob(TEXT("rgl.Scene.SyncTriggerMarginSlackM"), V, 5.0);
        return 5.0f;
    }
    return V;
}
static int32 SceneInRangeCreatesPerUpdate()
{
    const int32 V = CVarRGLSceneInRangeCreatesPerUpdate.GetValueOnGameThread();
    if (V < 0)
    {
        WarnInvalidKnob(TEXT("rgl.Scene.InRangeCreatesPerUpdate"), V, 16);
        return 16;
    }
    return V;
}

// The first RGL call of the sync's measured interval is timed (wait_ms=); other calls pass straight through.
#define RGL_PROBED(Api, Expr) WaitProbe.Call(TEXT(Api), [&]() { return (Expr); })

// Shipping-safe command-line switches for the rgl.* scene CVars (same reasoning as the skeletal ones above). Omitted
// switches keep the CVar defaults: 8 creates, 4 ms, unload factor 1.0, idle cache 0, 512 ISM instance changes, audit off,
// hit dump off (every 10th scan once a directory is given), the World Partition culling's knobs (above), the sync on the
// simulation clock every 1 s, the distance trigger 0.2 R capped 5 m below the margin between R and the range, 16 in-range
// creates per Update within 8 ms.
void FRGLSceneManager::ApplySceneSwitches(const TCHAR* CommandLine, EConsoleVariableFlags SetBy)
{
    if (!CommandLine)
    {
        return;
    }
    int32 I = 0;
    float F = 0.0f;
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-max-mesh-creates-per-update="), I))
    {
        CVarRGLSceneMaxMeshCreatesPerUpdate->Set(I, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-create-budget-ms="), F))
    {
        CVarRGLSceneCreateBudgetMs->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-unload-distance-factor="), F))
    {
        CVarRGLSceneUnloadDistanceFactor->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-idle-mesh-cache-max="), I))
    {
        CVarRGLSceneIdleMeshCacheMax->Set(I, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-max-ism-instance-changes-per-update="), I))
    {
        CVarRGLSceneMaxIsmInstanceChangesPerUpdate->Set(I, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-audit="), I))
    {
        CVarRGLSceneAudit->Set(I, SetBy);
    }
    FString S;
    if (FParse::Value(CommandLine, TEXT("-rgl-debug-hit-dump="), S))
    {
        CVarRGLDebugHitDumpDir->Set(*S, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-debug-hit-dump-every="), I))
    {
        CVarRGLDebugHitDumpEvery->Set(I, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-sync-clock="), I))
    {
        CVarRGLSceneSyncClock->Set(I, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-sync-interval-seconds="), F))
    {
        CVarRGLSceneSyncIntervalSeconds->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-sync-trigger-ratio="), F))
    {
        CVarRGLSceneSyncTriggerRatio->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-sync-trigger-margin-slack-m="), F))
    {
        CVarRGLSceneSyncTriggerMarginSlackM->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-in-range-creates-per-update="), I))
    {
        CVarRGLSceneInRangeCreatesPerUpdate->Set(I, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-in-range-create-budget-ms="), F))
    {
        CVarRGLSceneInRangeCreateBudgetMs->Set(F, SetBy);
    }
    // The World Partition culling of non-Static parts (7b).
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-wp-movable-culling="), I))
    {
        CVarRGLSceneWpMovableCulling->Set(I, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-movable-min-margin-m="), F))
    {
        CVarRGLSceneMovableMinMarginM->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-movable-sensor-speed-bound-mps="), F))
    {
        CVarRGLSceneMovableSensorSpeedBoundMps->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-movable-object-speed-bound-mps="), F))
    {
        CVarRGLSceneMovableObjectSpeedBoundMps->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-movable-max-update-gap-seconds="), F))
    {
        CVarRGLSceneMovableMaxUpdateGapSeconds->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-movable-create-latency-seconds="), F))
    {
        CVarRGLSceneMovableCreateLatencySeconds->Set(F, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-max-movable-entity-changes-per-update="), I))
    {
        CVarRGLSceneMaxMovableEntityChangesPerUpdate->Set(I, SetBy);
    }
    if (FParse::Value(CommandLine, TEXT("-rgl-scene-movable-entity-budget-ms="), F))
    {
        CVarRGLSceneMovableEntityBudgetMs->Set(F, SetBy);
    }
}

// The process command line's switches, once (the first manager, or the first hit-dump query).
static void ApplySceneCommandLineOverrides()
{
    static bool bApplied = false; if (bApplied) return; bApplied = true;
    FRGLSceneManager::ApplySceneSwitches(FCommandLine::Get(), ECVF_SetByCommandline);
}

void FRGLSceneManager::SetSyncInterval(float Seconds)
{
    CVarRGLSceneSyncIntervalSeconds->Set(FMath::Max(0.1f, Seconds), ECVF_SetByCode);
}

FRGLSceneManager::FSceneKnobs FRGLSceneManager::ReadSceneKnobs()
{
    FSceneKnobs K;
    K.MaxMeshCreatesPerUpdate = SceneMaxMeshCreatesPerUpdate();
    K.CreateBudgetMs = SceneCreateBudgetMs();
    K.UnloadDistanceFactor = SceneUnloadDistanceFactor();
    K.IdleMeshCacheMax = SceneIdleMeshCacheMax();
    K.MaxIsmInstanceChangesPerUpdate = SceneMaxIsmInstanceChangesPerUpdate();
    K.SyncClock = SceneSyncClock();
    K.SyncIntervalSeconds = SceneSyncIntervalSeconds();
    K.SyncTriggerRatio = SceneSyncTriggerRatio();
    K.SyncTriggerMarginSlackM = SceneSyncTriggerMarginSlackM();
    K.InRangeCreatesPerUpdate = SceneInRangeCreatesPerUpdate();
    K.InRangeCreateBudgetMs = SceneInRangeCreateBudgetMs();
    K.WpMovableCulling = SceneWpMovableCulling();
    K.MovableMinMarginM = SceneMovableMinMarginM();
    K.MovableSensorSpeedBoundMps = SceneMovableSensorSpeedBoundMps();
    K.MovableObjectSpeedBoundMps = SceneMovableObjectSpeedBoundMps();
    K.MovableMaxUpdateGapSeconds = SceneMovableMaxUpdateGapSeconds();
    K.MovableCreateLatencySeconds = SceneMovableCreateLatencySeconds();
    K.MaxMovableEntityChangesPerUpdate = SceneMaxMovableEntityChangesPerUpdate();
    K.MovableEntityBudgetMs = SceneMovableEntityBudgetMs();
    return K;
}

FString FRGLSceneManager::FormatSceneConfigLine(double SimTime, const FSceneKnobs& K)
{
    // The sync's knobs (clock, interval, the range-edge ones) together after the older keys, then the 7b keys;
    // wp_culling = the culling applies to this world (World Partition and enabled).
    return FString::Printf(
        TEXT("rgl_scene_config t=%.3f max_mesh_creates_per_update=%d create_budget_ms=%.2f unload_distance_factor=%.2f "
             "idle_mesh_cache_max=%d max_ism_instance_changes_per_update=%d unload_margin_cm=%.0f sync_clock=%s "
             "sync_interval_s=%.2f sync_trigger_ratio=%.2f sync_trigger_margin_slack_m=%.1f in_range_creates_per_update=%d "
             "in_range_create_budget_ms=%.2f wp_culling=%d wp_movable_culling=%d movable_min_margin_m=%.1f "
             "movable_sensor_speed_bound_mps=%.1f movable_object_speed_bound_mps=%.1f movable_max_update_gap_seconds=%.3f "
             "movable_create_latency_seconds=%.3f max_movable_entity_changes_per_update=%d movable_entity_budget_ms=%.2f"),
        SimTime, K.MaxMeshCreatesPerUpdate, K.CreateBudgetMs, K.UnloadDistanceFactor, K.IdleMeshCacheMax,
        K.MaxIsmInstanceChangesPerUpdate, K.UnloadMarginCm, K.SyncClock == 1 ? TEXT("sim") : TEXT("update"),
        K.SyncIntervalSeconds, K.SyncTriggerRatio, K.SyncTriggerMarginSlackM, K.InRangeCreatesPerUpdate,
        K.InRangeCreateBudgetMs, K.WpMap, K.WpMovableCulling, K.MovableMinMarginM, K.MovableSensorSpeedBoundMps,
        K.MovableObjectSpeedBoundMps, K.MovableMaxUpdateGapSeconds, K.MovableCreateLatencySeconds,
        K.MaxMovableEntityChangesPerUpdate, K.MovableEntityBudgetMs);
}

TMap<UWorld*, FRGLSceneManager*> FRGLSceneManager::Instances;

FRGLSceneManager::FRGLSceneManager()
{
    // Apply the Shipping-safe command-line switches before anything reads the CVars.
    ApplySkeletalCommandLineOverrides();
    ApplySceneCommandLineOverrides();

    // Use the default scene (nullptr).
    // RGL treats nullptr as the implicit default scene.
    Scene = nullptr;

    bSkeletalApiAvailable = RGLDynLoader::IsSkeletalApiAvailable();
    if (!bSkeletalApiAvailable)
    {
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager: RGL skeleton API unavailable; skeletal meshes will not be registered."));
    }
}

FRGLSceneManager::~FRGLSceneManager()
{
    // The LevelAddedToWorld binding points at this instance: drop it before anything else
    // (OnWorldCleanup -> DestroyInstance during load_world, stale-world cleanup, module shutdown).
    if (LevelAddedHandle.IsValid())
    {
        FWorldDelegates::LevelAddedToWorld.Remove(LevelAddedHandle);
        LevelAddedHandle.Reset();
    }
    PendingAddedLevels.Empty();

    TeardownSkeletal();

    // Destroy ground plane
    if (GroundEntity)
    {
        rgl_entity_destroy(GroundEntity);
        GroundEntity = nullptr;
    }
    if (GroundMesh)
    {
        rgl_mesh_destroy(GroundMesh);
        GroundMesh = nullptr;
    }

    // Destroy all entities first (they reference meshes)
    for (auto& Pair : EntityMap)
    {
        if (Pair.Value.Entity)
        {
            rgl_entity_destroy(Pair.Value.Entity);
            Pair.Value.Entity = nullptr;
        }
    }
    EntityMap.Empty();

    // Destroy all ISMC entities
    for (auto& Pair : ISMCEntityMap)
    {
        for (rgl_entity_t Entity : Pair.Value.Entities)
        {
            if (Entity)
            {
                rgl_entity_destroy(Entity);
            }
        }
    }
    ISMCEntityMap.Empty();

    // Destroy all cached meshes (live and idle alike: idle meshes live in MeshCache too, so each RGL
    // mesh is destroyed exactly once here and IdleMeshLru only drops its keys).
    for (auto& Pair : MeshCache)
    {
        if (Pair.Value.Mesh)
        {
            rgl_mesh_destroy(Pair.Value.Mesh);
        }
    }
    MeshCache.Empty();
    MeshRefCounts.Empty();
    IdleMeshLru.Empty();
    PendingCreates.Empty();
    PendingIsmReevals.Empty();
    FailedMeshRetrySync.Empty();
}

FRGLSceneManager& FRGLSceneManager::GetInstance(UWorld* World)
{
    // Clean up stale entries for worlds that are no longer valid
    TArray<UWorld*> StaleWorlds;
    for (auto& Pair : Instances)
    {
        if (!IsValid(Pair.Key))
        {
            StaleWorlds.Add(Pair.Key);
        }
    }
    for (UWorld* Stale : StaleWorlds)
    {
        RGLLog::Info("RGLSceneManager: Cleaning up stale instance for destroyed world");
        delete Instances[Stale];
        Instances.Remove(Stale);
    }

    FRGLSceneManager** Found = Instances.Find(World);
    if (Found && *Found)
    {
        return **Found;
    }

    FRGLSceneManager* NewInstance = new FRGLSceneManager();
    NewInstance->BindWorld(World);
    Instances.Add(World, NewInstance);
    return *NewInstance;
}

void FRGLSceneManager::DestroyInstance(UWorld* World)
{
    FRGLSceneManager** Found = Instances.Find(World);
    if (Found && *Found)
    {
        delete *Found;
        Instances.Remove(World);
    }
}

void FRGLSceneManager::DestroyAllInstances()
{
    for (auto& Pair : Instances) delete Pair.Value;
    Instances.Empty();
}

// ============================================================================
// Update: called each frame before raytrace
// ============================================================================

void FRGLSceneManager::Update(UWorld* World, double SimulationTime)
{
    if (!World)
    {
        return;
    }

    // Frame skip: avoid redundant updates if already called this frame
    const uint64 CurrentFrame = GFrameCounter;
    if (bInitialized && CurrentFrame == LastUpdateFrame)
    {
        return;
    }
    LastUpdateFrame = CurrentFrame;
    UpdateStep(World, SimulationTime);
}

void FRGLSceneManager::UpdateStep(UWorld* World, double SimulationTime)
{
    // Wall time between two Updates that ran (frame_ms of the rgl_scene_audit line).
    const double UpdateWallSeconds = FPlatformTime::Seconds();
    LastFrameMs = (LastUpdateWallSeconds > 0.0) ? (UpdateWallSeconds - LastUpdateWallSeconds) * 1000.0 : 0.0;
    LastUpdateWallSeconds = UpdateWallSeconds;

    // 7b: whether this world culls its non-Static parts by distance is known before the first scan, so that
    // InitializeFromWorld leaves the parts the culling chooses to its queue (no far part is created to be removed later).
    bWpActive = UsesWpNonStaticCulling(World);
    const bool bStartupUpdate = !bInitialized;
    if (!bInitialized)
    {
        InitializeFromWorld(World);
        bInitialized = true;
    }

    // Track sim time for stale-sensor eviction (used inside SyncWorldComponents).
    CurrentSimTime = (SimulationTime >= 0.0) ? SimulationTime : World->GetTimeSeconds();

    const FSceneKnobs Knobs = LogSceneConfigIfChanged();

    // Sync on a time interval OR when any sensor has moved far enough that newly
    // relevant geometry could be missed before the next periodic sync
    // (Phase 1: high-speed safety; P8: the distance trigger stays below the margin between R and the range).
    // The time, sensor and distance conditions are all evaluated (the sensor loop is a handful of entries) so the
    // telemetry can name the trigger; a sync happens when any of them, or the level condition below, holds.
    // The time condition's clock (rgl.Scene.SyncClock): simulation time since the last sync (1, the default; time going
    // back before the last sync is due at once rather than waiting to catch up), or the world dt added once per scene
    // Update (0, the original timer: with Updates less often than every frame it runs slower than simulation time).
    const float IntervalSeconds = SceneSyncIntervalSeconds();
    TimeSinceLastSync += WorldDeltaSeconds(World);
    const bool bTimeDue = (SceneSyncClock() == 1)
        ? (LastSyncSimTime < 0.0 || CurrentSimTime < LastSyncSimTime || CurrentSimTime - LastSyncSimTime >= IntervalSeconds)
        : (TimeSinceLastSync >= IntervalSeconds);
    bool bNewSensor = false;   // a sensor without a previous sync (startup, newly spawned or re-registered sensor)
    bool bMoved = false;
    for (const auto& Pair : RegisteredSensors)
    {
        const FSensorRegistration& S = Pair.Value;
        if (!S.bLastSyncValid)
        {
            bNewSensor = true;
        }
        else if (FVector::Dist(S.Position, S.LastSyncPosition) > SyncTriggerCm(S, Knobs))
        {
            bMoved = true;
        }
    }
    // P5: a level added since the last sync whose static geometry reaches some sensor's R (World Partition cells
    // stream in between the periodic syncs). Looked at whenever no sensor is new, so the trigger name follows the
    // priority init > level > distance > time; the levels' actors only, at most once per 0.3 s (sim) (D5).
    const bool bLevel = !bNewSensor && ConsumeRelevantLevelAdds();
    // 7b (World Partition only; O(sensors), no candidate is read): the U / refresh / speed contracts, and a sync forced
    // by the deadline (the sim time the last culling sync's margin assumed would pass before the next one) or by a
    // reset (time going back, a sensor pose jump, a knob change).
    bool bWpDeadline = false;
    bool bWpReset = false;
    bool bWpJump = false;
    if (bWpActive)
    {
        WpCheckContracts(World, bWpDeadline, bWpReset, bWpJump);
    }
    bWpSyncDiscontinuous = bNewSensor || bWpJump;
    const bool bShouldSync = bTimeDue || bNewSensor || bMoved || bLevel || bWpDeadline || bWpReset;
    const TCHAR* Trigger = bNewSensor ? TEXT("init") : (bLevel ? TEXT("level") : (bMoved ? TEXT("distance") :
        (bTimeDue ? TEXT("time") : (bWpReset ? TEXT("wp_reset") : TEXT("wp_deadline")))));

    const double StepStartSeconds = FPlatformTime::Seconds();
    FSceneSyncStats SyncStats;
    // Sensors synced for the first time by this Update: their rgl_scene_sensor lines follow the sync line.
    TArray<const void*, TInlineAllocator<8>> FirstSyncSensors;
    if (bShouldSync)
    {
        // wait_ms=: the first RGL call from here to elapsed_ms is timed (nothing is called, reordered or added for it).
        WaitProbe.Arm();
        SyncWorldComponents(World, SyncStats);
        TimeSinceLastSync = 0.0f;
        LastSyncSimTime = CurrentSimTime;
        for (auto& Pair : RegisteredSensors)
        {
            FSensorRegistration& S = Pair.Value;
            if (!S.bLastSyncValid)
            {
                FirstSyncSensors.Add(Pair.Key);
            }
            if (bWpActive && !S.bLastSyncValid)
            {
                S.WpInitSyncSimTime = CurrentSimTime;
                S.WpInitPoseTime = S.LastUpdateTime;
                S.bWpStartupPosePending = true;
                S.bWpStartupUpdatePending = true;
            }
            S.LastSyncPosition = S.Position;
            S.bLastSyncValid = true;
        }
        if (bWpActive)
        {
            // Snapshot the sensor set active at the interval start, before another Update can add sensors.
            bWpStartupSceneIntervalPending = !RegisteredSensors.IsEmpty();
            for (const auto& Pair : RegisteredSensors)
            {
                bWpStartupSceneIntervalPending &= Pair.Value.bWpStartupUpdatePending &&
                    Pair.Value.WpInitSyncSimTime == CurrentSimTime;
            }
        }
        if (!bNewSensor && !bLevel && !bMoved && !bTimeDue && !bWpReset && bWpDeadline)
        {
            ++WpDeadlineSyncs;
        }
#if !UE_BUILD_SHIPPING
        ++SyncCount_ForTest;
        LastSyncTrigger_ForTest = Trigger;
#endif
    }

    // Time-sliced creation of the meshes queued by this or an earlier sync: a step on every Update, sync
    // due or not. A sensor's first sync (startup, or a sensor spawned later) is drained in full, as before:
    // that is not the driving stall, and localization start-up should see the complete scene. Otherwise the queued parts
    // already inside some sensor's range go first, outside the budget (up to the in-range caps, P8).
    if (!QueueEpisode.bActive && PendingCreates.Num() > 0)
    {
        QueueEpisode = FQueueEpisode();
        QueueEpisode.bActive = true;
        QueueEpisode.StartSimTime = CurrentSimTime;
    }
    FDrainStats Drain;
    DrainPendingCreates(/*bUnlimited=*/bNewSensor, Drain);
    if (QueueEpisode.bActive)
    {
        ++QueueEpisode.Updates;
        QueueEpisode.Created += Drain.Created;
        QueueEpisode.Failed += Drain.Failed;
        QueueEpisode.Dropped += Drain.Dropped;
    }

    // 7b: prefetch work takes a budgeted step on every Update; R6 raw-range discovery jobs finish without a budget.
    // A sync Update includes both in elapsed_ms; the Update that initialized the scene drains it in full (the start-up scene is complete, as above).
#if !UE_BUILD_SHIPPING
    WpAddedLastUpdate_ForTest = 0;
    WpRemovedLastUpdate_ForTest = 0;
#endif
    WpInRangeAddedLastUpdate = 0;
    WpInRangeMsLastUpdate = 0.0;
    if (bWpActive && WpJobs.Num() > 0)
    {
        WpDrain(/*bUnlimited=*/bStartupUpdate);
    }

    // P3 (D7): the Static ISM groups a sync queued are compared with the sensor spheres on the Updates without a sync,
    // so the frame of a sync does not also carry the instance changes; at most rgl.Scene.MaxIsmInstanceChangesPerUpdate
    // changes per Update. Its time is reported as ism_ms / ism_max_ms, not in the next sync's elapsed_ms.
    if (!bShouldSync)
    {
        FIsmReevalStats Ism;
        DrainIsmReevaluations(SceneMaxIsmInstanceChangesPerUpdate(), Ism);
        IsmAddedSinceLog += Ism.Added;
        IsmRemovedSinceLog += Ism.Removed;
        IsmMsSinceLog += Ism.Ms;
        IsmMaxMsSinceLog = FMath::Max(IsmMaxMsSinceLog, Ism.Ms);
    }

    if (bShouldSync)
    {
        // The sync and this Update's create step (the ISM re-evaluation never runs in a sync's Update).
        const double ElapsedMs = (FPlatformTime::Seconds() - StepStartSeconds) * 1000.0;
        WaitProbe.Disarm();
        // Totals for the scene size (outside elapsed_ms): every entity (ISM instances and live skeletal entities
        // included, the ground plane not), and the non-Static static mesh ones.
        int32 EntitiesTotal = 0;
        int32 NonStaticRegistered = 0;
        CountSceneEntities(EntitiesTotal, NonStaticRegistered);
        double WpQueueAgeMax = 0.0;
        for (const FWpJob& J : WpJobs)
        {
            if (J.bAdd)
            {
                WpQueueAgeMax = FMath::Max(WpQueueAgeMax, CurrentSimTime - J.SinceSimTime);
            }
        }
        int32 WpPromoted = 0;
        for (const auto& Pair : WpCandidates)
        {
            WpPromoted += Pair.Value.bMoved ? 1 : 0;
        }
        FString SyncLine = FString::Printf(
            TEXT("rgl_scene_sync t=%.3f trigger=%s scanned=%d added_cached=%d queued=%d created=%d removed=%d "
                 "idle_destroyed=%d pending=%d elapsed_ms=%.2f drain_ms=%.2f failed=%d drain_reused=%d dropped=%d "
                 "range_skipped=%d sensors=%d entities=%d ismc=%d meshes=%d idle=%d dead_ismc=%d "
                 "ism_added=%d ism_removed=%d ism_pending=%d ism_empty_skipped=%d ism_ms=%.2f ism_max_ms=%.2f "
                 "scan_ms=%.2f reg_ms=%.2f remove_ms=%.2f skel_ms=%.2f"),
            CurrentSimTime, Trigger, SyncStats.ScannedActors, SyncStats.AddedCached, SyncStats.Queued,
            Drain.Created, SyncStats.Removed, IdleDestroyedSinceLog, PendingCreates.Num(), ElapsedMs, Drain.Ms,
            MeshFailuresSinceLog, Drain.RegisteredCached, Drain.Dropped, SyncStats.RangeSkipped, RegisteredSensors.Num(),
            EntityMap.Num(), ISMCEntityMap.Num(), MeshCache.Num(), IdleMeshLru.Num(), DeadIsmcSinceLog,
            IsmAddedSinceLog, IsmRemovedSinceLog, IsmPendingAtSync, IsmEmptySkippedSinceLog, IsmMsSinceLog, IsmMaxMsSinceLog,
            SyncStats.ScanMs, SyncStats.RegMs, SyncStats.RemoveMs, SyncStats.SkelMs);
        // 7b keys (after every older key; the gate reads named keys only). wait_ms: the first RGL call of the measured
        // interval (it carries the wait for the previous raytrace graph and its own work; 0.00 / none = no call).
        // wp_* : wp = the culling applies; sync bound, create latency and margin of this sync; pending jobs and the age
        // of the oldest waiting add (current values); check/entity counts and times since the previous line (max =
        // the longest single sync's look); promoted = candidates seen moving (current); violations since the previous
        // line; deadline syncs (whole run); entities_total / nonstatic_registered (current). wp_inrange_* count/time only
        // this Update's raw-range discovery burst, including upload and the first API wait (not since the last line).
        // P8 keys last: in_range_creates / in_range_ms = this Update's drain step, as created= / drain_ms= (the queued
        // parts inside a range created outside the create budget, and their wall time).
        SyncLine += FString::Printf(
            TEXT(" wait_ms=%.2f wait_api=%s wp=%d wp_sync_bound_s=%.2f wp_create_latency_s=%.2f wp_margin_m=%.1f "
                 "wp_pending=%d wp_queue_age_max_s=%.2f wp_check_n=%d wp_check_instances=%d wp_check_ms=%.2f "
                 "wp_check_max_ms=%.2f wp_entity_added=%d wp_entity_removed=%d wp_entity_ms=%.2f wp_promoted=%d "
                 "wp_violations=%d wp_deadline_syncs=%d wp_entities_total=%d wp_nonstatic_registered=%d "
                 "wp_inrange_added=%d wp_inrange_ms=%.2f in_range_creates=%d in_range_ms=%.2f"),
            WaitProbe.Ms, WaitProbe.Api, bWpActive ? 1 : 0, bWpActive ? WpAssumedSyncSeconds : 0.0,
            bWpActive ? static_cast<double>(WpSyncKnobs.CreateLatencyS) : 0.0, bWpActive ? WpMarginCm / 100.0 : 0.0,
            WpJobs.Num(), WpQueueAgeMax, WpCheckN, WpCheckInstances, WpCheckMs, WpCheckMaxMs, WpEntityAdded,
            WpEntityRemoved, WpEntityMs, WpPromoted, WpViolationsSinceLog, WpDeadlineSyncs,
            EntitiesTotal, NonStaticRegistered, WpInRangeAddedLastUpdate, WpInRangeMsLastUpdate,
            Drain.InRangeCreates, Drain.InRangeMs);
        EmitSceneLine(SyncLine);
#if !UE_BUILD_SHIPPING
        LastSyncLine_ForTest = SyncLine;
        LastSensorLines_ForTest.Reset();
#endif
        // Once per sensor (per registration), at its first sync: the numbers in effect for it (after the sync line, so
        // that elapsed_ms does not include the printing).
        for (const void* SensorId : FirstSyncSensors)
        {
            if (const FSensorRegistration* S = RegisteredSensors.Find(SensorId))
            {
                const FString SensorLine = FString::Printf(
                    TEXT("rgl_scene_sensor t=%.3f sensor=0x%llx registration_m=%.1f range_m=%.1f trigger_m=%.1f"),
                    CurrentSimTime, static_cast<unsigned long long>(reinterpret_cast<UPTRINT>(SensorId)),
                    S->RegistrationDistanceCm / 100.0, SensorRangeCm(*S) / 100.0, SyncTriggerCm(*S, Knobs) / 100.0);
                EmitSceneLine(SensorLine);
#if !UE_BUILD_SHIPPING
                LastSensorLines_ForTest.Add(SensorLine);
#endif
            }
        }
        WpCheckN = 0;
        WpCheckInstances = 0;
        WpCheckMs = 0.0;
        WpCheckMaxMs = 0.0;
        WpEntityAdded = 0;
        WpEntityRemoved = 0;
        WpEntityMs = 0.0;
        WpViolationsSinceLog = 0;
        IdleDestroyedSinceLog = 0;
        MeshFailuresSinceLog = 0;
        DeadIsmcSinceLog = 0;
        IsmAddedSinceLog = 0;
        IsmRemovedSinceLog = 0;
        IsmEmptySkippedSinceLog = 0;
        IsmMsSinceLog = 0.0;
        IsmMaxMsSinceLog = 0.0;

        // P4: after the sync line, so its elapsed_ms does not include the streaming queries. Returns at once on a map
        // without World Partition.
        CheckStreamCoverage(World);
    }

    if (QueueEpisode.bActive && PendingCreates.Num() == 0)
    {
        EmitSceneLine(FString::Printf(
            TEXT("rgl_scene_queue_drained t=%.3f wait_s=%.3f updates=%d created=%d failed=%d dropped=%d"),
            CurrentSimTime, CurrentSimTime - QueueEpisode.StartSimTime, QueueEpisode.Updates,
            QueueEpisode.Created, QueueEpisode.Failed, QueueEpisode.Dropped));
        QueueEpisode = FQueueEpisode();
    }

    // Update scene time for velocity computation and ROS2 timestamp synchronization.
    // Use SimulationTime (elapsed since episode start) if provided, otherwise fall back
    // to World->GetTimeSeconds() which includes UE5 engine startup time (~7s offset).
    const double SceneTime = (SimulationTime >= 0.0) ? SimulationTime : World->GetTimeSeconds();
    const uint64 TimeNs = static_cast<uint64>(SceneTime * 1e9);
    RGL_CHECK(rgl_scene_set_time(Scene, TimeNs));

    // Update only dynamic entity transforms (static entities skip after first set)
    UpdateTransforms();
    // 7b: the registered instances of ISM candidates seen moving follow their instance transforms.
    if (bWpActive && WpMovedIsm.Num() > 0)
    {
        WpUpdateMovedInstanceTransforms();
    }

    // Re-pose every registered skeletal entity for this scene time (no skip optimization).
    if (SkeletalEnabled())
    {
        UpdateSkeletalPoses();
    }
}

// ============================================================================
// InitializeFromWorld: first-time scan
// ============================================================================

void FRGLSceneManager::InitializeFromWorld(UWorld* World)
{
    RGLLog::Info("RGLSceneManager: Initializing from world...");
    RGLLog::Info("RGLSceneManager: Initializing with registered sensors=",
                 RegisteredSensors.Num());

    int32 EntityCount = 0;
    int32 SkippedCount = 0;
    int32 RangeSkippedCount = 0;
    int32 ActorCount = 0;
    int32 CompCount = 0;

    // [DEBUG] Diagnose what types of collision-bearing actors exist in the world
#if !UE_BUILD_SHIPPING
    {
        int32 LandscapeCount = 0;
        int32 StaticMeshActorCount = 0;
        int32 CollisionCompCount = 0;
        int32 NoStaticMeshCompCount = 0;

        for (TActorIterator<AActor> It(World); It; ++It)
        {
            AActor* Actor = *It;
            if (!IsValid(Actor)) continue;

            FString ClassName = Actor->GetClass()->GetName();

            if (ClassName.Contains(TEXT("Landscape")))
            {
                ++LandscapeCount;
            }

            TArray<UStaticMeshComponent*> SMComps;
            Actor->GetComponents<UStaticMeshComponent>(SMComps);
            if (SMComps.Num() > 0)
            {
                ++StaticMeshActorCount;
            }
            else
            {
                // Check if this actor has any primitive component with collision
                TArray<UPrimitiveComponent*> PrimComps;
                Actor->GetComponents<UPrimitiveComponent>(PrimComps);
                for (UPrimitiveComponent* Prim : PrimComps)
                {
                    if (Prim && Prim->IsCollisionEnabled())
                    {
                        ++CollisionCompCount;
                        // Log first few non-StaticMesh collision actors
                        static int32 sNonSMLog = 0;
                        if (sNonSMLog < 5)
                        {
                            RGLLog::Info("[DEBUG] Non-SM collision actor:",
                                         TCHAR_TO_UTF8(*Actor->GetName()),
                                         "class:", TCHAR_TO_UTF8(*ClassName),
                                         "comp:", TCHAR_TO_UTF8(*Prim->GetClass()->GetName()));
                            ++sNonSMLog;
                        }
                        break;
                    }
                }
                ++NoStaticMeshCompCount;
            }
        }

        RGLLog::Info("[DEBUG] World actor types: Landscape=", LandscapeCount,
                     "WithStaticMesh=", StaticMeshActorCount,
                     "WithoutStaticMesh=", NoStaticMeshCompCount,
                     "NonSM-Collision=", CollisionCompCount);
    }
#endif

    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!IsValid(Actor))
        {
            continue;
        }
        ++ActorCount;

        ScanComponents.Reset();   // one array for every actor (P7)
        Actor->GetComponents<UStaticMeshComponent>(ScanComponents);

        for (UStaticMeshComponent* Comp : ScanComponents)
        {
            if (IsValid(Comp) && Comp->GetStaticMesh())
            {
                ++CompCount;
            }

            bool bOutOfRange = false;
            if (!ShouldRegisterComponent(Comp, bOutOfRange))
            {
                if (bOutOfRange)
                {
                    ++RangeSkippedCount;
                }
                continue;
            }

            if (RegisterComponent(Comp))
            {
                ++EntityCount;
            }
            else
            {
                ++SkippedCount;
            }
        }
    }
    ScanComponents.Reset();

    if (SkeletalEnabled())
    {
        SyncSkeletalComponents(World);
    }

    const int32 MeshCount = MeshCache.Num();
    RGLLog::Info("RGLSceneManager: Scanned", ActorCount, "actors,", CompCount,
                 "mesh components. Uploaded", MeshCount, "unique meshes, created",
                 EntityCount, "entities (skipped", SkippedCount,
                 ", range-skipped", RangeSkippedCount, ")");

    const int32 ISMCCompCount = ISMCEntityMap.Num();
    int32 ISMCEntityTotal = 0;
    for (const auto& Pair : ISMCEntityMap)
    {
        ISMCEntityTotal += Pair.Value.Entities.Num();
    }
    RGLLog::Info("RGLSceneManager: ISMC components=", ISMCCompCount,
                 " total ISMC instances=", ISMCEntityTotal);

#if !UE_BUILD_SHIPPING
    // Print extraction path statistics
    {
        extern int32 GRGLExtractRender, GRGLExtractGPU, GRGLExtractChaos, GRGLExtractFailed;
        extern int64 GRGLTotalVertices, GRGLTotalTriangles;
        RGLLog::Info("[DEBUG] Mesh extraction paths: RenderData=", GRGLExtractRender,
                     "GPUBuffer=", GRGLExtractGPU,
                     "ChaosCollision=", GRGLExtractChaos, "Failed=", GRGLExtractFailed);
        RGLLog::Info("[DEBUG] Total scene geometry: vertices=", GRGLTotalVertices,
                     "triangles=", GRGLTotalTriangles);

        int32 StaticCount = 0, DynamicCount = 0;
        for (const auto& Pair : EntityMap)
        {
            if (Pair.Value.bIsStatic) ++StaticCount; else ++DynamicCount;
        }
        RGLLog::Info("[DEBUG] Entity mobility: Static=", StaticCount, "Dynamic=", DynamicCount);
        RGLLog::Info("[DEBUG] ISMC entity map: components=", ISMCEntityMap.Num());
    }
#endif
}

// ============================================================================
// Mesh upload: extract vertex/index data from UStaticMesh
// ============================================================================

/// Try to extract geometry from Chaos TriMeshes (complex collision, full triangle detail).
/// This is the primary path — preserves concave geometry unlike ConvexElems.
static bool ExtractFromChaosTriMeshes(UStaticMesh* StaticMesh,
                                      TArray<rgl_vec3f>& OutVertices,
                                      TArray<rgl_vec3i>& OutIndices)
{
    UBodySetup* BodySetup = StaticMesh->GetBodySetup();
    if (!BodySetup)
    {
        return false;
    }

    if (BodySetup->ChaosTriMeshes.Num() > 0)
    {
        // Merge all tri-meshes into one vertex/index set
        int32 VertexOffset = 0;

        for (const auto& TriMeshPtr : BodySetup->ChaosTriMeshes)
        {
            if (!TriMeshPtr.IsValid())
            {
                continue;
            }

            const Chaos::FTriangleMeshImplicitObject& TriMesh = *TriMeshPtr;
            const auto& Particles = TriMesh.Particles();
            const auto& Elements = TriMesh.Elements();
            const int32 NumVerts = static_cast<int32>(Particles.Size());
            const int32 NumTris = Elements.GetNumTriangles();

            if (NumVerts == 0 || NumTris == 0)
            {
                continue;
            }

            // Append vertices (in local space, cm -> m)
            const int32 PrevVertCount = OutVertices.Num();
            OutVertices.AddUninitialized(NumVerts);
            for (int32 i = 0; i < NumVerts; ++i)
            {
                const Chaos::FVec3 P = Particles.GetX(i);
                OutVertices[PrevVertCount + i] = { {
                    static_cast<float>(P[0]) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(P[1]) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(P[2]) * RGLCoord::UE_TO_RGL
                } };
            }

            // Append indices (offset by accumulated vertex count)
            const int32 PrevTriCount = OutIndices.Num();
            OutIndices.AddUninitialized(NumTris);

            if (Elements.RequiresLargeIndices())
            {
                const auto& IdxBuf = Elements.GetLargeIndexBuffer();
                for (int32 i = 0; i < NumTris; ++i)
                {
                    OutIndices[PrevTriCount + i] = { {
                        static_cast<int32>(IdxBuf[i][0]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][1]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][2]) + VertexOffset
                    } };
                }
            }
            else
            {
                const auto& IdxBuf = Elements.GetSmallIndexBuffer();
                for (int32 i = 0; i < NumTris; ++i)
                {
                    OutIndices[PrevTriCount + i] = { {
                        static_cast<int32>(IdxBuf[i][0]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][1]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][2]) + VertexOffset
                    } };
                }
            }

            VertexOffset += NumVerts;
        }

        return OutVertices.Num() > 0 && OutIndices.Num() > 0;
    }

    return false;
}

/// Fallback: extract from ConvexElems or BoxElems (simplified convex geometry).
/// Only used as last resort — convex hulls lose concave features.
static bool ExtractFromChaosConvexOrBox(UStaticMesh* StaticMesh,
                                        TArray<rgl_vec3f>& OutVertices,
                                        TArray<rgl_vec3i>& OutIndices)
{
    UBodySetup* BodySetup = StaticMesh->GetBodySetup();
    if (!BodySetup)
    {
        return false;
    }

    const FKAggregateGeom& AggGeom = BodySetup->AggGeom;
    if (AggGeom.ConvexElems.Num() > 0)
    {
        int32 VertexOffset = 0;
        for (const FKConvexElem& Convex : AggGeom.ConvexElems)
        {
            const int32 NumVerts = Convex.VertexData.Num();
            const int32 NumIdx = Convex.IndexData.Num();
            const int32 NumTris = NumIdx / 3;
            if (NumVerts == 0 || NumTris == 0)
            {
                continue;
            }

            const int32 PrevVertCount = OutVertices.Num();
            OutVertices.AddUninitialized(NumVerts);
            for (int32 i = 0; i < NumVerts; ++i)
            {
                const FVector& V = Convex.VertexData[i];
                OutVertices[PrevVertCount + i] = { {
                    static_cast<float>(V.X) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(V.Y) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(V.Z) * RGLCoord::UE_TO_RGL
                } };
            }

            const int32 PrevTriCount = OutIndices.Num();
            OutIndices.AddUninitialized(NumTris);
            for (int32 i = 0; i < NumTris; ++i)
            {
                OutIndices[PrevTriCount + i] = { {
                    static_cast<int32>(Convex.IndexData[i * 3 + 0]) + VertexOffset,
                    static_cast<int32>(Convex.IndexData[i * 3 + 1]) + VertexOffset,
                    static_cast<int32>(Convex.IndexData[i * 3 + 2]) + VertexOffset
                } };
            }

            VertexOffset += NumVerts;
        }

        return OutVertices.Num() > 0 && OutIndices.Num() > 0;
    }

    // Fallback: try box collision elements (common for walls, floors, building shells)
    if (AggGeom.BoxElems.Num() > 0)
    {
        int32 VertexOffset = 0;
        for (const FKBoxElem& Box : AggGeom.BoxElems)
        {
            // Generate 8 vertices and 12 triangles for each box
            const float HX = static_cast<float>(Box.X) * 0.5f * RGLCoord::UE_TO_RGL; // half-extent in meters
            const float HY = static_cast<float>(Box.Y) * 0.5f * RGLCoord::UE_TO_RGL;
            const float HZ = static_cast<float>(Box.Z) * 0.5f * RGLCoord::UE_TO_RGL;

            // Box center offset (in meters)
            const FVector C = Box.Center;
            const float CX = static_cast<float>(C.X) * RGLCoord::UE_TO_RGL;
            const float CY = static_cast<float>(C.Y) * RGLCoord::UE_TO_RGL;
            const float CZ = static_cast<float>(C.Z) * RGLCoord::UE_TO_RGL;

            const int32 PrevVert = OutVertices.Num();
            OutVertices.AddUninitialized(8);
            OutVertices[PrevVert + 0] = {{ CX - HX, CY - HY, CZ - HZ }};
            OutVertices[PrevVert + 1] = {{ CX + HX, CY - HY, CZ - HZ }};
            OutVertices[PrevVert + 2] = {{ CX + HX, CY + HY, CZ - HZ }};
            OutVertices[PrevVert + 3] = {{ CX - HX, CY + HY, CZ - HZ }};
            OutVertices[PrevVert + 4] = {{ CX - HX, CY - HY, CZ + HZ }};
            OutVertices[PrevVert + 5] = {{ CX + HX, CY - HY, CZ + HZ }};
            OutVertices[PrevVert + 6] = {{ CX + HX, CY + HY, CZ + HZ }};
            OutVertices[PrevVert + 7] = {{ CX - HX, CY + HY, CZ + HZ }};

            const int32 V = VertexOffset;
            const int32 PrevTri = OutIndices.Num();
            OutIndices.AddUninitialized(12);
            // Bottom face
            OutIndices[PrevTri + 0]  = {{ V+0, V+2, V+1 }};
            OutIndices[PrevTri + 1]  = {{ V+0, V+3, V+2 }};
            // Top face
            OutIndices[PrevTri + 2]  = {{ V+4, V+5, V+6 }};
            OutIndices[PrevTri + 3]  = {{ V+4, V+6, V+7 }};
            // Front face
            OutIndices[PrevTri + 4]  = {{ V+0, V+1, V+5 }};
            OutIndices[PrevTri + 5]  = {{ V+0, V+5, V+4 }};
            // Back face
            OutIndices[PrevTri + 6]  = {{ V+2, V+3, V+7 }};
            OutIndices[PrevTri + 7]  = {{ V+2, V+7, V+6 }};
            // Left face
            OutIndices[PrevTri + 8]  = {{ V+0, V+4, V+7 }};
            OutIndices[PrevTri + 9]  = {{ V+0, V+7, V+3 }};
            // Right face
            OutIndices[PrevTri + 10] = {{ V+1, V+2, V+6 }};
            OutIndices[PrevTri + 11] = {{ V+1, V+6, V+5 }};

            VertexOffset += 8;
        }

        return OutVertices.Num() > 0 && OutIndices.Num() > 0;
    }

    return false;
}

/// Try to extract geometry from render vertex/index buffers (requires CPU access).
static bool ExtractFromRenderData(UStaticMesh* StaticMesh,
                                  TArray<rgl_vec3f>& OutVertices,
                                  TArray<rgl_vec3i>& OutIndices)
{
    const FStaticMeshRenderData* RenderData = StaticMesh->GetRenderData();
    if (!RenderData || RenderData->LODResources.Num() == 0)
    {
        return false;
    }

    const FStaticMeshLODResources& LOD = RenderData->LODResources[0];
    const FPositionVertexBuffer& VB = LOD.VertexBuffers.PositionVertexBuffer;
    const uint32 NumVertices = VB.GetNumVertices();

    if (NumVertices == 0 || VB.GetStride() == 0)
    {
        return false;
    }

    // CPU data must be accessible (otherwise VertexPosition() dereferences null -> SIGSEGV).
    // This check is required in ALL build configurations — even Development packaged builds
    // release CPU-side vertex data after GPU upload.
    if (!StaticMesh->bAllowCPUAccess && !VB.GetAllowCPUAccess())
    {
        return false;
    }

    OutVertices.SetNum(NumVertices);
    for (uint32 i = 0; i < NumVertices; ++i)
    {
        const FVector3f& Pos = VB.VertexPosition(i);
        OutVertices[i] = { {
            Pos.X * RGLCoord::UE_TO_RGL,
            Pos.Y * RGLCoord::UE_TO_RGL,
            Pos.Z * RGLCoord::UE_TO_RGL
        } };
    }

    FIndexArrayView IndexView = LOD.IndexBuffer.GetArrayView();
    const int32 NumTriangles = IndexView.Num() / 3;
    if (NumTriangles == 0)
    {
        OutVertices.Empty();
        return false;
    }

    OutIndices.SetNum(NumTriangles);
    for (int32 i = 0; i < NumTriangles; ++i)
    {
        const int32 Idx0 = static_cast<int32>(IndexView[i * 3 + 0]);
        const int32 Idx1 = static_cast<int32>(IndexView[i * 3 + 1]);
        const int32 Idx2 = static_cast<int32>(IndexView[i * 3 + 2]);

        if (Idx0 >= static_cast<int32>(NumVertices) ||
            Idx1 >= static_cast<int32>(NumVertices) ||
            Idx2 >= static_cast<int32>(NumVertices))
        {
            OutVertices.Empty();
            OutIndices.Empty();
            return false;
        }

        OutIndices[i] = { { Idx0, Idx1, Idx2 } };
    }

    return true;
}

/// Extract geometry by reading GPU vertex/index buffers via RHI readback.
/// Works in ALL build configurations (Shipping included) since render data is always on GPU.
/// Must be called from game thread (uses FlushRenderingCommands).
/// Uses the coarsest available LOD to minimize polygon count for raytrace performance.
static bool ExtractFromGPUBuffer(UStaticMesh* StaticMesh,
                                  TArray<rgl_vec3f>& OutVertices,
                                  TArray<rgl_vec3i>& OutIndices)
{
    const FStaticMeshRenderData* RenderData = StaticMesh->GetRenderData();
    if (!RenderData || RenderData->LODResources.Num() == 0)
    {
        return false;
    }

    const FStaticMeshLODResources& LOD = RenderData->LODResources[0];

    // --- Vertex buffer readback ---
    const FPositionVertexBuffer& VB = LOD.VertexBuffers.PositionVertexBuffer;
    const uint32 NumVertices = VB.GetNumVertices();
    if (NumVertices == 0)
    {
        return false;
    }

    const FBufferRHIRef& VBRef = VB.VertexBufferRHI;
    if (!VBRef.IsValid())
    {
        return false;
    }

    // Flush rendering to ensure the buffer is up to date
    FlushRenderingCommands();

    const uint32 VBSize = VBRef->GetSize();
    const uint32 Stride = VBSize / NumVertices;
    if (Stride < sizeof(FVector3f))
    {
        return false;
    }

    // Lock the GPU buffer for reading
    void* LockedVB = nullptr;
    ENQUEUE_RENDER_COMMAND(LockVertexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        LockedVB = RHICmdList.LockBuffer(VBRef, 0, VBSize, RLM_ReadOnly);
    });
    FlushRenderingCommands();

    if (!LockedVB)
    {
        return false;
    }

    OutVertices.SetNum(NumVertices);
    for (uint32 i = 0; i < NumVertices; ++i)
    {
        const FVector3f* Pos = reinterpret_cast<const FVector3f*>(
            static_cast<const uint8*>(LockedVB) + i * Stride);
        OutVertices[i] = { {
            Pos->X * RGLCoord::UE_TO_RGL,
            Pos->Y * RGLCoord::UE_TO_RGL,
            Pos->Z * RGLCoord::UE_TO_RGL
        } };
    }

    ENQUEUE_RENDER_COMMAND(UnlockVertexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        RHICmdList.UnlockBuffer(VBRef);
    });
    FlushRenderingCommands();

    // --- Index buffer readback ---
    const FRawStaticIndexBuffer& IB = LOD.IndexBuffer;
    const FBufferRHIRef& IBRef = IB.IndexBufferRHI;
    if (!IBRef.IsValid())
    {
        OutVertices.Empty();
        return false;
    }

    const uint32 IBSize = IBRef->GetSize();
    const bool bIs32Bit = (IB.Is32Bit());
    const uint32 IndexStride = bIs32Bit ? 4 : 2;
    const uint32 NumIndices = IBSize / IndexStride;
    const uint32 NumTriangles = NumIndices / 3;

    if (NumTriangles == 0)
    {
        OutVertices.Empty();
        return false;
    }

    void* LockedIB = nullptr;
    ENQUEUE_RENDER_COMMAND(LockIndexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        LockedIB = RHICmdList.LockBuffer(IBRef, 0, IBSize, RLM_ReadOnly);
    });
    FlushRenderingCommands();

    if (!LockedIB)
    {
        OutVertices.Empty();
        return false;
    }

    OutIndices.SetNum(NumTriangles);
    for (uint32 i = 0; i < NumTriangles; ++i)
    {
        int32 Idx0, Idx1, Idx2;
        if (bIs32Bit)
        {
            const uint32* Indices32 = static_cast<const uint32*>(LockedIB);
            Idx0 = static_cast<int32>(Indices32[i * 3 + 0]);
            Idx1 = static_cast<int32>(Indices32[i * 3 + 1]);
            Idx2 = static_cast<int32>(Indices32[i * 3 + 2]);
        }
        else
        {
            const uint16* Indices16 = static_cast<const uint16*>(LockedIB);
            Idx0 = static_cast<int32>(Indices16[i * 3 + 0]);
            Idx1 = static_cast<int32>(Indices16[i * 3 + 1]);
            Idx2 = static_cast<int32>(Indices16[i * 3 + 2]);
        }

        if (Idx0 >= static_cast<int32>(NumVertices) ||
            Idx1 >= static_cast<int32>(NumVertices) ||
            Idx2 >= static_cast<int32>(NumVertices))
        {
            OutVertices.Empty();
            OutIndices.Empty();
            ENQUEUE_RENDER_COMMAND(UnlockIndexBuffer)([&](FRHICommandListImmediate& RHICmdList)
            {
                RHICmdList.UnlockBuffer(IBRef);
            });
            FlushRenderingCommands();
            return false;
        }

        OutIndices[i] = { { Idx0, Idx1, Idx2 } };
    }

    ENQUEUE_RENDER_COMMAND(UnlockIndexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        RHICmdList.UnlockBuffer(IBRef);
    });
    FlushRenderingCommands();

    return true;
}

rgl_mesh_t FRGLSceneManager::UploadMesh(UStaticMesh* StaticMesh)
{
    if (!StaticMesh)
    {
        return nullptr;
    }

    // Check cache first (a live or an idle entry)
    if (rgl_mesh_t CachedMesh = FindCachedMesh(StaticMesh))
    {
        return CachedMesh;
    }

    if (IsMeshRetryDeferred(StaticMesh))
    {
        return nullptr;
    }

    TArray<rgl_vec3f> Vertices;
    TArray<rgl_vec3i> Indices;

    // Extraction priority:
    // 1. Chaos TriMeshes — complex collision with full triangle detail (fast raytrace)
    // 2. Render data (CPU access) — if bAllowCPUAccess is set
    // 3. GPU buffer readback — render mesh from GPU (full fidelity, heavy)
    // 4. Chaos ConvexElems/BoxElems — simplified convex hulls (last resort)
    //
    // Note: AWSIM uses render meshes or direct collision trimeshes, never convex hulls.
    // ConvexElems lose concave features (e.g., spiral towers become simple polyhedra).
    // GPU readback preserves full visual fidelity, so it is preferred over ConvexElems.
    if (ExtractFromChaosTriMeshes(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractChaos;
#endif
    }
    else if (ExtractFromRenderData(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractRender;
#endif
    }
    else if (ExtractFromGPUBuffer(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractGPU;
#endif
    }
    else if (ExtractFromChaosConvexOrBox(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractChaos;  // Still count as Chaos path
#endif
    }
    else
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractFailed;
#endif
        RememberMeshFailure(StaticMesh);
        return nullptr;
    }

    if (Vertices.Num() == 0 || Indices.Num() == 0)
    {
        RememberMeshFailure(StaticMesh);
        return nullptr;
    }

#if !UE_BUILD_SHIPPING
    // [DEBUG] Log first mesh's vertex bounds
    {
        static int32 sMeshLogCount = 0;
        if (sMeshLogCount < 2)
        {
            float minX=1e9, minY=1e9, minZ=1e9, maxX=-1e9, maxY=-1e9, maxZ=-1e9;
            for (int32 i = 0; i < Vertices.Num(); ++i)
            {
                minX = FMath::Min(minX, Vertices[i].value[0]);
                minY = FMath::Min(minY, Vertices[i].value[1]);
                minZ = FMath::Min(minZ, Vertices[i].value[2]);
                maxX = FMath::Max(maxX, Vertices[i].value[0]);
                maxY = FMath::Max(maxY, Vertices[i].value[1]);
                maxZ = FMath::Max(maxZ, Vertices[i].value[2]);
            }
            RGLLog::Info("[DEBUG] Mesh", sMeshLogCount,
                         "name:", TCHAR_TO_UTF8(*StaticMesh->GetName()),
                         "verts:", Vertices.Num(), "tris:", Indices.Num(),
                         "bounds(m): min(", minX, minY, minZ, ") max(", maxX, maxY, maxZ, ")");
            ++sMeshLogCount;
        }
    }
#endif

#if !UE_BUILD_SHIPPING
    // Track total scene geometry
    extern int64 GRGLTotalVertices, GRGLTotalTriangles;
    GRGLTotalVertices += Vertices.Num();
    GRGLTotalTriangles += Indices.Num();
#endif

    // Upload to RGL
    rgl_mesh_t RGLMesh = nullptr;
    rgl_status_t Status = RGL_PROBED("rgl_mesh_create", rgl_mesh_create(
        &RGLMesh,
        Vertices.GetData(),
        static_cast<int32_t>(Vertices.Num()),
        Indices.GetData(),
        static_cast<int32_t>(Indices.Num())));

    if (Status != RGL_SUCCESS || !RGLMesh)
    {
        const char* ErrMsg = nullptr;
        rgl_get_last_error_string(&ErrMsg);
        UE_LOG(LogCarlaRGL, Warning,
               TEXT("RGLSceneManager: Failed to upload mesh '%s' (%d verts, %d tris): %s"),
               *StaticMesh->GetName(),
               Vertices.Num(), Indices.Num(),
               ErrMsg ? *FString(UTF8_TO_TCHAR(ErrMsg)) : TEXT("unknown"));
        RememberMeshFailure(StaticMesh);
        return nullptr;
    }

    // Cache the mesh
    const FObjectKey MeshKey(StaticMesh);
    MeshCache.Add(MeshKey).Mesh = RGLMesh;
    FailedMeshRetrySync.Remove(MeshKey);
#if !UE_BUILD_SHIPPING
    ++MeshUploads_ForTest;
#endif

    return RGLMesh;
}

bool FRGLSceneManager::IsMeshRetryDeferred(UStaticMesh* StaticMesh) const
{
    const uint64* RetrySync = FailedMeshRetrySync.Find(FObjectKey(StaticMesh));
    return RetrySync && SceneSyncSerial < *RetrySync;
}

void FRGLSceneManager::RememberMeshFailure(UStaticMesh* StaticMesh)
{
    // Count syncs rather than Updates so retries stay infrequent even with many LiDAR scans per sync.
    // Keep this state outside PendingCreates: rebuilding that queue must not reset the back-off.
    FailedMeshRetrySync.Add(FObjectKey(StaticMesh), SceneSyncSerial + 16);
    ++MeshFailuresSinceLog;
}

rgl_mesh_t FRGLSceneManager::FindCachedMesh(const UStaticMesh* StaticMesh) const
{
    // A cache entry (live or idle) can outlive its asset: an idle one outlives the entities that kept the asset alive,
    // and a live one lasts until the next Update removes the entities of a destroyed component. If the asset was
    // collected and its address handed to another UStaticMesh, that mesh has another FObjectKey (GC serial) and misses
    // here, so the old geometry is never served for it (P2). A stale entry leaves with its last reference, or, kept as
    // idle (rgl.Scene.IdleMeshCacheMax > 0), at the next sync's PurgeIdleMeshes.
    const FCachedMesh* Found = StaticMesh ? MeshCache.Find(FObjectKey(StaticMesh)) : nullptr;
    return Found ? Found->Mesh : nullptr;
}

void FRGLSceneManager::DestroyIdleMeshAt(int32 LruIndex)
{
    const FObjectKey Key = IdleMeshLru[LruIndex];
    IdleMeshLru.RemoveAt(LruIndex);
    if (MeshRefCounts.Contains(Key))
    {
        return;   // referenced again: not idle (defensive; AddMeshReference removes revived keys)
    }
    if (FCachedMesh* Cached = MeshCache.Find(Key))
    {
        if (Cached->Mesh)
        {
            RglDestroyChecked(RGL_PROBED("rgl_mesh_destroy", rgl_mesh_destroy(Cached->Mesh)), TEXT("rgl_mesh_destroy"));
        }
        MeshCache.Remove(Key);
    }
    ++IdleDestroyedSinceLog;
}

void FRGLSceneManager::TrimIdleMeshes(int32 MaxIdle)
{
    while (IdleMeshLru.Num() > MaxIdle)
    {
        DestroyIdleMeshAt(0);   // least recently idled first
    }
}

void FRGLSceneManager::PurgeIdleMeshes()
{
    // An idle entry whose key no longer resolves (its asset was collected or is garbage) can never be looked up again.
    for (int32 i = IdleMeshLru.Num() - 1; i >= 0; --i)
    {
        const FObjectKey& Key = IdleMeshLru[i];
        if (!MeshCache.Contains(Key) || !Key.ResolveObjectPtr())
        {
            DestroyIdleMeshAt(i);
        }
    }
    // Applies a lowered rgl.Scene.IdleMeshCacheMax (0 empties the idle set).
    TrimIdleMeshes(SceneIdleMeshCacheMax());
}

void FRGLSceneManager::AddMeshReference(const FObjectKey& MeshKey, int32 Count)
{
    if (MeshKey == FObjectKey() || Count <= 0)
    {
        return;
    }

    int32& RefCount = MeshRefCounts.FindOrAdd(MeshKey);
    if (RefCount == 0)
    {
        IdleMeshLru.RemoveSingle(MeshKey);   // revived from the idle LRU (no-op for a fresh mesh)
    }
    RefCount += Count;
}

void FRGLSceneManager::ReleaseMeshReference(const FObjectKey& MeshKey, int32 Count)
{
    if (MeshKey == FObjectKey() || Count <= 0)
    {
        return;
    }

    int32* RefCount = MeshRefCounts.Find(MeshKey);
    if (!RefCount)
    {
        return;
    }

    *RefCount -= Count;
    if (*RefCount > 0)
    {
        return;
    }

    MeshRefCounts.Remove(MeshKey);

    FCachedMesh* Cached = MeshCache.Find(MeshKey);
    if (!Cached)
    {
        return;
    }

    // Keep the uploaded GPU mesh for re-entry; RGL discards its GAS when the last entity is removed.
    const int32 MaxIdle = SceneIdleMeshCacheMax();
    if (MaxIdle > 0 && Cached->Mesh)
    {
        IdleMeshLru.RemoveSingle(MeshKey);
        IdleMeshLru.Add(MeshKey);
        TrimIdleMeshes(MaxIdle);
        return;
    }

    if (Cached->Mesh)
    {
        RGL_PROBED("rgl_mesh_destroy", rgl_mesh_destroy(Cached->Mesh));
    }
    MeshCache.Remove(MeshKey);
}

// ============================================================================
// Entity management
// ============================================================================

bool FRGLSceneManager::IsWithinAnySensor(const FVector& CenterCm, float BoundsRadiusCm, float ExtraCm, float RadiusScale) const
{
    for (const auto& Pair : RegisteredSensors)
    {
        const FSensorRegistration& S = Pair.Value;
        const float EffectiveDistanceCm = S.RegistrationDistanceCm * RadiusScale + BoundsRadiusCm + ExtraCm;
        if (FVector::DistSquared(CenterCm, S.Position) <= FMath::Square(EffectiveDistanceCm))
        {
            return true;
        }
    }
    return false;
}

bool FRGLSceneManager::IsComponentWithinAnySensor(UStaticMeshComponent* Component, float ExtraCm, float RadiusScale) const
{
    if (!IsValid(Component))
    {
        return false;
    }
    return IsWithinAnySensor(Component->Bounds.Origin, Component->Bounds.SphereRadius, ExtraCm, RadiusScale);
}

bool FRGLSceneManager::IsWithinAnySensorRange(const FVector& CenterCm, float BoundsRadiusCm) const
{
    for (const auto& Pair : RegisteredSensors)
    {
        const double ReachCm = static_cast<double>(SensorRangeCm(Pair.Value)) + BoundsRadiusCm;
        if (FVector::DistSquared(CenterCm, Pair.Value.Position) <= ReachCm * ReachCm)
        {
            return true;
        }
    }
    return false;
}

float FRGLSceneManager::NearestSensorGapCm(const FVector& CenterCm, float BoundsRadiusCm) const
{
    float Best = TNumericLimits<float>::Max();
    for (const auto& Pair : RegisteredSensors)
    {
        Best = FMath::Min(Best, static_cast<float>(FVector::Dist(CenterCm, Pair.Value.Position)) - BoundsRadiusCm);
    }
    return Best;
}

// An ISM instance that never gets an entity (the registration rule): a near-zero scale on any axis.
static bool IsZeroScaleInstance(const FTransform& InstanceTransform)
{
    const FVector Scale = InstanceTransform.GetScale3D();
    return FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z);
}

// The radius of an ISM instance's culling sphere: the mesh's bounds radius times the instance's largest absolute scale.
static float InstanceRadiusCm(const UStaticMesh* StaticMesh, const FTransform& InstanceTransform)
{
    const FVector Scale = InstanceTransform.GetScale3D();
    const float MaxScale = FMath::Max(FMath::Max(FMath::Abs(Scale.X), FMath::Abs(Scale.Y)), FMath::Abs(Scale.Z));
    return static_cast<float>(StaticMesh->GetBounds().SphereRadius * MaxScale);
}

bool FRGLSceneManager::IsInstanceWithinAnySensor(
    const UInstancedStaticMeshComponent* Component,
    const UStaticMesh* StaticMesh,
    const FTransform& InstanceTransform,
    float ExtraCm,
    float RadiusScale) const
{
    if (!IsValid(Component) || !StaticMesh)
    {
        return false;
    }
    return IsWithinAnySensor(InstanceTransform.GetLocation(), InstanceRadiusCm(StaticMesh, InstanceTransform), ExtraCm, RadiusScale);
}

bool FRGLSceneManager::CollectInRangeInstances(const UInstancedStaticMeshComponent* Component, const UStaticMesh* StaticMesh,
                                               TArray<int32>& OutIndices, bool bStopAtFirst) const
{
    OutIndices.Reset();
    if (!IsValid(Component) || !StaticMesh)
    {
        return false;
    }
    const int32 NumInstances = Component->GetInstanceCount();
    for (int32 i = 0; i < NumInstances; ++i)
    {
        FTransform InstanceTransform;
        if (!Component->GetInstanceTransform(i, InstanceTransform, /*bWorldSpace=*/true) || IsZeroScaleInstance(InstanceTransform))
        {
            continue;
        }
        if (IsInstanceWithinAnySensor(Component, StaticMesh, InstanceTransform, 0.0f))
        {
            OutIndices.Add(i);
            if (bStopAtFirst)
            {
                break;
            }
        }
    }
    return OutIndices.Num() > 0;
}

FRGLSceneManager::EStaticGate FRGLSceneManager::GateStaticComponent(const UStaticMeshComponent* Component,
                                                                    bool bRememberVerdict) const
{
    // An unregistered component is not part of the loaded world (its level is being streamed out, or the owner
    // unregistered it): registering it would put a ghost into the scene (P1).
    const UStaticMesh* Mesh = IsValid(Component) ? Component->GetStaticMesh() : nullptr;
    if (!Mesh || !Component->IsRegistered() || !Component->IsVisible())
    {
        return EStaticGate::Reject;
    }

    const FVector Scale = Component->GetComponentTransform().GetScale3D();
    if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
    {
        return EStaticGate::Reject;
    }

    // Sky domes and spheres are valid static meshes but should never take part in LiDAR ray tracing. Only the dome
    // words count, not "sky" anywhere in a name: that also dropped CARLA's shared skyscraper parts (SM_Skysc*).
    // The verdict depends on the mesh alone: remembered per mesh, no name string per component (P7).
    if (IsSkyDomeMesh(Mesh, bRememberVerdict))
    {
        return EStaticGate::ExcludedByName;
    }
    // Size guard: a sky mesh the dome words miss (a map nobody checked) is still left out when the old rule's names
    // match and it is wider than 2 km. The radius is looked at first; the names are built only for such a component.
    if (Component->Bounds.SphereRadius > HugeSkyRadiusCm && IsLegacySkyName(Component))
    {
        return EStaticGate::ExcludedByName;
    }

    return EStaticGate::Pass;
}

bool FRGLSceneManager::IsSkyDomeMesh(const UStaticMesh* Mesh, bool bRemember) const
{
    const FObjectKey Key(Mesh);   // generation-aware: a mesh that takes a collected one's address has another key
    if (const bool* Verdict = SkyVerdictByMesh.Find(Key))
    {
        return *Verdict;
    }
    const bool bDome = IsSkyDomeMeshName(Mesh->GetName());
    if (bRemember)
    {
        SkyVerdictByMesh.Add(Key, bDome);
    }
    return bDome;
}

bool FRGLSceneManager::IsSkyDomeMeshName(const FString& MeshName)
{
    static const TCHAR* const DomeWords[] = { TEXT("SkySphere"), TEXT("SkyDome"), TEXT("Sky_Sphere"), TEXT("Sky_Dome") };
    for (const TCHAR* Word : DomeWords)
    {
        if (MeshName.Contains(Word, ESearchCase::IgnoreCase))
        {
            return true;
        }
    }
    return false;
}

bool FRGLSceneManager::IsLegacySkyName(const UStaticMeshComponent* Component)
{
    if (!Component)
    {
        return false;
    }
    const AActor* Owner = Component->GetOwner();
    const UStaticMesh* Mesh = Component->GetStaticMesh();
    return (Owner && Owner->GetName().Contains(TEXT("Sky"), ESearchCase::IgnoreCase)) ||
           Component->GetName().Contains(TEXT("Sky"), ESearchCase::IgnoreCase) ||
           (Mesh && Mesh->GetName().Contains(TEXT("Sky"), ESearchCase::IgnoreCase));
}

bool FRGLSceneManager::ShouldRegisterComponent(UStaticMeshComponent* Component, bool& bOutOfRange) const
{
    bOutOfRange = false;

    // Validity, mesh, visibility, scale and the name rule (shared with AuditScene).
    if (GateStaticComponent(Component) != EStaticGate::Pass)
    {
        return false;
    }

    // Movable actors are kept even if no sensor has registered yet.
    if (Component->Mobility != EComponentMobility::Static)
    {
        // 7b: on a World Partition map, a non-Static part the culling chooses (not protected) is registered by the
        // culling's queue only, never on this path (no immediate cached registration, no unlimited first scan).
        if (bWpActive && ClassifyWpProtection(Component) == EWpProtect::None)
        {
            return false;
        }
        return true;
    }

    // Static geometry on large maps is large enough to exhaust OptiX if uploaded wholesale.
    // Register only meshes whose bounds intersect the union of all sensor spheres.
    if (RegisteredSensors.Num() == 0)
    {
        bOutOfRange = true;
        return false;
    }

    if (!IsComponentWithinAnySensor(Component, 0.0f))
    {
        bOutOfRange = true;
        return false;
    }

    return true;
}

bool FRGLSceneManager::ShouldKeepRegisteredComponent(UStaticMeshComponent* Component) const
{
    bool bOutOfRange = false;
    if (ShouldRegisterComponent(Component, bOutOfRange))
    {
        return true;
    }
    // 7b: a non-Static part the culling chooses stays until its queue removes it (distance) or the gate rejects it.
    if (bWpActive && IsValid(Component) && Component->Mobility != EComponentMobility::Static &&
        GateStaticComponent(Component) == EStaticGate::Pass)
    {
        return true;
    }

    if (!bOutOfRange || !IsValid(Component))
    {
        return false;
    }

    // Keep a wider unload radius than the load radius to prevent repeated
    // destroy/recreate near the boundary while sensors move:
    // RegistrationDistance * rgl.Scene.UnloadDistanceFactor + UnregistrationHysteresisCm (factor 1 = original).
    return IsComponentWithinAnySensor(Component, UnregistrationHysteresisCm, SceneUnloadDistanceFactor());
}

bool FRGLSceneManager::RegisterComponent(UStaticMeshComponent* Component)
{
    if (!IsValid(Component) || EntityMap.Contains(Component))
    {
        return false;
    }

    // ISMC branch: expand all instances into separate RGL entities
    if (UInstancedStaticMeshComponent* ISMComp = Cast<UInstancedStaticMeshComponent>(Component))
    {
        if (ISMCEntityMap.Contains(ISMComp))
        {
            return false;
        }
        return RegisterISMComponent(ISMComp);
    }

    UStaticMesh* StaticMesh = Component->GetStaticMesh();
    if (!StaticMesh)
    {
        return false;
    }

    rgl_mesh_t RGLMesh = UploadMesh(StaticMesh);
    if (!RGLMesh)
    {
        return false;
    }
    const FObjectKey MeshKey(StaticMesh);
    AddMeshReference(MeshKey);

    rgl_entity_t Entity = nullptr;
    rgl_status_t Status = RGL_PROBED("rgl_entity_create", rgl_entity_create(&Entity, Scene, RGLMesh));
    if (Status != RGL_SUCCESS || !Entity)
    {
        ReleaseMeshReference(MeshKey);
        const char* ErrMsg = nullptr;
        rgl_get_last_error_string(&ErrMsg);
        UE_LOG(LogCarlaRGL, Warning,
               TEXT("RGLSceneManager: Failed to create entity for '%s': %s"),
               *Component->GetName(),
               ErrMsg ? *FString(UTF8_TO_TCHAR(ErrMsg)) : TEXT("unknown"));
        return false;
    }

    // Set initial transform
    const FTransform WorldTransform = Component->GetComponentTransform();
    rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(WorldTransform);
    RGL_CHECK(RGL_PROBED("rgl_entity_set_transform", rgl_entity_set_transform(Entity, &RGLTransform)));

#if !UE_BUILD_SHIPPING
    // [DEBUG] Log first 3 entities' positions
    {
        static int32 sEntityLogCount = 0;
        if (sEntityLogCount < 3)
        {
            const FVector Pos = WorldTransform.GetLocation();
            RGLLog::Info("[DEBUG] Entity", sEntityLogCount,
                         "name:", TCHAR_TO_UTF8(*Component->GetOwner()->GetName()),
                         "UE5pos(cm):", Pos.X, Pos.Y, Pos.Z,
                         "RGLpos(m):", RGLTransform.value[0][3], RGLTransform.value[1][3], RGLTransform.value[2][3]);
            ++sEntityLogCount;
        }
    }
#endif

    // Store mapping with static/dynamic classification
    FEntityInfo Info;
    Info.Entity = Entity;
    Info.Component = Component;
    Info.MeshKey = MeshKey;
    Info.bIsStatic = (Component->Mobility == EComponentMobility::Static);
    Info.bTransformInitialized = true;
    Info.LastTransform = WorldTransform;
    EntityMap.Add(Component, Info);

    return true;
}

void FRGLSceneManager::UnregisterComponent(UStaticMeshComponent* Component)
{
    FEntityInfo* Info = EntityMap.Find(Component);
    if (!Info)
    {
        return;
    }

    if (Info->Entity)
    {
        RGL_PROBED("rgl_entity_destroy", rgl_entity_destroy(Info->Entity));
        Info->Entity = nullptr;
    }
    ReleaseMeshReference(Info->MeshKey);   // the stored key: the asset may be collected already
    EntityMap.Remove(Component);
}

bool FRGLSceneManager::RegisterISMComponent(UInstancedStaticMeshComponent* Component)
{
    if (!IsValid(Component))
    {
        return false;
    }

    UStaticMesh* StaticMesh = Component->GetStaticMesh();
    if (!StaticMesh)
    {
        return false;
    }

    // Choose the instances before touching the mesh. A Static ISM gets entities for its instances inside R and keeps
    // the sphere of every instance for the re-evaluation (P3); a Movable ISM gets all of its instances, as before.
    const int32 NumInstances = Component->GetInstanceCount();
    const bool bStatic = (Component->Mobility == EComponentMobility::Static);
    TArray<FSphere3f> Spheres;
    TArray<int32> Chosen;
    TArray<FTransform> ChosenTransforms;
    if (bStatic)
    {
        Spheres.SetNumUninitialized(FMath::Max(0, NumInstances));
    }
    int32 RangeSkippedCount = 0;
    for (int32 i = 0; i < NumInstances; ++i)
    {
        FTransform InstanceTransform;
        // Skip instances with zero/near-zero scale (and those without a transform)
        const bool bUsable = Component->GetInstanceTransform(i, InstanceTransform, /*bWorldSpace=*/true) &&
                             !IsZeroScaleInstance(InstanceTransform);
        if (bStatic)
        {
            if (!bUsable)
            {
                Spheres[i] = FSphere3f(FVector3f::ZeroVector, -1.0f);
                continue;
            }
            Spheres[i] = FSphere3f(FVector3f(InstanceTransform.GetLocation()), InstanceRadiusCm(StaticMesh, InstanceTransform));
            if (!IsInstanceWithinAnySensor(Component, StaticMesh, InstanceTransform, 0.0f))
            {
                ++RangeSkippedCount;
                continue;
            }
        }
        else if (!bUsable)
        {
            continue;
        }
        Chosen.Add(i);
        ChosenTransforms.Add(InstanceTransform);
    }
    if (Chosen.Num() == 0)
    {
        ++IsmEmptySkippedSinceLog;   // nothing to register: no mesh upload for it
        return false;
    }

    rgl_mesh_t RGLMesh = UploadMesh(StaticMesh);
    if (!RGLMesh)
    {
        return false;
    }
    const FObjectKey MeshKey(StaticMesh);
    AddMeshReference(MeshKey); // Temporary reference until at least one entity owns the mesh.

    FISMCEntityGroup Group;
    Group.Component = Component;
    Group.MeshKey = MeshKey;
    Group.OriginalInstanceCount = NumInstances;
    Group.Entities.Reserve(Chosen.Num());
    Group.InstanceSpheres = MoveTemp(Spheres);
    Group.RegisteredMask.Init(false, NumInstances);

    int32 SuccessCount = 0;
    for (int32 k = 0; k < Chosen.Num(); ++k)
    {
        rgl_entity_t Entity = nullptr;
        rgl_status_t Status = RGL_PROBED("rgl_entity_create", rgl_entity_create(&Entity, Scene, RGLMesh));
        if (Status != RGL_SUCCESS || !Entity)
        {
            continue;
        }

        rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(ChosenTransforms[k]);
        RGL_CHECK(RGL_PROBED("rgl_entity_set_transform", rgl_entity_set_transform(Entity, &RGLTransform)));

        Group.Entities.Add(Entity);
        Group.InstanceIndices.Add(Chosen[k]);
        Group.InstanceWorldTransforms.Add(ChosenTransforms[k]);
        Group.RegisteredMask[Chosen[k]] = true;
        ++SuccessCount;
    }

    if (SuccessCount == 0)
    {
        ReleaseMeshReference(MeshKey);
        return false;
    }

    Group.MeshRefCount = SuccessCount;
    ISMCEntityMap.Add(Component, MoveTemp(Group));
    AddMeshReference(MeshKey, SuccessCount);
    ReleaseMeshReference(MeshKey); // Drop the temporary upload reference.

    RGLLog::Info("RGLSceneManager: Registered ISMC '",
                 TCHAR_TO_UTF8(*Component->GetName()),
                 "' instances=", NumInstances,
                 " entities=", SuccessCount,
                 " range-skipped=", RangeSkippedCount,
                 " mesh='", TCHAR_TO_UTF8(*StaticMesh->GetName()), "'");

    return true;
}

void FRGLSceneManager::UnregisterISMComponent(UInstancedStaticMeshComponent* Component)
{
    FISMCEntityGroup* Group = ISMCEntityMap.Find(Component);
    if (!Group)
    {
        return;
    }

    for (rgl_entity_t Entity : Group->Entities)
    {
        if (Entity)
        {
            RGL_PROBED("rgl_entity_destroy", rgl_entity_destroy(Entity));
        }
    }
    ReleaseMeshReference(Group->MeshKey, Group->MeshRefCount);   // the stored key: the asset may be collected already
    ISMCEntityMap.Remove(Component);
}

// ============================================================================
// Transform update
// ============================================================================

void FRGLSceneManager::UpdateTransforms()
{
    TArray<UStaticMeshComponent*> ToRemove;

    for (auto& Pair : EntityMap)
    {
        FEntityInfo& Info = Pair.Value;

        // Use weak pointer to safely detect destroyed components
        if (!Info.Component.IsValid())
        {
            ToRemove.Add(Pair.Key);
            continue;
        }

        UStaticMeshComponent* Comp = Info.Component.Get();
        if (!IsValid(Comp) || !Comp->IsRegistered())
        {
            ToRemove.Add(Pair.Key);
            continue;
        }

        // Optimization: skip static entities after initial transform is set
        if (Info.bIsStatic && Info.bTransformInitialized)
        {
            continue;
        }

        const FTransform WorldTransform = Comp->GetComponentTransform();

        // Skip entities with zero/near-zero scale to avoid NaN in transforms
        const FVector Scale = WorldTransform.GetScale3D();
        if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
        {
            continue;
        }

        // Optimization: skip dynamic entities whose transform hasn't changed
        if (Info.bTransformInitialized && WorldTransform.Equals(Info.LastTransform, 0.01f))
        {
            continue;
        }

        rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(WorldTransform);
        RGL_CHECK(RGL_PROBED("rgl_entity_set_transform", rgl_entity_set_transform(Info.Entity, &RGLTransform)));

        Info.LastTransform = WorldTransform;
        Info.bTransformInitialized = true;
    }

    for (UStaticMeshComponent* Comp : ToRemove)
    {
        UnregisterComponent(Comp);
    }

    // P1: an ISM group whose component is gone or no longer registered (its World Partition cell is being streamed
    // out, or the owner unregistered it) leaves the RGL scene in this Update too, instead of staying until the next
    // sync as ghosts. Instance transforms are not touched here.
    TArray<UInstancedStaticMeshComponent*> DeadIsm;
    for (const auto& Pair : ISMCEntityMap)
    {
        const UInstancedStaticMeshComponent* Comp = Pair.Value.Component.Get();
        if (!IsValid(Comp) || !Comp->IsRegistered())
        {
            DeadIsm.Add(Pair.Key);
        }
    }
    for (UInstancedStaticMeshComponent* Comp : DeadIsm)
    {
        UnregisterISMComponent(Comp);
    }
    DeadIsmcSinceLog += DeadIsm.Num();
}

// ============================================================================
// Periodic world sync: detect new/removed components
// ============================================================================

void FRGLSceneManager::SyncWorldComponents(UWorld* World, FSceneSyncStats& OutStats)
{
    ++SceneSyncSerial;
    for (auto It = FailedMeshRetrySync.CreateIterator(); It; ++It)
    {
        if (!It.Key().ResolveObjectPtr())
        {
            It.RemoveCurrent(); // No strong asset reference; GC address reuse cannot inherit a failure.
        }
    }
    // P7: the sky-name verdicts of meshes that are gone (no lookup can reach them again).
    for (auto It = SkyVerdictByMesh.CreateIterator(); It; ++It)
    {
        if (!It.Key().ResolveObjectPtr())
        {
            It.RemoveCurrent();
        }
    }

    // Evict sensors that stopped refreshing (paused but not destroyed) so their
    // surrounding geometry can be released and stop holding VRAM.
    {
        TArray<const void*> StaleSensors;
        for (const auto& Pair : RegisteredSensors)
        {
            if (CurrentSimTime - Pair.Value.LastUpdateTime > SensorStaleTimeoutSeconds)
            {
                StaleSensors.Add(Pair.Key);
            }
        }
        for (const void* Id : StaleSensors)
        {
            RegisteredSensors.Remove(Id);
        }
    }

    // Drop idle meshes whose asset is gone and apply the current rgl.Scene.IdleMeshCacheMax before the
    // scan decides what is resident.
    PurgeIdleMeshes();

    // 7b: a culling sync fixes its horizon and margin first; a world that no longer culls (rgl.Scene.WpMovableCulling
    // switched off live) drops the culling's state here, and the scan below registers those parts the original way.
    if (bWpActive)
    {
        WpBeginSync(WorldDeltaSeconds(World));
    }
    else if (WpCandidates.Num() > 0 || WpJobs.Num() > 0 || WpMovedIsm.Num() > 0)
    {
        WpTeardown();
    }
    for (auto It = WpStickyProtected.CreateIterator(); It; ++It)
    {
        if (!It.Key().ResolveObjectPtr())
        {
            It.RemoveCurrent();
        }
    }

    // P7: the scan's containers are members; Reset keeps their allocations from one sync to the next.
    TSet<UStaticMeshComponent*>& CurrentComponents = ScanCurrentComponents;
    TSet<UInstancedStaticMeshComponent*>& CurrentISMComponents = ScanCurrentIsmComponents;
    TArray<FPendingCreate>& NewPending = ScanNewPending;
    CurrentComponents.Reset();
    CurrentISMComponents.Reset();
    NewPending.Reset();
    ScanSkeletalCandidates.Reset();
    int32 RemovedRegular = 0;
    int32 RemovedISM = 0;
    const bool bSkeletal = SkeletalEnabled();

    // The RGL calls made during the scan are timed apart (reg_ms) from the scan itself (scan_ms).
    double RegSeconds = 0.0;
    const double ScanStartSeconds = FPlatformTime::Seconds();

    // NOTE: a full-world actor scan on the game thread; its cost scales with the map, not with what changed
    // (see scan_ms of a sync with created=0). A spatial index is the next step if that cost matters.
    // One scan serves the static and the skeletal syncs: the skeletal sync used to visit every actor again.
    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!IsValid(Actor))
        {
            continue;
        }
        ++OutStats.ScannedActors;

        if (bSkeletal)
        {
            ScanActorSkeletal.Reset();
            Actor->GetComponents<USkeletalMeshComponent>(ScanActorSkeletal);
            ScanSkeletalCandidates.Append(ScanActorSkeletal);
        }

        ScanComponents.Reset();
        Actor->GetComponents<UStaticMeshComponent>(ScanComponents);

        for (UStaticMeshComponent* Comp : ScanComponents)
        {
            // 7b: a non-Static part the World Partition culling chooses goes to the culling (its candidates and queue),
            // never to the original path below. The sets keep the original removals away from it.
            if (bWpActive && IsValid(Comp) && Comp->Mobility != EComponentMobility::Static &&
                GateStaticComponent(Comp) == EStaticGate::Pass && ClassifyWpProtection(Comp) == EWpProtect::None)
            {
                UInstancedStaticMeshComponent* WpIsm = Cast<UInstancedStaticMeshComponent>(Comp);
                bool bWpSeen = false;
                if (WpIsm)
                {
                    CurrentISMComponents.Add(WpIsm, &bWpSeen);
                }
                else
                {
                    CurrentComponents.Add(Comp, &bWpSeen);
                }
                if (!bWpSeen)
                {
                    const double WpStart = FPlatformTime::Seconds();
                    const double WpRegBefore = WpVisitRegSeconds;
                    WpVisitCandidate(Comp, WpIsm);
                    const double WpMs = (FPlatformTime::Seconds() - WpStart) * 1000.0;
                    WpSyncCheckMs += WpMs;
                    RegSeconds += WpVisitRegSeconds - WpRegBefore;   // its RGL calls (a rebuilt group) are reg_ms
                }
                continue;
            }

            bool bOutOfRange = false;
            if (!ShouldRegisterComponent(Comp, bOutOfRange))
            {
                if (bOutOfRange)
                {
                    ++OutStats.RangeSkipped;
                }
                continue;
            }

            // Classify: ISMC or regular. The sets also de-duplicate the queue built below.
            bool bAlreadySeen = false;
            if (UInstancedStaticMeshComponent* ISMComp = Cast<UInstancedStaticMeshComponent>(Comp))
            {
                CurrentISMComponents.Add(ISMComp, &bAlreadySeen);
                if (bAlreadySeen)
                {
                    continue;
                }
                if (const FISMCEntityGroup* Group = ISMCEntityMap.Find(ISMComp))
                {
                    // Registered: a Static group's instances are re-evaluated after the scan (P3). A group whose
                    // instance count or mesh changed since its registration is rebuilt instead (its instance indices
                    // may name other instances now, review focus 3): it leaves the scene here and is registered again
                    // below like a new component.
                    if (Group->Component.Get() == ISMComp && Group->OriginalInstanceCount == ISMComp->GetInstanceCount() &&
                        Group->MeshKey == FObjectKey(ISMComp->GetStaticMesh()))
                    {
                        continue;
                    }
                    const double RegStartSeconds = FPlatformTime::Seconds();
                    UnregisterISMComponent(ISMComp);
                    RegSeconds += FPlatformTime::Seconds() - RegStartSeconds;
                    ++RemovedISM;
                }
                // A Static ISM none of whose instances reaches R is not queued: registering it would upload its mesh
                // for no entity (P3).
                if (ISMComp->Mobility == EComponentMobility::Static &&
                    !CollectInRangeInstances(ISMComp, ISMComp->GetStaticMesh(), IsmScanIndices, /*bStopAtFirst=*/true))
                {
                    ++IsmEmptySkippedSinceLog;
                    continue;
                }
            }
            else
            {
                CurrentComponents.Add(Comp, &bAlreadySeen);
                if (bAlreadySeen || EntityMap.Contains(Comp))
                {
                    continue;
                }
            }

            UStaticMesh* StaticMesh = Comp->GetStaticMesh();
            if (IsMeshRetryDeferred(StaticMesh))
            {
                continue; // A remembered failure consumes no create budget and is not requeued yet.
            }

            // Only meshes with live entities can bypass the budget: reviving an idle mesh rebuilds GAS
            // at the next raytrace, just like creating a new mesh. The scan runs before the removals below (D6): a
            // component that replaces a just-destroyed one with the same asset finds that one's mesh still live.
            if (FindCachedMesh(StaticMesh) && MeshRefCounts.FindRef(FObjectKey(StaticMesh)) > 0)
            {
                const double RegStartSeconds = FPlatformTime::Seconds();
                const bool bRegistered = RegisterComponent(Comp);   // dispatches ISMCs to RegisterISMComponent
                RegSeconds += FPlatformTime::Seconds() - RegStartSeconds;
                if (bRegistered)
                {
                    ++OutStats.AddedCached;
                }
                continue;
            }

            // New meshes and refcount-zero idle meshes share the per-Update create budget.
            FPendingCreate& Pending = NewPending.AddDefaulted_GetRef();
            Pending.Component = Comp;
            Pending.GapCm = NearestSensorGapCm(Comp->Bounds.Origin, Comp->Bounds.SphereRadius);
        }
    }
    ScanComponents.Reset();
    ScanActorSkeletal.Reset();
    OutStats.RegMs = RegSeconds * 1000.0;
    OutStats.ScanMs = FMath::Max(0.0, (FPlatformTime::Seconds() - ScanStartSeconds - RegSeconds) * 1000.0);
    // The scan has seen every level added so far: none of them needs a level-triggered sync any more (P5).
    PendingAddedLevels.Reset();
    // 7b: candidates the scan no longer saw are dropped; the new queue replaces the old one (the waits are kept).
    if (bWpActive)
    {
        WpEndSync();
    }

    // Remove regular entities for components that no longer exist or moved out
    // of the LiDAR-relevant window.
    const double RemoveStartSeconds = FPlatformTime::Seconds();
    TArray<UStaticMeshComponent*>& ToRemove = ScanToRemove;
    ToRemove.Reset();
    for (auto& Pair : EntityMap)
    {
        // A destroyed component leaves a dangling raw key in this non-UObject map
        // (GC does not null it: e.g. LargeMap tile stream-out, actor destruction).
        // Detect it via the weak pointer and remove without dereferencing the raw
        // key through ShouldKeepRegisteredComponent. Mirrors UpdateTransforms's guard.
        if (!Pair.Value.Component.IsValid())
        {
            ToRemove.Add(Pair.Key);
            continue;
        }
        if (!CurrentComponents.Contains(Pair.Key) && !ShouldKeepRegisteredComponent(Pair.Key))
        {
            ToRemove.Add(Pair.Key);
        }
    }
    for (UStaticMeshComponent* Comp : ToRemove)
    {
        UnregisterComponent(Comp);
        ++RemovedRegular;
    }

    ToRemove.Reset();

    // Remove ISMC entities for components that no longer exist or moved out
    // of the LiDAR-relevant window.
    TArray<UInstancedStaticMeshComponent*>& ISMCToRemove = ScanIsmToRemove;
    ISMCToRemove.Reset();
    for (auto& Pair : ISMCEntityMap)
    {
        // Same dangling-raw-key guard as the regular-entity loop above.
        if (!Pair.Value.Component.IsValid())
        {
            ISMCToRemove.Add(Pair.Key);
            continue;
        }
        if (!CurrentISMComponents.Contains(Pair.Key) && !ShouldKeepRegisteredComponent(Pair.Key))
        {
            ISMCToRemove.Add(Pair.Key);
        }
    }
    for (UInstancedStaticMeshComponent* Comp : ISMCToRemove)
    {
        UnregisterISMComponent(Comp);
        ++RemovedISM;
    }
    ISMCToRemove.Reset();
    OutStats.Removed = RemovedRegular + RemovedISM;
    OutStats.RemoveMs = (FPlatformTime::Seconds() - RemoveStartSeconds) * 1000.0;

    // The new queue replaces the old one: what the scan no longer saw in range is dropped, and the
    // order follows the sensors' current positions (nearest surface first). The old queue's array is kept for the next
    // sync's scan.
    NewPending.Sort([](const FPendingCreate& A, const FPendingCreate& B) { return A.GapCm < B.GapCm; });
    Swap(PendingCreates, NewPending);
    NewPending.Reset();
    OutStats.Queued = PendingCreates.Num();
    // The sets hold raw component pointers: emptied (allocations kept) so that nothing stale waits for the next sync.
    CurrentComponents.Reset();
    CurrentISMComponents.Reset();

    // P3: every registered Static ISM group is compared with the sensor spheres again, nearest first, on the next
    // Updates without a sync (DrainIsmReevaluations). The queue is rebuilt like PendingCreates: a group the previous
    // round had not reached yet is queued again with the rest (ism_pending counts them).
    IsmScanIndices.Reset();
    IsmPendingAtSync = PendingIsmReevals.Num();
    PendingIsmReevals.Reset();
    for (const auto& Pair : ISMCEntityMap)
    {
        const UInstancedStaticMeshComponent* Comp = Pair.Value.Component.Get();
        if (!Comp || Comp->Mobility != EComponentMobility::Static || Pair.Value.InstanceSpheres.Num() == 0)
        {
            continue;
        }
        FPendingIsmReeval& Reeval = PendingIsmReevals.AddDefaulted_GetRef();
        Reeval.Component = Pair.Value.Component;
        Reeval.GapCm = NearestSensorGapCm(Comp->Bounds.Origin, Comp->Bounds.SphereRadius);
    }
    PendingIsmReevals.Sort([](const FPendingIsmReeval& A, const FPendingIsmReeval& B) { return A.GapCm < B.GapCm; });

    // Skeletal components are tracked in a separate map with their own budget /
    // hysteresis rules; a live kill-switch flip tears the whole skeletal path down. Their candidates come from the
    // scan above (the same actors, in the same order as a scan of its own would visit them).
    const double SkelStartSeconds = FPlatformTime::Seconds();
    if (bSkeletal)
    {
        SyncSkeletalCandidates(ScanSkeletalCandidates);
    }
    else if (SkeletalEntityMap.Num() > 0)
    {
        TeardownSkeletal();
    }
    ScanSkeletalCandidates.Reset();
    OutStats.SkelMs = (FPlatformTime::Seconds() - SkelStartSeconds) * 1000.0;
    // The per-sync summary is the rgl_scene_sync line printed by Update() after the drain step.
}

void FRGLSceneManager::DrainPendingCreates(bool bUnlimited, FDrainStats& OutStats)
{
    if (PendingCreates.Num() == 0)
    {
        return;
    }

    const double StartSeconds = FPlatformTime::Seconds();
    const int32 MaxCreates = bUnlimited ? 0 : SceneMaxMeshCreatesPerUpdate();
    const double BudgetSeconds = bUnlimited ? 0.0 : static_cast<double>(SceneCreateBudgetMs()) / 1000.0;
    int32 BudgetedCreates = 0;
    // P8: the queued parts inside some sensor's range, created outside the budget above (an unlimited drain needs no
    // priority). Their own count and wall time (the budget's ms leave them out); the first one always proceeds.
    const int32 InRangeMax = bUnlimited ? 0 : SceneInRangeCreatesPerUpdate();
    const double InRangeBudgetSeconds = static_cast<double>(SceneInRangeCreateBudgetMs()) / 1000.0;
    double InRangeSeconds = 0.0;

    // In-place compaction: consumed or dropped entries are removed, waiting ones keep their order.
    int32 Write = 0;
    for (int32 Read = 0; Read < PendingCreates.Num(); ++Read)
    {
        UStaticMeshComponent* Comp = PendingCreates[Read].Component.Get();
        UInstancedStaticMeshComponent* ISMComp = Cast<UInstancedStaticMeshComponent>(Comp);
        if (!Comp || (ISMComp ? ISMCEntityMap.Contains(ISMComp) : EntityMap.Contains(Comp)))
        {
            ++OutStats.Dropped;   // destroyed (stale weak pointer) or already registered
            continue;
        }

        UStaticMesh* StaticMesh = Comp->GetStaticMesh();
        if (IsMeshRetryDeferred(StaticMesh))
        {
            continue; // Includes other queued components sharing an asset that failed earlier this step.
        }
        const bool bResident = (FindCachedMesh(StaticMesh) != nullptr);
        const bool bNeedsCreate = !bResident || MeshRefCounts.FindRef(FObjectKey(StaticMesh)) <= 0;

        // Re-check what the scan checked: still valid, visible, and within registration range.
        bool bOutOfRange = false;
        if (!ShouldRegisterComponent(Comp, bOutOfRange))
        {
            ++OutStats.Dropped;
            continue;
        }

        // P8: a part whose bounds reach some sensor's range now takes an in-range slot while one is left: the count, and
        // the wall time of this Update's in-range creations (checked between creations; the first always proceeds).
        // Beyond those caps it waits for the budget below like any other part.
        const bool bInRangeSlot = bNeedsCreate && InRangeMax > 0 && OutStats.InRangeCreates < InRangeMax &&
            (OutStats.InRangeCreates == 0 || InRangeBudgetSeconds <= 0.0 || InRangeSeconds < InRangeBudgetSeconds) &&
            IsWithinAnySensorRange(Comp->Bounds.Origin, Comp->Bounds.SphereRadius);

        // Failed extractions/uploads do not satisfy the minimum progress guarantee. Once a creatable
        // item ran, elapsed time (including failed attempts) can stop further creations this Update. The in-range
        // creations' time is not counted against this budget.
        if (bNeedsCreate && !bInRangeSlot && BudgetedCreates > 0 &&
            ((MaxCreates > 0 && BudgetedCreates >= MaxCreates) ||
             (BudgetSeconds > 0.0 && FPlatformTime::Seconds() - StartSeconds - InRangeSeconds >= BudgetSeconds)))
        {
            if (Write != Read)
            {
                PendingCreates[Write] = MoveTemp(PendingCreates[Read]);
            }
            ++Write;
            continue;
        }

        const double InRangeStartSeconds = bInRangeSlot ? FPlatformTime::Seconds() : 0.0;
        const int32 EmptySkippedBefore = IsmEmptySkippedSinceLog;
        const bool bRegistered = RegisterComponent(Comp);   // dispatches ISMCs to RegisterISMComponent
        if (bInRangeSlot)
        {
            InRangeSeconds += FPlatformTime::Seconds() - InRangeStartSeconds;   // failed attempts included
        }
        if (!bRegistered && IsmEmptySkippedSinceLog != EmptySkippedBefore)
        {
            ++OutStats.Dropped;   // an ISM with no instance in R any more: nothing was uploaded, no budget used
            continue;
        }
        if (!bNeedsCreate)
        {
            // An earlier creation/reactivation made the mesh live: additional entities share its GAS.
            if (bRegistered)
            {
                ++OutStats.RegisteredCached;
            }
            continue;
        }

        if (IsMeshRetryDeferred(StaticMesh))
        {
            ++OutStats.Failed; // Actual extraction/upload failure; do not block the first creatable item.
            continue;
        }

        // A successful upload (even if registration found no eligible instances), or an idle
        // reactivation, consumes a slot (an in-range one, or the budget's). The ms budget does not include RGL's
        // raytrace-time GAS build.
        if (bInRangeSlot)
        {
            ++OutStats.InRangeCreates;
        }
        else
        {
            ++BudgetedCreates;
        }
        if (!bResident && FindCachedMesh(StaticMesh))
        {
            ++OutStats.Created;
        }
    }
    PendingCreates.SetNum(Write);

    OutStats.InRangeMs = InRangeSeconds * 1000.0;
    OutStats.Ms = (FPlatformTime::Seconds() - StartSeconds) * 1000.0;
}

void FRGLSceneManager::DrainIsmReevaluations(int32 MaxChanges, FIsmReevalStats& OutStats)
{
    if (PendingIsmReevals.Num() == 0)
    {
        return;
    }

    const double StartSeconds = FPlatformTime::Seconds();
    int32 Changes = 0;
    int32 Done = 0;   // groups finished from the head of the queue
    for (; Done < PendingIsmReevals.Num(); ++Done)
    {
        if (MaxChanges > 0 && Changes >= MaxChanges)
        {
            break;   // the cap was reached at a group boundary: the rest waits for the next Update
        }
        UInstancedStaticMeshComponent* Comp = PendingIsmReevals[Done].Component.Get();
        FISMCEntityGroup* Group = Comp ? ISMCEntityMap.Find(Comp) : nullptr;
        // Gone or unregistered (UpdateTransforms drops the group), or its instance count or mesh changed since the
        // registration (the next sync rebuilds it; its spheres and indices no longer describe the component).
        if (!Group || Group->Component.Get() != Comp || !Comp->IsRegistered() ||
            Group->InstanceSpheres.Num() != Comp->GetInstanceCount() || Group->MeshKey != FObjectKey(Comp->GetStaticMesh()))
        {
            continue;
        }
        if (!ReevaluateIsmGroup(Comp, *Group, MaxChanges, Changes, OutStats))
        {
            break;   // the cap was reached inside this group: it stays at the head
        }
        if (Group->Entities.Num() == 0)
        {
            UnregisterISMComponent(Comp);   // no instance within the unload radius any more
        }
    }
    if (Done > 0)
    {
        PendingIsmReevals.RemoveAt(0, Done, EAllowShrinking::No);
    }
    OutStats.Ms += (FPlatformTime::Seconds() - StartSeconds) * 1000.0;
}

bool FRGLSceneManager::ReevaluateIsmGroup(UInstancedStaticMeshComponent* Component, FISMCEntityGroup& Group, int32 MaxChanges,
                                          int32& InOutChanges, FIsmReevalStats& OutStats)
{
    const int32 NumInstances = Group.InstanceSpheres.Num();
    auto CapReached = [MaxChanges, &InOutChanges]() { return MaxChanges > 0 && InOutChanges >= MaxChanges; };

    // Additions: an instance without an entity whose sphere reaches R (RegisterISMComponent's rule). The group holds
    // entities, so its mesh is live; the instance's transform is read only now, to place the new entity.
    rgl_mesh_t Mesh = nullptr;
    for (int32 i = 0; i < NumInstances; ++i)
    {
        const FSphere3f& Sphere = Group.InstanceSpheres[i];
        if (Sphere.W < 0.0f || Group.RegisteredMask[i] || !IsWithinAnySensor(FVector(Sphere.Center), Sphere.W, 0.0f))
        {
            continue;
        }
        if (CapReached())
        {
            return false;
        }
        if (!Mesh)
        {
            const FCachedMesh* Cached = MeshCache.Find(Group.MeshKey);
            Mesh = Cached ? Cached->Mesh : nullptr;
            if (!Mesh)
            {
                break;   // defensive: no resident mesh for a group with entities; leave the additions to a rebuild
            }
        }
        FTransform InstanceTransform;
        if (!Component->GetInstanceTransform(i, InstanceTransform, /*bWorldSpace=*/true) || IsZeroScaleInstance(InstanceTransform))
        {
            Group.InstanceSpheres[i].W = -1.0f;   // not registrable any more
            continue;
        }
        rgl_entity_t Entity = nullptr;
        if (RGL_PROBED("rgl_entity_create", rgl_entity_create(&Entity, Scene, Mesh)) != RGL_SUCCESS || !Entity)
        {
            continue;   // tried again by the next sync's round
        }
        rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(InstanceTransform);
        RGL_CHECK(RGL_PROBED("rgl_entity_set_transform", rgl_entity_set_transform(Entity, &RGLTransform)));
        Group.Entities.Add(Entity);
        Group.InstanceIndices.Add(i);
        Group.InstanceWorldTransforms.Add(InstanceTransform);
        Group.RegisteredMask[i] = true;
        ++Group.MeshRefCount;
        AddMeshReference(Group.MeshKey);
        ++InOutChanges;
        ++OutStats.Added;
    }

    // Removals: an entity whose instance sphere is beyond R x rgl.Scene.UnloadDistanceFactor + the unload margin (the
    // rule ShouldKeepRegisteredComponent applies to whole components). Back to front: RemoveAtSwap only moves entries
    // that were looked at already.
    const float UnloadScale = SceneUnloadDistanceFactor();
    for (int32 k = Group.Entities.Num() - 1; k >= 0; --k)
    {
        const int32 Index = Group.InstanceIndices[k];
        const FSphere3f& Sphere = Group.InstanceSpheres[Index];
        if (Sphere.W >= 0.0f && IsWithinAnySensor(FVector(Sphere.Center), Sphere.W, UnregistrationHysteresisCm, UnloadScale))
        {
            continue;
        }
        if (CapReached())
        {
            return false;
        }
        if (Group.Entities[k])
        {
            RGL_PROBED("rgl_entity_destroy", rgl_entity_destroy(Group.Entities[k]));
        }
        Group.Entities.RemoveAtSwap(k, EAllowShrinking::No);
        Group.InstanceIndices.RemoveAtSwap(k, EAllowShrinking::No);
        Group.InstanceWorldTransforms.RemoveAtSwap(k, EAllowShrinking::No);
        Group.RegisteredMask[Index] = false;
        --Group.MeshRefCount;
        ReleaseMeshReference(Group.MeshKey);
        ++InOutChanges;
        ++OutStats.Removed;
    }
    return true;
}

FRGLSceneManager::FSceneKnobs FRGLSceneManager::LogSceneConfigIfChanged()
{
    FSceneKnobs K = ReadSceneKnobs();
    K.UnloadMarginCm = UnregistrationHysteresisCm;
    K.WpMap = bWpActive ? 1 : 0;
    if (bSceneKnobsLogged && K == LoggedSceneKnobs)
    {
        return K;
    }
    bSceneKnobsLogged = true;
    LoggedSceneKnobs = K;
    const FString Line = FormatSceneConfigLine(CurrentSimTime, K);
    EmitSceneLine(Line);
#if !UE_BUILD_SHIPPING
    LastConfigLine_ForTest = Line;
#endif
    return K;
}

// ============================================================================
// World Partition culling of non-Static parts (7b)
// ============================================================================
// On a World Partition map (rgl.Scene.WpMovableCulling = 1), a non-Static static mesh part that is not protected
// (ClassifyWpProtection) and has not been seen moving is registered only when its bounds (ISM: each instance's) reach
// some sensor's load sphere: R_load = max(R, range + M), M = max(MinMargin, (V_sensor + V_object) * H), H = T_sync_max +
// L_create + U (ComputeWpCullHorizon). It leaves beyond R_load * UnloadDistanceFactor + the unload margin, for every
// sensor. Every sync looks at all candidates again (the full scan, as before); a part whose transform changed (or with
// a velocity) is promoted and kept at any distance from then on. Adds and removals go through one queue: one entity per
// change, under rgl.Scene.MaxMovableEntityChangesPerUpdate / MovableEntityBudgetMs and the mesh upload budget, nearest to
// reach a range first. A culled part closes at most (V_sensor + V_object) * H on a sensor's range before the next sync
// has looked at it and its entity exists, as long as U, the speed bounds and L_create hold; a deadline on the sim clock
// forces that sync even when the world dt shrinks. Violations of the contracts are logged
// (rgl_scene_wp_cull_violation); a teleport, a new sensor or time going back forces a sync but is not covered.

namespace RGLWpCull
{
    /// Same tolerances as the audit's stale-instance test: 1 cm, ~0.01 deg, 0.1 % of scale.
    static bool TransformChanged(const FTransform& Now, const FTransform& Before)
    {
        return !Now.GetLocation().Equals(Before.GetLocation(), 1.0) ||
               !Now.GetRotation().Equals(Before.GetRotation(), 1.0e-4) ||
               !Now.GetScale3D().Equals(Before.GetScale3D(), 1.0e-3);
    }
    static bool IsZeroScale(const FTransform& T)
    {
        const FVector S = T.GetScale3D();
        return FMath::IsNearlyZero(S.X) || FMath::IsNearlyZero(S.Y) || FMath::IsNearlyZero(S.Z);
    }
    /// A velocity of the part or its owner above 1 cm/s.
    static bool HasVelocity(const UStaticMeshComponent* Comp)
    {
        const AActor* Owner = Comp->GetOwner();
        return Comp->GetComponentVelocity().SizeSquared() > 1.0 || (Owner && Owner->GetVelocity().SizeSquared() > 1.0);
    }
}

FRGLSceneManager::FWpCullHorizon FRGLSceneManager::ComputeWpCullHorizon(EWpCullClock Clock, double IntervalSeconds,
                                                                         double WorldDtSeconds, double MaxUpdateGapSeconds,
                                                                         double CreateLatencySeconds)
{
    FWpCullHorizon H;
    const double U = MaxUpdateGapSeconds;
    const double D = (FMath::IsFinite(WorldDtSeconds) && WorldDtSeconds > 0.0) ? WorldDtSeconds : U;
    if (Clock == EWpCullClock::SimClock)
    {
        H.SyncBoundSeconds = IntervalSeconds + U;
    }
    else
    {
        H.SyncBoundSeconds = FMath::CeilToDouble(IntervalSeconds / D) * FMath::Max(U, D) + U;
    }
    H.HorizonSeconds = H.SyncBoundSeconds + CreateLatencySeconds + U;
    return H;
}

double FRGLSceneManager::ComputeWpMarginM(double HorizonSeconds, double SensorSpeedMps, double ObjectSpeedMps, double MinMarginM)
{
    return FMath::Max(MinMarginM, (SensorSpeedMps + ObjectSpeedMps) * HorizonSeconds);
}

bool FRGLSceneManager::UsesWpNonStaticCulling(const UWorld* World) const
{
    if (SceneWpMovableCulling() == 0)
    {
        return false;
    }
#if !UE_BUILD_SHIPPING
    if (WpMapOverride_ForTest.IsSet())
    {
        return WpMapOverride_ForTest.GetValue();
    }
#endif
    return World && World->GetWorldPartition() != nullptr;   // as CheckStreamCoverage and AuditScene decide
}

float FRGLSceneManager::WorldDeltaSeconds(const UWorld* World) const
{
#if !UE_BUILD_SHIPPING
    if (WorldDtOverride_ForTest.IsSet())
    {
        return WorldDtOverride_ForTest.GetValue();
    }
#endif
    return World ? World->GetDeltaSeconds() : 0.0f;
}

FRGLSceneManager::FWpKnobs FRGLSceneManager::CurrentWpKnobs() const
{
    FWpKnobs K;
    K.Enable = SceneWpMovableCulling();
    K.MinMarginM = SceneMovableMinMarginM();
    K.SensorSpeedMps = SceneMovableSensorSpeedBoundMps();
    K.ObjectSpeedMps = SceneMovableObjectSpeedBoundMps();
    K.MaxUpdateGapS = SceneMovableMaxUpdateGapSeconds();
    K.CreateLatencyS = SceneMovableCreateLatencySeconds();
    K.IntervalS = SceneSyncIntervalSeconds();
    K.Clock = SceneSyncClock();
    return K;
}

const TCHAR* FRGLSceneManager::WpProtectName(EWpProtect Reason)
{
    switch (Reason)
    {
    case EWpProtect::Pawn:              return TEXT("pawn");
    case EWpProtect::MovementComponent: return TEXT("movement_component");
    case EWpProtect::Skeletal:          return TEXT("skeletal");
    case EWpProtect::Physics:           return TEXT("physics");
    case EWpProtect::SensorParent:      return TEXT("sensor_parent");
    default:                            return TEXT("none");
    }
}

FRGLSceneManager::EWpProtect FRGLSceneManager::ClassifyWpProtection(const UStaticMeshComponent* Component, bool bRemember) const
{
    if (!Component)
    {
        return EWpProtect::None;
    }
    const FObjectKey Key(Component);
    if (const EWpProtect* Sticky = WpStickyProtected.Find(Key))
    {
        return *Sticky;   // once protected, protected for the part's life
    }
    EWpProtect Reason = EWpProtect::None;
    // The part and the components it is attached to (across actors): physics, or a skeletal mesh to ride on.
    int32 Depth = 0;
    for (const USceneComponent* S = Component; S && Reason == EWpProtect::None && Depth < 64; S = S->GetAttachParent(), ++Depth)
    {
        if (const UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(S))
        {
            if (Prim->IsSimulatingPhysics())
            {
                Reason = EWpProtect::Physics;
                break;
            }
        }
        if (S != Component && S->IsA<USkeletalMeshComponent>())
        {
            Reason = EWpProtect::Skeletal;
        }
    }
    // The owner and the actors it is attached to: a Pawn (vehicles, walkers), an actor that moves itself (movement
    // component) or carries a skeletal mesh, a sensor's carrier (self-cutout vehicle, audit parent). Cached per actor
    // within one sync (the sync paths only).
    const AActor* Owner = Component->GetOwner();
    if (Reason == EWpProtect::None && Owner)
    {
        if (bRemember && WpCachedActor == Owner && WpCachedActorSerial == SceneSyncSerial)
        {
            Reason = WpCachedActorVerdict;
        }
        else
        {
            EWpProtect ActorReason = EWpProtect::None;
            int32 ActorDepth = 0;
            for (const AActor* A = Owner; A && ActorReason == EWpProtect::None && ActorDepth < 64; A = A->GetAttachParentActor(), ++ActorDepth)
            {
                if (A->IsA<APawn>())
                {
                    ActorReason = EWpProtect::Pawn;
                }
                else if (A->FindComponentByClass<UMovementComponent>())
                {
                    ActorReason = EWpProtect::MovementComponent;
                }
                else if (A->FindComponentByClass<USkeletalMeshComponent>())
                {
                    ActorReason = EWpProtect::Skeletal;
                }
                else
                {
                    for (const auto& Pair : RegisteredSensors)
                    {
                        if (Pair.Value.CutoutVehicle.Get() == A || Pair.Value.AuditParent.Get() == A)
                        {
                            ActorReason = EWpProtect::SensorParent;
                            break;
                        }
                    }
                }
            }
            if (bRemember)
            {
                WpCachedActor = Owner;
                WpCachedActorSerial = SceneSyncSerial;
                WpCachedActorVerdict = ActorReason;
            }
            Reason = ActorReason;
        }
    }
    if (Reason != EWpProtect::None && bRemember)
    {
        WpStickyProtected.Add(Key, Reason);
    }
    return Reason;
}

void FRGLSceneManager::WpViolation(const TCHAR* Kind, double Value, double Bound)
{
    ++WpViolationsSinceLog;
    int32& N = WpViolationCounts.FindOrAdd(FString(Kind));
    ++N;
    if (N <= 20 || (N % 1000) == 0)   // the first ones of each kind, then one in 1000 (the sync line counts them all)
    {
        EmitSceneLine(FString::Printf(TEXT("rgl_scene_wp_cull_violation t=%.3f kind=%s value=%.3f bound=%.3f n=%d"),
                                      CurrentSimTime, Kind, Value, Bound, N));
    }
}

void FRGLSceneManager::WpCheckContracts(const UWorld* World, bool& bOutDeadline, bool& bOutReset, bool& bOutJump)
{
    const FWpKnobs K = CurrentWpKnobs();
    const double U = K.MaxUpdateGapS;
    const double Tol = U * 1.0e-3 + 1.0e-6;
    // R10: this interval was classified at its START, after that Update's init sync. Sensors joining now must
    // neither invalidate its exemption nor excuse an existing sensor's later gap. Consume the snapshot once.
    const bool bStartupUpdateInterval = bWpStartupSceneIntervalPending;
    bWpStartupSceneIntervalPending = false;
    const double Dt = FMath::Max(0.0f, WorldDeltaSeconds(World));
    auto StartupGap = [this, U, Dt](const TCHAR* Kind, double Gap, const void* Sensor, double InitSync)
    {
        EmitSceneLine(FString::Printf(TEXT("rgl_scene_wp_cull_startup_gap t=%.3f kind=%s value=%.6f limit=%.6f world_dt=%.6f sensor=%p init_sync=%.3f"),
                                      CurrentSimTime, Kind, Gap, U, Dt, Sensor, InitSync));
    };
    if (WpPrevUpdateSimTime >= 0.0)
    {
        if (CurrentSimTime < WpPrevUpdateSimTime)
        {
            bOutReset = true;   // time went back: no continuity to rely on
            bOutJump = true;
        }
        else if (CurrentSimTime - WpPrevUpdateSimTime > U + Tol)
        {
            const double Gap = CurrentSimTime - WpPrevUpdateSimTime;
            if (bStartupUpdateInterval && Gap <= U + Dt + Tol)
            {
                StartupGap(TEXT("update_gap"), Gap, nullptr, WpPrevUpdateSimTime);
            }
            else
            {
                WpViolation(TEXT("update_gap"), Gap, U);
            }
        }
    }
    WpPrevUpdateSimTime = CurrentSimTime;
    for (auto& Pair : RegisteredSensors)
    {
        FSensorRegistration& S = Pair.Value;
        S.bWpStartupUpdatePending = false; // consumed even after a normal, oversized or rewound first interval
        if (S.bTimeRewound)
        {
            S.bTimeRewound = false;
            S.bWpStartupPosePending = false;
            bOutReset = true;
            bOutJump = true;
        }
        if (!S.bRefreshed)
        {
            continue;
        }
        S.bRefreshed = false;
        const bool bStartupPoseInterval = S.bWpStartupPosePending && S.LastUpdateTime > S.WpInitPoseTime;
        if (S.LastRefreshGapSeconds > U + Tol)
        {
            if (bStartupPoseInterval && S.LastRefreshGapSeconds <= U + Dt + Tol)
            {
                StartupGap(TEXT("pose_gap"), S.LastRefreshGapSeconds, Pair.Key, S.WpInitSyncSimTime);
            }
            else
            {
                WpViolation(TEXT("pose_gap"), S.LastRefreshGapSeconds, U);
            }
        }
        // Consume even a normal first refresh; only the first interval can use the exemption.
        if (bStartupPoseInterval)
        {
            S.bWpStartupPosePending = false;
        }
        // Faster than the bound: a pose jump (teleport) rather than motion the margin covers. Looked at again now.
        if (S.ObservedSpeedCmps > K.SensorSpeedMps * 100.0f * (1.0f + 1.0e-3f))
        {
            WpViolation(TEXT("sensor_speed"), S.ObservedSpeedCmps / 100.0, K.SensorSpeedMps);
            bOutReset = true;
            bOutJump = true;
        }
    }
    if (!bWpSyncValid)
    {
        return;
    }
    if (K != WpSyncKnobs)
    {
        bOutReset = true;   // the margin was worked out with other inputs
    }
    if (CurrentSimTime < WpLastSyncSimTime)
    {
        bOutReset = true;
        bOutJump = true;
    }
    // The next Update may come up to U later: sync now if that would pass the time the last culling sync assumed. With a
    // fixed dt the periodic sync comes first (T_sync_max holds its rounding), so this fires only when the dt shrank on
    // the Update clock; on the simulation clock (T_sync_max = I + U) the periodic sync always comes first.
    else if (CurrentSimTime - WpLastSyncSimTime + U > WpAssumedSyncSeconds + 1.0e-6)
    {
        bOutDeadline = true;
    }
}

void FRGLSceneManager::WpBeginSync(float Dt)
{
    const FWpKnobs K = CurrentWpKnobs();
    // The periodic sync's own clock and interval (the time trigger in UpdateStep reads the same two knobs).
    const EWpCullClock Clock = (K.Clock == 1) ? EWpCullClock::SimClock : EWpCullClock::UpdateClock;
    const FWpCullHorizon H = ComputeWpCullHorizon(Clock, K.IntervalS, Dt, K.MaxUpdateGapS, K.CreateLatencyS);
    WpAssumedSyncSeconds = H.SyncBoundSeconds;
    WpHorizonSeconds = H.HorizonSeconds;
    WpMarginCm = ComputeWpMarginM(H.HorizonSeconds, K.SensorSpeedMps, K.ObjectSpeedMps, K.MinMarginM) * 100.0;
    WpLastSyncSimTime = CurrentSimTime;
    WpSyncKnobs = K;
    bWpSyncValid = true;
    ScanWpJobs.Reset();
    WpSyncCheckMs = 0.0;
}

float FRGLSceneManager::WpLoadRadiusCm(const FSensorRegistration& S) const
{
    return FMath::Max(S.RegistrationDistanceCm, SensorRangeCm(S) + static_cast<float>(WpMarginCm));
}

bool FRGLSceneManager::WpInAnyLoadSphere(const FVector& CenterCm, float RadiusCm) const
{
    for (const auto& Pair : RegisteredSensors)
    {
        const double R = WpLoadRadiusCm(Pair.Value) + RadiusCm;
        if (FVector::DistSquared(CenterCm, Pair.Value.Position) <= R * R)
        {
            return true;
        }
    }
    return false;
}

bool FRGLSceneManager::WpInAnyUnloadSphere(const FVector& CenterCm, float RadiusCm) const
{
    const float Factor = SceneUnloadDistanceFactor();
    for (const auto& Pair : RegisteredSensors)
    {
        const double R = WpLoadRadiusCm(Pair.Value) * Factor + UnregistrationHysteresisCm + RadiusCm;
        if (FVector::DistSquared(CenterCm, Pair.Value.Position) <= R * R)
        {
            return true;
        }
    }
    return false;
}

double FRGLSceneManager::WpReachSimTime(const FVector& CenterCm, float RadiusCm) const
{
    const double ClosingCmps = FMath::Max(1.0, (static_cast<double>(WpSyncKnobs.SensorSpeedMps) + WpSyncKnobs.ObjectSpeedMps) * 100.0);
    double Best = TNumericLimits<double>::Max();
    for (const auto& Pair : RegisteredSensors)
    {
        const double GapCm = FVector::Dist(CenterCm, Pair.Value.Position) - RadiusCm - SensorRangeCm(Pair.Value);
        Best = FMath::Min(Best, CurrentSimTime + FMath::Max(0.0, GapCm) / ClosingCmps);
    }
    return Best;
}

FSphere FRGLSceneManager::WpInstanceSphere(const UStaticMesh* Mesh, const FTransform& InstanceTransform)
{
    const FBoxSphereBounds B = Mesh ? Mesh->GetBounds() : FBoxSphereBounds(ForceInit);
    const FVector S = InstanceTransform.GetScale3D();
    const double MaxScale = FMath::Max3(FMath::Abs(S.X), FMath::Abs(S.Y), FMath::Abs(S.Z));
    return FSphere(InstanceTransform.TransformPosition(B.Origin), B.SphereRadius * MaxScale);
}

void FRGLSceneManager::WpVisitCandidate(UStaticMeshComponent* Comp, UInstancedStaticMeshComponent* Ism)
{
    const FObjectKey Key(Comp);
    FWpCandidate* Found = WpCandidates.Find(Key);
    const bool bNew = (Found == nullptr);
    FWpCandidate& C = bNew ? WpCandidates.Add(Key) : *Found;
    if (bNew)
    {
        C.Component = Comp;
        C.bIsm = (Ism != nullptr);
    }
    UStaticMesh* Mesh = Comp->GetStaticMesh();
    const FObjectKey MeshKey(Mesh);
    // A mesh whose extraction failed waits for its retry without a job (as the original scan; the audit counts it as
    // unextractable, not missing).
    const bool bMeshDeferred = IsMeshRetryDeferred(Mesh);
    const double Elapsed = CurrentSimTime - C.SeenSimTime;
    const double ObjectBoundCmps = WpSyncKnobs.ObjectSpeedMps * 100.0;
    ++WpCheckN;

    auto Promote = [this, &C, &Key]()
    {
        if (!C.bMoved)
        {
            C.bMoved = true;
            if (C.bIsm)
            {
                WpMovedIsm.AddUnique(Key);
            }
        }
    };
    auto CheckSpeed = [this, Elapsed, ObjectBoundCmps](const FSphere& Before, const FSphere& Now)
    {
        // The bounds' surface: the center's move plus the radius' change (turns and scale included through both).
        if (Elapsed > 0.0)
        {
            const double Speed = (FVector::Dist(Before.Center, Now.Center) + FMath::Abs(Now.W - Before.W)) / Elapsed;
            if (Speed > ObjectBoundCmps * (1.0 + 1.0e-3))
            {
                WpViolation(TEXT("object_speed"), Speed / 100.0, ObjectBoundCmps / 100.0);
            }
        }
    };
    // The contract covers a job when the part was known before this sync with the same layout (set below for ISMs),
    // and this sync does not follow a discontinuity.
    bool bKnown = !bNew && C.MeshKey == MeshKey;
    auto AddJob = [this, Comp, &Key, &C, &bKnown](int32 Instance, bool bAdd, const FVector& Center, float Radius, double Since,
                                                   bool bWasCulled)
    {
        FWpJob& J = ScanWpJobs.AddDefaulted_GetRef();
        J.bCovered = bKnown && !bWpSyncDiscontinuous;
        J.bLateCovered = J.bCovered && bWasCulled;
        J.Component = Comp;
        J.Key = Key;
        J.Instance = Instance;
        J.Generation = C.Generation;
        J.bAdd = bAdd;
        J.SinceSimTime = Since;
        J.ReachSimTime = bAdd ? WpReachSimTime(Center, Radius) : TNumericLimits<double>::Max();
        // Reach time is now exactly when bounds intersect ANY sensor's raw range. The same sphere helper supplies
        // offset-pivot/scale-aware ISM bounds; neither registration radius nor prefetch margin makes this urgent.
        J.bInRangeAtDiscovery = bAdd && J.ReachSimTime <= CurrentSimTime;
        J.GapCm = NearestSensorGapCm(Center, Radius);
    };

    if (!Ism)
    {
        // ---- a regular part ----
        const FTransform T = Comp->GetComponentTransform();
        if (!bNew && C.MeshKey == MeshKey && RGLWpCull::TransformChanged(T, C.Transform))
        {
            CheckSpeed(WpInstanceSphere(Mesh, C.Transform), WpInstanceSphere(Mesh, T));
            Promote();
        }
        if (!C.bMoved && RGLWpCull::HasVelocity(Comp))
        {
            Promote();
        }
        if (!bNew && C.MeshKey != MeshKey)
        {
            ++C.Generation;   // another mesh: the old jobs drop out
        }
        // An entity built from another mesh (swapped) leaves now; the queue builds the new one.
        if (const FEntityInfo* Info = EntityMap.Find(Comp))
        {
            if (Info->MeshKey != MeshKey)
            {
                const double R0 = FPlatformTime::Seconds();
                UnregisterComponent(Comp);
                WpVisitRegSeconds += FPlatformTime::Seconds() - R0;
            }
        }
        C.MeshKey = MeshKey;
        C.Transform = T;
        C.SeenSimTime = CurrentSimTime;
        C.SeenSyncSerial = SceneSyncSerial;

        const FVector Origin = Comp->Bounds.Origin;
        const float Radius = static_cast<float>(Comp->Bounds.SphereRadius);
        const bool bRegistered = EntityMap.Contains(Comp);
        const bool bWanted = C.bMoved || WpInAnyLoadSphere(Origin, Radius);
        const bool bWasCulled = bKnown && C.bCulledAtSync;
        if (bWanted && !bRegistered && !bMeshDeferred)
        {
            if (C.WantSince < 0.0)
            {
                C.WantSince = CurrentSimTime;
            }
            AddJob(INDEX_NONE, true, Origin, Radius, C.WantSince, bWasCulled);
        }
        else
        {
            C.WantSince = -1.0;
        }
        if (!bWanted && bRegistered && !WpInAnyUnloadSphere(Origin, Radius))
        {
            AddJob(INDEX_NONE, false, Origin, Radius, CurrentSimTime, false);
        }
        C.bCulledAtSync = !bWanted && !bRegistered;
        return;
    }

    // ---- an ISM: every instance on its own ----
    const int32 NumInstances = Ism->GetInstanceCount();
    FISMCEntityGroup* Group = ISMCEntityMap.Find(Ism);
    // A group of another component at a reused address, or built for another mesh / instance count (also one the
    // original path built before this part was a candidate): it leaves now; the queue builds the instances again.
    if (Group && (Group->Component.Get() != Ism || Group->MeshKey != MeshKey || Group->OriginalInstanceCount != NumInstances))
    {
        const double R0 = FPlatformTime::Seconds();
        UnregisterISMComponent(Ism);
        WpVisitRegSeconds += FPlatformTime::Seconds() - R0;
        Group = nullptr;
    }
    const bool bLayoutChanged = !bNew && (C.InstanceCount != NumInstances || C.MeshKey != MeshKey);
    bKnown = !bNew && !bLayoutChanged;
    if (bLayoutChanged)
    {
        ++C.Generation;   // old instance indices may name other instances now: their jobs drop out
        C.InstanceLocal.Reset();
        C.InstanceSpheres.Reset();
        C.InstanceWantSince.Reset();
        C.InstanceCulled.Empty();
    }
    const bool bHistory = bKnown && C.InstanceLocal.Num() == NumInstances && C.InstanceSpheres.Num() == NumInstances;
    const int32 OldWant = C.InstanceWantSince.Num();
    C.InstanceLocal.SetNum(NumInstances);
    C.InstanceSpheres.SetNum(NumInstances);
    C.InstanceWantSince.SetNum(NumInstances);
    for (int32 i = OldWant; i < NumInstances; ++i)
    {
        C.InstanceWantSince[i] = -1.0;
    }
    if (C.InstanceCulled.Num() != NumInstances)
    {
        C.InstanceCulled.Init(false, NumInstances);   // no history of being culled (new part or new layout)
    }
    WpCheckInstances += NumInstances;

    // Pass 1: which instances changed since the last sync (their local matrix, or the component's transform), and their
    // spheres. An unchanged instance keeps its recorded sphere (nothing is decomposed); a changed one is read again,
    // and with a history it moved (the whole component is promoted; its speed is checked).
    const FTransform CompTransform = Comp->GetComponentTransform();
    const bool bCompSame = bHistory && CompTransform.Equals(C.Transform, 0.0);
    const TArray<FInstancedStaticMeshInstanceData>& Data = Ism->PerInstanceSMData;
    const int32 NumData = FMath::Min(NumInstances, Data.Num());
    bool bMovedNow = false;
    for (int32 i = 0; i < NumInstances; ++i)
    {
        const bool bHasData = i < NumData;
        if (bCompSame && bHasData && FMemory::Memcmp(&Data[i].Transform, &C.InstanceLocal[i], sizeof(Data[i].Transform)) == 0)
        {
            continue;   // unchanged: the recorded sphere stands
        }
        FTransform T;
        const bool bOk = Ism->GetInstanceTransform(i, T, /*bWorldSpace=*/true) && !RGLWpCull::IsZeroScale(T);
        const FSphere NewSphere = bOk ? WpInstanceSphere(Mesh, T) : FSphere(FVector::ZeroVector, -1.0);
        if (bHistory)
        {
            const FTransform Before = FTransform(FMatrix(C.InstanceLocal[i])) * C.Transform;
            if (!bOk || RGLWpCull::TransformChanged(T, Before))
            {
                bMovedNow = true;
                const FSphere3f& Old = C.InstanceSpheres[i];
                if (bOk && Old.W >= 0.0f)
                {
                    CheckSpeed(FSphere(FVector(Old.Center), Old.W), NewSphere);
                }
            }
        }
        if (bHasData)
        {
            C.InstanceLocal[i] = Data[i].Transform;
        }
        C.InstanceSpheres[i] = FSphere3f(FVector3f(NewSphere.Center), static_cast<float>(NewSphere.W));
    }
    if (bMovedNow || (!C.bMoved && RGLWpCull::HasVelocity(Comp)))
    {
        Promote();   // one moving instance keeps the whole component
    }
    C.Transform = CompTransform;
    C.MeshKey = MeshKey;
    C.InstanceCount = NumInstances;
    C.SeenSimTime = CurrentSimTime;
    C.SeenSyncSerial = SceneSyncSerial;

    // Pass 2: choose the instances.
    for (int32 i = 0; i < NumInstances; ++i)
    {
        const FSphere3f& Sphere = C.InstanceSpheres[i];
        const FVector Center(Sphere.Center);
        const bool bValid = Sphere.W >= 0.0f;
        const bool bRegistered = Group && Group->RegisteredMask.IsValidIndex(i) && Group->RegisteredMask[i];
        const bool bWanted = bValid && (C.bMoved || WpInAnyLoadSphere(Center, Sphere.W));
        const bool bWasCulled = bKnown && C.InstanceCulled[i];
        if (bWanted && !bRegistered && !bMeshDeferred)
        {
            if (C.InstanceWantSince[i] < 0.0)
            {
                C.InstanceWantSince[i] = CurrentSimTime;
            }
            AddJob(i, true, Center, Sphere.W, C.InstanceWantSince[i], bWasCulled);
        }
        else
        {
            C.InstanceWantSince[i] = -1.0;
        }
        if (bRegistered && (!bValid || (!bWanted && !WpInAnyUnloadSphere(Center, Sphere.W))))
        {
            AddJob(i, false, Center, FMath::Max(0.0f, Sphere.W), CurrentSimTime, false);
        }
        C.InstanceCulled[i] = bValid && !bWanted && !bRegistered;
    }
}

void FRGLSceneManager::WpEndSync()
{
    // Candidates the scan no longer saw (gone, hidden, unregistered, now protected or Static): their state goes; the
    // original rules handle their entities (the gate and the removal loops, or the original registration).
    for (auto It = WpCandidates.CreateIterator(); It; ++It)
    {
        const FWpCandidate& C = It.Value();
        if (C.SeenSyncSerial == SceneSyncSerial)
        {
            continue;
        }
        // Protected now (attached to a vehicle, physics on, ...) with a partial group: the original path keeps a
        // registered group as it is, so this one leaves the scene and the next sync registers it whole.
        if (UInstancedStaticMeshComponent* Ism = Cast<UInstancedStaticMeshComponent>(C.Component.Get()))
        {
            if (const FISMCEntityGroup* Group = ISMCEntityMap.Find(Ism))
            {
                if (Group->Component.Get() == Ism && Ism->Mobility != EComponentMobility::Static &&
                    Group->Entities.Num() < Ism->GetInstanceCount())
                {
                    UnregisterISMComponent(Ism);
                }
            }
        }
        It.RemoveCurrent();
    }
    for (int32 m = WpMovedIsm.Num() - 1; m >= 0; --m)
    {
        if (!WpCandidates.Contains(WpMovedIsm[m]))
        {
            WpMovedIsm.RemoveAtSwap(m, EAllowShrinking::No);
        }
    }
    // Adds first, the soonest to reach a range first (then the nearest, then a stable order); removals after them.
    ScanWpJobs.Sort([](const FWpJob& A, const FWpJob& B)
    {
        if (A.bAdd != B.bAdd)
        {
            return A.bAdd;
        }
        if (A.bAdd && A.ReachSimTime != B.ReachSimTime)
        {
            return A.ReachSimTime < B.ReachSimTime;
        }
        if (A.GapCm != B.GapCm)
        {
            return A.bAdd ? (A.GapCm < B.GapCm) : (A.GapCm > B.GapCm);
        }
        if (A.Key != B.Key)
        {
            return A.Key < B.Key;
        }
        return A.Instance < B.Instance;
    });
    Swap(WpJobs, ScanWpJobs);
    ScanWpJobs.Reset();
    WpCheckMs += WpSyncCheckMs;
    WpCheckMaxMs = FMath::Max(WpCheckMaxMs, WpSyncCheckMs);
}

void FRGLSceneManager::WpTeardown()
{
    // Groups the culling built (possibly partial) leave the scene; the original rule registers those ISMs whole at this
    // sync's scan. Regular entities stay: the original rule keeps every non-Static part.
    for (const auto& Pair : WpCandidates)
    {
        if (UInstancedStaticMeshComponent* Ism = Cast<UInstancedStaticMeshComponent>(Pair.Value.Component.Get()))
        {
            if (const FISMCEntityGroup* Group = ISMCEntityMap.Find(Ism))
            {
                if (Group->Component.Get() == Ism)
                {
                    UnregisterISMComponent(Ism);
                }
            }
        }
    }
    WpCandidates.Empty();
    WpJobs.Empty();
    ScanWpJobs.Empty();
    WpMovedIsm.Empty();
    bWpSyncValid = false;
    WpPrevUpdateSimTime = -1.0;
    bWpStartupSceneIntervalPending = false;
}

bool FRGLSceneManager::WpAddRegular(UStaticMeshComponent* Comp)
{
    return !EntityMap.Contains(Comp) && RegisterComponent(Comp);   // the original registration of one entity
}

bool FRGLSceneManager::WpAddInstance(UInstancedStaticMeshComponent* Comp, int32 Instance, const FTransform& InstanceTransform)
{
    UStaticMesh* Mesh = Comp->GetStaticMesh();
    if (!Mesh)
    {
        return false;
    }
    const FObjectKey MeshKey(Mesh);
    const int32 NumInstances = Comp->GetInstanceCount();
    FISMCEntityGroup* Group = ISMCEntityMap.Find(Comp);
    if (Group && (Group->Component.Get() != Comp || Group->MeshKey != MeshKey || Group->OriginalInstanceCount != NumInstances))
    {
        UnregisterISMComponent(Comp);   // built for another layout
        Group = nullptr;
    }
    if (Group && Group->RegisteredMask.IsValidIndex(Instance) && Group->RegisteredMask[Instance])
    {
        return true;
    }
    rgl_mesh_t RGLMesh = UploadMesh(Mesh);
    if (!RGLMesh)
    {
        return false;
    }
    AddMeshReference(MeshKey);   // held by the new entity (released again if it cannot be made)
    rgl_entity_t Entity = nullptr;
    if (RGL_PROBED("rgl_entity_create", rgl_entity_create(&Entity, Scene, RGLMesh)) != RGL_SUCCESS || !Entity)
    {
        ReleaseMeshReference(MeshKey);
        return false;
    }
    rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(InstanceTransform);
    RGL_CHECK(RGL_PROBED("rgl_entity_set_transform", rgl_entity_set_transform(Entity, &RGLTransform)));
    if (!Group)
    {
        FISMCEntityGroup NewGroup;
        NewGroup.Component = Comp;
        NewGroup.MeshKey = MeshKey;
        NewGroup.OriginalInstanceCount = NumInstances;
        NewGroup.RegisteredMask.Init(false, NumInstances);
        Group = &ISMCEntityMap.Add(Comp, MoveTemp(NewGroup));
    }
    Group->Entities.Add(Entity);
    Group->InstanceIndices.Add(Instance);
    Group->InstanceWorldTransforms.Add(InstanceTransform);
    Group->RegisteredMask[Instance] = true;
    ++Group->MeshRefCount;
    return true;
}

bool FRGLSceneManager::WpRemoveInstance(UInstancedStaticMeshComponent* Comp, int32 Instance)
{
    FISMCEntityGroup* Group = ISMCEntityMap.Find(Comp);
    if (!Group || !Group->RegisteredMask.IsValidIndex(Instance) || !Group->RegisteredMask[Instance])
    {
        return false;
    }
    for (int32 k = Group->Entities.Num() - 1; k >= 0; --k)
    {
        if (Group->InstanceIndices[k] != Instance)
        {
            continue;
        }
        if (Group->Entities[k])
        {
            RGL_PROBED("rgl_entity_destroy", rgl_entity_destroy(Group->Entities[k]));
        }
        Group->Entities.RemoveAtSwap(k, EAllowShrinking::No);
        Group->InstanceIndices.RemoveAtSwap(k, EAllowShrinking::No);
        Group->InstanceWorldTransforms.RemoveAtSwap(k, EAllowShrinking::No);
        Group->RegisteredMask[Instance] = false;
        --Group->MeshRefCount;
        ReleaseMeshReference(Group->MeshKey);
        if (Group->Entities.Num() == 0)
        {
            UnregisterISMComponent(Comp);   // no entity left: the group goes (its references are released already)
        }
        return true;
    }
    return false;
}

void FRGLSceneManager::WpDrain(bool bUnlimited)
{
    const double StartSeconds = FPlatformTime::Seconds();
    const int32 MaxChanges = bUnlimited ? 0 : SceneMaxMovableEntityChangesPerUpdate();
    const double BudgetSeconds = bUnlimited ? 0.0 : static_cast<double>(SceneMovableEntityBudgetMs()) / 1000.0;
    const int32 MaxUploads = bUnlimited ? 0 : SceneMaxMeshCreatesPerUpdate();
    const double UploadBudgetSeconds = bUnlimited ? 0.0 : static_cast<double>(SceneCreateBudgetMs()) / 1000.0;
    const double Latency = CurrentWpKnobs().CreateLatencyS;
    int32 Changes = 0;
    int32 Uploads = 0;
    double UploadSeconds = 0.0;
    double ClockStart = -1.0;   // the ms budget runs from the end of the first change (that call may carry the graph wait)
    bool bStop = false;
    int32 Write = 0;
    for (int32 Read = 0; Read < WpJobs.Num(); ++Read)
    {
        FWpJob& J = WpJobs[Read];
        auto Keep = [this, &Write, Read]()
        {
            if (Write != Read)
            {
                WpJobs[Write] = MoveTemp(WpJobs[Read]);
            }
            ++Write;
        };
        if (bStop && !J.bInRangeAtDiscovery)
        {
            Keep();
            continue;
        }
        UStaticMeshComponent* Comp = J.Component.Get();
        const FWpCandidate* C = WpCandidates.Find(J.Key);
        if (!Comp || !C || C->Generation != J.Generation || !Comp->IsRegistered() || !Comp->GetStaticMesh())
        {
            continue;   // gone, or the layout changed: the next sync queues what is needed
        }
        if (!J.bInRangeAtDiscovery && Changes > 0 && ((MaxChanges > 0 && Changes >= MaxChanges) ||
                            (BudgetSeconds > 0.0 && ClockStart >= 0.0 && FPlatformTime::Seconds() - ClockStart >= BudgetSeconds)))
        {
            bStop = true;
            Keep();
            continue;
        }
        UInstancedStaticMeshComponent* Ism = Cast<UInstancedStaticMeshComponent>(Comp);
        bool bDone = false;
        if (J.bAdd)
        {
            UStaticMesh* Mesh = Comp->GetStaticMesh();
            if (IsMeshRetryDeferred(Mesh))
            {
                continue;   // a failed mesh waits for its retry; a later sync queues it again
            }
            const bool bNeedsUpload = !FindCachedMesh(Mesh) || MeshRefCounts.FindRef(FObjectKey(Mesh)) <= 0;
            if (!J.bInRangeAtDiscovery && bNeedsUpload && Uploads > 0 && ((MaxUploads > 0 && Uploads >= MaxUploads) ||
                                                (UploadBudgetSeconds > 0.0 && UploadSeconds >= UploadBudgetSeconds)))
            {
                Keep();   // the mesh upload budget (rgl.Scene.MaxMeshCreatesPerUpdate / CreateBudgetMs) is spent
                continue;
            }
            const double U0 = FPlatformTime::Seconds();
            FVector Center = FVector::ZeroVector;
            float Radius = 0.0f;
            if (Ism)
            {
                FTransform T;
                if (J.Instance < 0 || J.Instance >= Ism->GetInstanceCount() ||
                    !Ism->GetInstanceTransform(J.Instance, T, /*bWorldSpace=*/true) || RGLWpCull::IsZeroScale(T))
                {
                    continue;
                }
                const FSphere S = WpInstanceSphere(Mesh, T);
                Center = S.Center;
                Radius = static_cast<float>(S.W);
                bDone = WpAddInstance(Ism, J.Instance, T);
            }
            else
            {
                Center = Comp->Bounds.Origin;
                Radius = static_cast<float>(Comp->Bounds.SphereRadius);
                bDone = WpAddRegular(Comp);
            }
            if (bNeedsUpload && !J.bInRangeAtDiscovery)
            {
                ++Uploads;
                UploadSeconds += FPlatformTime::Seconds() - U0;
            }
            if (!bDone)
            {
                if (EntityMap.Contains(Comp) && !Ism)
                {
                    continue;   // registered already
                }
                Keep();   // could not be made now (a failed upload is deferred): tried again
                continue;
            }
            // Contracts: made after its bounds already reached a sensor's range (a part culled at the previous sync), or
            // later than L_create (a part known at the previous sync).
            if (J.bLateCovered)
            {
                for (const auto& Pair : RegisteredSensors)
                {
                    const double R = SensorRangeCm(Pair.Value) + Radius;
                    if (FVector::DistSquared(Center, Pair.Value.Position) <= R * R)
                    {
                        WpViolation(TEXT("late_create"), CurrentSimTime - J.SinceSimTime, Latency);
                        break;
                    }
                }
            }
            if (J.bCovered)
            {
                if (!J.bAgeReported && CurrentSimTime - J.SinceSimTime > Latency + 1.0e-6)
                {
                    WpViolation(TEXT("queue_age"), CurrentSimTime - J.SinceSimTime, Latency);
                }
            }
            if (FWpCandidate* MutableC = WpCandidates.Find(J.Key))
            {
                if (Ism && MutableC->InstanceWantSince.IsValidIndex(J.Instance))
                {
                    MutableC->InstanceWantSince[J.Instance] = -1.0;
                }
                else if (!Ism)
                {
                    MutableC->WantSince = -1.0;
                }
            }
            ++WpEntityAdded;
            if (J.bInRangeAtDiscovery)
            {
                ++WpInRangeAddedLastUpdate;
                WpInRangeMsLastUpdate += (FPlatformTime::Seconds() - U0) * 1000.0;
            }
#if !UE_BUILD_SHIPPING
            ++WpAddedLastUpdate_ForTest;
#endif
        }
        else
        {
            if (Ism)
            {
                bDone = WpRemoveInstance(Ism, J.Instance);
            }
            else if (EntityMap.Contains(Comp))
            {
                UnregisterComponent(Comp);
                bDone = true;
            }
            if (!bDone)
            {
                continue;   // nothing registered any more
            }
            ++WpEntityRemoved;
#if !UE_BUILD_SHIPPING
            ++WpRemovedLastUpdate_ForTest;
#endif
        }
        if (!J.bInRangeAtDiscovery)
        {
            ++Changes;
            if (ClockStart < 0.0)
            {
                ClockStart = FPlatformTime::Seconds();
            }
        }
    }
    WpJobs.SetNum(Write);
    // The waits that already passed L_create (one line per job and sync; the sync line counts all).
    for (FWpJob& J : WpJobs)
    {
        if (J.bAdd && J.bCovered && !J.bAgeReported && CurrentSimTime - J.SinceSimTime > Latency + 1.0e-6)
        {
            J.bAgeReported = true;
            WpViolation(TEXT("queue_age"), CurrentSimTime - J.SinceSimTime, Latency);
        }
    }
    WpEntityMs += (FPlatformTime::Seconds() - StartSeconds) * 1000.0;
}

void FRGLSceneManager::WpUpdateMovedInstanceTransforms()
{
    for (int32 m = WpMovedIsm.Num() - 1; m >= 0; --m)
    {
        const FWpCandidate* C = WpCandidates.Find(WpMovedIsm[m]);
        UInstancedStaticMeshComponent* Ism = C ? Cast<UInstancedStaticMeshComponent>(C->Component.Get()) : nullptr;
        if (!Ism)
        {
            continue;
        }
        FISMCEntityGroup* Group = ISMCEntityMap.Find(Ism);
        if (!Group || Group->Component.Get() != Ism || !Ism->IsRegistered())
        {
            continue;
        }
        const int32 NumInstances = Ism->GetInstanceCount();
        for (int32 k = 0; k < Group->Entities.Num(); ++k)
        {
            const int32 Index = Group->InstanceIndices[k];
            FTransform T;
            if (Index < 0 || Index >= NumInstances || !Ism->GetInstanceTransform(Index, T, /*bWorldSpace=*/true) ||
                RGLWpCull::IsZeroScale(T) || !RGLWpCull::TransformChanged(T, Group->InstanceWorldTransforms[k]))
            {
                continue;
            }
            rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(T);
            RGL_CHECK(RGL_PROBED("rgl_entity_set_transform", rgl_entity_set_transform(Group->Entities[k], &RGLTransform)));
            Group->InstanceWorldTransforms[k] = T;
        }
    }
}

void FRGLSceneManager::CountSceneEntities(int32& OutTotal, int32& OutNonStatic) const
{
    OutTotal = EntityMap.Num();
    OutNonStatic = 0;
    for (const auto& Pair : EntityMap)
    {
        OutNonStatic += Pair.Value.bIsStatic ? 0 : 1;
    }
    for (const auto& Pair : ISMCEntityMap)
    {
        const int32 N = Pair.Value.Entities.Num();
        OutTotal += N;
        const UInstancedStaticMeshComponent* Comp = Pair.Value.Component.Get();
        if (Comp && Comp->Mobility != EComponentMobility::Static)
        {
            OutNonStatic += N;
        }
    }
    for (const auto& Pair : SkeletalEntityMap)
    {
        OutTotal += Pair.Value.Entity ? 1 : 0;
    }
}

// ============================================================================
// Ground plane: a large flat mesh at Z=0 to represent road surface
// ============================================================================

void FRGLSceneManager::CreateGroundPlane()
{
    // Create a 2km × 2km quad centered at the origin, at Z=0.
    // This covers the typical LiDAR range (100m) around any sensor position.
    // The actual ground height varies, but Z=0 is a reasonable approximation
    // for flat urban roads in CARLA.
    static constexpr float HALF_SIZE = 1000.0f; // 1000m = 1km half-extent

    rgl_vec3f Vertices[4] = {
        {{ -HALF_SIZE, -HALF_SIZE, 0.0f }},
        {{  HALF_SIZE, -HALF_SIZE, 0.0f }},
        {{  HALF_SIZE,  HALF_SIZE, 0.0f }},
        {{ -HALF_SIZE,  HALF_SIZE, 0.0f }}
    };

    // Two triangles forming the quad (both face upward)
    rgl_vec3i Indices[2] = {
        {{ 0, 1, 2 }},
        {{ 0, 2, 3 }}
    };

    rgl_status_t Status = rgl_mesh_create(&GroundMesh, Vertices, 4, Indices, 2);
    if (Status != RGL_SUCCESS || !GroundMesh)
    {
        RGLLog::Info("RGLSceneManager: Failed to create ground plane mesh");
        return;
    }

    Status = rgl_entity_create(&GroundEntity, Scene, GroundMesh);
    if (Status != RGL_SUCCESS || !GroundEntity)
    {
        RGLLog::Info("RGLSceneManager: Failed to create ground plane entity");
        rgl_mesh_destroy(GroundMesh);
        GroundMesh = nullptr;
        return;
    }

    // Place at origin with identity rotation (already in meters, Z=0).
    rgl_mat3x4f GroundTf = RGLCoord::Identity();
    RGL_CHECK(rgl_entity_set_transform(GroundEntity, &GroundTf));

    RGLLog::Info("RGLSceneManager: Ground plane created (2km x 2km)");
}

void FRGLSceneManager::UpdateGroundPlane(UWorld* World, const FTransform& SensorTransform)
{
    if (!GroundEntity || !World)
    {
        return;
    }

    // Trace a ray straight down from the sensor to find the actual ground level.
    const FVector SensorPos = SensorTransform.GetLocation();
    const FVector TraceStart = SensorPos;
    const FVector TraceEnd = SensorPos - FVector(0, 0, 10000.0f); // 100m downward

    FHitResult HitResult;
    FCollisionQueryParams TraceParams;
    TraceParams.bTraceComplex = false;

    float GroundZ;
    if (World->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd,
                                          ECC_WorldStatic, TraceParams))
    {
        GroundZ = static_cast<float>(HitResult.ImpactPoint.Z) * RGLCoord::UE_TO_RGL;
    }
    else
    {
        // Fallback: assume ground is 2m below sensor
        GroundZ = static_cast<float>(SensorPos.Z - 200.0f) * RGLCoord::UE_TO_RGL;
    }

    rgl_mat3x4f GroundTf = RGLCoord::Identity();
    GroundTf.value[0][3] = static_cast<float>(SensorPos.X) * RGLCoord::UE_TO_RGL;
    GroundTf.value[1][3] = static_cast<float>(SensorPos.Y) * RGLCoord::UE_TO_RGL;
    GroundTf.value[2][3] = GroundZ;

    RGL_CHECK(rgl_entity_set_transform(GroundEntity, &GroundTf));
}

// ============================================================================
// Skeletal mesh path
// ============================================================================

void FRGLSceneManager::RglDestroyChecked(rgl_status_t Status, const TCHAR* Api)
{
    if (Status != RGL_SUCCESS)
    {
        const char* Err = nullptr; rgl_get_last_error_string(&Err);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] %s failed (status %d): %s"), Api, (int)Status, Err ? *FString(UTF8_TO_TCHAR(Err)) : TEXT("?"));
    }
}

void FRGLSceneManager::UnregisterSensor(const void* SensorId)
{
    RegisteredSensors.Remove(SensorId);
    // Update() only runs from sensor ticks; with no sensor left nothing would restore tick policies or free entities.
    if (RegisteredSensors.Num() == 0 && SkeletalEntityMap.Num() > 0) TeardownSkeletal();
}

bool FRGLSceneManager::SkeletalEnabled() const
{
    bool bApi = bSkeletalApiAvailable;
#if !UE_BUILD_SHIPPING
    if (CVarRGLSkeletalDebugForceApiUnavailable.GetValueOnGameThread() != 0) bApi = false;
#endif
    return bApi && CVarRGLSkeletalEnable.GetValueOnGameThread() != 0;
}

void FRGLSceneManager::WarnSkeletalOnce(const USkeletalMeshComponent* Comp, const USkeletalMesh* Mesh, const FString& Msg)
{
    // (component, asset, reason) identity. FObjectKey carries the object's serial number, so an
    // address recycled by GC starts with a clean slate; the reason hash lets a *different* later
    // failure for the same pair (e.g. permanent skip after "not readable yet") still be logged once.
    const FSkeletalWarnKey Key = MakeTuple(FObjectKey(Comp), FObjectKey(Mesh), GetTypeHash(Msg));
    if (SkeletalWarned.Contains(Key)) return;
    SkeletalWarned.Add(Key);
    UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] %s (%s / %s)"), *Msg, Comp ? *Comp->GetPathName() : TEXT("?"), Mesh ? *Mesh->GetName() : TEXT("?"));
}

// Non-distance disqualifiers (return false with bOutOfRange=false) are applied without hysteresis.
bool FRGLSceneManager::ShouldRegisterSkeletalComponent(USkeletalMeshComponent* Comp, bool& bOutOfRange, bool& bRetryLater, FString& OutReason) const
{
    bOutOfRange = false; bRetryLater = false;
    if (!IsValid(Comp) || !Comp->IsRegistered() || !Comp->GetSkeletalMeshAsset() || !Comp->IsVisible() || Comp->bRenderStatic) return false;
    const FVector Scale = Comp->GetComponentTransform().GetScale3D();
    if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z)) return false;
    if (CVarRGLSkeletalScope.GetValueOnGameThread() == 0) { const AActor* O = Comp->GetOwner(); if (!O || !O->IsA<APawn>()) return false; }
    if (Comp->bNoSkeletonUpdate) return false;
    const FBoxSphereBounds B = Comp->Bounds;
    if (RegisteredSensors.Num() == 0 || !IsWithinAnySensor(B.Origin, B.SphereRadius, 0.0f)) { bOutOfRange = true; return false; }
    // Permanently skipped after a non-recoverable extraction/upload failure (spec §6.1);
    // hard disqualifier, so no hysteresis and no repeated extraction attempts.
    if (SkeletalSkipped.Contains(FSkeletalSkipKey(FObjectKey(Comp), FObjectKey(Comp->GetSkeletalMeshAsset())))) return false;
    if (RGLSkeletal::SelectReadableLOD(Comp->GetSkeletalMeshAsset(), OutReason, CVarRGLSkeletalMinLOD.GetValueOnGameThread()) < 0) { bRetryLater = true; return false; }
    return true;
}

rgl_mesh_t FRGLSceneManager::GetOrUploadSkeletalMesh(USkeletalMesh* Mesh, int32& OutLOD, int32& OutRawBoneNum, FString& OutReason)
{
    if (!Mesh) { OutReason = TEXT("null skeletal mesh"); return nullptr; }
    // Resolve the cache key WITHOUT extracting: SelectReadableLOD only inspects buffer
    // sizes, whereas ExtractSkeletalMesh walks every vertex. N pawns sharing one asset
    // must therefore cost one extraction, not N (spec §4.2).
    // MinLOD is part of the selection, and the cache key carries the LOD actually used, so a live
    // MinLOD change simply produces new cache entries when components are (re-)registered.
    const int32 MinLOD = CVarRGLSkeletalMinLOD.GetValueOnGameThread();
    const int32 LOD = RGLSkeletal::SelectReadableLOD(Mesh, OutReason, MinLOD);
    if (LOD < 0) return nullptr;   // §3.1 readiness not met: retry later, OutLOD stays < 0
    // From here on OutLOD >= 0, which tells the caller readiness passed and any later
    // failure is non-recoverable and must be skipped permanently (spec §6.1).
    OutLOD = LOD;
    const FSkeletalMeshKey Key{ Mesh, LOD };
    if (rgl_mesh_t* Cached = SkeletalMeshCache.Find(Key))
    {
        OutRawBoneNum = Mesh->GetRefSkeleton().GetRawBoneNum();
        return *Cached;
    }

    FRGLSkeletalMeshData Data;
    if (!RGLSkeletal::ExtractSkeletalMesh(Mesh, Data, OutReason, MinLOD)) return nullptr;
    // ExtractSkeletalMesh selects the same lowest readable LOD; a divergence would put
    // the entity on a mesh cached under the wrong key.
    if (Data.LODIndex != LOD) { OutReason = FString::Printf(TEXT("LOD mismatch: SelectReadableLOD=%d ExtractSkeletalMesh=%d"), LOD, Data.LODIndex); return nullptr; }
    OutRawBoneNum = Data.RawBoneNum;
    rgl_mesh_t M = nullptr;
    if (RGL_PROBED("rgl_mesh_create", rgl_mesh_create(&M, Data.Vertices.GetData(), Data.Vertices.Num(), Data.Indices.GetData(), Data.Indices.Num())) != RGL_SUCCESS || !M) { OutReason = TEXT("rgl_mesh_create failed"); return nullptr; }
    if (rgl_mesh_set_bone_weights(M, Data.Weights.GetData(), Data.Weights.Num()) != RGL_SUCCESS) { RglDestroyChecked(rgl_mesh_destroy(M), TEXT("rgl_mesh_destroy")); OutReason = TEXT("rgl_mesh_set_bone_weights failed"); return nullptr; }
    if (rgl_mesh_set_restposes(M, Data.RestposesInv.GetData(), Data.RestposesInv.Num()) != RGL_SUCCESS) { RglDestroyChecked(rgl_mesh_destroy(M), TEXT("rgl_mesh_destroy")); OutReason = TEXT("rgl_mesh_set_restposes failed"); return nullptr; }
    SkeletalMeshCache.Add(Key, M);
    UE_LOG(LogCarlaRGL, Log, TEXT("RGLSceneManager[skeletal] uploaded %s LOD=%d verts=%d tris=%d rawBones=%d"), *Mesh->GetName(), Data.LODIndex, Data.Vertices.Num(), Data.Indices.Num(), Data.RawBoneNum);
    // Shipping has NO_LOGGING, so the line above vanishes there. One stderr breadcrumb per process
    // is enough to tell an operator that the skeletal path is actually live in a packaged server.
    static bool bFirstSkeletalUploadLogged = false;
    if (!bFirstSkeletalUploadLogged)
    {
        bFirstSkeletalUploadLogged = true;
        fprintf(stderr, "CarlaRGL[skeletal]: first skeletal mesh uploaded (%s LOD=%d verts=%d bones=%d); skeletal path active\n",
                TCHAR_TO_UTF8(*Mesh->GetName()), Data.LODIndex, Data.Vertices.Num(), Data.RawBoneNum);
    }
    return M;
}

void FRGLSceneManager::ReleaseSkeletalMeshReference(const FSkeletalMeshKey& Key)
{
    int32* Count = SkeletalMeshRefCounts.Find(Key); if (!Count) return;
    if (--(*Count) <= 0)
    {
        if (rgl_mesh_t* M = SkeletalMeshCache.Find(Key)) { if (*M) RglDestroyChecked(RGL_PROBED("rgl_mesh_destroy", rgl_mesh_destroy(*M)), TEXT("rgl_mesh_destroy")); SkeletalMeshCache.Remove(Key); }
        SkeletalMeshRefCounts.Remove(Key);
    }
}

void FRGLSceneManager::ApplyTickPolicy(USkeletalMeshComponent* Comp, FSkeletalEntityInfo& Info, bool bEnable)
{
    if (!IsValid(Comp)) return;
    if (bEnable && !Info.bTickPolicyApplied)
    {
        Info.SavedTickOption = Comp->VisibilityBasedAnimTickOption; Info.bSavedURO = Comp->bEnableUpdateRateOptimizations;
        Comp->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones;
        Comp->bEnableUpdateRateOptimizations = false; Info.bTickPolicyApplied = true;
    }
    else if (!bEnable && Info.bTickPolicyApplied)
    {
        Comp->VisibilityBasedAnimTickOption = Info.SavedTickOption; Comp->bEnableUpdateRateOptimizations = Info.bSavedURO;
        Info.bTickPolicyApplied = false;
    }
}

bool FRGLSceneManager::RegisterSkeletalComponent(USkeletalMeshComponent* Comp)
{
    if (!IsValid(Comp) || SkeletalEntityMap.Contains(Comp)) return false;
    if (WantsSelfCutout(Comp)) return RegisterSkeletalCutoutComponent(Comp);   // own meshes, never the shared cache
    USkeletalMesh* Mesh = Comp->GetSkeletalMeshAsset();
    FString Reason; int32 LOD = -1, RawBoneNum = 0;
    rgl_mesh_t M = GetOrUploadSkeletalMesh(Mesh, LOD, RawBoneNum, Reason);
    if (!M)
    {
        // LOD >= 0 means readiness passed and extraction/upload itself failed: not a
        // §3.1 condition, so never retry this (component, asset) pair (spec §6.1).
        // Readiness failures cannot reach here (ShouldRegisterSkeletalComponent returns bRetryLater).
        if (LOD >= 0)
        {
            SkeletalSkipped.Add(FSkeletalSkipKey(FObjectKey(Comp), FObjectKey(Mesh)));
            // Shipping has no log: a permanent skip silently removes an actor from the point cloud,
            // so leave a stderr breadcrumb (LOD >= 0 implies Mesh != nullptr; Comp is IsValid here).
            fprintf(stderr, "CarlaRGL[skeletal]: permanently skipping %s (%s): %s\n",
                    TCHAR_TO_UTF8(*Comp->GetPathName()), TCHAR_TO_UTF8(*Mesh->GetName()), TCHAR_TO_UTF8(*Reason));
        }
        WarnSkeletalOnce(Comp, Mesh, TEXT("skipped: ") + Reason);
        return false;
    }
    FSkeletalEntityInfo Info;
    Info.Component = Comp; Info.SkeletalMesh = Mesh; Info.LODIndex = LOD; Info.RawBoneNum = RawBoneNum;
    Info.PoseScratch.SetNumUninitialized(RawBoneNum);
    ApplyTickPolicy(Comp, Info, CVarRGLSkeletalAlwaysTickPose.GetValueOnGameThread() != 0);
    SkeletalMeshRefCounts.FindOrAdd(FSkeletalMeshKey{ Mesh, LOD })++;
    SkeletalEntityMap.Add(Comp, MoveTemp(Info));   // Entity stays nullptr until the first valid pose (spec §4.3/§4.5)
    return true;
}

void FRGLSceneManager::UnregisterSkeletalComponent(USkeletalMeshComponent* Key)
{
    FSkeletalEntityInfo* Info = SkeletalEntityMap.Find(Key); if (!Info) return;
    if (Info->bCutout)
    {
        DestroyCutout(*Info);   // owns its mesh; nothing to release in the shared cache
        if (USkeletalMeshComponent* Live = Info->Component.Get()) ApplyTickPolicy(Live, *Info, false);   // never through the raw key
        SkeletalEntityMap.Remove(Key);
        return;
    }
    if (Info->Entity) { RglDestroyChecked(RGL_PROBED("rgl_entity_destroy", rgl_entity_destroy(Info->Entity)), TEXT("rgl_entity_destroy")); Info->Entity = nullptr; }
    ReleaseSkeletalMeshReference(FSkeletalMeshKey{ Info->SkeletalMesh, Info->LODIndex });
    if (USkeletalMeshComponent* Live = Info->Component.Get()) ApplyTickPolicy(Live, *Info, false);   // never through the raw key
    SkeletalEntityMap.Remove(Key);
}

void FRGLSceneManager::TeardownSkeletal()
{
    TArray<USkeletalMeshComponent*> Keys; SkeletalEntityMap.GetKeys(Keys);
    for (USkeletalMeshComponent* K : Keys) UnregisterSkeletalComponent(K);
    for (auto& P : SkeletalMeshCache) if (P.Value) RglDestroyChecked(RGL_PROBED("rgl_mesh_destroy", rgl_mesh_destroy(P.Value)), TEXT("rgl_mesh_destroy"));
    SkeletalMeshCache.Empty(); SkeletalMeshRefCounts.Empty();
    SkeletalSkipped.Empty(); SkeletalWarned.Empty();
}

void FRGLSceneManager::UpdateSkeletalPoses()
{
    // Self cutout wanted or no longer wanted (a cutout sensor attached, destroyed or evicted): move the
    // component to the other path before posing, so it is back in the scene this tick. Skipped
    // entirely while no sensor uses the self cutout and no component is on that path.
    bool bCutoutInPlay = AnyCutoutSensor();
    for (auto It = SkeletalEntityMap.CreateConstIterator(); It && !bCutoutInPlay; ++It) bCutoutInPlay = It.Value().bCutout;
    if (bCutoutInPlay)
    {
        TArray<USkeletalMeshComponent*> ToSwitch;
        for (auto& Pair : SkeletalEntityMap)
        {
            USkeletalMeshComponent* Live = Pair.Value.Component.Get();
            if (Live && IsValid(Live) && Live->IsRegistered() && WantsSelfCutout(Live) != Pair.Value.bCutout) ToSwitch.Add(Pair.Key);
        }
        for (USkeletalMeshComponent* K : ToSwitch) { UnregisterSkeletalComponent(K); RegisterSkeletalComponent(K); }
    }

    TArray<USkeletalMeshComponent*> ToRemove;
    for (auto& Pair : SkeletalEntityMap)
    {
        FSkeletalEntityInfo& Info = Pair.Value;
        USkeletalMeshComponent* Comp = Info.Component.Get();
        if (!Comp || !IsValid(Comp) || !Comp->IsRegistered()) { ToRemove.Add(Pair.Key); continue; }
        if (Comp->GetSkeletalMeshAsset() != Info.SkeletalMesh) { ToRemove.Add(Pair.Key); continue; }   // swapped: re-register at next sync
        bool bPose = RGLSkeletal::BuildWorldPose(Comp, Info.RawBoneNum, Info.PoseScratch);
#if !UE_BUILD_SHIPPING
        if (CVarRGLSkeletalDebugForcePoseFailure.GetValueOnGameThread() != 0) bPose = false;
#endif
        if (!bPose)
        {
            if (Info.Entity) { RglDestroyChecked(rgl_entity_destroy(Info.Entity), TEXT("rgl_entity_destroy")); Info.Entity = nullptr; }
            WarnSkeletalOnce(Comp, Info.SkeletalMesh, TEXT("no complete pose this tick; pending"));
            continue;
        }
        if (Info.bCutout)
        {
            UpdateCutoutEntity(Comp, Info);   // on failure the entity stays pending and is retried next tick
            continue;
        }
        if (!Info.Entity)
        {
            rgl_mesh_t* M = SkeletalMeshCache.Find(FSkeletalMeshKey{ Info.SkeletalMesh, Info.LODIndex });
            if (!M || !*M) { ToRemove.Add(Pair.Key); continue; }
            rgl_entity_t E = nullptr;
            if (rgl_entity_create(&E, Scene, *M) != RGL_SUCCESS || !E) { WarnSkeletalOnce(Comp, Info.SkeletalMesh, TEXT("rgl_entity_create failed; pending")); continue; }
            Info.Entity = E;   // identity transform; world-space poses follow. Never call rgl_entity_set_transform.
        }
        if (rgl_entity_set_pose_world(Info.Entity, Info.PoseScratch.GetData(), Info.RawBoneNum) != RGL_SUCCESS)
        {
            RglDestroyChecked(rgl_entity_destroy(Info.Entity), TEXT("rgl_entity_destroy")); Info.Entity = nullptr;
            WarnSkeletalOnce(Comp, Info.SkeletalMesh, TEXT("rgl_entity_set_pose_world failed; pending"));
        }
    }
    for (USkeletalMeshComponent* K : ToRemove) UnregisterSkeletalComponent(K);
}

void FRGLSceneManager::SyncSkeletalComponents(UWorld* World)
{
    TArray<USkeletalMeshComponent*> Candidates;
    TArray<USkeletalMeshComponent*> Comps;
    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It; if (!IsValid(Actor)) continue;
        Comps.Reset(); Actor->GetComponents<USkeletalMeshComponent>(Comps);
        Candidates.Append(Comps);
    }
    SyncSkeletalCandidates(Candidates);
}

void FRGLSceneManager::SyncSkeletalCandidates(const TArray<USkeletalMeshComponent*>& Candidates)
{
    const bool bTickPolicy = CVarRGLSkeletalAlwaysTickPose.GetValueOnGameThread() != 0;
    const int32 MaxEntities = FMath::Max(0, CVarRGLSkeletalMaxEntities.GetValueOnGameThread());
    for (auto& Pair : SkeletalEntityMap) if (USkeletalMeshComponent* Live = Pair.Value.Component.Get()) ApplyTickPolicy(Live, Pair.Value, bTickPolicy);

    // 1) Evaluate every skeletal component: eligible set (with distance), hard-disqualified set, retry set.
    struct FCand { USkeletalMeshComponent* Comp; float DistSq; };
    TArray<FCand> Eligible; TSet<USkeletalMeshComponent*> Disqualified; int32 RangeSkipped = 0, RetryLater = 0;
    for (USkeletalMeshComponent* C : Candidates)
    {
        bool bOut = false, bRetry = false; FString Reason;
        if (!ShouldRegisterSkeletalComponent(C, bOut, bRetry, Reason))
        {
            if (bRetry) { ++RetryLater; WarnSkeletalOnce(C, C->GetSkeletalMeshAsset(), TEXT("not readable yet, will retry: ") + Reason); }
            else if (bOut) ++RangeSkipped;
            else Disqualified.Add(C);          // scope/visibility/scale/bNoSkeletonUpdate/bRenderStatic: no hysteresis
            continue;
        }
        float Best = TNumericLimits<float>::Max();
        for (const auto& S : RegisteredSensors) Best = FMath::Min(Best, (float)FVector::DistSquared(S.Value.Position, C->Bounds.Origin));
        Eligible.Add({ C, Best });
    }
    // 2) Unregister: dead, hard-disqualified, or out of range beyond hysteresis. Registrations that
    //    survive this step but are NOT eligible ("retained" by hysteresis) still occupy VRAM, so
    //    they are ranked alongside the eligible candidates in step 3 (spec §4.2-5).
    TSet<USkeletalMeshComponent*> EligibleSet; for (const FCand& C : Eligible) EligibleSet.Add(C.Comp);
    TArray<USkeletalMeshComponent*> ToRemove; TArray<FCand> Retained;
    for (auto& Pair : SkeletalEntityMap)
    {
        USkeletalMeshComponent* Live = Pair.Value.Component.Get();
        if (!Live || !IsValid(Live) || Disqualified.Contains(Pair.Key)) { ToRemove.Add(Pair.Key); continue; }
        if (EligibleSet.Contains(Pair.Key)) continue;   // already ranked as an eligible candidate
        const FBoxSphereBounds B = Live->Bounds;
        if (!IsWithinAnySensor(B.Origin, B.SphereRadius, UnregistrationHysteresisCm)) { ToRemove.Add(Pair.Key); continue; }
        float Best = TNumericLimits<float>::Max();
        for (const auto& S : RegisteredSensors) Best = FMath::Min(Best, (float)FVector::DistSquared(S.Value.Position, B.Origin));
        Retained.Add({ Pair.Key, Best });
    }
    for (USkeletalMeshComponent* K : ToRemove) UnregisterSkeletalComponent(K);
    // 3) Budget over the UNION of eligible candidates and hysteresis-retained registrations,
    //    nearest first. Everything past MaxEntities is unregistered FIRST (so MaxEntities=0 really
    //    empties the set), then the in-budget newcomers are registered.
    TArray<FCand> Ranked = MoveTemp(Eligible);
    Ranked.Append(Retained);
    Ranked.Sort([](const FCand& A, const FCand& B) { return A.DistSq < B.DistSq; });
    TArray<USkeletalMeshComponent*> OverBudget;
    for (int32 i = MaxEntities; i < Ranked.Num(); ++i)
        if (SkeletalEntityMap.Contains(Ranked[i].Comp)) OverBudget.Add(Ranked[i].Comp);
    for (USkeletalMeshComponent* K : OverBudget) UnregisterSkeletalComponent(K);
    int32 Added = 0;
    for (int32 i = 0, N = FMath::Min(MaxEntities, Ranked.Num()); i < N; ++i)
    {
        USkeletalMeshComponent* C = Ranked[i].Comp;
        if (!SkeletalEntityMap.Contains(C) && RegisterSkeletalComponent(C)) ++Added;
    }
    if (Added || ToRemove.Num() || OverBudget.Num())
        RGLLog::Info("RGLSceneManager[skeletal]: registered=", SkeletalEntityMap.Num(), " cachedMeshes=", SkeletalMeshCache.Num(),
                     " added=", Added, " removed=", ToRemove.Num() + OverBudget.Num(), " range-skipped=", RangeSkipped, " retry-later=", RetryLater);
}

// ============================================================================
// Self cutout (rgl_self_cutout_radius)
// ============================================================================
// A vehicle carrying a sensor with a cutout radius is registered on its own path: its skeletal mesh is
// extracted per component (no shared cache) and only the triangles that are not near any of its cutout
// sensors (RGLSelfCutout::ClassifyTriangles) go into the RGL scene. The near triangles are a hole for
// every sensor in the scene, other vehicles' included; the RGL raytrace graphs are not touched. The kept
// mesh is rebuilt whenever the set of cutout sensors or their settings change.

void FRGLSceneManager::SetSensorSelfCutout(const void* SensorId, AActor* Vehicle, float RadiusCm, const FTransform& SensorWorldTransform,
                                           float MinElevationDeg, float MaxElevationDeg)
{
    FSensorRegistration* S = RegisteredSensors.Find(SensorId);
    if (!S) return;   // RegisterSensor() first
    const bool bVehicle = IsValid(Vehicle);
    S->CutoutVehicle = bVehicle ? Vehicle : nullptr;
    S->CutoutRadiusCm = (bVehicle && RadiusCm > 0.0f) ? RadiusCm : 0.0f;
    S->CutoutMinElevationDeg = FMath::Min(MinElevationDeg, MaxElevationDeg);
    S->CutoutMaxElevationDeg = FMath::Max(MinElevationDeg, MaxElevationDeg);
    if (bVehicle)
    {
        // Sensor and vehicle transforms of the same tick: the relative pose stays valid in later ticks,
        // when this sensor's own world transform may lag behind the vehicle (sensors tick in turn).
        S->CutoutSensorToVehicle = SensorWorldTransform.GetRelativeTransform(Vehicle->GetActorTransform());
    }
}

bool FRGLSceneManager::AnyCutoutSensor() const
{
    for (const auto& Pair : RegisteredSensors) if (Pair.Value.CutoutRadiusCm > 0.0f) return true;
    return false;
}

bool FRGLSceneManager::WantsSelfCutout(const USkeletalMeshComponent* Comp) const
{
    const AActor* Owner = Comp ? Comp->GetOwner() : nullptr;
    if (!Owner) return false;
    for (const auto& Pair : RegisteredSensors)
        if (Pair.Value.CutoutRadiusCm > 0.0f && Pair.Value.CutoutVehicle.Get() == Owner) return true;
    return false;
}

void FRGLSceneManager::GatherCutoutConfig(const AActor* Owner, TArray<RGLSelfCutout::FSensorConfig>& Out) const
{
    Out.Reset();
    if (!Owner) return;
    for (const auto& Pair : RegisteredSensors)
    {
        const FSensorRegistration& S = Pair.Value;
        if (!(S.CutoutRadiusCm > 0.0f) || S.CutoutVehicle.Get() != Owner) continue;
        RGLSelfCutout::FSensorConfig& C = Out.AddDefaulted_GetRef();
        C.Key = reinterpret_cast<UPTRINT>(Pair.Key);
        C.RadiusCm = S.CutoutRadiusCm;
        C.MinElevationDeg = S.CutoutMinElevationDeg;
        C.MaxElevationDeg = S.CutoutMaxElevationDeg;
        C.SensorToOwner = S.CutoutSensorToVehicle;
    }
    RGLSelfCutout::SortConfig(Out);
}

bool FRGLSceneManager::RegisterSkeletalCutoutComponent(USkeletalMeshComponent* Comp)
{
    USkeletalMesh* Mesh = Comp->GetSkeletalMeshAsset();
    FString Reason;
    const int32 MinLOD = CVarRGLSkeletalMinLOD.GetValueOnGameThread();
    const int32 LOD = RGLSkeletal::SelectReadableLOD(Mesh, Reason, MinLOD);
    if (LOD < 0)
    {
        WarnSkeletalOnce(Comp, Mesh, TEXT("skipped: ") + Reason);   // readiness (spec §3.1): retried at a later sync
        return false;
    }
    TSharedPtr<FRGLSkeletalMeshData> Data = MakeShared<FRGLSkeletalMeshData>();
    bool bOk = RGLSkeletal::ExtractSkeletalMesh(Mesh, *Data, Reason, MinLOD);
    if (bOk && Data->LODIndex != LOD) { bOk = false; Reason = FString::Printf(TEXT("LOD mismatch: SelectReadableLOD=%d ExtractSkeletalMesh=%d"), LOD, Data->LODIndex); }
    if (bOk && Data->Indices.Num() == 0) { bOk = false; Reason = TEXT("no triangles"); }
    if (!bOk)
    {
        // Readiness passed, so this is not recoverable (spec §6.1): same permanent skip as the shared path.
        SkeletalSkipped.Add(FSkeletalSkipKey(FObjectKey(Comp), FObjectKey(Mesh)));
        fprintf(stderr, "CarlaRGL[skeletal]: permanently skipping %s (%s): %s\n",
                TCHAR_TO_UTF8(*Comp->GetPathName()), TCHAR_TO_UTF8(*Mesh->GetName()), TCHAR_TO_UTF8(*Reason));
        WarnSkeletalOnce(Comp, Mesh, TEXT("skipped: ") + Reason);
        return false;
    }
    FSkeletalEntityInfo Info;
    Info.Component = Comp; Info.SkeletalMesh = Mesh; Info.LODIndex = LOD; Info.RawBoneNum = Data->RawBoneNum;
    Info.PoseScratch.SetNumUninitialized(Data->RawBoneNum);
    Info.bCutout = true;
    Info.CutoutSource = Data;
    ApplyTickPolicy(Comp, Info, CVarRGLSkeletalAlwaysTickPose.GetValueOnGameThread() != 0);
    UE_LOG(LogCarlaRGL, Log, TEXT("RGLSceneManager[cutout] registered %s (%s LOD=%d verts=%d tris=%d)"),
           *Comp->GetPathName(), *Mesh->GetName(), LOD, Data->Vertices.Num(), Data->Indices.Num());
    SkeletalEntityMap.Add(Comp, MoveTemp(Info));   // the kept mesh is built from the first valid pose
    return true;
}

void FRGLSceneManager::DestroyCutout(FSkeletalEntityInfo& Info)
{
    if (Info.Entity) { RglDestroyChecked(RGL_PROBED("rgl_entity_destroy", rgl_entity_destroy(Info.Entity)), TEXT("rgl_entity_destroy")); Info.Entity = nullptr; }   // entity before its mesh
    if (Info.CutoutMesh) { RglDestroyChecked(RGL_PROBED("rgl_mesh_destroy", rgl_mesh_destroy(Info.CutoutMesh)), TEXT("rgl_mesh_destroy")); Info.CutoutMesh = nullptr; }
    Info.CutoutConfig.Reset();
    Info.CutoutKeptTriangles = 0;
    Info.CutoutRemovedTriangles = 0;
    Info.bCutoutBuilt = false;
}

bool FRGLSceneManager::UpdateCutoutEntity(USkeletalMeshComponent* Comp, FSkeletalEntityInfo& Info)
{
    TArray<RGLSelfCutout::FSensorConfig> Config;
    GatherCutoutConfig(Comp->GetOwner(), Config);
    if (!Info.bCutoutBuilt || !RGLSelfCutout::SameConfig(Config, Info.CutoutConfig))
    {
        DestroyCutout(Info);
        if (!BuildCutoutMesh(Comp, Info, Config))
        {
            DestroyCutout(Info);
            return false;
        }
    }
    if (!Info.CutoutMesh) return true;   // every triangle is near a cutout sensor: nothing to register
    // From here on as on the shared path: entity on the first valid pose, re-posed every tick, dropped
    // (mesh kept) when the pose is lost.
    if (!Info.Entity)
    {
        rgl_entity_t E = nullptr;
        if (rgl_entity_create(&E, Scene, Info.CutoutMesh) != RGL_SUCCESS || !E)
        {
            WarnSkeletalOnce(Comp, Info.SkeletalMesh, TEXT("self cutout: rgl_entity_create failed; pending"));
            return false;
        }
        Info.Entity = E;   // identity transform; world-space poses follow. Never call rgl_entity_set_transform.
    }
    if (rgl_entity_set_pose_world(Info.Entity, Info.PoseScratch.GetData(), Info.RawBoneNum) != RGL_SUCCESS)
    {
        RglDestroyChecked(rgl_entity_destroy(Info.Entity), TEXT("rgl_entity_destroy")); Info.Entity = nullptr;
        WarnSkeletalOnce(Comp, Info.SkeletalMesh, TEXT("self cutout: rgl_entity_set_pose_world failed; pending"));
        return false;
    }
    return true;
}

bool FRGLSceneManager::BuildCutoutMesh(USkeletalMeshComponent* Comp, FSkeletalEntityInfo& Info, const TArray<RGLSelfCutout::FSensorConfig>& Config)
{
    const double StartSeconds = FPlatformTime::Seconds();
    const FRGLSkeletalMeshData* Src = Info.CutoutSource.Get();
    const AActor* Owner = Comp->GetOwner();
    if (!Src || !Owner) return false;

    // World positions under this tick's pose; each sensor placed from this tick's vehicle pose and its
    // fixed pose relative to the vehicle.
    TArray<FVector> WorldM;
    if (!RGLSelfCutout::SkinVertices(*Src, Info.PoseScratch, WorldM))
    {
        WarnSkeletalOnce(Comp, Info.SkeletalMesh, TEXT("self cutout: pose does not match the extracted mesh; pending"));
        return false;
    }
    const FTransform OwnerWorld = Owner->GetActorTransform();
    TArray<RGLSelfCutout::FSensor> Sensors;
    for (const RGLSelfCutout::FSensorConfig& C : Config)
    {
        const FTransform SensorWorld = C.SensorToOwner * OwnerWorld;
        RGLSelfCutout::FSensor& S = Sensors.AddDefaulted_GetRef();
        S.OriginM = SensorWorld.GetLocation() * RGLCoord::UE_TO_RGL;
        S.Rotation = SensorWorld.GetRotation();
        S.RadiusM = C.RadiusCm * RGLCoord::UE_TO_RGL;
        S.MinElevationDeg = C.MinElevationDeg - RGLSelfCutout::ElevationMarginDeg;
        S.MaxElevationDeg = C.MaxElevationDeg + RGLSelfCutout::ElevationMarginDeg;
    }
    TArray<int32> Owners;
    RGLSelfCutout::ClassifyTriangles(WorldM, Src->Indices, Sensors, Owners);

    // Removed triangles per sensor; a triangle near several sensors is counted once, under "shared".
    TArray<int32> NearCount;
    NearCount.Init(0, Config.Num());
    int32 SharedCount = 0;
    for (const int32 O : Owners)
    {
        if (O == RGLSelfCutout::OwnerShared) ++SharedCount;
        else if (NearCount.IsValidIndex(O)) ++NearCount[O];
    }
    int32 RemovedTotal = SharedCount;
    for (const int32 N : NearCount) RemovedTotal += N;

    // The kept mesh: the triangles near no cutout sensor, compacted to the vertices they use. It keeps
    // every restpose, so the component's full world pose drives it as on the shared path.
    TArray<rgl_vec3f> KeptVertices; TArray<rgl_vec3i> KeptIndices; TArray<rgl_bone_weights_t> KeptWeights;
    const int32 Kept = RGLSelfCutout::ExtractPart(*Src, Owners, RGLSelfCutout::OwnerRest, KeptVertices, KeptIndices, KeptWeights);
    rgl_mesh_t M = nullptr;
    if (Kept > 0)
    {
        bool bOk = RGL_PROBED("rgl_mesh_create", rgl_mesh_create(&M, KeptVertices.GetData(), KeptVertices.Num(), KeptIndices.GetData(), KeptIndices.Num())) == RGL_SUCCESS && M;
        if (!bOk) M = nullptr;
        bOk = bOk && rgl_mesh_set_bone_weights(M, KeptWeights.GetData(), KeptWeights.Num()) == RGL_SUCCESS;
        bOk = bOk && rgl_mesh_set_restposes(M, Src->RestposesInv.GetData(), Src->RestposesInv.Num()) == RGL_SUCCESS;
        if (!bOk)
        {
            const char* Err = nullptr; rgl_get_last_error_string(&Err);
            if (M) RglDestroyChecked(rgl_mesh_destroy(M), TEXT("rgl_mesh_destroy"));
            WarnSkeletalOnce(Comp, Info.SkeletalMesh, FString::Printf(TEXT("self cutout: RGL mesh setup failed (%s); pending"),
                                                                     Err ? *FString(UTF8_TO_TCHAR(Err)) : TEXT("?")));
            return false;
        }
    }

    Info.CutoutMesh = M;
    Info.CutoutKeptTriangles = Kept;
    Info.CutoutRemovedTriangles = RemovedTotal;
    Info.CutoutConfig = Config;
    Info.bCutoutBuilt = true;
    ++CutoutBuildCount;

    // Sensor keys are the session handles; the backend's rgl_self_cutout_sensor line prints the same key.
    FString RemovedStr;
    for (int32 i = 0; i < Config.Num(); ++i)
    {
        RemovedStr += FString::Printf(TEXT("%s0x%llx:%d"), i ? TEXT(",") : TEXT(""), static_cast<unsigned long long>(Config[i].Key), NearCount[i]);
    }
    RemovedStr += FString::Printf(TEXT("%sshared:%d"), RemovedStr.IsEmpty() ? TEXT("") : TEXT(","), SharedCount);
    EmitSceneLine(FString::Printf(
        TEXT("rgl_self_cutout t=%.3f owner=%s comp=%s sensors=%d tris=%d kept=%d removed=%s removed_total=%d elapsed_ms=%.2f"),
        CurrentSimTime, *GetNameSafe(Owner), *Comp->GetName(), Config.Num(), Owners.Num(), Kept,
        *RemovedStr, RemovedTotal, (FPlatformTime::Seconds() - StartSeconds) * 1000.0));
    return true;
}

// ============================================================================
// Scene audit (read-only): the RGL static scene against the world UE has loaded
// ============================================================================
// Ghost = in RGL but not in the loaded world; missing = in the loaded world, eligible by the registration
// predicate, but not in RGL. "Loaded world" = registered components of visible levels (bIsVisible &&
// !bIsBeingRemoved). A level being added (components registered over several frames before bIsVisible) or
// removed (components unregistered over several frames while bIsVisible stays true) is in transition and its
// components count neither way. Nothing here changes the scene: no registration, no cache maintenance (no idle purge),
// no queue edits.

// Helpers are always called qualified (RGLSceneAudit::...): no using-directive, so neither UE's global FSphere
// nor same-named statics of other files in a unity build can make a name ambiguous.
namespace RGLSceneAudit
{
    struct FAuditSphere
    {
        FVector Center = FVector::ZeroVector;
        double  RadiusCm = 0.0;
    };

    /// The bounding sphere (center, radius) intersects one of the spheres (same test as IsWithinAnySensor).
    static bool InAnySphere(const TArray<FAuditSphere>& Spheres, const FVector& CenterCm, double BoundsRadiusCm)
    {
        for (const FAuditSphere& S : Spheres)
        {
            if (FVector::DistSquared(CenterCm, S.Center) <= FMath::Square(S.RadiusCm + BoundsRadiusCm))
            {
                return true;
            }
        }
        return false;
    }

    enum class ELevelState : uint8 { Visible, Hidden, InTransition };

    static ELevelState GetLevelState(const UWorld* World, const ULevel* Level)
    {
        if (!Level)
        {
            return ELevelState::Visible;   // nothing to judge by: the component's own registration decides
        }
        if (Level->bIsBeingRemoved || Level->bIsAssociatingLevel || Level->bIsDisassociatingLevel)
        {
            return ELevelState::InTransition;
        }
        if (World && (World->GetCurrentLevelPendingVisibility() == Level || World->GetCurrentLevelPendingInvisibility() == Level))
        {
            return ELevelState::InTransition;
        }
        return Level->bIsVisible ? ELevelState::Visible : ELevelState::Hidden;
    }

    static double MaxAbsScale(const FTransform& T)
    {
        const FVector S = T.GetScale3D();
        return FMath::Max3(FMath::Abs(S.X), FMath::Abs(S.Y), FMath::Abs(S.Z));
    }

    static bool IsZeroScale(const FTransform& T)
    {
        const FVector S = T.GetScale3D();
        return FMath::IsNearlyZero(S.X) || FMath::IsNearlyZero(S.Y) || FMath::IsNearlyZero(S.Z);
    }

    /// Instance moved, turned or rescaled since registration (1 cm / ~0.01 deg / 0.1 % of scale).
    static bool InstanceChanged(const FTransform& Now, const FTransform& Registered)
    {
        return !Now.GetLocation().Equals(Registered.GetLocation(), 1.0) ||
               !Now.GetRotation().Equals(Registered.GetRotation(), 1.0e-4) ||
               !Now.GetScale3D().Equals(Registered.GetScale3D(), 1.0e-3);
    }

    /// Bounding-sphere radius (cm, unscaled) of the asset behind MeshKey; 0 when it is gone.
    static double MeshRadiusCm(const FObjectKey& MeshKey)
    {
        const UStaticMesh* Mesh = Cast<UStaticMesh>(MeshKey.ResolveObjectPtr());
        return Mesh ? Mesh->GetBounds().SphereRadius : 0.0;
    }

    static bool IsHlodOwned(const UActorComponent* Component)
    {
        const AActor* Owner = Component ? Component->GetOwner() : nullptr;
        return Owner && Owner->IsA<AWorldPartitionHLOD>();
    }

    /// Landscape collision/render components, by class name: CarlaRGL does not link the Landscape module.
    static bool IsLandscapeClass(const UClass* Class)
    {
        static const FName HeightfieldCollision(TEXT("LandscapeHeightfieldCollisionComponent"));
        static const FName LandscapeComponent(TEXT("LandscapeComponent"));
        for (const UClass* C = Class; C; C = C->GetSuperClass())
        {
            if (C->GetFName() == HeightfieldCollision || C->GetFName() == LandscapeComponent)
            {
                return true;
            }
        }
        return false;
    }
}

void FRGLSceneManager::SetSensorAuditRange(const void* SensorId, float RangeCm)
{
    if (FSensorRegistration* S = RegisteredSensors.Find(SensorId))   // RegisterSensor() first
    {
        S->AuditRangeCm = RangeCm;
    }
}

void FRGLSceneManager::SetSensorAuditParent(const void* SensorId, AActor* RootParent)
{
    if (FSensorRegistration* S = RegisteredSensors.Find(SensorId))   // RegisterSensor() first
    {
        S->AuditParent = RootParent;
    }
}

float FRGLSceneManager::SensorRangeCm(const FSensorRegistration& S) const
{
    return (S.AuditRangeCm >= 0.0f) ? S.AuditRangeCm : FMath::Max(0.0f, S.RegistrationDistanceCm - UnregistrationHysteresisCm);
}

float FRGLSceneManager::SyncTriggerCm(const FSensorRegistration& S, const FSceneKnobs& K) const
{
    const float R = S.RegistrationDistanceCm;
    float TriggerCm = K.SyncTriggerRatio * R;
    if (K.SyncTriggerMarginSlackM >= 0.0f)
    {
        // A part outside R at the last sync is at least (R - range) from the range edge; stay slack metres below that.
        const float CapCm = FMath::Max(100.0f, (R - SensorRangeCm(S)) - K.SyncTriggerMarginSlackM * 100.0f);
        TriggerCm = FMath::Min(TriggerCm, CapCm);
    }
    return TriggerCm;
}

float FRGLSceneManager::StreamSourceRadiusCm(const AActor* RootParent)
{
    // The streaming source CARLA puts on spawned vehicles/walkers (CarlaEpisode.cpp) sits on the root actor.
    float SourceRadiusCm = -1.0f;
    if (RootParent)
    {
        if (const UWorldPartitionStreamingSourceComponent* Source =
                RootParent->FindComponentByClass<UWorldPartitionStreamingSourceComponent>())
        {
            for (const FStreamingSourceShape& Shape : Source->Shapes)
            {
                if (!Shape.bUseGridLoadingRange)   // a grid-bound shape has no radius of its own
                {
                    SourceRadiusCm = FMath::Max(SourceRadiusCm, Shape.Radius);
                }
            }
        }
    }
    return SourceRadiusCm;
}

// ============================================================================
// Stream coverage warning (P4): the scene follows the loaded world, so a range that World Partition does not stream
// in is not in it. Not a defect to fix here; the line tells the user how to get the range loaded.
// ============================================================================

FString FRGLSceneManager::FormatStreamWarningLine(double SimTime, const void* SensorId, float RangeM, const FString& ParentName,
                                                  float ParentStreamRadiusM)
{
    return FString::Printf(
        TEXT("rgl_scene_stream_warning t=%.3f sensor=0x%llx range_m=%.0f parent=%s parent_stream_radius_m=%.0f "
             "hint=attach_the_sensor_to_a_hero_vehicle_or_raise_tile_stream_distance"),
        SimTime, static_cast<unsigned long long>(reinterpret_cast<UPTRINT>(SensorId)), static_cast<double>(RangeM),
        ParentName.IsEmpty() ? TEXT("none") : *ParentName, static_cast<double>(ParentStreamRadiusM));
}

void FRGLSceneManager::CheckStreamCoverage(UWorld* World)
{
    UWorldPartition* WorldPartition = World ? World->GetWorldPartition() : nullptr;
    if (!WorldPartition)
    {
        return;   // not a World Partition map: the whole map is loaded
    }
    const UWorldPartitionSubsystem* WPSubsystem = World->GetSubsystem<UWorldPartitionSubsystem>();
    if (!WPSubsystem || !WorldPartition->IsInitialized())
    {
        return;
    }
    TArray<FWorldPartitionStreamingQuerySource> Query;
    FWorldPartitionStreamingQuerySource& Q = Query.AddDefaulted_GetRef();   // one sensor per query
    Q.bUseGridLoadingRange = false;
    Q.bSpatialQuery = true;
    for (auto& Pair : RegisteredSensors)
    {
        FSensorRegistration& S = Pair.Value;
        if (S.bStreamWarned)
        {
            continue;   // once per registration: no more queries for it
        }
        if (ParentRingHoldsRange(S))
        {
            S.UncoveredSinceSimTime = -1.0;   // as StepStreamCoverage does for a held range, without the query
            continue;
        }
        Q.Location = S.Position;
        Q.Radius = SensorRangeCm(S);
        const bool bStreamedIn =
            WPSubsystem->IsStreamingCompleted(EWorldPartitionRuntimeCellState::Activated, Query, /*bExactState=*/false);
        StepStreamCoverage(Pair.Key, S, bStreamedIn);
    }
}

void FRGLSceneManager::StepStreamCoverage(const void* SensorId, FSensorRegistration& S, bool bStreamedIn)
{
    if (S.bStreamWarned)
    {
        return;
    }
    // Streamed in: the parent is not looked at. The parent's own ring holds the whole range sphere: the sphere's cells
    // load with the ring, so a not-streamed-in answer is streaming in progress (start-up, teleport), not something the
    // hint can fix.
    if (bStreamedIn || ParentRingHoldsRange(S))
    {
        S.UncoveredSinceSimTime = -1.0;
        return;
    }
    if (S.UncoveredSinceSimTime < 0.0 || CurrentSimTime < S.UncoveredSinceSimTime)   // a clock that went back starts again
    {
        S.UncoveredSinceSimTime = CurrentSimTime;
    }
    if (CurrentSimTime - S.UncoveredSinceSimTime < StreamWarningSeconds)
    {
        return;
    }
    S.bStreamWarned = true;
    const float RangeCm = SensorRangeCm(S);
    const AActor* Parent = S.AuditParent.Get();
    const float ParentRadiusCm = StreamSourceRadiusCm(Parent);
    EmitSceneLine(FormatStreamWarningLine(CurrentSimTime, SensorId, RangeCm * RGLCoord::UE_TO_RGL,
                                          Parent ? Parent->GetName() : FString(),
                                          (ParentRadiusCm >= 0.0f) ? ParentRadiusCm * RGLCoord::UE_TO_RGL : -1.0f));
#if !UE_BUILD_SHIPPING
    ++StreamWarnings_ForTest;
#endif
}

bool FRGLSceneManager::ParentRingHoldsRange(const FSensorRegistration& S) const
{
    const AActor* Parent = S.AuditParent.Get();
    const float ParentRadiusCm = StreamSourceRadiusCm(Parent);
    return Parent && ParentRadiusCm >= 0.0f &&
           FVector::Dist(S.Position, Parent->GetActorLocation()) + SensorRangeCm(S) <= ParentRadiusCm;
}

FRGLSceneAuditResult FRGLSceneManager::AuditScene(UWorld* World, FRGLSceneAuditExtras* OutExtras) const
{
    namespace RA = RGLSceneAudit;

    const double StartSeconds = FPlatformTime::Seconds();
    FRGLSceneAuditResult R;
    if (!World)
    {
        return R;
    }
    R.Pending = PendingCreates.Num();
    R.PendingIsm = PendingIsmReevals.Num();
    R.WpPending = WpJobs.Num();
    R.LevelsAdded = LevelsAddedSinceAudit;
    R.SecondsSinceSync = (LastSyncSimTime >= 0.0) ? FMath::Max(0.0, CurrentSimTime - LastSyncSimTime) : -1.0;
    R.FrameMs = LastFrameMs;
    // 7b: on a world that culls its non-Static parts, an ISM instance of the new path is placed with the same geometry
    // as the culling uses (the mesh bounds' origin through the instance transform); a known candidate in range without
    // an entity is counted again as missing_wp_culled_in_range. Other worlds: unchanged.
    const bool bWpAudit = UsesWpNonStaticCulling(World);

    // missing_sync: R around each sensor where the last sync saw it (what the registration rule used).
    // missing_in_range / ghost_in_range: the range the rays reach, around the current position.
    // missing_steady: inside a SyncSphere and inside R around a sensor's current position (CurrentRSpheres).
    TArray<RA::FAuditSphere> SyncSpheres;
    TArray<RA::FAuditSphere> RangeSpheres;
    TArray<RA::FAuditSphere> CurrentRSpheres;
    float MinParentRadiusCm = -1.0f;
    for (const auto& Pair : RegisteredSensors)
    {
        const FSensorRegistration& S = Pair.Value;
        if (S.bLastSyncValid)
        {
            SyncSpheres.Add(RA::FAuditSphere{ S.LastSyncPosition, S.RegistrationDistanceCm });
        }
        CurrentRSpheres.Add(RA::FAuditSphere{ S.Position, S.RegistrationDistanceCm });
        RangeSpheres.Add(RA::FAuditSphere{ S.Position, SensorRangeCm(S) });

        const float SourceRadiusCm = StreamSourceRadiusCm(S.AuditParent.Get());
        if (SourceRadiusCm >= 0.0f)
        {
            MinParentRadiusCm = (MinParentRadiusCm < 0.0f) ? SourceRadiusCm : FMath::Min(MinParentRadiusCm, SourceRadiusCm);
        }
    }
    R.ParentStreamRadiusM = (MinParentRadiusCm >= 0.0f) ? MinParentRadiusCm * RGLCoord::UE_TO_RGL : -1.0f;

    // World Partition: is every sensor's range sphere streamed in (Activated)? Not covered = not loaded, which
    // by the definition of "correct" is not a defect, but tells which frames' missing counts can be trusted.
    if (World && World->GetWorldPartition() != nullptr)
    {
        R.StreamCovered = 0;
        const UWorldPartitionSubsystem* WPSubsystem = World->GetSubsystem<UWorldPartitionSubsystem>();
        if (WPSubsystem && World->GetWorldPartition()->IsInitialized())
        {
            TArray<FWorldPartitionStreamingQuerySource> Queries;
            for (const RA::FAuditSphere& S : RangeSpheres)
            {
                FWorldPartitionStreamingQuerySource& Q = Queries.AddDefaulted_GetRef();
                Q.Location = S.Center;
                Q.Radius = static_cast<float>(S.RadiusCm);
                Q.bUseGridLoadingRange = false;
                Q.bSpatialQuery = true;
            }
            const bool bCovered = (Queries.Num() == 0) ||
                WPSubsystem->IsStreamingCompleted(EWorldPartitionRuntimeCellState::Activated, Queries, /*bExactState=*/false);
            R.StreamCovered = bCovered ? 1 : 0;
        }
    }

    // ---- RGL side: every registered entity against its component ----
    for (const auto& Pair : EntityMap)
    {
        const FEntityInfo& Info = Pair.Value;
        ++R.Registered;
        if (!Info.bIsStatic)
        {
            ++R.MovableRegistered;
        }
        UStaticMeshComponent* Comp = Info.Component.Get();
        if (!Comp)
        {
            ++R.Ghost;   // destroyed or collected while still in the scene
            const double RadiusCm = RA::MeshRadiusCm(Info.MeshKey) * RA::MaxAbsScale(Info.LastTransform);
            R.GhostInRange += RA::InAnySphere(RangeSpheres, Info.LastTransform.GetLocation(), RadiusCm) ? 1 : 0;
            continue;
        }
        const RA::ELevelState LevelState = RA::GetLevelState(World, Comp->GetComponentLevel());
        if (LevelState == RA::ELevelState::InTransition)
        {
            continue;
        }
        if (RA::IsHlodOwned(Comp))
        {
            ++R.HlodRegistered;
        }
        if (Comp->IsA<USplineMeshComponent>())
        {
            ++R.SplineRegistered;   // the RGL entity is the undeformed mesh; ray_cast hits the deformed collision
        }
        if (!Comp->IsRegistered() || LevelState == RA::ELevelState::Hidden)
        {
            ++R.Ghost;
            R.GhostInRange += RA::InAnySphere(RangeSpheres, Comp->Bounds.Origin, Comp->Bounds.SphereRadius) ? 1 : 0;
            continue;
        }
        if (FObjectKey(Comp->GetStaticMesh()) != Info.MeshKey)
        {
            ++R.MeshMismatch;
        }
    }

    for (const auto& Pair : ISMCEntityMap)
    {
        const FISMCEntityGroup& Group = Pair.Value;
        const int32 NumEntities = Group.Entities.Num();
        R.Registered += NumEntities;
        const double MeshRadius = RA::MeshRadiusCm(Group.MeshKey);
        auto CountGhostInstance = [&](int32 EntityIndex)
        {
            ++R.Ghost;
            ++R.GhostIsm;
            if (Group.InstanceWorldTransforms.IsValidIndex(EntityIndex))
            {
                const FTransform& T = Group.InstanceWorldTransforms[EntityIndex];
                const int32 InRange = RA::InAnySphere(RangeSpheres, T.GetLocation(), MeshRadius * RA::MaxAbsScale(T)) ? 1 : 0;
                R.GhostInRange += InRange;
                R.GhostIsmInRange += InRange;
            }
        };

        UInstancedStaticMeshComponent* Comp = Group.Component.Get();
        if (!Comp)
        {
            for (int32 k = 0; k < NumEntities; ++k)
            {
                CountGhostInstance(k);
            }
            continue;
        }
        if (Comp->Mobility != EComponentMobility::Static)
        {
            R.MovableRegistered += NumEntities;
        }
        const RA::ELevelState LevelState = RA::GetLevelState(World, Comp->GetComponentLevel());
        if (LevelState == RA::ELevelState::InTransition)
        {
            continue;
        }
        if (RA::IsHlodOwned(Comp))
        {
            R.HlodRegistered += NumEntities;
        }
        if (!Comp->IsRegistered() || LevelState == RA::ELevelState::Hidden)
        {
            for (int32 k = 0; k < NumEntities; ++k)
            {
                CountGhostInstance(k);
            }
            continue;
        }
        if (FObjectKey(Comp->GetStaticMesh()) != Group.MeshKey)
        {
            R.MeshMismatch += NumEntities;
        }
        const int32 NumInstances = Comp->GetInstanceCount();
        for (int32 k = 0; k < NumEntities; ++k)
        {
            const int32 Index = Group.InstanceIndices.IsValidIndex(k) ? Group.InstanceIndices[k] : INDEX_NONE;
            if (Index < 0 || Index >= NumInstances)
            {
                CountGhostInstance(k);   // the instance was removed
                continue;
            }
            FTransform Now;
            if (Comp->GetInstanceTransform(Index, Now, /*bWorldSpace=*/true) &&
                Group.InstanceWorldTransforms.IsValidIndex(k) &&
                RA::InstanceChanged(Now, Group.InstanceWorldTransforms[k]))
            {
                ++R.StaleInstance;
            }
        }
    }

    // ---- Mesh cache: entries whose key no longer resolves (the asset was collected or is garbage) ----
    for (const auto& Pair : MeshCache)
    {
        if (!Pair.Key.ResolveObjectPtr())
        {
            ++R.CacheStale;
        }
    }

    // ---- World side: every registered primitive of a visible level ----
    // A static mesh component is judged with the registration predicate's distance-independent part
    // (GateStaticComponent) and the two spheres above; ISMs per instance, like RegisterISMComponent.
    // With OutExtras, the name rule is also counted over the whole loaded world, whatever the distance (once per
    // component), next to the old rule, and the components wider than 2 km are listed by mesh name.
    auto AddListed = [](TArray<FString>& List, const FString& Name)
    {
        if (List.Num() < FRGLSceneAuditExtras::MaxListed)
        {
            List.AddUnique(Name);
        }
    };
    auto CountWorldExclusion = [&](const UStaticMeshComponent* SMC, EStaticGate Gate)
    {
        const bool bExcluded = (Gate == EStaticGate::ExcludedByName);
        const bool bLegacy = IsLegacySkyName(SMC);
        const bool bHuge = SMC->Bounds.SphereRadius > HugeSkyRadiusCm;
        if (!bExcluded && !bLegacy && !bHuge)
        {
            return;
        }
        const FString MeshName = GetNameSafe(SMC->GetStaticMesh());
        if (bExcluded)
        {
            ++OutExtras->ExcludedWorld;
            AddListed(OutExtras->ExcludedMeshes, MeshName);
        }
        if (bLegacy)
        {
            ++OutExtras->ExcludedLegacyWorld;
            if (!bExcluded)
            {
                AddListed(OutExtras->LegacyOnlyMeshes, MeshName);
            }
        }
        if (bHuge)
        {
            AddListed(OutExtras->HugeMeshes, MeshName);
        }
    };
    auto AuditStaticComponent = [&](UStaticMeshComponent* SMC)
    {
        const EStaticGate Gate = GateStaticComponent(SMC, /*bRememberVerdict=*/false);   // read-only: no memo entry
        if (Gate == EStaticGate::Reject)
        {
            return;   // no mesh, hidden, zero scale: not a candidate (and not a defect)
        }
        if (OutExtras)
        {
            CountWorldExclusion(SMC, Gate);
        }
        const FBoxSphereBounds& Bounds = SMC->Bounds;
        const bool bCompInSync = RA::InAnySphere(SyncSpheres, Bounds.Origin, Bounds.SphereRadius);
        const bool bCompInRange = RA::InAnySphere(RangeSpheres, Bounds.Origin, Bounds.SphereRadius);
        if (!bCompInSync && !bCompInRange)
        {
            return;
        }
        UStaticMesh* Mesh = SMC->GetStaticMesh();

        UInstancedStaticMeshComponent* ISM = Cast<UInstancedStaticMeshComponent>(SMC);
        if (!ISM)
        {
            if (Gate == EStaticGate::ExcludedByName)
            {
                ++R.ExcludedByName;
                return;
            }
            const FEntityInfo* Info = EntityMap.Find(SMC);
            if (Info && Info->Component.Get() == SMC)
            {
                return;   // registered (a changed mesh is counted on the RGL side)
            }
            if (IsMeshRetryDeferred(Mesh))
            {
                ++R.Unextractable;
                return;
            }
            R.MissingSync += bCompInSync ? 1 : 0;
            R.MissingInRange += bCompInRange ? 1 : 0;
            R.MissingSteady += (bCompInSync && RA::InAnySphere(CurrentRSpheres, Bounds.Origin, Bounds.SphereRadius)) ? 1 : 0;
            if (bCompInRange && bWpAudit && SMC->Mobility != EComponentMobility::Static)
            {
                const FWpCandidate* WpC = WpCandidates.Find(FObjectKey(SMC));
                R.MissingWpCulledInRange += (WpC && WpC->MeshKey == FObjectKey(Mesh)) ? 1 : 0;
            }
            return;
        }

        const int32 NumInstances = ISM->GetInstanceCount();
        if (NumInstances <= 0)
        {
            return;
        }
        const FISMCEntityGroup* Group = ISMCEntityMap.Find(ISM);
        if (Group && Group->Component.Get() != ISM)
        {
            Group = nullptr;
        }
        TBitArray<> RegisteredInstances(false, NumInstances);
        if (Group)
        {
            for (const int32 Index : Group->InstanceIndices)
            {
                if (Index >= 0 && Index < NumInstances)
                {
                    RegisteredInstances[Index] = true;
                }
            }
        }
        const bool bDeferred = !Group && IsMeshRetryDeferred(Mesh);
        const double MeshRadius = Mesh->GetBounds().SphereRadius;
        // 7b: the new path's instances (non-Static, not protected, on a world that culls) are placed as the culling
        // places them; a candidate whose layout the last scan recorded counts its unregistered in-range instances.
        const bool bWpGeometry = bWpAudit && ISM->Mobility != EComponentMobility::Static &&
                                 ClassifyWpProtection(ISM, /*bRemember=*/false) == EWpProtect::None;
        const FWpCandidate* WpC = bWpGeometry ? WpCandidates.Find(FObjectKey(ISM)) : nullptr;
        if (WpC && (WpC->InstanceCount != NumInstances || WpC->MeshKey != FObjectKey(Mesh)))
        {
            WpC = nullptr;   // the layout changed since: the old history is void
        }
        for (int32 i = 0; i < NumInstances; ++i)
        {
            FTransform T;
            if (!ISM->GetInstanceTransform(i, T, /*bWorldSpace=*/true) || RA::IsZeroScale(T))
            {
                continue;
            }
            const double RadiusCm = MeshRadius * RA::MaxAbsScale(T);
            const FVector CenterCm = bWpGeometry ? WpInstanceSphere(Mesh, T).Center : T.GetLocation();
            const bool bInSync = RA::InAnySphere(SyncSpheres, CenterCm, RadiusCm);
            const bool bInRange = RA::InAnySphere(RangeSpheres, CenterCm, RadiusCm);
            if (!bInSync && !bInRange)
            {
                continue;
            }
            if (Gate == EStaticGate::ExcludedByName)
            {
                ++R.ExcludedByName;
                continue;
            }
            if (RegisteredInstances[i])
            {
                continue;
            }
            if (bDeferred)
            {
                ++R.Unextractable;
                continue;
            }
            R.MissingSync += bInSync ? 1 : 0;
            R.MissingInRange += bInRange ? 1 : 0;
            R.MissingIsmSync += bInSync ? 1 : 0;
            R.MissingIsmInRange += bInRange ? 1 : 0;
            R.MissingSteady += (bInSync && RA::InAnySphere(CurrentRSpheres, CenterCm, RadiusCm)) ? 1 : 0;
            R.MissingWpCulledInRange += (bInRange && WpC) ? 1 : 0;
        }
    };

    TArray<UPrimitiveComponent*> Primitives;
    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!IsValid(Actor) || RA::GetLevelState(World, Actor->GetLevel()) != RA::ELevelState::Visible)
        {
            continue;
        }
        Primitives.Reset();
        Actor->GetComponents(Primitives);
        for (UPrimitiveComponent* Prim : Primitives)
        {
            if (!IsValid(Prim) || !Prim->IsRegistered())
            {
                continue;   // not part of the loaded world
            }
            if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Prim))
            {
                AuditStaticComponent(SMC);
                continue;
            }
            // Geometry RGL never registers but ray_cast lidars hit (their trace channel, RayCastSemanticLidar.cpp).
            if (Prim->IsA<USkeletalMeshComponent>() || RA::IsLandscapeClass(Prim->GetClass()))
            {
                continue;
            }
            if (!Prim->IsQueryCollisionEnabled() || Prim->GetCollisionResponseToChannel(ECC_GameTraceChannel2) != ECR_Block)
            {
                continue;
            }
            const FBoxSphereBounds& Bounds = Prim->Bounds;
            if (RA::InAnySphere(SyncSpheres, Bounds.Origin, Bounds.SphereRadius) ||
                RA::InAnySphere(RangeSpheres, Bounds.Origin, Bounds.SphereRadius))
            {
                ++R.NonSmCollision;
            }
        }
    }

    R.AuditMs = (FPlatformTime::Seconds() - StartSeconds) * 1000.0;
    return R;
}

FString FRGLSceneManager::FormatAuditLine(uint64 Frame, double SimTime, const FRGLSceneAuditResult& R)
{
    return FString::Printf(
        TEXT("rgl_scene_audit frame=%llu sim_t=%.3f ghost=%d ghost_in_range=%d missing_sync=%d missing_in_range=%d "
             "mesh_mismatch=%d stale_instance=%d excluded_by_name=%d unextractable=%d non_sm_collision=%d "
             "hlod_registered=%d registered=%d pending=%d movable_registered=%d levels_added=%d since_sync_s=%.2f "
             "stream_covered=%d parent_stream_radius_m=%.1f frame_ms=%.2f audit_ms=%.3f "
             "missing_ism_sync=%d missing_ism_in_range=%d ghost_ism=%d ghost_ism_in_range=%d missing_steady=%d "
             "spline_registered=%d cache_stale=%d pending_ism=%d missing_wp_culled_in_range=%d wp_pending=%d"),
        static_cast<unsigned long long>(Frame), SimTime, R.Ghost, R.GhostInRange, R.MissingSync, R.MissingInRange,
        R.MeshMismatch, R.StaleInstance, R.ExcludedByName, R.Unextractable, R.NonSmCollision,
        R.HlodRegistered, R.Registered, R.Pending, R.MovableRegistered, R.LevelsAdded, R.SecondsSinceSync,
        R.StreamCovered, static_cast<double>(R.ParentStreamRadiusM), R.FrameMs, R.AuditMs,
        R.MissingIsmSync, R.MissingIsmInRange, R.GhostIsm, R.GhostIsmInRange, R.MissingSteady,
        R.SplineRegistered, R.CacheStale, R.PendingIsm, R.MissingWpCulledInRange, R.WpPending);
}

// ============================================================================
// Scene audit line (rgl.Scene.Audit / -rgl-scene-audit=)
// ============================================================================

void FRGLSceneManager::MaybeEmitAudit(UWorld* World, double SimTime, uint64 Frame)
{
    const int32 Every = SceneAuditEvery();
    if (Every <= 0)
    {
        LevelsAddedSinceAudit = 0;   // levels_added counts only while the audit is on
        return;
    }
    if (!World)
    {
        return;
    }
    // Every RGL sensor calls this after its Update; Update itself runs once per frame.
    if (bAuditFrameValid && Frame == LastAuditFrame)
    {
        return;
    }
    LastAuditFrame = Frame;
    bAuditFrameValid = true;

    ++AuditUpdateCount;
    if (AuditUpdateCount % static_cast<uint64>(Every) != 0)
    {
        return;
    }

    // The first audit line of the world also looks at the name rule over the whole loaded world (FRGLSceneAuditExtras).
    FRGLSceneAuditExtras Extras;
    const bool bExcludedLine = !bExcludedLineLogged;
    const FRGLSceneAuditResult Result = AuditScene(World, bExcludedLine ? &Extras : nullptr);
    LevelsAddedSinceAudit = 0;
    EmitSceneLine(FormatAuditLine(Frame, (SimTime >= 0.0) ? SimTime : CurrentSimTime, Result));
    if (bExcludedLine)
    {
        // Once per world, with the first audit line: what the name rule leaves out, what only the old rule left out,
        // and the meshes wider than 2 km (mesh names; spaces would break key=value parsing).
        bExcludedLineLogged = true;
        auto JoinNames = [](const TArray<FString>& Names)
        {
            return FString::Join(Names, TEXT(";")).Replace(TEXT(" "), TEXT("_"));
        };
        EmitSceneLine(FString::Printf(
            TEXT("rgl_scene_audit_excluded frame=%llu n=%d names=%s excluded_world=%d excluded_legacy_world=%d "
                 "legacy_only_meshes=%s huge_meshes=%s"),
            static_cast<unsigned long long>(Frame), Extras.ExcludedMeshes.Num(), *JoinNames(Extras.ExcludedMeshes),
            Extras.ExcludedWorld, Extras.ExcludedLegacyWorld, *JoinNames(Extras.LegacyOnlyMeshes),
            *JoinNames(Extras.HugeMeshes)));
    }
#if !UE_BUILD_SHIPPING
    ++AuditLinesEmitted_ForTest;
#endif
}

// ============================================================================
// Level-added sync (P5)
// ============================================================================

void FRGLSceneManager::BindWorld(UWorld* World)
{
    OwnerWorldKey = FObjectKey(World);
    if (!LevelAddedHandle.IsValid())
    {
        LevelAddedHandle = FWorldDelegates::LevelAddedToWorld.AddRaw(this, &FRGLSceneManager::OnLevelAddedToWorld);
    }
}

void FRGLSceneManager::OnLevelAddedToWorld(ULevel* Level, UWorld* World)
{
    if (FObjectKey(World) != OwnerWorldKey)
    {
        return;
    }
    ++LevelsAddedSinceAudit;
    if (Level)
    {
        PendingAddedLevels.AddUnique(TWeakObjectPtr<ULevel>(Level));
    }
}

bool FRGLSceneManager::ConsumeRelevantLevelAdds()
{
    if (PendingAddedLevels.Num() == 0)
    {
        return false;
    }
    // D5: a level-triggered sync at most every LevelSyncMinIntervalSeconds (sim); the levels wait meanwhile. A clock
    // that went back (a new episode in the same world) does not hold them.
    if (LastSyncSimTime >= 0.0)
    {
        const double SinceSyncSeconds = CurrentSimTime - LastSyncSimTime;
        if (SinceSyncSeconds >= 0.0 && SinceSyncSeconds < LevelSyncMinIntervalSeconds)
        {
            return false;
        }
    }

    // Relevant = the level brings a registered static mesh component whose bounds reach some sensor's R and that
    // passes the registration gate: what the next periodic sync would register. The gate matters: a sky dome's bounds
    // reach every sensor, but the name rule leaves it out, so its level alone must not force a sync. The cheap
    // tests run first, the gate's name checks only for a component already in range. Only the added levels' actors
    // are looked at, not the whole world.
    bool bRelevant = false;
    for (const TWeakObjectPtr<ULevel>& WeakLevel : PendingAddedLevels)
    {
        const ULevel* Level = WeakLevel.Get();
        if (!Level || Level->bIsBeingRemoved)
        {
            continue;
        }
        for (AActor* Actor : Level->Actors)
        {
            if (!IsValid(Actor))
            {
                continue;
            }
            LevelScanComponents.Reset();
            Actor->GetComponents<UStaticMeshComponent>(LevelScanComponents);
            for (UStaticMeshComponent* Comp : LevelScanComponents)
            {
                if (IsValid(Comp) && Comp->IsRegistered() && Comp->GetStaticMesh() && IsComponentWithinAnySensor(Comp, 0.0f) &&
                    GateStaticComponent(Comp) == EStaticGate::Pass)
                {
                    bRelevant = true;
                    break;
                }
            }
            if (bRelevant)
            {
                break;
            }
        }
        if (bRelevant)
        {
            break;
        }
    }
    LevelScanComponents.Reset();   // keeps the allocation for the next look
    PendingAddedLevels.Reset();
    return bRelevant;
}

// ============================================================================
// Debug hit dump settings (rgl.Debug.HitDump*; the dump itself is in RGLBackendImpl.cpp)
// ============================================================================

FString FRGLSceneManager::GetHitDumpDir()
{
    ApplySceneCommandLineOverrides();   // idempotent; the switches normally land with the first instance
    return CVarRGLDebugHitDumpDir.GetValueOnGameThread();
}

int32 FRGLSceneManager::GetHitDumpEvery()
{
    ApplySceneCommandLineOverrides();
    return FMath::Max(1, CVarRGLDebugHitDumpEvery.GetValueOnGameThread());
}

#if !UE_BUILD_SHIPPING
void FRGLSceneManager::SyncStaticNow_ForTest(UWorld* World, double Now)
{
    // What Update() does on a sync step, without the per-frame guard, the triggers or the create budget.
    CurrentSimTime = Now;
    bWpActive = UsesWpNonStaticCulling(World);
    bWpSyncDiscontinuous = false;
    FSceneSyncStats SyncStats;
    SyncWorldComponents(World, SyncStats);
    TimeSinceLastSync = 0.0f;
    LastSyncSimTime = Now;
    for (auto& Pair : RegisteredSensors)
    {
        Pair.Value.LastSyncPosition = Pair.Value.Position;
        Pair.Value.bLastSyncValid = true;
    }
    FDrainStats Drain;
    DrainPendingCreates(/*bUnlimited=*/true, Drain);
    // 7b: the culling's queue, in full.
    if (bWpActive && WpJobs.Num() > 0)
    {
        WpDrain(/*bUnlimited=*/true);
    }
    // The ISM re-evaluation the sync queued, in full (Update() runs it on the following Updates, capped).
    FIsmReevalStats Ism;
    DrainIsmReevaluations(/*MaxChanges=*/0, Ism);
    IsmAddedSinceLog += Ism.Added;
    IsmRemovedSinceLog += Ism.Removed;
    IsmMsSinceLog += Ism.Ms;
    IsmMaxMsSinceLog = FMath::Max(IsmMaxMsSinceLog, Ism.Ms);
    QueueEpisode = FQueueEpisode();
    bInitialized = true;   // the static scene is synced; a later Update() must not run InitializeFromWorld on top
}
void FRGLSceneManager::Update_ForTest(UWorld* World, double SimTime)
{
    if (World)
    {
        UpdateStep(World, SimTime);
    }
}
void FRGLSceneManager::OnLevelAddedToWorld_ForTest(ULevel* Level, UWorld* World)
{
    OnLevelAddedToWorld(Level, World);
}
void FRGLSceneManager::CheckStreamCoverage_ForTest(double SimTime, bool bIsWp, bool bCovered)
{
    CurrentSimTime = SimTime;
    if (!bIsWp)
    {
        return;   // as CheckStreamCoverage on a map without World Partition
    }
    for (auto& Pair : RegisteredSensors)
    {
        StepStreamCoverage(Pair.Key, Pair.Value, bCovered);
    }
}
void FRGLSceneManager::AddIdleMeshEntry_ForTest(const UStaticMesh* Asset)
{
    const FObjectKey Key(Asset);
    if (!Asset || MeshCache.Contains(Key))
    {
        return;
    }
    MeshCache.Add(Key);
    IdleMeshLru.Add(Key);
}
int32 FRGLSceneManager::GetSkyVerdictCount_ForTest() const
{
    return SkyVerdictByMesh.Num();
}
double FRGLSceneManager::GetWpQueueAgeMax_ForTest() const
{
    double Age = 0.0;
    for (const FWpJob& J : WpJobs)
    {
        if (J.bAdd)
        {
            Age = FMath::Max(Age, CurrentSimTime - J.SinceSimTime);
        }
    }
    return Age;
}
bool FRGLSceneManager::IsWpPromoted_ForTest(const UStaticMeshComponent* Comp) const
{
    const FWpCandidate* C = WpCandidates.Find(FObjectKey(Comp));
    return C && C->bMoved;
}
bool FRGLSceneManager::IsIsmGroupConsistent_ForTest(const UInstancedStaticMeshComponent* Comp) const
{
    const FISMCEntityGroup* G = ISMCEntityMap.Find(const_cast<UInstancedStaticMeshComponent*>(Comp));
    if (!G)
    {
        return true;   // no group: nothing to disagree
    }
    const int32 N = G->Entities.Num();
    if (G->InstanceIndices.Num() != N || G->InstanceWorldTransforms.Num() != N || G->MeshRefCount != N)
    {
        return false;
    }
    TSet<int32> Seen;
    for (const int32 Index : G->InstanceIndices)
    {
        bool bDup = false;
        Seen.Add(Index, &bDup);
        if (bDup || !G->RegisteredMask.IsValidIndex(Index) || !G->RegisteredMask[Index])
        {
            return false;
        }
    }
    int32 MaskBits = 0;
    for (int32 i = 0; i < G->RegisteredMask.Num(); ++i)
    {
        MaskBits += G->RegisteredMask[i] ? 1 : 0;
    }
    return MaskBits == N;
}
float FRGLSceneManager::GetWpLoadRadiusCm_ForTest(const void* SensorId) const
{
    const FSensorRegistration* S = RegisteredSensors.Find(SensorId);
    return (S && bWpSyncValid) ? WpLoadRadiusCm(*S) : -1.0f;
}
float FRGLSceneManager::SyncTriggerCm_ForTest(float RegistrationCm, float RangeCm) const
{
    FSensorRegistration S;
    S.RegistrationDistanceCm = RegistrationCm;
    S.AuditRangeCm = RangeCm;
    return SyncTriggerCm(S, ReadSceneKnobs());
}
int32 FRGLSceneManager::GetIsmEntityCount_ForTest(const UInstancedStaticMeshComponent* Comp) const
{
    const FISMCEntityGroup* Group = ISMCEntityMap.Find(const_cast<UInstancedStaticMeshComponent*>(Comp));
    return Group ? Group->Entities.Num() : 0;
}
FRGLSceneManager::FSceneCounts FRGLSceneManager::GetSceneCounts_ForTest() const
{
    FSceneCounts C;
    C.Entities = EntityMap.Num();
    C.IsmcGroups = ISMCEntityMap.Num();
    C.CachedMeshes = MeshCache.Num();
    C.IdleMeshes = IdleMeshLru.Num();
    return C;
}
void FRGLSceneManager::SyncSkeletalNow_ForTest(UWorld* World, double SimTime)
{
    CurrentSimTime = SimTime;
    if (SkeletalEnabled()) SyncSkeletalComponents(World); else if (SkeletalEntityMap.Num() > 0) TeardownSkeletal();
}
void FRGLSceneManager::UpdateSkeletalPosesNow_ForTest(double SimTime)
{
    CurrentSimTime = SimTime;
    const uint64 TimeNs = static_cast<uint64>(SimTime * 1e9);
    RGL_CHECK(rgl_scene_set_time(Scene, TimeNs));      // mirrors Update(): time before poses
    if (SkeletalEnabled()) UpdateSkeletalPoses();
}
int32 FRGLSceneManager::GetSkeletalLiveEntityCount_ForTest() const { int32 N = 0; for (const auto& P : SkeletalEntityMap) N += (P.Value.Entity != nullptr); return N; }
bool FRGLSceneManager::IsSkeletalCutout_ForTest(const USkeletalMeshComponent* Comp) const
{
    const FSkeletalEntityInfo* Info = SkeletalEntityMap.Find(const_cast<USkeletalMeshComponent*>(Comp));
    return Info && Info->bCutout;
}
int32 FRGLSceneManager::GetSkeletalCutoutKeptTriangles_ForTest(const USkeletalMeshComponent* Comp) const
{
    const FSkeletalEntityInfo* Info = SkeletalEntityMap.Find(const_cast<USkeletalMeshComponent*>(Comp));
    return (Info && Info->bCutout && Info->bCutoutBuilt) ? Info->CutoutKeptTriangles : -1;
}
int32 FRGLSceneManager::GetSkeletalCutoutRemovedTriangles_ForTest(const USkeletalMeshComponent* Comp) const
{
    const FSkeletalEntityInfo* Info = SkeletalEntityMap.Find(const_cast<USkeletalMeshComponent*>(Comp));
    return (Info && Info->bCutout && Info->bCutoutBuilt) ? Info->CutoutRemovedTriangles : -1;
}
#endif

#endif // WITH_RGL
