// Copyright (c) 2026 RGL Integration for CARLA.
// Scene audit (FRGLSceneManager::AuditScene / MaybeEmitAudit) and the debug hit dump, on the game world of
// EmptyMap (the hit dump also needs the GPU, like the other raytrace tests). The scene manager of a world is
// shared by every test, so counts are compared as differences around each step, and every test works around its
// own base point, 1 km from the others.

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "HAL/IConsoleManager.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLSceneManager.h"
#include "../RGLBackendImpl.h"
#include "../RGLCoordinateUtils.h"
#include "RGLSceneTestUtil.h"

namespace RGLSceneAuditTests
{
// kCube, BaseFor, SpawnMeshActor, Diff, FAuditFixture, ...: the shared helpers of the scene tests.
using namespace RGLSceneTestUtil;

// ============================================================================
// Task 3: AuditScene
// ============================================================================

// A static cube in range: registered by a sync, and no defect counter moves.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditCleanTest, "CarlaRGL.SceneAudit.Clean",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditCleanTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(0);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0001), Base, 5000.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    TestNotNull(TEXT("cube actor"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static)));
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(1, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("registered +1"), D.Registered, 1);
    TestNoDefects(*this, TEXT("clean"), D);
    TestEqual(TEXT("create queue drained"), R1.Pending, 0);
    TestEqual(TEXT("no sensor with a parent"), R1.ParentStreamRadiusM, -1.0f);
    return true;
}

// A registered cube whose component is unregistered (no sync since): a ghost, inside the range.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditGhostAfterUnregisterTest, "CarlaRGL.SceneAudit.GhostAfterUnregister",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditGhostAfterUnregisterTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(1);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0002), Base, 5100.0);
    AStaticMeshActor* A = F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static));
    if (!TestNotNull(TEXT("cube actor"), A)) return false;
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();

    A->GetStaticMeshComponent()->UnregisterComponent();
    const FRGLSceneAuditResult D = Diff(R1, F.Audit());
    TestEqual(TEXT("ghost +1"), D.Ghost, 1);
    TestEqual(TEXT("ghost_in_range +1"), D.GhostInRange, 1);
    TestEqual(TEXT("still registered in RGL"), D.Registered, 0);
    TestEqual(TEXT("an unregistered component is not missing"), D.MissingInRange, 0);
    return true;
}

// A cube spawned after the sync: missing from both spheres until the next sync registers it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditMissingBeforeSyncTest, "CarlaRGL.SceneAudit.MissingBeforeSync",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditMissingBeforeSyncTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(2);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0003), Base, 5200.0);
    F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static));
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();

    TestNotNull(TEXT("2nd cube"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(0.f, 6000.f, 0.f), EComponentMobility::Static)));
    const FRGLSceneAuditResult D = Diff(R1, F.Audit());
    TestEqual(TEXT("before sync: missing_in_range +1"), D.MissingInRange, 1);
    TestEqual(TEXT("before sync: missing_sync +1"), D.MissingSync, 1);
    TestEqual(TEXT("before sync: not registered"), D.Registered, 0);

    F.Sync();
    const FRGLSceneAuditResult D2 = Diff(R1, F.Audit());
    TestEqual(TEXT("after sync: missing_in_range"), D2.MissingInRange, 0);
    TestEqual(TEXT("after sync: missing_sync"), D2.MissingSync, 0);
    TestEqual(TEXT("after sync: registered +1"), D2.Registered, 1);
    return true;
}

// P8 with the legacy values of the range-edge knobs (rgl.Scene.SyncTriggerMarginSlackM = -1: the distance trigger is
// 0.2 R with no cap; rgl.Scene.InRangeCreatesPerUpdate = 0: no in-range priority), so the old behaviour stays reachable:
// range 100 m, R = 120 m, distance trigger 0.2 R = 24 m. A cube whose bounds start at 122 m is outside R at the sync; a
// 23 m move brings it inside the range without triggering a sync: missing in range, while the registration rule (the
// sphere at the last sync) had nothing to register. SceneSync.RangeEdgeGapClosed is the same with the defaults.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditRangeEdgeGapTest, "CarlaRGL.SceneAudit.RangeEdgeGap",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditRangeEdgeGapTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    // Before the fixture: its clean-up sync still runs with these values, then the guards put the defaults back.
    FFloatCVarGuard SlackGuard(TEXT("rgl.Scene.SyncTriggerMarginSlackM"));
    FCVarGuard InRangeGuard(TEXT("rgl.Scene.InRangeCreatesPerUpdate"));
    SetFloatCVar(TEXT("rgl.Scene.SyncTriggerMarginSlackM"), -1.0f);
    SetCVar(TEXT("rgl.Scene.InRangeCreatesPerUpdate"), 0);
    const FVector Base = BaseFor(3);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0004), Base, 5300.0);
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();

    const float BoundsRadiusCm = static_cast<float>(Cube->GetBounds().SphereRadius);
    F.Keep(SpawnMeshActor(World, Cube, Base + FVector(12200.f + BoundsRadiusCm, 0.f, 0.f), EComponentMobility::Static));
    F.Sync();
    const FRGLSceneAuditResult D1 = Diff(R1, F.Audit());
    TestEqual(TEXT("outside R at the sync: not registered"), D1.Registered, 0);
    TestEqual(TEXT("outside R at the sync: missing_sync"), D1.MissingSync, 0);
    TestEqual(TEXT("outside the range: missing_in_range"), D1.MissingInRange, 0);

    const float MoveCm = 2300.f;
    TestTrue(TEXT("the move stays below the legacy distance trigger (0.2 R)"),
             MoveCm < F.SM.SyncTriggerCm_ForTest(kRegistrationCm, kRangeCm) && MoveCm < 0.2f * kRegistrationCm);
    F.SensorPos = Base + FVector(MoveCm, 0.f, 0.f);
    F.T += 0.05;
    F.Refresh();
    F.SM.SetSyncInterval(3600.f);   // keep the 1 s timer out of this step: only the distance trigger is under test
    F.SM.Update(World, F.T);
    F.SM.SetSyncInterval(1.0f);     // the default
    const FRGLSceneAuditResult R2 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(2, F.T, R2));
    const FRGLSceneAuditResult D2 = Diff(R1, R2);
    TestEqual(TEXT("no sync: still not registered"), D2.Registered, 0);
    TestEqual(TEXT("inside the range now: missing_in_range +1"), D2.MissingInRange, 1);
    TestEqual(TEXT("the last sync's sphere still excludes it: missing_sync"), D2.MissingSync, 0);
    return true;
}

// A Movable cube whose mesh is swapped after registration: the entity keeps the old shape.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditMeshMismatchTest, "CarlaRGL.SceneAudit.MeshMismatch",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditMeshMismatchTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    UStaticMesh* Sphere = LoadMeshReady(kSphere); if (!TestNotNull(TEXT("sphere"), Sphere)) return false;
    const FVector Base = BaseFor(4);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0005), Base, 5400.0);
    AStaticMeshActor* A = F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Movable));
    if (!TestNotNull(TEXT("cube actor"), A)) return false;
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();

    TestTrue(TEXT("mesh swapped"), A->GetStaticMeshComponent()->SetStaticMesh(Sphere));
    const FRGLSceneAuditResult D = Diff(R1, F.Audit());
    TestEqual(TEXT("mesh_mismatch +1"), D.MeshMismatch, 1);
    TestEqual(TEXT("not a ghost"), D.Ghost, 0);
    TestEqual(TEXT("not missing"), D.MissingInRange, 0);
    return true;
}

// An instance added to a registered ISM after the sync is not in the RGL scene until a sync re-evaluates the group;
// the audit counts it as an ISM-instance miss (50 m from the sensor: inside the range and inside both R spheres).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditIsmInstanceAddedAfterSyncTest, "CarlaRGL.SceneAudit.IsmInstanceAddedAfterSync",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditIsmInstanceAddedAfterSyncTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(16);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0011), Base, 8000.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(0.f, 300.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    TestEqual(TEXT("2 instances registered"), Diff(R0, R1).Registered, 2);

    Ism->AddInstance(FTransform(FVector(1000.f, 0.f, 0.f)));   // no sync after this
    const FRGLSceneAuditResult D = Diff(R1, F.Audit());
    TestEqual(TEXT("the new instance is not registered yet"), D.Registered, 0);
    TestEqual(TEXT("missing_in_range +1"), D.MissingInRange, 1);
    TestEqual(TEXT("... and it is an ISM instance: missing_ism_in_range +1"), D.MissingIsmInRange, 1);
    TestEqual(TEXT("... inside both R spheres: missing_steady +1 (the ISM path)"), D.MissingSteady, 1);
    TestEqual(TEXT("not a ghost"), D.Ghost, 0);
    return true;
}

// An ISM instance moved after registration: the entity stays where the instance was.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditStaleInstanceTest, "CarlaRGL.SceneAudit.StaleInstance",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditStaleInstanceTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(5);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0006), Base, 5500.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(0.f, 300.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    TestEqual(TEXT("2 instances registered"), Diff(R0, R1).Registered, 2);

    FTransform Moved;
    TestTrue(TEXT("instance 0 transform"), Ism->GetInstanceTransform(0, Moved, /*bWorldSpace=*/true));
    Moved.AddToTranslation(FVector(500.f, 0.f, 0.f));
    TestTrue(TEXT("instance 0 moved 5 m"), Ism->UpdateInstanceTransform(0, Moved, /*bWorldSpace=*/true, /*bMarkRenderStateDirty=*/true));
    const FRGLSceneAuditResult D = Diff(R1, F.Audit());
    TestEqual(TEXT("stale_instance +1"), D.StaleInstance, 1);
    TestEqual(TEXT("not a ghost"), D.Ghost, 0);
    TestEqual(TEXT("not missing (the instance index is registered)"), D.MissingInRange, 0);
    return true;
}

// The name rule leaves out a sky sphere (the dome words on the mesh name): counted as excluded, not as missing. A
// transient mesh named SM_SkySphere stands in for the engine's asset, so the test does not depend on what the engine
// ships; it is cube-sized at most, so the 2 km size guard plays no part.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditExcludedByNameTest, "CarlaRGL.SceneAudit.ExcludedByName",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditExcludedByNameTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Sky = NewNamedTransientMesh(TEXT("SM_SkySphere")); if (!TestNotNull(TEXT("sky sphere mesh"), Sky)) return false;
    TestTrue(FString::Printf(TEXT("a dome word in the mesh name (%s)"), *Sky->GetName()),
             FRGLSceneManager::IsSkyDomeMeshName(Sky->GetName()));
    const FVector Base = BaseFor(6);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0007), Base, 5600.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AStaticMeshActor* A = F.Keep(SpawnMeshActor(World, Sky, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static));
    if (!TestNotNull(TEXT("sky sphere actor"), A)) return false;
    TestTrue(TEXT("bounds below the 2 km size guard"),
             A->GetStaticMeshComponent()->Bounds.SphereRadius < FRGLSceneManager::HugeSkyRadiusCm);
    F.Sync();
    const FRGLSceneAuditResult D = Diff(R0, F.Audit());
    TestEqual(TEXT("excluded_by_name +1"), D.ExcludedByName, 1);
    TestEqual(TEXT("not missing"), D.MissingInRange, 0);
    TestEqual(TEXT("not registered"), D.Registered, 0);
    return true;
}

// Review focus 5: a component without a mesh and an ISMC without instances are no candidates at all.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditNullMeshAndEmptyIsmcTest, "CarlaRGL.SceneAudit.NullMeshAndEmptyIsmc",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditNullMeshAndEmptyIsmcTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(7);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0008), Base, 5700.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    TestNotNull(TEXT("mesh-less actor"), F.Keep(SpawnMeshActor(World, nullptr, Base + FVector(3000.f, 0.f, 0.f), EComponentMobility::Static)));
    AActor* Owner = nullptr;
    TestNotNull(TEXT("empty ISMC"), SpawnIsm(World, Cube, Base + FVector(3500.f, 0.f, 0.f), {}, Owner));
    F.Keep(Owner);
    F.Sync();
    const FRGLSceneAuditResult D = Diff(R0, F.Audit());
    TestEqual(TEXT("missing_in_range"), D.MissingInRange, 0);
    TestEqual(TEXT("missing_sync"), D.MissingSync, 0);
    TestEqual(TEXT("mesh_mismatch"), D.MeshMismatch, 0);
    TestEqual(TEXT("registered"), D.Registered, 0);
    TestEqual(TEXT("pending"), D.Pending, 0);
    return true;
}

// The audit is read-only: the scene's counts (idle meshes included) and the audit itself do not change.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditDoesNotChangeSceneTest, "CarlaRGL.SceneAudit.DoesNotChangeScene",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditDoesNotChangeSceneTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    UStaticMesh* Sphere = LoadMeshReady(kSphere); if (!TestNotNull(TEXT("sphere"), Sphere)) return false;
    const FVector Base = BaseFor(8);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0009), Base, 5800.0);
    // Declared after the fixture: restored first, so the fixture's last sync trims the idle mesh again.
    FCVarGuard IdleGuard(TEXT("rgl.Scene.IdleMeshCacheMax"));
    SetCVar(TEXT("rgl.Scene.IdleMeshCacheMax"), 4);

    F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static));
    AStaticMeshActor* Ball = SpawnMeshActor(World, Sphere, Base + FVector(0.f, 6000.f, 0.f), EComponentMobility::Static);
    AActor* Owner = nullptr;
    SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f), { FVector::ZeroVector, FVector(0.f, 300.f, 0.f) }, Owner);
    F.Keep(Owner);
    F.Sync();
    if (Ball) Ball->Destroy();   // its mesh becomes an idle cache entry (its key still resolves: no purge takes it)
    F.Sync();

    const FRGLSceneManager::FSceneCounts C0 = F.SM.GetSceneCounts_ForTest();
    const FRGLSceneAuditResult A1 = F.Audit();
    const FRGLSceneAuditResult A2 = F.Audit();
    const FRGLSceneManager::FSceneCounts C1 = F.SM.GetSceneCounts_ForTest();
    AddInfo(FString::Printf(TEXT("entities=%d ismc=%d meshes=%d idle=%d"), C0.Entities, C0.IsmcGroups, C0.CachedMeshes, C0.IdleMeshes));
    TestTrue(TEXT("an idle mesh is resident"), C0.IdleMeshes >= 1);
    TestEqual(TEXT("entities unchanged"), C1.Entities, C0.Entities);
    TestEqual(TEXT("ISMC groups unchanged"), C1.IsmcGroups, C0.IsmcGroups);
    TestEqual(TEXT("cached meshes unchanged"), C1.CachedMeshes, C0.CachedMeshes);
    TestEqual(TEXT("idle meshes unchanged"), C1.IdleMeshes, C0.IdleMeshes);

    // Same result twice (audit_ms is the audit's own wall time and may differ).
    FRGLSceneAuditResult B1 = A1, B2 = A2;
    B1.AuditMs = B2.AuditMs = 0.0;
    const FString Line1 = FRGLSceneManager::FormatAuditLine(0, 0.0, B1);
    const FString Line2 = FRGLSceneManager::FormatAuditLine(0, 0.0, B2);
    AddInfo(Line1);
    TestTrue(TEXT("same counts twice"), Line2.Equals(Line1, ESearchCase::CaseSensitive));
    TestEqual(TEXT("same frame_ms"), A2.FrameMs, A1.FrameMs);
    TestEqual(TEXT("same since_sync_s"), A2.SecondsSinceSync, A1.SecondsSinceSync);
    return true;
}

// The line's keys, order and number formats (Python reads them).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditFormatLineTest, "CarlaRGL.SceneAudit.FormatLine",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditFormatLineTest::RunTest(const FString& Parameters)
{
    FRGLSceneAuditResult R;
    R.Ghost = 1;
    R.StreamCovered = -1;
    R.ParentStreamRadiusM = -1.0f;
    R.MissingWpCulledInRange = 3;
    R.WpPending = 4;
    const FString Line = FRGLSceneManager::FormatAuditLine(42, 1.5, R);
    const FString Expected = TEXT(
        "rgl_scene_audit frame=42 sim_t=1.500 ghost=1 ghost_in_range=0 missing_sync=0 missing_in_range=0 "
        "mesh_mismatch=0 stale_instance=0 excluded_by_name=0 unextractable=0 non_sm_collision=0 hlod_registered=0 "
        "registered=0 pending=0 movable_registered=0 levels_added=0 since_sync_s=0.00 stream_covered=-1 "
        "parent_stream_radius_m=-1.0 frame_ms=0.00 audit_ms=0.000 missing_ism_sync=0 missing_ism_in_range=0 "
        "ghost_ism=0 ghost_ism_in_range=0 missing_steady=0 spline_registered=0 cache_stale=0 pending_ism=0 "
        "missing_wp_culled_in_range=3 wp_pending=4");
    // Exact (string TestEqual ignores case).
    if (!TestTrue(TEXT("rgl_scene_audit line, exactly"), Line.Equals(Expected, ESearchCase::CaseSensitive)))
    {
        AddInfo(TEXT("got:      ") + Line);
        AddInfo(TEXT("expected: ") + Expected);
    }
    return true;
}

// Review focus 4: the audit's cost with 2,000 static cubes in range (logged; no threshold).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditCostTest, "CarlaRGL.SceneAudit.AuditCost",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditCostTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(9);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A000A), Base, 5900.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    const int32 NumCubes = 2000;   // 50 x 40 grid, 2 m pitch: all within 65 m of the sensor
    for (int32 i = 0; i < NumCubes; ++i)
    {
        const FVector Offset(200.f * static_cast<float>(i % 50) - 4900.f, 200.f * static_cast<float>(i / 50) - 3900.f, 0.f);
        F.Keep(SpawnMeshActor(World, Cube, Base + Offset, EComponentMobility::Static));
    }
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FString::Printf(TEXT("audit with %d cubes in range: audit_ms=%.3f"), NumCubes, R1.AuditMs));
    AddInfo(FRGLSceneManager::FormatAuditLine(3, F.T, R1));
    TestEqual(TEXT("all registered"), Diff(R0, R1).Registered, NumCubes);
    TestTrue(TEXT("audit_ms measured"), R1.AuditMs > 0.0);
    return true;
}

// ============================================================================
// Task 4: MaybeEmitAudit (rgl.Scene.Audit)
// ============================================================================

// Review focus 2: two sensors tick in the same frame -> one line.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditOncePerFrameTest, "CarlaRGL.SceneAudit.OncePerFrame",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditOncePerFrameTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    FCVarGuard AuditGuard(TEXT("rgl.Scene.Audit"));
    SetCVar(TEXT("rgl.Scene.Audit"), 1);
    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    const void* SensorA = reinterpret_cast<const void*>(0x5B0001);
    const void* SensorB = reinterpret_cast<const void*>(0x5B0002);
    const FVector Base = BaseFor(20);
    const double T = 6000.0;
    SM.RegisterSensor(SensorA, Base, kRegistrationCm, T);
    SM.RegisterSensor(SensorB, Base + FVector(1000.f, 0.f, 0.f), kRegistrationCm, T);
    const int32 Before = SM.AuditLinesEmitted_ForTest;
    const uint64 Frame = GFrameCounter + (1ull << 40);   // a frame no real tick will reach

    SM.MaybeEmitAudit(World, T, Frame);   // sensor A's tick
    SM.MaybeEmitAudit(World, T, Frame);   // sensor B's tick, same frame
    TestEqual(TEXT("one line for the frame"), SM.AuditLinesEmitted_ForTest - Before, 1);
    SM.MaybeEmitAudit(World, T + 0.05, Frame + 1);
    TestEqual(TEXT("one more for the next frame"), SM.AuditLinesEmitted_ForTest - Before, 2);

    SM.UnregisterSensor(SensorA);
    SM.UnregisterSensor(SensorB);
    return true;
}

// rgl.Scene.Audit = 3: one line every 3rd frame.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditEveryNthTest, "CarlaRGL.SceneAudit.EveryNth",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditEveryNthTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    FCVarGuard AuditGuard(TEXT("rgl.Scene.Audit"));
    SetCVar(TEXT("rgl.Scene.Audit"), 3);
    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    const int32 Before = SM.AuditLinesEmitted_ForTest;
    const uint64 Frame0 = GFrameCounter + (1ull << 40) + 100;
    for (uint64 f = 1; f <= 6; ++f)
    {
        SM.MaybeEmitAudit(World, 6100.0 + 0.05 * static_cast<double>(f), Frame0 + f);
    }
    TestEqual(TEXT("6 frames, every 3rd -> 2 lines"), SM.AuditLinesEmitted_ForTest - Before, 2);
    return true;
}

// Off unless asked for: no line. The LevelAddedToWorld binding does not depend on the audit: the level-triggered
// sync (P5) needs it for the instance's whole life.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditOffByDefaultTest, "CarlaRGL.SceneAudit.OffByDefault",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditOffByDefaultTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(TEXT("rgl.Scene.Audit"));
    if (!TestNotNull(TEXT("rgl.Scene.Audit"), CVar)) return false;
    TestEqual(TEXT("rgl.Scene.Audit defaults to 0"), CVar->GetInt(), 0);
    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    const int32 Before = SM.AuditLinesEmitted_ForTest;
    const uint64 Frame0 = GFrameCounter + (1ull << 40) + 200;
    for (uint64 f = 1; f <= 5; ++f)
    {
        SM.MaybeEmitAudit(World, 6200.0 + 0.05 * static_cast<double>(f), Frame0 + f);
    }
    TestEqual(TEXT("no line"), SM.AuditLinesEmitted_ForTest - Before, 0);
    TestTrue(TEXT("the level binding is always on (P5)"), FWorldDelegates::LevelAddedToWorld.IsBoundToObject(&SM));
    return true;
}

// Review focus 1: the audit is on when the world goes away (load_world: CleanupWorld -> OnWorldCleanup ->
// DestroyInstance). The manager goes with its delegate binding, and a manager of a new world counts only its own
// lines. Throwaway worlds: broadcasting OnWorldCleanup for the game world would tear down far more than RGL.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditWorldCleanupTest, "CarlaRGL.SceneAudit.WorldCleanup",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditWorldCleanupTest::RunTest(const FString& Parameters)
{
    FCVarGuard AuditGuard(TEXT("rgl.Scene.Audit"));
    SetCVar(TEXT("rgl.Scene.Audit"), 1);
    const uint64 Frame0 = GFrameCounter + (1ull << 40) + 300;

    UWorld* WorldA = UWorld::CreateWorld(EWorldType::Inactive, /*bInformEngineOfWorld=*/false);
    if (!TestNotNull(TEXT("world A"), WorldA)) return false;
    {
        FRGLSceneManager& A = FRGLSceneManager::GetInstance(WorldA);
        A.MaybeEmitAudit(WorldA, 1.0, Frame0);
        TestEqual(TEXT("A printed its line"), A.AuditLinesEmitted_ForTest, 1);
        TestTrue(TEXT("A listens to LevelAddedToWorld while the audit is on"), FWorldDelegates::LevelAddedToWorld.IsBoundToObject(&A));
        const void* APtr = &A;
        WorldA->DestroyWorld(/*bInformEngineOfWorld=*/false);   // CleanupWorld broadcasts OnWorldCleanup
        FRGLSceneManager::DestroyInstance(WorldA);              // no-op when the module's cleanup hook already ran
        TestFalse(TEXT("A's binding went with it"), FWorldDelegates::LevelAddedToWorld.IsBoundToObject(APtr));
    }

    UWorld* WorldB = UWorld::CreateWorld(EWorldType::Inactive, /*bInformEngineOfWorld=*/false);
    if (!TestNotNull(TEXT("world B"), WorldB)) return false;
    {
        FRGLSceneManager& B = FRGLSceneManager::GetInstance(WorldB);
        TestEqual(TEXT("a new manager starts with no line"), B.AuditLinesEmitted_ForTest, 0);
        B.MaybeEmitAudit(WorldB, 2.0, Frame0 + 1);
        B.MaybeEmitAudit(WorldB, 2.0, Frame0 + 1);
        TestEqual(TEXT("the new manager counts its own line only"), B.AuditLinesEmitted_ForTest, 1);
    }
    WorldB->DestroyWorld(/*bInformEngineOfWorld=*/false);
    FRGLSceneManager::DestroyInstance(WorldB);
    return true;
}

// ============================================================================
// Task 5: debug hit dump (rgl.Debug.HitDumpDir)
// ============================================================================

// One scan of a 4 m box 15 m in front of a 32-channel lidar, no client listening: the dumped points are all on
// the box, in CARLA world metres with y's sign kept (the sensor sits at y = -3500 m).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditHitDumpRoundTripTest, "CarlaRGL.SceneAudit.HitDumpRoundTrip",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditHitDumpRoundTripTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;

    const FString Dir = FPaths::ConvertRelativePathToFull(
        FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation"), TEXT("RGLHitDumpRoundTrip")));
    IFileManager::Get().DeleteDirectory(*Dir, /*RequireExists=*/false, /*Tree=*/true);
    FStringCVarGuard DirGuard(TEXT("rgl.Debug.HitDumpDir"));
    FCVarGuard EveryGuard(TEXT("rgl.Debug.HitDumpEvery"));
    IConsoleManager::Get().FindConsoleVariable(TEXT("rgl.Debug.HitDumpDir"))->Set(*Dir, ECVF_SetByCode);
    SetCVar(TEXT("rgl.Debug.HitDumpEvery"), 10);   // a session's first scan is dumped whatever N is

    const FVector SensorCm(-200000.f, -350000.f, kZ);   // CARLA (-2000, -3500, 300) m
    AStaticMeshActor* Box = SpawnMeshActor(World, Cube, SensorCm + FVector(1500.f, 0.f, 0.f), EComponentMobility::Static,
                                           NAME_None, FVector(4.f));
    if (!TestNotNull(TEXT("box"), Box)) return false;
    const FBox BoxCm = Box->GetStaticMeshComponent()->Bounds.GetBox();
    const FBox BoxM = FBox(BoxCm.Min * RGLCoord::UE_TO_RGL, BoxCm.Max * RGLCoord::UE_TO_RGL).ExpandBy(0.05);

    FRGLSessionConfig Config;
    Config.LidarDesc.Channels = 32;
    Config.LidarDesc.Range = 5000.f;              // 50 m (cm)
    Config.LidarDesc.PointsPerSecond = 115200;    // 360 points per channel per 0.1 s: 1 degree
    Config.LidarDesc.RotationFrequency = 10.f;
    Config.LidarDesc.UpperFovLimit = 10.f;
    Config.LidarDesc.LowerFovLimit = -10.f;
    Config.LidarDesc.HorizontalFov = 360.f;

    const double T = 7000.0;
    FRGLBackendImpl Backend;
    Backend.SetSimulationTime(T);
    FRGLSessionHandle Handle = Backend.CreateSession(Config, World);
    if (!TestNotNull(TEXT("session"), Handle)) { Box->Destroy(); return false; }

    // Put the box into the scene whatever Update()'s once-per-frame guard decides inside Tick().
    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    SM.RegisterSensor(Handle, SensorCm, FMath::Max(5000.f, Config.LidarDesc.Range + 2000.f), T);
    SM.SyncStaticNow_ForTest(World, T);

    float Angle = 0.f;
    const FTransform SensorTf(FRotator::ZeroRotator, SensorCm);
    Backend.Tick(Handle, SensorTf, 0.1f, Angle, /*bNeedResults=*/false, /*bNeedWorldPoints=*/false);
    Backend.DestroySession(Handle);
    Box->Destroy();
    SM.SyncStaticNow_ForTest(World, T + 0.05);

    TArray<FString> Files;
    IFileManager::Get().FindFiles(Files, *Dir, TEXT(".bin"));
    TestEqual(TEXT("one file for one scan"), Files.Num(), 1);
    if (Files.Num() >= 1)
    {
        TArray<uint8> Bytes;
        TestTrue(TEXT("file read"), FFileHelper::LoadFileToArray(Bytes, *FPaths::Combine(Dir, Files[0])));
        if (TestTrue(TEXT("40-byte header"), Bytes.Num() >= 40))
        {
            uint32 Magic = 0, Count = 0;
            double SimT = 0.0;
            float Pose[6] = {};
            FMemory::Memcpy(&Magic, Bytes.GetData() + 0, 4);
            FMemory::Memcpy(&Count, Bytes.GetData() + 4, 4);
            FMemory::Memcpy(&SimT, Bytes.GetData() + 8, 8);
            FMemory::Memcpy(Pose, Bytes.GetData() + 16, 24);
            TestTrue(FString::Printf(TEXT("magic 0x%08x"), Magic), Magic == 0x52474c48u);
            TestEqual(TEXT("size = 40 + 12 * count"), Bytes.Num(), 40 + 12 * static_cast<int32>(Count));
            TestTrue(FString::Printf(TEXT(">= 50 hits on the box (%u)"), Count), Count >= 50u);
            TestTrue(FString::Printf(TEXT("sim_t %.3f"), SimT), FMath::IsNearlyEqual(SimT, T, 1.0e-6));
            TestTrue(FString::Printf(TEXT("sensor x/y/z in CARLA m, y keeps its sign (%.2f %.2f %.2f)"),
                         static_cast<double>(Pose[0]), static_cast<double>(Pose[1]), static_cast<double>(Pose[2])),
                FMath::IsNearlyEqual(static_cast<double>(Pose[0]), SensorCm.X * 0.01, 0.01) &&
                FMath::IsNearlyEqual(static_cast<double>(Pose[1]), SensorCm.Y * 0.01, 0.01) &&
                FMath::IsNearlyEqual(static_cast<double>(Pose[2]), SensorCm.Z * 0.01, 0.01));
            TestTrue(TEXT("sensor roll/pitch/yaw 0"),
                FMath::Abs(Pose[3]) < 1.0e-3f && FMath::Abs(Pose[4]) < 1.0e-3f && FMath::Abs(Pose[5]) < 1.0e-3f);

            int32 Outside = 0;
            FVector FirstOutside = FVector::ZeroVector;
            for (uint32 i = 0; i < Count && 40 + 12 * (static_cast<int64>(i) + 1) <= Bytes.Num(); ++i)
            {
                float P[3];
                FMemory::Memcpy(P, Bytes.GetData() + 40 + 12 * i, 12);
                const FVector PointM(P[0], P[1], P[2]);
                if (!BoxM.IsInside(PointM))
                {
                    if (Outside == 0) FirstOutside = PointM;
                    ++Outside;
                }
            }
            AddInfo(FString::Printf(TEXT("hits=%u outside=%d box_m=%s first_outside=%s"),
                Count, Outside, *BoxM.ToString(), *FirstOutside.ToString()));
            TestEqual(TEXT("every hit within 5 cm of the box (CARLA m)"), Outside, 0);
        }
    }
    IFileManager::Get().DeleteDirectory(*Dir, /*RequireExists=*/false, /*Tree=*/true);
    return true;
}

// ============================================================================
// 7b: a known culled part that comes into range is not hidden by a waiting queue
// ============================================================================

// World Partition culling (the seam): a part found far away (300 m) by a sync is culled; three parts in R_load but out of
// range wait in the queue (one change per Update). The far part is put 50 m from the sensor with no Update: the audit
// counts it as missing_wp_culled_in_range and missing_in_range although jobs are pending, and changes nothing. A sync with
// its drain registers it (both back to 0). The same for one instance of an ISM; its other far instance is not counted.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditWpLateApproachIsNotMaskedByPendingTest, "CarlaRGL.SceneAudit.WpLateApproachIsNotMaskedByPending",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditWpLateApproachIsNotMaskedByPendingTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(77);
    FWpTestScope W(World, true);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 1);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0920), Base, 14200.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();
    AStaticMeshActor* Late = F.Keep(SpawnMeshActor(World, Cube, Base + FVector(30000.f, 0.f, 0.f), EComponentMobility::Movable));
    for (int32 i = 0; i < 3; ++i)
    {
        F.Keep(SpawnMeshActor(World, Cube, Base + FVector(11000.f + 500.f * i, 2000.f * (i - 1), 0.f), EComponentMobility::Movable));
    }
    AActor* O = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base, { FVector(30000.f, 3000.f, 0.f), FVector(32000.f, 3000.f, 0.f) }, O,
                                                  EComponentMobility::Movable);
    F.Keep(O);
    if (!TestTrue(TEXT("parts"), Late && Ism)) return false;
    // Updates until the interval's sync has found them (budgeted: 1 change per Update).
    for (int32 k = 0; k < 21; ++k)
    {
        F.T += 0.1;
        F.Refresh();
        F.SM.Update_ForTest(World, F.T);
    }
    TestTrue(TEXT("jobs are pending"), F.SM.GetWpPendingJobs_ForTest() > 0);
    TestTrue(TEXT("the far part is a known candidate"), F.SM.IsWpCandidate_ForTest(Late->GetStaticMeshComponent()));
    TestFalse(TEXT("the far part is culled"), F.SM.IsEntityRegistered_ForTest(Late->GetStaticMeshComponent()));

    Late->SetActorLocation(Base + FVector(5000.f, 0.f, 0.f));   // into the range, no Update
    const FRGLSceneManager::FSceneCounts C0 = F.SM.GetSceneCounts_ForTest();
    const int32 Jobs0 = F.SM.GetWpPendingJobs_ForTest();
    const FRGLSceneAuditResult D1 = Diff(R0, F.Audit());
    AddInfo(FRGLSceneManager::FormatAuditLine(77, F.T, F.Audit()));
    TestEqual(TEXT("missing_wp_culled_in_range +1"), D1.MissingWpCulledInRange, 1);
    TestEqual(TEXT("missing_in_range +1"), D1.MissingInRange, 1);
    const FRGLSceneManager::FSceneCounts C1 = F.SM.GetSceneCounts_ForTest();
    TestEqual(TEXT("the audit changed no entity"), C1.Entities, C0.Entities);
    TestEqual(TEXT("the audit changed no mesh"), C1.CachedMeshes, C0.CachedMeshes);
    TestEqual(TEXT("the audit changed no job"), F.SM.GetWpPendingJobs_ForTest(), Jobs0);
    F.Sync();
    const FRGLSceneAuditResult D2 = Diff(R0, F.Audit());
    TestEqual(TEXT("after the sync: missing_wp_culled_in_range 0"), D2.MissingWpCulledInRange, 0);
    TestEqual(TEXT("after the sync: missing_in_range 0"), D2.MissingInRange, 0);

    // One ISM instance moved into the range (the component does not move); the other stays 320 m away.
    FTransform T;
    Ism->GetInstanceTransform(0, T, /*bWorldSpace=*/true);
    T.SetLocation(Base + FVector(5000.f, 3000.f, 0.f));
    Ism->UpdateInstanceTransform(0, T, /*bWorldSpace=*/true, /*bMarkRenderStateDirty=*/true, /*bTeleport=*/true);
    Ism->UpdateBounds();   // what the end of the frame does (the render state update); the test stays in one frame
    const FRGLSceneAuditResult D3 = Diff(R0, F.Audit());
    TestEqual(TEXT("ISM: missing_wp_culled_in_range +1 (the far instance is not counted)"), D3.MissingWpCulledInRange, 1);
    TestEqual(TEXT("ISM: missing_in_range +1"), D3.MissingInRange, 1);
    F.Sync();
    const FRGLSceneAuditResult D4 = Diff(R0, F.Audit());
    TestEqual(TEXT("ISM after the sync: missing_wp_culled_in_range 0"), D4.MissingWpCulledInRange, 0);
    TestEqual(TEXT("ISM after the sync: missing_in_range 0"), D4.MissingInRange, 0);
    return true;
}

// 7b: the audit line shows the culling's queue (wp_pending, current jobs), so that a line whose queue is not done is not
// counted as steady (summarize). Three parts in R_load but out of range, one change per Update: after the sync that found
// them, jobs wait; after a full drain, none.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditWpPendingShownTest, "CarlaRGL.SceneAudit.WpPendingShown",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditWpPendingShownTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(81);
    FWpTestScope W(World, true);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 1);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0921), Base, 14300.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();
    TestEqual(TEXT("nothing waits at first"), R0.WpPending, 0);
    for (int32 i = 0; i < 3; ++i)
    {
        F.Keep(SpawnMeshActor(World, Cube, Base + FVector(11000.f + 500.f * i, 2000.f * (i - 1), 0.f), EComponentMobility::Movable));
    }
    for (int32 k = 0; k < 20 && F.SM.GetWpPendingJobs_ForTest() == 0; ++k)
    {
        F.T += 0.1;
        F.Refresh();
        F.SM.Update_ForTest(World, F.T);
    }
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(81, F.T, R1));
    TestEqual(TEXT("wp_pending = the jobs waiting"), R1.WpPending, F.SM.GetWpPendingJobs_ForTest());
    TestTrue(TEXT("jobs wait"), R1.WpPending > 0);
    TestTrue(TEXT("the line carries wp_pending"), FRGLSceneManager::FormatAuditLine(81, F.T, R1).Contains(
        FString::Printf(TEXT(" wp_pending=%d"), R1.WpPending)));
    F.Sync();
    TestEqual(TEXT("drained: wp_pending 0"), F.Audit().WpPending, 0);
    return true;
}
}
#endif
