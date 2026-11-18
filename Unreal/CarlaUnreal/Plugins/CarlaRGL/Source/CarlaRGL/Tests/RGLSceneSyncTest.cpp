// Copyright (c) 2026 RGL Integration for CARLA.
// Scene synchronization of FRGLSceneManager on the game world of EmptyMap, judged with the scene audit.
// Shared fixture and helpers: RGLSceneTestUtil.h (each test works around its own BaseFor slot).
// First the audit counts the World Partition fixes are judged by: missing_steady (the miss that the last sync
// should already have registered), the ISM part of the ghosts, and spline mesh components in the scene.
// Then the fixes: unregistered ISM components leave the scene at once (P1), and adding a level with geometry in some
// sensor's R syncs (P5).

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/World.h"
#include "Engine/Level.h"
#include "Engine/StaticMesh.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SplineMeshComponent.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLSceneManager.h"
#include "RGLSceneTestUtil.h"

namespace RGLSceneSyncTests
{
using namespace RGLSceneTestUtil;

// missing_steady counts a miss only inside both R spheres: around the sensor at the last sync (what that sync had
// to register) and around the sensor now (what is still needed). The sensor moves 23 m (below the 24 m distance
// trigger, and more than the 20 m between R and the range) with no Update, then three cubes appear:
// A 110 m behind the last sync point (inside the last sync's R, 133 m from the sensor = outside its current R),
// B 50 m ahead (inside both), and C whose bounds start 122 m ahead of the last sync point (outside the last sync's
// R, 99 m from the sensor = inside its current R and its range): only B is a steady miss. An audit that used the
// current R alone would count C too; one that used the last sync's R alone would count A.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncMissingSteadyNeedsBothSpheresTest, "CarlaRGL.SceneSync.MissingSteadyNeedsBothSpheres",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncMissingSteadyNeedsBothSpheresTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(31);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0020), Base, 8100.0);
    F.Sync();                                    // the last sync: R = 120 m around P0 = Base

    const float MoveCm = 2300.f;
    TestTrue(TEXT("the move stays below the distance trigger (0.2 R)"), MoveCm < 0.2f * kRegistrationCm);
    TestTrue(TEXT("the move is larger than the gap between R and the range"), MoveCm > kRegistrationCm - kRangeCm);
    F.SensorPos = Base + FVector(MoveCm, 0.f, 0.f);
    F.Refresh();                                 // no Update: the last sync's sphere stays at P0
    const FRGLSceneAuditResult R0 = F.Audit();

    const float BoundsRadiusCm = static_cast<float>(Cube->GetBounds().SphereRadius);
    TestNotNull(TEXT("cube A"), F.Keep(SpawnMeshActor(World, Cube, Base - FVector(11000.f, 0.f, 0.f), EComponentMobility::Static)));
    TestNotNull(TEXT("cube B"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static)));
    TestNotNull(TEXT("cube C"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(12200.f + BoundsRadiusCm, 0.f, 0.f),
                                                      EComponentMobility::Static)));
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(31, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("A and B inside the last sync's R (C is not): missing_sync +2"), D.MissingSync, 2);
    TestEqual(TEXT("only B inside both R spheres: missing_steady +1"), D.MissingSteady, 1);
    TestEqual(TEXT("B and C inside the range: missing_in_range +2"), D.MissingInRange, 2);
    TestEqual(TEXT("not ISM instances"), D.MissingIsmSync, 0);
    TestEqual(TEXT("not registered"), D.Registered, 0);
    return true;
}

// A spline mesh component is a static mesh component: the sync registers it (with the undeformed mesh), and the
// audit counts it as spline_registered.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSplineMeshCountedTest, "CarlaRGL.SceneSync.SplineMeshCounted",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSplineMeshCountedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(32);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0021), Base, 8200.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    // Mesh and mobility are set while the component is unregistered (see SpawnMeshActor).
    AActor* Owner = F.Keep(World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity));
    if (!TestNotNull(TEXT("owner"), Owner)) return false;
    USplineMeshComponent* Spline = NewObject<USplineMeshComponent>(Owner);
    Spline->SetMobility(EComponentMobility::Static);
    Spline->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Spline->SetStaticMesh(Cube);
    Owner->SetRootComponent(Spline);
    Spline->SetRelativeLocation_Direct(Base + FVector(5000.f, 0.f, 0.f));
    Spline->RegisterComponent();
    TestTrue(TEXT("spline mesh registered with the cube"), Spline->IsRegistered() && Spline->GetStaticMesh() == Cube);

    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(32, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("registered +1"), D.Registered, 1);
    TestEqual(TEXT("spline_registered +1"), D.SplineRegistered, 1);
    TestEqual(TEXT("not missing"), D.MissingInRange, 0);
    return true;
}

// A registered ISM whose component is unregistered, with no Update since: both instances are ghosts, and both are
// counted in the ISM part of the ghosts (in range: 40 m from the sensor).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncGhostIsmCountedTest, "CarlaRGL.SceneSync.GhostIsmCounted",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncGhostIsmCountedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    bool bWasCompiling = false;
    UStaticMesh* Cube = LoadMeshReady(kCube, &bWasCompiling); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    AddInfo(FString::Printf(TEXT("cube_was_compiling=%d"), bWasCompiling ? 1 : 0));
    const FVector Base = BaseFor(33);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0022), Base, 8300.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(0.f, 300.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FString::Printf(TEXT("ism bounds origin=%s radius=%.1f instances=%d "), *Ism->Bounds.Origin.ToString(),
        Ism->Bounds.SphereRadius, Ism->GetInstanceCount()) + FRGLSceneManager::FormatAuditLine(330, F.T, R1));
    TestEqual(TEXT("2 instances registered"), Diff(R0, R1).Registered, 2);

    Ism->UnregisterComponent();
    const FRGLSceneAuditResult R2 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(33, F.T, R2));
    const FRGLSceneAuditResult D = Diff(R1, R2);
    TestEqual(TEXT("ghost +2"), D.Ghost, 2);
    TestEqual(TEXT("ghost_ism +2"), D.GhostIsm, 2);
    TestEqual(TEXT("ghost_in_range +2"), D.GhostInRange, 2);
    TestEqual(TEXT("ghost_ism_in_range +2"), D.GhostIsmInRange, 2);
    TestEqual(TEXT("still in the RGL scene"), D.Registered, 0);
    return true;
}

// ============================================================================
// P1: unregistered ISM components leave the scene at once
// ============================================================================

// An ISM whose component is not registered (its level is being streamed out, or the owner unregistered it) is no
// candidate: a sync does not register its instances, so they never become ghosts.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncUnregisteredIsmNotRegisteredTest, "CarlaRGL.SceneSync.UnregisteredIsmNotRegistered",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncUnregisteredIsmNotRegisteredTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(10);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0030), Base, 8400.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(0.f, 300.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    Ism->UnregisterComponent();
    TestFalse(TEXT("the ISMC is unregistered"), Ism->IsRegistered());
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(10, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("not registered"), D.Registered, 0);
    TestEqual(TEXT("no ISM ghost"), D.GhostIsm, 0);
    TestEqual(TEXT("no ghost"), D.Ghost, 0);
    return true;
}

// A registered ISM whose component is unregistered leaves the RGL scene at the next Update, sync due or not (the
// sync timer is stretched so that no sync runs): no ghost waits for the next sync.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncIsmGhostRemovedOnUpdateTest, "CarlaRGL.SceneSync.IsmGhostRemovedOnUpdate",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncIsmGhostRemovedOnUpdateTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(11);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0031), Base, 8500.0);
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

    Ism->UnregisterComponent();
    F.SM.SetSyncInterval(3600.f);   // no time-triggered sync in this step (the fixture puts 1 s back)
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    F.T += 0.1;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    const FRGLSceneAuditResult R2 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(11, F.T, R2));
    const FRGLSceneAuditResult D = Diff(R1, R2);
    TestEqual(TEXT("no sync ran"), F.SM.SyncCount_ForTest - Syncs0, 0);
    TestEqual(TEXT("both instances left the RGL scene: registered -2"), D.Registered, -2);
    TestEqual(TEXT("no ghost"), D.Ghost, 0);
    TestEqual(TEXT("no ISM ghost"), D.GhostIsm, 0);
    return true;
}

// Review focus 2 (a cell streams out and straight back in, before GC): the same ISMC unregistered and registered
// again between two Updates is neither duplicated nor left as a ghost.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncIsmReRegisteredNoDuplicateTest, "CarlaRGL.SceneSync.IsmReRegisteredNoDuplicate",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncIsmReRegisteredNoDuplicateTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(15);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0032), Base, 8800.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(0.f, 300.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    TestEqual(TEXT("2 instances registered"), Diff(R0, F.Audit()).Registered, 2);

    Ism->UnregisterComponent();
    Ism->RegisterComponent();
    TestTrue(TEXT("the ISMC is registered again"), Ism->IsRegistered());
    F.SM.SetSyncInterval(3600.f);
    F.T += 0.1;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(15, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("registered once: +2"), D.Registered, 2);
    TestEqual(TEXT("no ghost"), D.Ghost, 0);
    TestEqual(TEXT("no ISM ghost"), D.GhostIsm, 0);
    TestEqual(TEXT("nothing missing"), D.MissingInRange, 0);
    return true;
}

// ============================================================================
// P5: a sync when a level with geometry in some sensor's R is added
// ============================================================================

// A cube appears after the sync and its level is added: the next Update syncs ("level") although no other trigger
// is due, and the cube is registered.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncLevelAddedForcesSyncTest, "CarlaRGL.SceneSync.LevelAddedForcesSync",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncLevelAddedForcesSyncTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(12);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0033), Base, 8600.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    TestNotNull(TEXT("cube"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(6000.f, 0.f, 0.f), EComponentMobility::Static)));
    TestEqual(TEXT("before: the cube is missing"), Diff(R0, F.Audit()).MissingInRange, 1);
    F.SM.SetSyncInterval(3600.f);   // only the level trigger can sync here
    F.T += 0.5;
    F.Refresh();
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    F.SM.OnLevelAddedToWorld_ForTest(World->PersistentLevel, World);
    F.SM.Update_ForTest(World, F.T);
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(12, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("one sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("trigger=level"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("level")));
    TestEqual(TEXT("the cube is registered: +1"), D.Registered, 1);
    TestEqual(TEXT("nothing missing"), D.MissingInRange, 0);
    return true;
}

// D5: a level-triggered sync waits until 0.3 s (sim) have passed since the previous sync; the level added in
// between is kept and syncs once the interval is over (checked 1 ms past the limit: the times are not exact).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncLevelSyncMinIntervalTest, "CarlaRGL.SceneSync.LevelSyncMinInterval",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncLevelSyncMinIntervalTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(12);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0034), Base, 8650.0);
    F.Sync();
    F.SM.SetSyncInterval(3600.f);

    // a first level sync (as in LevelAddedForcesSync)
    TestNotNull(TEXT("cube A"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(6000.f, 0.f, 0.f), EComponentMobility::Static)));
    F.T += 0.5;
    F.Refresh();
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    F.SM.OnLevelAddedToWorld_ForTest(World->PersistentLevel, World);
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("the first level sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    const double TSync = F.T;
    const FRGLSceneAuditResult R0 = F.Audit();

    // a second level 0.1 s later: kept, no sync
    TestNotNull(TEXT("cube B"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(0.f, 6000.f, 0.f), EComponentMobility::Static)));
    F.SM.OnLevelAddedToWorld_ForTest(World->PersistentLevel, World);
    F.T = TSync + 0.1;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("+0.1 s: no sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("+0.1 s: cube B still missing"), Diff(R0, F.Audit()).MissingInRange, 1);

    // 0.301 s after the first sync: the kept level syncs
    F.T = TSync + 0.301;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(121, F.T, R1));
    TestEqual(TEXT("+0.301 s: one more sync"), F.SM.SyncCount_ForTest - Syncs0, 2);
    TestEqual(TEXT("+0.301 s: trigger=level"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("level")));
    TestEqual(TEXT("+0.301 s: cube B registered"), Diff(R0, R1).Registered, 1);
    TestEqual(TEXT("+0.301 s: nothing missing"), Diff(R0, R1).MissingInRange, 0);
    return true;
}

// Review focus 1 (a teleport or respawn makes WP add many cells in one frame): ten level events before one Update
// give one sync, and the next Update (past the 0.3 s interval) has nothing left to sync for.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncManyLevelAddsOneSyncTest, "CarlaRGL.SceneSync.ManyLevelAddsOneSync",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncManyLevelAddsOneSyncTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(13);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0035), Base, 8700.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    TestNotNull(TEXT("cube"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(6000.f, 0.f, 0.f), EComponentMobility::Static)));
    F.SM.SetSyncInterval(3600.f);
    F.T += 0.5;
    F.Refresh();
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    for (int32 i = 0; i < 10; ++i)
    {
        F.SM.OnLevelAddedToWorld_ForTest(World->PersistentLevel, World);
    }
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("ten level events: one sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("the cube is registered"), Diff(R0, F.Audit()).Registered, 1);
    F.T += 0.5;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("the next Update: still one sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    return true;
}

// A level without static meshes in any sensor's R does not sync: the sensor is 5 km above this test's base, where
// a cube sits that would be in R of a sensor at the base.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncIrrelevantLevelAddDoesNotSyncTest, "CarlaRGL.SceneSync.IrrelevantLevelAddDoesNotSync",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncIrrelevantLevelAddDoesNotSyncTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(40);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0036), Base + FVector(0.f, 0.f, 500000.f), 8900.0);
    F.Sync();   // F10: the sensor's first sync, so the Update below has no "init" to run

    TestNotNull(TEXT("cube"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(6000.f, 0.f, 0.f), EComponentMobility::Static)));
    F.SM.SetSyncInterval(3600.f);
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    F.SM.OnLevelAddedToWorld_ForTest(World->PersistentLevel, World);
    F.T += 0.5;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("no sync"), F.SM.SyncCount_ForTest - Syncs0, 0);
    return true;
}
}
#endif
