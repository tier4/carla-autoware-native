// Copyright (c) 2026 RGL Integration for CARLA.
// Scene synchronization of FRGLSceneManager on the game world of EmptyMap, judged with the scene audit.
// Shared fixture and helpers: RGLSceneTestUtil.h (each test works around its own BaseFor slot).
// First the audit counts the World Partition fixes are judged by: missing_steady (the miss that the last sync
// should already have registered), the ISM part of the ghosts, and spline mesh components in the scene.
// Then the fixes: unregistered ISM components leave the scene at once (P1), adding a level with geometry in some
// sensor's R syncs (P5), the instances of Static ISMs are re-evaluated against the sensor spheres (P3), the name
// rule leaves out sky domes and spheres only (N1), and the mesh cache is keyed by the asset's identity (P2).

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/World.h"
#include "Engine/Level.h"
#include "Engine/StaticMesh.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SplineMeshComponent.h"
#include "UObject/Package.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/GarbageCollection.h"
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

// Review focus 1 (a teleport or respawn makes WP add many cells in one frame): ten distinct levels added before one
// Update give one sync, and the next Update (past the 0.3 s interval) has nothing left to sync for. Nine of them are
// empty transient levels and come first; the last is the persistent level, which holds the cube.
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

    TArray<TStrongObjectPtr<ULevel>> EmptyLevels;   // kept alive until the test ends (the list holds weak pointers)
    for (int32 i = 0; i < 9; ++i)
    {
        EmptyLevels.Emplace(NewObject<ULevel>(GetTransientPackage(), NAME_None, RF_Transient));
    }
    TestNotNull(TEXT("cube"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(6000.f, 0.f, 0.f), EComponentMobility::Static)));
    F.SM.SetSyncInterval(3600.f);
    F.T += 0.5;
    F.Refresh();
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    for (const TStrongObjectPtr<ULevel>& Level : EmptyLevels)
    {
        F.SM.OnLevelAddedToWorld_ForTest(Level.Get(), World);
    }
    F.SM.OnLevelAddedToWorld_ForTest(World->PersistentLevel, World);
    TestEqual(TEXT("ten distinct levels wait"), F.SM.GetPendingAddedLevelCount_ForTest(), 10);
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("ten level events: one sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("the list is empty after the sync"), F.SM.GetPendingAddedLevelCount_ForTest(), 0);
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

// ============================================================================
// P3: the instances of a Static ISM are re-evaluated against the sensor spheres
// ============================================================================

// The instances of a registered Static ISM are not chosen once: instance 0 at 40 m is registered, instance 1 at 200 m
// is out of R; the sensor moves 150 m (instance 1 is now 50 m away, instance 0 110 m behind, both inside R) and a
// sync runs: both are registered and no ISM instance is missing.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncIsmReevaluatedTest, "CarlaRGL.SceneSync.IsmReevaluated",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncIsmReevaluatedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(46);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0040), Base, 9100.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(16000.f, 0.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    TestEqual(TEXT("only instance 0 registered: +1"), Diff(R0, F.Audit()).Registered, 1);

    F.SensorPos = Base + FVector(15000.f, 0.f, 0.f);
    F.Sync();
    const FRGLSceneAuditResult R2 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(46, F.T, R2));
    const FRGLSceneAuditResult D = Diff(R0, R2);
    TestEqual(TEXT("both instances registered: +2"), D.Registered, 2);
    TestEqual(TEXT("the group has 2 entities"), F.SM.GetIsmEntityCount_ForTest(Ism), 2);
    TestEqual(TEXT("nothing missing in range"), D.MissingInRange, 0);
    TestEqual(TEXT("no ISM instance missing in range"), D.MissingIsmInRange, 0);
    TestEqual(TEXT("no ghost"), D.Ghost, 0);
    return true;
}

// An instance that the sensor left behind leaves the RGL scene beyond R x rgl.Scene.UnloadDistanceFactor + 20 m:
// instances at 40 m and 300 m; the sensor moves 250 m, so instance 0 is 210 m away (beyond 140 m) and instance 1 is
// 50 m away. One entity is left, instance 1's (nothing missing, no ghost).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncIsmInstanceDroppedBeyondUnloadRadiusTest, "CarlaRGL.SceneSync.IsmInstanceDroppedBeyondUnloadRadius",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncIsmInstanceDroppedBeyondUnloadRadiusTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(14);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0041), Base, 9200.0);
    FFloatCVarGuard UnloadGuard(TEXT("rgl.Scene.UnloadDistanceFactor"));
    SetFloatCVar(TEXT("rgl.Scene.UnloadDistanceFactor"), 1.0f);   // unload radius = 120 m + 20 m
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(26000.f, 0.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    TestEqual(TEXT("instance 0 registered"), F.SM.GetIsmEntityCount_ForTest(Ism), 1);

    F.SensorPos = Base + FVector(25000.f, 0.f, 0.f);
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(14, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("one entity left"), F.SM.GetIsmEntityCount_ForTest(Ism), 1);
    TestEqual(TEXT("registered +1"), D.Registered, 1);
    TestEqual(TEXT("instance 1 (50 m) is not missing"), D.MissingInRange, 0);
    TestEqual(TEXT("no ghost"), D.Ghost, 0);
    TestEqual(TEXT("no ISM ghost"), D.GhostIsm, 0);
    return true;
}

// M5 / D7: an Update that syncs only queues the ISM groups; the instance changes run on the next Update without a
// sync (cap 0 = unlimited). Instance 0 at 40 m, instance 1 at 200 m; the sensor moves 160 m (a distance sync).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncIsmChangesDeferredOnSyncUpdateTest, "CarlaRGL.SceneSync.IsmChangesDeferredOnSyncUpdate",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncIsmChangesDeferredOnSyncUpdateTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(47);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0042), Base, 9300.0);
    FCVarGuard CapGuard(TEXT("rgl.Scene.MaxIsmInstanceChangesPerUpdate"));
    SetCVar(TEXT("rgl.Scene.MaxIsmInstanceChangesPerUpdate"), 0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(16000.f, 0.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    TestEqual(TEXT("instance 0 registered"), F.SM.GetIsmEntityCount_ForTest(Ism), 1);

    F.SensorPos = Base + FVector(16000.f, 0.f, 0.f);
    F.T += 0.1;
    F.Refresh();
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("a sync ran"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("trigger=distance"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("distance")));
    TestEqual(TEXT("the sync Update changes no instance"), F.SM.GetIsmEntityCount_ForTest(Ism), 1);
    TestEqual(TEXT("the group waits: pending_ism 1"), Diff(R0, F.Audit()).PendingIsm, 1);

    F.SM.SetSyncInterval(3600.f);   // the next Update does not sync (the fixture puts 1 s back)
    F.T += 0.1;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(47, F.T, R1));
    TestEqual(TEXT("no sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("the next Update adds instance 1"), F.SM.GetIsmEntityCount_ForTest(Ism), 2);
    TestEqual(TEXT("pending_ism 0"), Diff(R0, R1).PendingIsm, 0);
    TestEqual(TEXT("nothing missing in range"), Diff(R0, R1).MissingInRange, 0);
    return true;
}

// rgl.Scene.MaxIsmInstanceChangesPerUpdate = 1: three instances come into R (at 200, 205 and 210 m; the sensor moves
// 160 m) and are added one per Update without a sync; the group waits at the head of the queue until it has nothing
// left to change, then leaves it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncIsmChangesCappedPerUpdateTest, "CarlaRGL.SceneSync.IsmChangesCappedPerUpdate",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncIsmChangesCappedPerUpdateTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(17);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0043), Base, 9400.0);
    FCVarGuard CapGuard(TEXT("rgl.Scene.MaxIsmInstanceChangesPerUpdate"));
    SetCVar(TEXT("rgl.Scene.MaxIsmInstanceChangesPerUpdate"), 1);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
        { FVector::ZeroVector, FVector(16000.f, 0.f, 0.f), FVector(16500.f, 0.f, 0.f), FVector(17000.f, 0.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    TestEqual(TEXT("instance 0 registered"), F.SM.GetIsmEntityCount_ForTest(Ism), 1);

    F.SensorPos = Base + FVector(16000.f, 0.f, 0.f);
    auto Step = [&F, World]() { F.T += 0.1; F.Refresh(); F.SM.Update_ForTest(World, F.T); };
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    Step();
    TestEqual(TEXT("a distance sync ran"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("the sync Update changes no instance"), F.SM.GetIsmEntityCount_ForTest(Ism), 1);

    F.SM.SetSyncInterval(3600.f);
    Step();
    TestEqual(TEXT("1st Update without a sync: one instance added"), F.SM.GetIsmEntityCount_ForTest(Ism), 2);
    TestEqual(TEXT("1st Update without a sync: the group still waits"), Diff(R0, F.Audit()).PendingIsm, 1);
    Step();
    Step();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(17, F.T, R1));
    TestEqual(TEXT("3rd Update without a sync: all four registered"), F.SM.GetIsmEntityCount_ForTest(Ism), 4);
    TestEqual(TEXT("3rd Update without a sync: nothing waits"), Diff(R0, R1).PendingIsm, 0);
    TestEqual(TEXT("no other sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("nothing missing in range"), Diff(R0, R1).MissingInRange, 0);
    return true;
}

// A Static ISM whose bounds reach R but none of whose instances do is neither queued nor uploaded: a cylinder ISMC
// 60 m from the sensor with its two instances 150 m to either side of it (162 m from the sensor). The cylinder is used
// by no other test, so an upload would show in the counter.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncIsmWithoutInstanceInRangeDoesNotUploadTest, "CarlaRGL.SceneSync.IsmWithoutInstanceInRangeDoesNotUpload",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncIsmWithoutInstanceInRangeDoesNotUploadTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cylinder = LoadMeshReady(kCylinder); if (!TestNotNull(TEXT("cylinder"), Cylinder)) return false;
    const FVector Base = BaseFor(18);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0044), Base, 9500.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cylinder, Base + FVector(6000.f, 0.f, 0.f),
                                                  { FVector(0.f, -15000.f, 0.f), FVector(0.f, 15000.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    TestTrue(TEXT("the ISMC's bounds reach R"),
             FVector::Dist(Ism->Bounds.Origin, Base) - Ism->Bounds.SphereRadius < kRegistrationCm);
    const int32 Uploads0 = F.SM.MeshUploads_ForTest;
    F.Sync();
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(18, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("no mesh uploaded"), F.SM.MeshUploads_ForTest - Uploads0, 0);
    TestEqual(TEXT("not registered"), D.Registered, 0);
    TestEqual(TEXT("not queued"), D.Pending, 0);
    TestEqual(TEXT("no group"), F.SM.GetIsmEntityCount_ForTest(Ism), 0);
    TestEqual(TEXT("nothing missing in range"), D.MissingInRange, 0);
    return true;
}

// Review focus 3: the instance count of a registered ISM changes at run time. The group is rebuilt at the next sync
// and no old instance index stays in use: 2 instances, then one added 10 m further (3), then instance 0 removed (2).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncIsmInstanceCountChangeReRegistersTest, "CarlaRGL.SceneSync.IsmInstanceCountChangeReRegisters",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncIsmInstanceCountChangeReRegistersTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(19);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0045), Base, 9600.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(0.f, 300.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    TestEqual(TEXT("2 instances registered"), F.SM.GetIsmEntityCount_ForTest(Ism), 2);

    Ism->AddInstance(FTransform(FVector(1000.f, 0.f, 0.f)));   // 50 m from the sensor
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(191, F.T, R1));
    const FRGLSceneAuditResult D1 = Diff(R0, R1);
    TestEqual(TEXT("added: 3 entities"), F.SM.GetIsmEntityCount_ForTest(Ism), 3);
    TestEqual(TEXT("added: registered +3"), D1.Registered, 3);
    TestEqual(TEXT("added: no ghost"), D1.Ghost, 0);
    TestEqual(TEXT("added: nothing missing"), D1.MissingInRange, 0);

    TestTrue(TEXT("instance 0 removed"), Ism->RemoveInstance(0));
    F.Sync();
    const FRGLSceneAuditResult R2 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(192, F.T, R2));
    const FRGLSceneAuditResult D2 = Diff(R0, R2);
    TestEqual(TEXT("removed: 2 entities"), F.SM.GetIsmEntityCount_ForTest(Ism), 2);
    TestEqual(TEXT("removed: registered +2"), D2.Registered, 2);
    TestEqual(TEXT("removed: no ghost"), D2.Ghost, 0);
    TestEqual(TEXT("removed: no stale instance"), D2.StaleInstance, 0);
    TestEqual(TEXT("removed: nothing missing"), D2.MissingInRange, 0);
    return true;
}

// ============================================================================
// N1: the name rule leaves out sky domes and spheres, not every name with "sky" in it
// ============================================================================

// The dome words on the mesh name, case-insensitive; CARLA's skyscraper parts and other "sky" names are no dome.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSkyDomeNamesTest, "CarlaRGL.SceneSync.SkyDomeNames",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSkyDomeNamesTest::RunTest(const FString& Parameters)
{
    const TCHAR* const Domes[] = { TEXT("SM_SkySphere"), TEXT("sm_skydome_01"), TEXT("Sky_Sphere_LOD0"), TEXT("SM_Sky_Dome") };
    const TCHAR* const Others[] = { TEXT("SM_Skysc24_Mid_Wall_A"), TEXT("SM_SkyWheel"), TEXT("Skyscraper_01"), TEXT("SM_Sky"), TEXT("") };
    for (const TCHAR* Name : Domes)
    {
        TestTrue(FString::Printf(TEXT("a sky dome: '%s'"), Name), FRGLSceneManager::IsSkyDomeMeshName(FString(Name)));
    }
    for (const TCHAR* Name : Others)
    {
        TestFalse(FString::Printf(TEXT("no sky dome: '%s'"), Name), FRGLSceneManager::IsSkyDomeMeshName(FString(Name)));
    }
    return true;
}

// A building whose actor name carries "Sky" (like CARLA's shared skyscraper parts, SM_Skysc*) is registered like any
// other mesh: the old rule left it out.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSkyscraperRegisteredTest, "CarlaRGL.SceneSync.SkyscraperRegistered",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSkyscraperRegisteredTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(21);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0050), Base, 9700.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AStaticMeshActor* A = F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static,
                                                FName(TEXT("Skyscraper_01"))));
    if (!TestNotNull(TEXT("building"), A)) return false;
    TestTrue(FString::Printf(TEXT("owner name carries Sky (%s)"), *A->GetName()), A->GetName().Contains(TEXT("Sky")));
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(21, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("registered +1"), D.Registered, 1);
    TestEqual(TEXT("excluded_by_name"), D.ExcludedByName, 0);
    TestEqual(TEXT("not missing"), D.MissingInRange, 0);
    return true;
}

// Review focus 4: a sky mesh on a map nobody checked, under a name the dome words miss, is still left out when the old
// rule's names match and its bounds are wider than 2 km (the size guard). The same name on a cube of ordinary size is
// registered.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncHugeLegacySkyStillExcludedTest, "CarlaRGL.SceneSync.HugeLegacySkyStillExcluded",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncHugeLegacySkyStillExcludedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    TestFalse(TEXT("the mesh name has no dome word"), FRGLSceneManager::IsSkyDomeMeshName(Cube->GetName()));
    const FVector Base = BaseFor(60);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0051), Base, 9800.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AStaticMeshActor* Huge = F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static,
                                                   FName(TEXT("BP_Sky_Test")), FVector(3000.f)));
    if (!TestNotNull(TEXT("huge sky actor"), Huge)) return false;
    const double HugeRadiusCm = Huge->GetStaticMeshComponent()->Bounds.SphereRadius;
    TestTrue(FString::Printf(TEXT("bounds radius above 2 km (%.0f cm)"), HugeRadiusCm), HugeRadiusCm > FRGLSceneManager::HugeSkyRadiusCm);
    F.Sync();
    const FRGLSceneAuditResult D1 = Diff(R0, F.Audit());
    TestEqual(TEXT("huge: excluded_by_name +1"), D1.ExcludedByName, 1);
    TestEqual(TEXT("huge: not registered"), D1.Registered, 0);
    TestEqual(TEXT("huge: not missing"), D1.MissingInRange, 0);

    AStaticMeshActor* Small = F.Keep(SpawnMeshActor(World, Cube, Base + FVector(0.f, 5000.f, 0.f), EComponentMobility::Static,
                                                    FName(TEXT("BP_Sky_Test"))));
    if (!TestNotNull(TEXT("small actor"), Small)) return false;
    TestTrue(FString::Printf(TEXT("same owner name (%s)"), *Small->GetName()), Small->GetName().Contains(TEXT("BP_Sky_Test")));
    F.Sync();
    const FRGLSceneAuditResult R2 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(60, F.T, R2));
    const FRGLSceneAuditResult D2 = Diff(R0, R2);
    TestEqual(TEXT("scale 1: registered +1"), D2.Registered, 1);
    TestEqual(TEXT("scale 1: the huge one stays excluded"), D2.ExcludedByName, 1);
    TestEqual(TEXT("scale 1: not missing"), D2.MissingInRange, 0);
    return true;
}

// The audit's world-wide view of the name rule (the rgl_scene_audit_excluded line): two buildings named Skyscraper_01,
// at 50 m and at 5 km (outside every sphere), are what only the old rule left out.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncExcludedWorldCountsTest, "CarlaRGL.SceneSync.ExcludedWorldCounts",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncExcludedWorldCountsTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(22);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0052), Base, 9900.0);
    F.Sync();
    FRGLSceneAuditExtras E0;
    const FRGLSceneAuditResult R0 = F.SM.AuditScene(World, &E0);

    TestNotNull(TEXT("building at 50 m"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f),
                                                                EComponentMobility::Static, FName(TEXT("Skyscraper_01")))));
    TestNotNull(TEXT("building at 5 km"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(500000.f, 0.f, 0.f),
                                                                EComponentMobility::Static, FName(TEXT("Skyscraper_01")))));
    F.Sync();
    FRGLSceneAuditExtras E1;
    const FRGLSceneAuditResult R1 = F.SM.AuditScene(World, &E1);
    AddInfo(FString::Printf(TEXT("excluded_world=%d excluded_legacy_world=%d legacy_only_meshes=%s huge_meshes=%s excluded=%s"),
        E1.ExcludedWorld, E1.ExcludedLegacyWorld, *FString::Join(E1.LegacyOnlyMeshes, TEXT(";")),
        *FString::Join(E1.HugeMeshes, TEXT(";")), *FString::Join(E1.ExcludedMeshes, TEXT(";"))));
    const int32 LegacyOnly0 = E0.ExcludedLegacyWorld - E0.ExcludedWorld;
    const int32 LegacyOnly1 = E1.ExcludedLegacyWorld - E1.ExcludedWorld;
    TestTrue(FString::Printf(TEXT("the old rule left out at least 2 more (%d)"), LegacyOnly1), LegacyOnly1 >= 2);
    TestEqual(TEXT("both buildings counted, wherever they are"), LegacyOnly1 - LegacyOnly0, 2);
    TestEqual(TEXT("the new rule leaves out neither"), E1.ExcludedWorld - E0.ExcludedWorld, 0);
    TestTrue(TEXT("legacy_only_meshes has Cube"), E1.LegacyOnlyMeshes.Contains(FString(TEXT("Cube"))));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("the 50 m one registered"), D.Registered, 1);
    TestEqual(TEXT("excluded_by_name (inside the spheres)"), D.ExcludedByName, 0);
    return true;
}

// ============================================================================
// P2: the mesh cache is keyed by the asset's identity (FObjectKey), not by its address
// ============================================================================

// L3 (D6 keeps the order inside a sync: the scan, then the removals): an NPC is destroyed and the same model spawned in
// its place between two syncs. The scan registers the new component on the mesh the old entity still holds, so nothing
// is uploaded and nothing waits in the create queue. With rgl.Scene.IdleMeshCacheMax = 0 (set here), removing the old
// entity before the scan would destroy the cube's RGL mesh and upload it again.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSameAssetRespawnReusesLiveMeshTest, "CarlaRGL.SceneSync.SameAssetRespawnReusesLiveMesh",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSameAssetRespawnReusesLiveMeshTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(23);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0060), Base, 10000.0);
    FCVarGuard IdleGuard(TEXT("rgl.Scene.IdleMeshCacheMax"));
    SetCVar(TEXT("rgl.Scene.IdleMeshCacheMax"), 0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    const FVector Loc = Base + FVector(5000.f, 0.f, 0.f);
    AStaticMeshActor* A = F.Keep(SpawnMeshActor(World, Cube, Loc, EComponentMobility::Movable));
    if (!TestNotNull(TEXT("cube A"), A)) return false;
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    TestEqual(TEXT("A registered: movable_registered +1"), Diff(R0, R1).MovableRegistered, 1);

    const int32 Uploads1 = F.SM.MeshUploads_ForTest;
    A->Destroy();
    TestNotNull(TEXT("cube B"), F.Keep(SpawnMeshActor(World, Cube, Loc, EComponentMobility::Movable)));
    F.Sync();
    const FRGLSceneAuditResult R2 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(23, F.T, R2));
    const FRGLSceneAuditResult D = Diff(R1, R2);
    TestEqual(TEXT("the live mesh is reused: no upload"), F.SM.MeshUploads_ForTest - Uploads1, 0);
    TestEqual(TEXT("B replaces A: registered +0"), D.Registered, 0);
    TestEqual(TEXT("B replaces A: movable_registered +0"), D.MovableRegistered, 0);
    TestEqual(TEXT("no ghost"), D.Ghost, 0);
    TestEqual(TEXT("cache_stale"), D.CacheStale, 0);
    TestEqual(TEXT("nothing waits in the create queue"), D.Pending, 0);
    return true;
}

// P2 (best effort): A is destroyed and garbage collected with no Update in between, so A's entity still holds the cache
// entry of A's mesh M1 when another mesh, M2, takes M1's address. B, given M2, must get an upload of its own, not M1's
// shape (a 1 m box where M2 is a 2 m one). B is spawned without a mesh before the collections, so its component cannot
// take the address of A's component (the entity map is keyed by component address, outside this test). The rounds go on
// until the allocator hands M1's address to a new mesh, at most 32; without a reuse the test warns and passes. In each
// round M1 must be gone after the collection (checked by its key), and up to 16 meshes are made after it, because the
// same collection frees other blocks of the allocator too; the one at M1's address, if any, is M2. M1 and M2 are
// transient box meshes, not copies of engine meshes (see NewTransientBoxMesh). rgl.Scene.IdleMeshCacheMax = 0: an RGL
// mesh is destroyed when its last entity leaves.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncGcReusedAddressGetsFreshMeshTest, "CarlaRGL.SceneSync.GcReusedAddressGetsFreshMesh",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncGcReusedAddressGetsFreshMeshTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    const FVector Base = BaseFor(24);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0061), Base, 10100.0);
    FCVarGuard IdleGuard(TEXT("rgl.Scene.IdleMeshCacheMax"));
    SetCVar(TEXT("rgl.Scene.IdleMeshCacheMax"), 0);
    const FVector Loc = Base + FVector(5000.f, 0.f, 0.f);
    AStaticMeshActor* B = F.Keep(SpawnMeshActor(World, nullptr, Loc, EComponentMobility::Movable));
    if (!TestNotNull(TEXT("actor B"), B)) return false;
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    constexpr int32 MaxRounds = 32;
    constexpr int32 MaxCandidates = 16;
    int32 Collected = 0;
    for (int32 Round = 0; Round < MaxRounds; ++Round)
    {
        UStaticMesh* M1 = NewTransientBoxMesh(TEXT("RGLGcReuseM1"), 100.f);
        if (!TestNotNull(TEXT("M1"), M1)) return false;
        AStaticMeshActor* A = SpawnMeshActor(World, M1, Loc, EComponentMobility::Movable);   // not kept: collected below
        if (!TestNotNull(TEXT("actor A"), A)) return false;
        const int32 Uploads0 = F.SM.MeshUploads_ForTest;
        F.Sync();
        if (F.SM.MeshUploads_ForTest == Uploads0)
        {
            A->Destroy();
            AddWarning(TEXT("transient box mesh not uploadable"));
            return true;
        }
        const UPTRINT M1Address = reinterpret_cast<UPTRINT>(M1);
        const FObjectKey M1Key(M1);
        A->Destroy();                              // no Update: A's entity keeps M1's cache entry referenced
        CollectGarbage(GARBAGE_COLLECTION_KEEPFLAGS);
        if (M1Key.ResolveObjectPtrEvenIfGarbage() != nullptr)
        {
            F.Sync();                              // M1 survived the collection; A's entity leaves before the next round
            continue;
        }
        ++Collected;
        UStaticMesh* M2 = nullptr;
        int32 Candidate = 0;
        for (; Candidate < MaxCandidates && !M2; ++Candidate)
        {
            UStaticMesh* Mesh = NewNamedTransientMesh(TEXT("RGLGcReuseM2"));   // no body setup yet: one allocation each
            if (reinterpret_cast<UPTRINT>(Mesh) == M1Address)
            {
                M2 = Mesh;
            }
        }
        if (!M2)
        {
            F.Sync();                              // A's entity leaves before the next round
            continue;
        }
        AddBoxCollision(M2, 200.f);
        AddInfo(FString::Printf(TEXT("gc_reuse reused=1 round=%d candidate=%d collected_rounds=%d"), Round, Candidate - 1,
                                Collected));
        TestTrue(TEXT("B takes M2"), B->GetStaticMeshComponent()->SetStaticMesh(M2));
        const int32 Uploads1 = F.SM.MeshUploads_ForTest;
        F.Sync();
        const FRGLSceneAuditResult R1 = F.Audit();
        AddInfo(FRGLSceneManager::FormatAuditLine(24, F.T, R1));
        const FRGLSceneAuditResult D = Diff(R0, R1);
        TestEqual(TEXT("M2 gets its own upload: +1"), F.SM.MeshUploads_ForTest - Uploads1, 1);
        TestEqual(TEXT("mesh_mismatch"), D.MeshMismatch, 0);
        TestEqual(TEXT("cache_stale"), D.CacheStale, 0);
        TestEqual(TEXT("B registered and A gone: registered +1"), D.Registered, 1);
        TestEqual(TEXT("no ghost"), D.Ghost, 0);
        return true;
    }
    AddWarning(FString::Printf(TEXT("gc_reuse reused=0 rounds=%d collected_rounds=%d candidates=%d: no address reuse"),
                               MaxRounds, Collected, MaxCandidates));
    return true;
}

// F14 (deterministic): an idle cache entry whose key no longer resolves (its asset is marked as garbage) counts as
// cache_stale until the next sync's purge destroys it; an idle entry whose asset lives stays. rgl.Scene.IdleMeshCacheMax
// is raised so that the purge, not the idle cap, decides.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncStaleCacheKeyPurgedTest, "CarlaRGL.SceneSync.StaleCacheKeyPurged",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncStaleCacheKeyPurgedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    const FVector Base = BaseFor(34);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0062), Base, 10200.0);
    // Declared after the fixture: restored first, so the fixture's last sync trims the live entry again.
    FCVarGuard IdleGuard(TEXT("rgl.Scene.IdleMeshCacheMax"));
    SetCVar(TEXT("rgl.Scene.IdleMeshCacheMax"), 16);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();
    const FRGLSceneManager::FSceneCounts C0 = F.SM.GetSceneCounts_ForTest();

    TStrongObjectPtr<UStaticMesh> Live(NewNamedTransientMesh(TEXT("RGLCacheKeyLive")));
    UStaticMesh* Dead = NewNamedTransientMesh(TEXT("RGLCacheKeyDead"));
    F.SM.AddIdleMeshEntry_ForTest(Live.Get());
    F.SM.AddIdleMeshEntry_ForTest(Dead);
    const FRGLSceneManager::FSceneCounts C1 = F.SM.GetSceneCounts_ForTest();
    TestEqual(TEXT("two idle entries: cached meshes +2"), C1.CachedMeshes - C0.CachedMeshes, 2);
    TestEqual(TEXT("two idle entries: idle meshes +2"), C1.IdleMeshes - C0.IdleMeshes, 2);
    TestEqual(TEXT("both keys resolve: cache_stale +0"), Diff(R0, F.Audit()).CacheStale, 0);

    Dead->MarkAsGarbage();
    const FRGLSceneAuditResult R2 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(34, F.T, R2));
    TestEqual(TEXT("the dead key: cache_stale +1"), Diff(R0, R2).CacheStale, 1);

    F.Sync();
    const FRGLSceneAuditResult R3 = F.Audit();
    const FRGLSceneManager::FSceneCounts C3 = F.SM.GetSceneCounts_ForTest();
    TestEqual(TEXT("after the purge: cache_stale +0"), Diff(R0, R3).CacheStale, 0);
    TestEqual(TEXT("after the purge: the live entry stays (cached meshes +1)"), C3.CachedMeshes - C0.CachedMeshes, 1);
    TestEqual(TEXT("after the purge: idle meshes +1"), C3.IdleMeshes - C0.IdleMeshes, 1);
    return true;
}
}
#endif
