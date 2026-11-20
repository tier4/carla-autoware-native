// Copyright (c) 2026 RGL Integration for CARLA.
// Scene synchronization of FRGLSceneManager on the game world of EmptyMap, judged with the scene audit.
// Shared fixture and helpers: RGLSceneTestUtil.h (each test works around its own BaseFor slot).
// First the audit counts the World Partition fixes are judged by: missing_steady (the miss that the last sync
// should already have registered), the ISM part of the ghosts, and spline mesh components in the scene.
// Then the fixes: unregistered ISM components leave the scene at once (P1), adding a level with geometry in some
// sensor's R syncs (P5), the instances of Static ISMs are re-evaluated against the sensor spheres (P3), the name
// rule leaves out sky domes and spheres only (N1), and the mesh cache is keyed by the asset's identity (P2).
// Then the warning for a sensor whose range is not streamed in on a World Partition map (P4). Last, the lighter sync
// (P7): the sky-name verdict remembered per mesh, and the sync's cost on its line. At the end, the World Partition
// culling of non-Static parts (7b): the policy and its seam, the original rule elsewhere, the distance rule per part and
// per ISM instance, the protected parts, approaching parts and sensors, the budget, the contracts and the deadline.

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/Parse.h"
#include "Engine/World.h"
#include "Engine/Level.h"
#include "Engine/StaticMesh.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Components/WorldPartitionStreamingSourceComponent.h"
#include "WorldPartition/WorldPartitionStreamingSource.h"
#include "UObject/Package.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/GarbageCollection.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/RotatingMovementComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
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

// ============================================================================
// P4: one rgl_scene_stream_warning per sensor whose range stays not streamed in on a World Partition map
// ============================================================================

// EmptyMap is not a World Partition map: the check that runs after each sync prints nothing, however long the sensor
// has been there. Three distance-triggered syncs 6 s (sim) apart: the last one is 12 s after the first, beyond the 5 s a
// World Partition map would wait.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncNoStreamWarningOnNonWpWorldTest, "CarlaRGL.SceneSync.NoStreamWarningOnNonWpWorld",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncNoStreamWarningOnNonWpWorldTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    TestTrue(TEXT("EmptyMap has no World Partition"), World->GetWorldPartition() == nullptr);
    const FVector Base = BaseFor(25);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0070), Base, 10300.0);
    F.SM.SetSyncInterval(1.0f);
    F.Sync();
    const int32 Warnings0 = F.SM.StreamWarnings_ForTest;
    const int32 Syncs0 = F.SM.SyncCount_ForTest;

    const float StepCm = 3000.f;
    TestTrue(TEXT("each step is beyond the distance trigger (0.2 R)"), StepCm > 0.2f * kRegistrationCm);
    for (int32 Step = 0; Step < 3; ++Step)
    {
        F.T += 6.0;
        F.SensorPos += FVector(StepCm, 0.f, 0.f);
        F.Refresh();
        F.SM.Update_ForTest(World, F.T);
    }
    TestEqual(TEXT("three syncs ran (the check runs after each)"), F.SM.SyncCount_ForTest - Syncs0, 3);
    TestEqual(TEXT("no stream warning on a map without World Partition"), F.SM.StreamWarnings_ForTest - Warnings0, 0);
    return true;
}

// The checks below go through the seam CheckStreamCoverage_ForTest, which gives each check the World Partition answer
// (EmptyMap cannot stream). The fixture's sensor has no parent: nothing explains its uncovered range. The check times
// are whole seconds, so "5 s after" is exact.
static void CheckStreamAt(FAuditFixture& F, double At, bool bIsWp, bool bCovered)
{
    F.T = At;
    F.Refresh();
    F.SM.CheckStreamCoverage_ForTest(At, bIsWp, bCovered);
}

// Not streamed in since T0: no line at T0 + 4.9 s, one line at T0 + 5 s, still one at T0 + 20 s, and still one after
// the range is covered and uncovered again (once per sensor).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncStreamWarningOnceAfterFiveSecondsTest, "CarlaRGL.SceneSync.StreamWarningOnceAfterFiveSeconds",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncStreamWarningOnceAfterFiveSecondsTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    const FVector Base = BaseFor(35);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0071), Base, 10400.0);
    F.Sync();   // the sync also evicts sensors that other tests left behind (stale)
    const int32 W0 = F.SM.StreamWarnings_ForTest;
    const double T0 = 10401.0;

    CheckStreamAt(F, T0, true, false);
    TestEqual(TEXT("not streamed in at T0: no line yet"), F.SM.StreamWarnings_ForTest - W0, 0);
    CheckStreamAt(F, T0 + 4.9, true, false);
    TestEqual(TEXT("4.9 s not streamed in: no line"), F.SM.StreamWarnings_ForTest - W0, 0);
    CheckStreamAt(F, T0 + 5.0, true, false);
    TestEqual(TEXT("5 s not streamed in: one line"), F.SM.StreamWarnings_ForTest - W0, 1);
    CheckStreamAt(F, T0 + 20.0, true, false);
    TestEqual(TEXT("20 s not streamed in: still one line"), F.SM.StreamWarnings_ForTest - W0, 1);
    CheckStreamAt(F, T0 + 21.0, true, true);
    CheckStreamAt(F, T0 + 22.0, true, false);
    CheckStreamAt(F, T0 + 40.0, true, false);
    TestEqual(TEXT("covered, then 18 s not streamed in again: still one line (once per sensor)"),
              F.SM.StreamWarnings_ForTest - W0, 1);
    return true;
}

// The 5 s must be continuous: a check that finds the range streamed in starts the count again. Nothing is counted on a
// map without World Partition (bIsWp = false).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncStreamWarningClockRestartsWhenCoveredTest, "CarlaRGL.SceneSync.StreamWarningClockRestartsWhenCovered",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncStreamWarningClockRestartsWhenCoveredTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    const FVector Base = BaseFor(36);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0072), Base, 10500.0);
    F.Sync();
    const int32 W0 = F.SM.StreamWarnings_ForTest;
    const double T0 = 10501.0;

    CheckStreamAt(F, T0, false, false);
    CheckStreamAt(F, T0 + 20.0, false, false);
    TestEqual(TEXT("not a World Partition map: no line after 20 s"), F.SM.StreamWarnings_ForTest - W0, 0);

    CheckStreamAt(F, T0 + 21.0, true, false);
    CheckStreamAt(F, T0 + 25.0, true, true);    // covered: the count starts again
    CheckStreamAt(F, T0 + 27.0, true, false);
    CheckStreamAt(F, T0 + 31.9, true, false);   // 10.9 s since T0 + 21, but 4.9 s since T0 + 27
    TestEqual(TEXT("4.9 s since the range was last streamed in: no line"), F.SM.StreamWarnings_ForTest - W0, 0);
    CheckStreamAt(F, T0 + 32.0, true, false);
    TestEqual(TEXT("5 s since the range was last streamed in: one line"), F.SM.StreamWarnings_ForTest - W0, 1);
    return true;
}

// Time during which the parent's own streaming source holds the whole range sphere is streaming in progress (the ring
// loads it; the default 3 km hero ring needs more than 5 s at start-up), not a configuration the hint can fix: it does
// not count. A ring smaller than the range does not hold it, and the count runs as for a sensor without a parent.
// The source is not registered: EmptyMap has no World Partition subsystem to take it, and only its shapes are read.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncStreamWarningNotWhileParentRingHoldsRangeTest, "CarlaRGL.SceneSync.StreamWarningNotWhileParentRingHoldsRange",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncStreamWarningNotWhileParentRingHoldsRangeTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    const FVector Base = BaseFor(37);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0073), Base, 10600.0);
    AStaticMeshActor* Parent = F.Keep(SpawnMeshActor(World, nullptr, Base, EComponentMobility::Movable));
    if (!TestNotNull(TEXT("parent actor"), Parent)) return false;
    UWorldPartitionStreamingSourceComponent* Source = NewObject<UWorldPartitionStreamingSourceComponent>(Parent);
    FStreamingSourceShape Shape;
    Shape.bUseGridLoadingRange = false;
    Shape.Radius = 300000.f;   // 3 km, like the default hero ring
    Source->Shapes.Add(Shape);
    TestTrue(TEXT("the parent carries the source"), Parent->FindComponentByClass<UWorldPartitionStreamingSourceComponent>() == Source);
    F.SM.SetSensorAuditParent(F.Sensor, Parent);
    F.Sync();
    const int32 W0 = F.SM.StreamWarnings_ForTest;
    const double T0 = 10601.0;

    CheckStreamAt(F, T0, true, false);
    CheckStreamAt(F, T0 + 20.0, true, false);
    TestEqual(TEXT("the 3 km ring holds the 100 m range: 20 s not streamed in, no line"), F.SM.StreamWarnings_ForTest - W0, 0);

    Source->Shapes[0].Radius = 5000.f;   // 50 m: smaller than the range
    CheckStreamAt(F, T0 + 21.0, true, false);
    CheckStreamAt(F, T0 + 25.9, true, false);
    TestEqual(TEXT("50 m ring: 4.9 s not streamed in, no line"), F.SM.StreamWarnings_ForTest - W0, 0);
    CheckStreamAt(F, T0 + 26.0, true, false);
    TestEqual(TEXT("50 m ring: 5 s not streamed in, one line"), F.SM.StreamWarnings_ForTest - W0, 1);
    return true;
}

// The warning line: one grep-friendly key=value line, documented in README_RGL.md.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncStreamWarningLineFormatTest, "CarlaRGL.SceneSync.StreamWarningLineFormat",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncStreamWarningLineFormatTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("with a parent"),
              FRGLSceneManager::FormatStreamWarningLine(12.3456, reinterpret_cast<const void*>(0x5A0074), 200.0f,
                                                        TEXT("BP_Car_C_1"), 200.0f),
              FString(TEXT("rgl_scene_stream_warning t=12.346 sensor=0x5a0074 range_m=200 parent=BP_Car_C_1 "
                           "parent_stream_radius_m=200 hint=attach_the_sensor_to_a_hero_vehicle_or_raise_tile_stream_distance")));
    TestEqual(TEXT("without a parent"),
              FRGLSceneManager::FormatStreamWarningLine(7.0, reinterpret_cast<const void*>(0x5A0075), 100.0f, FString(), -1.0f),
              FString(TEXT("rgl_scene_stream_warning t=7.000 sensor=0x5a0075 range_m=100 parent=none "
                           "parent_stream_radius_m=-1 hint=attach_the_sensor_to_a_hero_vehicle_or_raise_tile_stream_distance")));
    return true;
}

// ============================================================================
// P7: a lighter static sync (the sky-name verdict is remembered per mesh; the scan's arrays are reused)
// ============================================================================

// The name rule's verdict belongs to the mesh, not to the component: a Movable cube is registered, leaves the scene when
// its mesh becomes a sky sphere, and comes back when the cube returns. A transient mesh named SM_SkySphere stands in for
// the engine's asset (no dependency on engine content; it is never uploaded).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSkyVerdictFollowsMeshSwapTest, "CarlaRGL.SceneSync.SkyVerdictFollowsMeshSwap",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSkyVerdictFollowsMeshSwapTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    TStrongObjectPtr<UStaticMesh> Sky(NewNamedTransientMesh(TEXT("SM_SkySphere")));
    if (!TestNotNull(TEXT("sky sphere mesh"), Sky.Get())) return false;
    TestTrue(FString::Printf(TEXT("a dome word in the mesh name (%s)"), *Sky->GetName()), FRGLSceneManager::IsSkyDomeMeshName(Sky->GetName()));
    const FVector Base = BaseFor(26);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0080), Base, 11000.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AStaticMeshActor* A = F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Movable));
    if (!TestNotNull(TEXT("movable cube"), A)) return false;
    UStaticMeshComponent* Comp = A->GetStaticMeshComponent();
    F.Sync();
    const FRGLSceneAuditResult D1 = Diff(R0, F.Audit());
    TestEqual(TEXT("cube: registered +1"), D1.Registered, 1);
    TestEqual(TEXT("cube: excluded_by_name +0"), D1.ExcludedByName, 0);

    Comp->SetStaticMesh(Sky.Get());
    if (!TestTrue(TEXT("the sky mesh is applied"), Comp->GetStaticMesh() == Sky.Get())) return false;
    F.Sync();
    const FRGLSceneAuditResult R2 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(26, F.T, R2));
    const FRGLSceneAuditResult D2 = Diff(R0, R2);
    TestEqual(TEXT("sky sphere: registered 0"), D2.Registered, 0);
    TestEqual(TEXT("sky sphere: excluded_by_name +1"), D2.ExcludedByName, 1);
    TestEqual(TEXT("sky sphere: not missing"), D2.MissingInRange, 0);

    Comp->SetStaticMesh(Cube);
    if (!TestTrue(TEXT("the cube is applied again"), Comp->GetStaticMesh() == Cube)) return false;
    F.Sync();
    const FRGLSceneAuditResult D3 = Diff(R0, F.Audit());
    TestEqual(TEXT("cube again: registered +1"), D3.Registered, 1);
    TestEqual(TEXT("cube again: excluded_by_name +0"), D3.ExcludedByName, 0);
    TestEqual(TEXT("cube again: not missing"), D3.MissingInRange, 0);
    TestEqual(TEXT("cube again: no ghost"), D3.Ghost, 0);
    return true;
}

// The cost of a sync with 2,000 static cubes in range (logged; no threshold), and the scan's own time on the sync line
// (scan_ms) that the P7 measurement reads. The line comes from an Update whose sync the sensor's 30 m move triggers
// (more than the 24 m distance trigger; every cube stays inside R).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSyncCostLoggedTest, "CarlaRGL.SceneSync.SyncCostLogged",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSyncCostLoggedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(27);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0081), Base, 11100.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    const int32 NumCubes = 2000;   // 50 x 40 grid, 2 m pitch: all within 65 m of the sensor
    for (int32 i = 0; i < NumCubes; ++i)
    {
        const FVector Offset(200.f * static_cast<float>(i % 50) - 4900.f, 200.f * static_cast<float>(i / 50) - 3900.f, 0.f);
        F.Keep(SpawnMeshActor(World, Cube, Base + Offset, EComponentMobility::Static));
    }
    const double S0 = FPlatformTime::Seconds();
    F.Sync();   // registers the cubes
    const double S1 = FPlatformTime::Seconds();
    F.Sync();   // nothing new: the scan alone
    const double S2 = FPlatformTime::Seconds();
    const FRGLSceneAuditResult R1 = F.Audit();
    TestEqual(TEXT("all registered"), Diff(R0, R1).Registered, NumCubes);

    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    F.T += 0.1;
    F.SensorPos += FVector(3000.f, 0.f, 0.f);
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("the move triggered one sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("trigger"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("distance")));
    const FString Line = F.SM.LastSyncLine_ForTest;
    AddInfo(Line);
    float ScanMs = -1.0f;
    TestTrue(TEXT("the sync line has scan_ms="), Line.Contains(TEXT(" scan_ms="), ESearchCase::CaseSensitive) &&
                                                   FParse::Value(*Line, TEXT("scan_ms="), ScanMs) && ScanMs >= 0.0f);
    TestEqual(TEXT("still all registered after the move"), Diff(R0, F.Audit()).Registered, NumCubes);
    AddInfo(FString::Printf(TEXT("sync_cost cubes=%d sync_ms_first=%.3f sync_ms_second=%.3f line_scan_ms=%.3f"),
                            NumCubes, (S1 - S0) * 1000.0, (S2 - S1) * 1000.0, ScanMs));
    return true;
}

// The verdict memo does not keep meshes that are gone: an entry whose key no longer resolves (the mesh is garbage) is
// dropped at the next sync. The sync that first sees a mesh adds its entry.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSkyVerdictMemoPurgedTest, "CarlaRGL.SceneSync.SkyVerdictMemoPurged",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSkyVerdictMemoPurgedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    const FVector Base = BaseFor(38);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0082), Base, 11200.0);
    F.Sync();
    const int32 N0 = F.SM.GetSkyVerdictCount_ForTest();

    UStaticMesh* Dome = NewNamedTransientMesh(TEXT("SM_SkyDome"));
    if (!TestNotNull(TEXT("dome mesh"), Dome)) return false;
    AStaticMeshActor* A = SpawnMeshActor(World, Dome, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Movable);
    if (!TestNotNull(TEXT("dome actor"), A)) return false;
    F.Sync();
    TestEqual(TEXT("the sync that saw the dome remembers its verdict: +1"), F.SM.GetSkyVerdictCount_ForTest() - N0, 1);

    A->Destroy();
    Dome->MarkAsGarbage();
    F.Sync();
    TestEqual(TEXT("the mesh is garbage: its entry is gone at the next sync"), F.SM.GetSkyVerdictCount_ForTest() - N0, 0);
    return true;
}

// ============================================================================
// 7b: the World Partition culling of non-Static parts (the map is taken as a World Partition one through the seam)
// ============================================================================
// FWpTestScope (RGLSceneTestUtil.h) fixes the numbers: speed bounds 5 m/s, U 0.1 s, L_create 1 s, I 1 s, dt 0.05 s ->
// T_sync_max 2.1 s, H 3.2 s, M 32 m; the fixture's sensor (R 120 m, range 100 m): R_load 132 m, R_unload 152 m. The
// engine cube's bounds radius is 0.87 m. Parts are put along +X of the test's base (the other slots are 1 km away in Y).

namespace
{
constexpr float kLoadCm = 13200.f;     // R_load with FWpTestScope's numbers
constexpr float kUnloadCm = 15200.f;   // R_unload

// A Movable static mesh part on an actor of ActorClass (AActor, APawn, ...), no collision, at Loc.
UStaticMeshComponent* SpawnOwnedPart(UWorld* World, UClass* ActorClass, UStaticMesh* Mesh, const FVector& Loc, AActor*& OutActor)
{
    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    OutActor = World->SpawnActor<AActor>(ActorClass, FTransform::Identity, Params);
    if (!OutActor) return nullptr;
    UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(OutActor);
    C->SetMobility(EComponentMobility::Movable);
    C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    C->SetStaticMesh(Mesh);
    OutActor->SetRootComponent(C);
    C->SetRelativeLocation_Direct(Loc);
    C->RegisterComponent();
    return C;
}

// A point on a band of the given distances (cm) in front of Base (+X), spread over +-25 degrees.
FVector BandPoint(const FVector& Base, int32 i, int32 N, float MinCm, float MaxCm)
{
    const float R = MinCm + (MaxCm - MinCm) * (static_cast<float>(i % 17) / 16.f);
    const float A = FMath::DegreesToRadians(-25.f + 50.f * static_cast<float>(i) / static_cast<float>(FMath::Max(1, N - 1)));
    return Base + FVector(R * FMath::Cos(A), R * FMath::Sin(A), 0.f);
}

// Counts the raytrace frames (audits after an Update) with a part missing inside the range.
struct FMissFrames
{
    int32 Frames = 0;
    int32 Bad = 0;
    FString First;
    void Look(const FRGLSceneAuditResult& D, double T)
    {
        ++Frames;
        if (D.MissingInRange != 0 || D.MissingWpCulledInRange != 0)
        {
            if (Bad++ == 0) First = FString::Printf(TEXT("t=%.2f missing_in_range=%d missing_wp_culled_in_range=%d"), T,
                                                    D.MissingInRange, D.MissingWpCulledInRange);
        }
    }
};

// One Update at T + Step with the fixture's sensor refreshed.
void StepUpdate(FAuditFixture& F, UWorld* World, double Step = 0.1)
{
    F.T += Step;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
}
}

// The policy: the seam answers "World Partition" for the map (true / false); unset, the world's own World Partition
// decides (EmptyMap has none); rgl.Scene.WpMovableCulling = 0 turns it off either way. Nothing else enters it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpNonStaticCullingPolicyTest, "CarlaRGL.SceneSync.WpNonStaticCullingPolicy",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpNonStaticCullingPolicyTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    FCVarGuard Enable(TEXT("rgl.Scene.WpMovableCulling"));
    SetCVar(TEXT("rgl.Scene.WpMovableCulling"), 1);
    {
        FRGLSceneManager::FWpScope_ForTest S(SM, true, 0.05f);
        TestTrue(TEXT("seam true: culling on"), SM.UsesWpNonStaticCulling(World));
    }
    {
        FRGLSceneManager::FWpScope_ForTest S(SM, false, 0.05f);
        TestFalse(TEXT("seam false: culling off"), SM.UsesWpNonStaticCulling(World));
    }
    TestFalse(TEXT("the seam is unset again"), SM.WpMapOverride_ForTest.IsSet());
    TestEqual(TEXT("unset: the world's World Partition decides"), SM.UsesWpNonStaticCulling(World), World->GetWorldPartition() != nullptr);
    SetCVar(TEXT("rgl.Scene.WpMovableCulling"), 0);
    {
        FRGLSceneManager::FWpScope_ForTest S(SM, true, 0.05f);
        TestFalse(TEXT("enable 0, seam true: off"), SM.UsesWpNonStaticCulling(World));
    }
    TestFalse(TEXT("enable 0, unset: off"), SM.UsesWpNonStaticCulling(World));
    return true;
}

// Not a World Partition map: with the culling enabled or not, every non-Static part is registered at any distance as
// before (regular Movable and Stationary at 1 km; Movable and Stationary ISMs, near and far instances all), the sync
// line's counts are the same, and the culling keeps no state.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncNonWpNonStaticRegistrationUnchangedTest, "CarlaRGL.SceneSync.NonWpNonStaticRegistrationUnchanged",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncNonWpNonStaticRegistrationUnchangedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(62);
    TArray<FString> Lines;
    for (int32 Enable = 0; Enable <= 1; ++Enable)
    {
        FWpTestScope W(World, /*bWp=*/false);
        SetCVar(TEXT("rgl.Scene.WpMovableCulling"), Enable);
        FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0901), Base, 12000.0 + 100.0 * Enable);
        F.Sync();
        const FRGLSceneAuditResult R0 = F.Audit();
        F.Keep(SpawnPart(World, Cube, Base + FVector(100000.f, 0.f, 0.f), EComponentMobility::Movable));
        F.Keep(SpawnPart(World, Cube, Base + FVector(100000.f, 1000.f, 0.f), EComponentMobility::Stationary));
        AActor* O1 = nullptr; AActor* O2 = nullptr;
        SpawnIsm(World, Cube, Base, { FVector(5000.f, 0.f, 0.f), FVector(100000.f, 0.f, 0.f) }, O1, EComponentMobility::Movable);
        SpawnIsm(World, Cube, Base, { FVector(5000.f, 500.f, 0.f), FVector(100000.f, 500.f, 0.f) }, O2, EComponentMobility::Stationary);
        F.Keep(O1); F.Keep(O2);
        F.Sync();
        const FRGLSceneAuditResult D = Diff(R0, F.Audit());
        TestEqual(FString::Printf(TEXT("enable %d: all 6 non-Static entities registered"), Enable), D.Registered, 6);
        TestEqual(FString::Printf(TEXT("enable %d: no candidate"), Enable), F.SM.GetWpCandidateCount_ForTest(), 0);
        TestEqual(FString::Printf(TEXT("enable %d: no job"), Enable), F.SM.GetWpPendingJobs_ForTest(), 0);
        // A distance sync (30 m) after it: same trigger and counts both times.
        F.SensorPos = Base + FVector(3000.f, 0.f, 0.f);
        StepUpdate(F, World);
        const FString Line = F.SM.LastSyncLine_ForTest;
        AddInfo(Line);
        TestTrue(TEXT("the line says wp=0"), Line.Contains(TEXT(" wp=0 ")));
        FString Kept;
        for (const TCHAR* Key : { TEXT("trigger="), TEXT("queued="), TEXT("created="), TEXT("removed="), TEXT("added_cached=") })
        {
            FString V; FParse::Value(*Line, Key, V); Kept += FString(Key) + V + TEXT(" ");
        }
        Lines.Add(Kept);
    }
    TestEqual(TEXT("same sync counts with the culling enabled or not"), Lines[1], Lines[0]);
    return true;
}

// World Partition: a Movable and a Stationary part 300 m away (beyond R_load) are not registered and their mesh is
// not uploaded; one of each 50 m away is (+2). With the culling off, the far ones are registered too.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpNonStaticFarComponentsCulledTest, "CarlaRGL.SceneSync.WpNonStaticFarComponentsCulled",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpNonStaticFarComponentsCulledTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    TStrongObjectPtr<UStaticMesh> FarMesh(NewTransientBoxMesh(TEXT("WpFarOnly"), 100.f));
    TStrongObjectPtr<UStaticMesh> NearMesh(NewTransientBoxMesh(TEXT("WpNearOnly"), 100.f));
    const FVector Base = BaseFor(63);
    FWpTestScope W(World, true);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0902), Base, 12200.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();
    const int32 Uploads0 = F.SM.MeshUploads_ForTest;
    AStaticMeshActor* FarM = F.Keep(SpawnPart(World, FarMesh.Get(), Base + FVector(30000.f, 0.f, 0.f), EComponentMobility::Movable));
    AStaticMeshActor* FarS = F.Keep(SpawnPart(World, FarMesh.Get(), Base + FVector(30000.f, 2000.f, 0.f), EComponentMobility::Stationary));
    AStaticMeshActor* NearM = F.Keep(SpawnPart(World, NearMesh.Get(), Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Movable));
    AStaticMeshActor* NearS = F.Keep(SpawnPart(World, NearMesh.Get(), Base + FVector(5000.f, 500.f, 0.f), EComponentMobility::Stationary));
    if (!TestTrue(TEXT("parts"), FarM && FarS && NearM && NearS)) return false;
    F.Sync();
    const FRGLSceneAuditResult D1 = Diff(R0, F.Audit());
    AddInfo(F.SM.LastSyncLine_ForTest);
    TestEqual(TEXT("the two near parts registered: +2"), D1.Registered, 2);
    TestFalse(TEXT("far Movable not registered"), F.SM.IsEntityRegistered_ForTest(FarM->GetStaticMeshComponent()));
    TestFalse(TEXT("far Stationary not registered"), F.SM.IsEntityRegistered_ForTest(FarS->GetStaticMeshComponent()));
    TestTrue(TEXT("far Movable is a candidate"), F.SM.IsWpCandidate_ForTest(FarM->GetStaticMeshComponent()));
    TestTrue(TEXT("far Stationary is a candidate"), F.SM.IsWpCandidate_ForTest(FarS->GetStaticMeshComponent()));
    TestEqual(TEXT("the far-only mesh has no reference"), F.SM.GetMeshRefCount_ForTest(FarMesh.Get()), 0);
    TestEqual(TEXT("one upload (the near mesh), none for the far-only mesh"), F.SM.MeshUploads_ForTest - Uploads0, 1);
    TestEqual(TEXT("nothing missing in range"), D1.MissingInRange, 0);

    SetCVar(TEXT("rgl.Scene.WpMovableCulling"), 0);
    F.Sync();
    const FRGLSceneAuditResult D2 = Diff(R0, F.Audit());
    TestEqual(TEXT("culling off: all four registered"), D2.Registered, 4);
    TestEqual(TEXT("culling off: no candidate left"), F.SM.GetWpCandidateCount_ForTest(), 0);
    return true;
}

// World Partition, per ISM instance: a Movable and a Stationary ISM with a near instance (50 m), a far one (300 m) and
// (Movable) a zero-scale near one get one entity each, although the components' bounds reach the sensor. An ISM with only
// far instances gets no group and no upload, and stays a known candidate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpNonStaticIsmPerInstanceTest, "CarlaRGL.SceneSync.WpNonStaticIsmPerInstance",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpNonStaticIsmPerInstanceTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    TStrongObjectPtr<UStaticMesh> FarMesh(NewTransientBoxMesh(TEXT("WpFarIsmOnly"), 100.f));
    const FVector Base = BaseFor(64);
    FWpTestScope W(World, true);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0903), Base, 12300.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();
    AActor* O1 = nullptr; AActor* O2 = nullptr; AActor* O3 = nullptr;
    UInstancedStaticMeshComponent* Mov = SpawnIsm(World, Cube, Base, { FVector(5000.f, 0.f, 0.f), FVector(30000.f, 0.f, 0.f) }, O1,
                                                  EComponentMobility::Movable);
    F.Keep(O1);
    if (!TestNotNull(TEXT("Movable ISM"), Mov)) return false;
    Mov->AddInstance(FTransform(FRotator::ZeroRotator, FVector(6000.f, 0.f, 0.f), FVector(0.f, 1.f, 1.f)));   // zero scale
    UInstancedStaticMeshComponent* Sta = SpawnIsm(World, Cube, Base, { FVector(5000.f, 800.f, 0.f), FVector(30000.f, 800.f, 0.f) }, O2,
                                                  EComponentMobility::Stationary);
    F.Keep(O2);
    UInstancedStaticMeshComponent* FarOnly = SpawnIsm(World, FarMesh.Get(), Base, { FVector(30000.f, 0.f, 0.f), FVector(32000.f, 0.f, 0.f) },
                                                      O3, EComponentMobility::Movable);
    F.Keep(O3);
    if (!TestTrue(TEXT("ISMs"), Sta && FarOnly)) return false;
    TestTrue(TEXT("the Movable ISM's bounds reach R"), FVector::Dist(Mov->Bounds.Origin, Base) - Mov->Bounds.SphereRadius < kRegistrationCm);
    F.Sync();
    const FRGLSceneAuditResult D = Diff(R0, F.Audit());
    AddInfo(F.SM.LastSyncLine_ForTest);
    TestEqual(TEXT("Movable ISM: only the near instance"), F.SM.GetIsmEntityCount_ForTest(Mov), 1);
    TestEqual(TEXT("Stationary ISM: only the near instance"), F.SM.GetIsmEntityCount_ForTest(Sta), 1);
    TestEqual(TEXT("far-only ISM: no group"), F.SM.GetIsmEntityCount_ForTest(FarOnly), 0);
    TestEqual(TEXT("far-only ISM: no mesh reference (no upload)"), F.SM.GetMeshRefCount_ForTest(FarMesh.Get()), 0);
    TestTrue(TEXT("far-only ISM stays a candidate"), F.SM.IsWpCandidate_ForTest(FarOnly));
    TestTrue(TEXT("Movable group consistent"), F.SM.IsIsmGroupConsistent_ForTest(Mov));
    TestEqual(TEXT("registered +2"), D.Registered, 2);
    TestEqual(TEXT("nothing missing in range"), D.MissingInRange, 0);
    return true;
}

// World Partition, the instance sphere of the new path: a mesh whose bounds' origin is 200 m off its pivot, an instance
// 300 m from sensor B with scale (-1, 2, 1): its bounds sit 100 m from B (inside B's R_load) and 412 m from A. It is
// registered (the location alone would leave it out), and leaves when B is gone (outside every unload sphere).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpInstanceBoundsUseMeshCenterTest, "CarlaRGL.SceneSync.WpInstanceBoundsUseMeshCenter",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpInstanceBoundsUseMeshCenterTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    TStrongObjectPtr<UStaticMesh> Off(NewTransientBoxMesh(TEXT("WpOffCentre"), 100.f));
    Off->SetExtendedBounds(FBoxSphereBounds(FVector(20000.f, 0.f, 0.f), FVector(50.f), 87.f));
    TestEqual(TEXT("bounds origin 200 m off the pivot"), Off->GetBounds().Origin.X, 20000.0);
    const FVector Base = BaseFor(65);
    FWpTestScope W(World, true);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0904), Base, 12400.0);
    const void* SensorB = reinterpret_cast<const void*>(0x5A0905);
    const FVector PosB = Base + FVector(0.f, 40000.f, 0.f);
    auto RefreshB = [&]() { F.SM.RegisterSensor(SensorB, PosB, kRegistrationCm, F.T); F.SM.SetSensorAuditRange(SensorB, kRangeCm); };
    RefreshB();
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();
    AActor* O = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Off.Get(), FVector::ZeroVector, {}, O, EComponentMobility::Movable);
    F.Keep(O);
    if (!TestNotNull(TEXT("ISM"), Ism)) return false;
    const FTransform T(FRotator::ZeroRotator, Base + FVector(30000.f, 40000.f, 0.f), FVector(-1.f, 2.f, 1.f));
    Ism->AddInstance(T, /*bWorldSpace=*/true);
    const FVector Center = T.TransformPosition(FVector(20000.f, 0.f, 0.f));
    AddInfo(FString::Printf(TEXT("instance sphere centre to A %.0f cm, to B %.0f cm"), FVector::Dist(Center, Base), FVector::Dist(Center, PosB)));
    TestTrue(TEXT("the sphere is inside B's load sphere"), FVector::Dist(Center, PosB) < kLoadCm);
    TestTrue(TEXT("the instance location is outside B's unload sphere"), FVector::Dist(T.GetLocation(), PosB) > kUnloadCm + 200.f);
    RefreshB();
    F.Sync();
    TestEqual(TEXT("registered through B"), F.SM.GetIsmEntityCount_ForTest(Ism), 1);
    const FRGLSceneAuditResult D = Diff(R0, F.Audit());   // the audit places it with the same geometry
    TestEqual(TEXT("not missing"), D.MissingInRange, 0);
    TestEqual(TEXT("no stale instance"), D.StaleInstance, 0);
    F.SM.UnregisterSensor(SensorB);
    F.Sync();
    TestEqual(TEXT("B gone: outside every unload sphere, removed"), F.SM.GetIsmEntityCount_ForTest(Ism), 0);
    return true;
}

// Parts that keep the original registration at any distance (300 m here): a part of a Pawn, a part on an actor attached
// to a Pawn (a door), a part of an actor with a movement component, a part on a physics body, a part of an actor with a
// skeletal mesh, a part of a sensor's carrier. A part attached to an ordinary actor's fixed root is culled.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpProtectedPartsKeepLegacyRegistrationTest, "CarlaRGL.SceneSync.WpProtectedPartsKeepLegacyRegistration",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpProtectedPartsKeepLegacyRegistrationTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(66);
    FWpTestScope W(World, true);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0906), Base, 12500.0);
    F.Sync();
    using EP = FRGLSceneManager::EWpProtect;
    struct FCase { const TCHAR* Name; UStaticMeshComponent* Comp; EP Expected; };
    TArray<FCase> Cases;
    AActor* A = nullptr;
    UStaticMeshComponent* PawnPart = SpawnOwnedPart(World, APawn::StaticClass(), Cube, Base + FVector(30000.f, 0.f, 0.f), A);
    F.Keep(A);
    AActor* PawnActor = A;
    Cases.Add({ TEXT("pawn part"), PawnPart, EP::Pawn });
    UStaticMeshComponent* Door = SpawnOwnedPart(World, AActor::StaticClass(), Cube, Base + FVector(30000.f, 300.f, 0.f), A);
    F.Keep(A);
    A->AttachToActor(PawnActor, FAttachmentTransformRules::KeepWorldTransform);
    Cases.Add({ TEXT("door attached to a pawn"), Door, EP::Pawn });
    UStaticMeshComponent* Rotating = SpawnOwnedPart(World, AActor::StaticClass(), Cube, Base + FVector(30000.f, 600.f, 0.f), A);
    F.Keep(A);
    URotatingMovementComponent* Mover = NewObject<URotatingMovementComponent>(A);
    Mover->RotationRate = FRotator::ZeroRotator;
    Mover->RegisterComponent();
    Cases.Add({ TEXT("movement component owner"), Rotating, EP::MovementComponent });
    AStaticMeshActor* Body = F.Keep(SpawnMeshActor(World, Cube, Base + FVector(30000.f, 900.f, 0.f), EComponentMobility::Movable));
    Body->GetStaticMeshComponent()->SetSimulatePhysics(true);
    UStaticMeshComponent* Rider = SpawnOwnedPart(World, AActor::StaticClass(), Cube, Base + FVector(30000.f, 900.f, 200.f), A);
    F.Keep(A);
    A->AttachToActor(Body, FAttachmentTransformRules::KeepWorldTransform);
    const bool bPhysics = Body->GetStaticMeshComponent()->IsSimulatingPhysics();
    AddInfo(FString::Printf(TEXT("physics body simulating=%d"), bPhysics ? 1 : 0));
    if (bPhysics)
    {
        Cases.Add({ TEXT("physics body"), Body->GetStaticMeshComponent(), EP::Physics });
        Cases.Add({ TEXT("part on a physics body"), Rider, EP::Physics });
    }
    UStaticMeshComponent* SkelOwned = SpawnOwnedPart(World, AActor::StaticClass(), Cube, Base + FVector(30000.f, 1200.f, 0.f), A);
    F.Keep(A);
    USkeletalMeshComponent* Skel = NewObject<USkeletalMeshComponent>(A);
    Skel->SetupAttachment(SkelOwned);
    Skel->RegisterComponent();
    Cases.Add({ TEXT("actor with a skeletal mesh"), SkelOwned, EP::Skeletal });
    UStaticMeshComponent* Carrier = SpawnOwnedPart(World, AActor::StaticClass(), Cube, Base + FVector(30000.f, 1500.f, 0.f), A);
    F.Keep(A);
    F.SM.SetSensorAuditParent(F.Sensor, A);
    Cases.Add({ TEXT("sensor carrier"), Carrier, EP::SensorParent });
    // An ordinary actor: a fixed scene root and the part attached to it.
    AActor* Plain = F.Keep(World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity));
    USceneComponent* Root = NewObject<USceneComponent>(Plain);
    Root->SetMobility(EComponentMobility::Movable);
    Plain->SetRootComponent(Root);
    Root->SetRelativeLocation_Direct(Base + FVector(30000.f, 1800.f, 0.f));
    Root->RegisterComponent();
    UStaticMeshComponent* Fixed = NewObject<UStaticMeshComponent>(Plain);
    Fixed->SetMobility(EComponentMobility::Movable);
    Fixed->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Fixed->SetStaticMesh(Cube);
    Fixed->SetupAttachment(Root);
    Fixed->RegisterComponent();

    for (const FCase& C : Cases)
    {
        const EP Got = F.SM.ClassifyWpProtection(C.Comp, /*bRemember=*/false);
        TestEqual(FString::Printf(TEXT("%s: %s"), C.Name, FRGLSceneManager::WpProtectName(C.Expected)),
                  FString(FRGLSceneManager::WpProtectName(Got)), FString(FRGLSceneManager::WpProtectName(C.Expected)));
    }
    TestEqual(TEXT("ordinary fixed root: none"), FString(FRGLSceneManager::WpProtectName(F.SM.ClassifyWpProtection(Fixed, false))),
              FString(TEXT("none")));
    F.Sync();
    for (const FCase& C : Cases)
    {
        TestTrue(FString::Printf(TEXT("%s registered at 300 m"), C.Name), F.SM.IsEntityRegistered_ForTest(C.Comp));
    }
    TestFalse(TEXT("ordinary fixed-root part culled at 300 m"), F.SM.IsEntityRegistered_ForTest(Fixed));
    TestTrue(TEXT("ordinary fixed-root part is a candidate"), F.SM.IsWpCandidate_ForTest(Fixed));
    F.SM.SetSensorAuditParent(F.Sensor, nullptr);
    if (bPhysics) Body->GetStaticMeshComponent()->SetSimulatePhysics(false);
    return true;
}

// Skeletal meshes keep their own rules (distance, MaxEntities, cutout): the same near (20 m) and far (300 m) pawns give
// the same skeletal registrations with the culling on and off, and no skeletal component becomes a candidate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpSkeletalPolicyUnchangedTest, "CarlaRGL.SceneSync.WpSkeletalPolicyUnchanged",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpSkeletalPolicyUnchangedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Carla/Static/Pedestrian/EuroG02_G2/SK_EuroG02_A_G2.SK_EuroG02_A_G2"));
    if (!TestNotNull(TEXT("walker SK"), Mesh)) return false;
    FCVarGuard SkelEnable(TEXT("rgl.SkeletalMesh.Enable"));
    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 1);
    const FVector Base = BaseFor(67);
    int32 Registered[2] = { 0, 0 };
    for (int32 Wp = 0; Wp <= 1; ++Wp)
    {
        FWpTestScope W(World, Wp == 1);
        FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0907), Base, 12600.0 + 100.0 * Wp);
        F.Sync();
        const int32 Before = F.SM.GetSkeletalRegisteredCount_ForTest();
        const int32 Cand0 = F.SM.GetWpCandidateCount_ForTest();
        for (const float X : { 2000.f, 30000.f })
        {
            APawn* P = F.Keep(World->SpawnActor<APawn>(APawn::StaticClass(), FTransform::Identity));
            USkeletalMeshComponent* C = NewObject<USkeletalMeshComponent>(P);
            P->SetRootComponent(C);
            C->SetSkeletalMeshAsset(Mesh);
            C->RegisterComponent();
            P->SetActorLocation(Base + FVector(X, 0.f, 0.f));
            C->RefreshBoneTransforms();
        }
        F.Sync();
        Registered[Wp] = F.SM.GetSkeletalRegisteredCount_ForTest() - Before;
        TestEqual(FString::Printf(TEXT("wp %d: no skeletal candidate"), Wp), F.SM.GetWpCandidateCount_ForTest() - Cand0, 0);
    }
    AddInfo(FString::Printf(TEXT("skeletal registered: off %d, on %d"), Registered[0], Registered[1]));
    TestEqual(TEXT("the near pawn only (off)"), Registered[0], 1);
    TestEqual(TEXT("same skeletal registrations with the culling on"), Registered[1], Registered[0]);
    return true;
}

// The guarantee for a part that starts moving: the sensor is still, an ordinary Movable part starts 145 m away (beyond
// R_load) and is culled, then comes at 5 m/s (the bound); 30 other parts wait in the queue (cap 2 per Update). Syncs come
// only from the interval. At every Update, nothing known is missing inside the range; the part is promoted at the sync that
// sees it move, registered before its bounds reach the range, and kept afterwards even when it jumps away.
// On both sync clocks (rgl.Scene.SyncClock): the Update clock (T_sync_max 2.1 s, R_load 132 m; a periodic sync every
// 20 Updates of dt 0.05 s) and the simulation clock (1.1 s, 122 m; a periodic sync every 1 s = 10 Updates of 0.1 s).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpApproachingObjectBeforeRangeTest, "CarlaRGL.SceneSync.WpApproachingObjectBeforeRange",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpApproachingObjectBeforeRangeTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(68);
    for (int32 Clock = 0; Clock <= 1; ++Clock)
    {
        const TCHAR* ClockName = Clock ? TEXT("sim clock") : TEXT("update clock");
        auto Msg = [ClockName](const TCHAR* What) { return FString::Printf(TEXT("%s: %s"), ClockName, What); };
        FWpTestScope W(World, true, 0.05f, 5.0f, Clock);
        SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 2);
        FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0908), Base, 12700.0 + 50.0 * Clock);
        F.Sync();
        const FRGLSceneAuditResult R0 = F.Audit();
        AStaticMeshActor* Mover = F.Keep(SpawnPart(World, Cube, Base + FVector(14500.f, 0.f, 0.f)));
        if (!TestNotNull(*Msg(TEXT("mover")), Mover)) return false;
        for (int32 i = 0; i < 30; ++i)
        {
            F.Keep(SpawnPart(World, Cube, BandPoint(Base, i, 30, 10500.f, 13000.f)));
        }
        UStaticMeshComponent* MC = Mover->GetStaticMeshComponent();
        TestTrue(*Msg(TEXT("the mover starts beyond R_load + its radius")),
                 FVector::Dist(MC->Bounds.Origin, Base) > F.SM.GetWpLoadRadiusCm_ForTest(F.Sensor) + MC->Bounds.SphereRadius);
        const int32 Syncs0 = F.SM.SyncCount_ForTest;
        const int32 Deadline0 = F.SM.WpDeadlineSyncs_ForTest();
        FMissFrames Miss;
        double RegisteredAtGapCm = -1.0;
        int32 OverCap = 0;
        for (int32 k = 0; k < 100; ++k)
        {
            Mover->SetActorLocation(Mover->GetActorLocation() - FVector(50.f, 0.f, 0.f));   // 5 m/s
            StepUpdate(F, World);
            Miss.Look(Diff(R0, F.Audit()), F.T);
            OverCap += (F.SM.WpAddedLastUpdate_ForTest + F.SM.WpRemovedLastUpdate_ForTest > 2) ? 1 : 0;
            if (RegisteredAtGapCm < 0.0 && F.SM.IsEntityRegistered_ForTest(MC))
            {
                RegisteredAtGapCm = FVector::Dist(MC->Bounds.Origin, Base) - MC->Bounds.SphereRadius - kRangeCm;
            }
        }
        const int32 Syncs = F.SM.SyncCount_ForTest - Syncs0;
        AddInfo(FString::Printf(TEXT("%s: frames=%d bad=%d first=%s syncs=%d deadline_syncs=%d registered_at_gap_m=%.1f last_trigger=%s"),
                                ClockName, Miss.Frames, Miss.Bad, *Miss.First, Syncs, F.SM.WpDeadlineSyncs_ForTest() - Deadline0,
                                RegisteredAtGapCm / 100.0, *F.SM.LastSyncTrigger_ForTest));
        TestEqual(*Msg(TEXT("at most 2 changes per Update")), OverCap, 0);
        TestEqual(*Msg(TEXT("no frame with a part missing in range")), Miss.Bad, 0);
        TestTrue(*Msg(TEXT("registered before its bounds reached the range")), RegisteredAtGapCm > 0.0);
        TestTrue(*Msg(TEXT("promoted")), F.SM.IsWpPromoted_ForTest(MC));
        // 10 s of simulation in 100 Updates: the periodic sync follows the clock (every 1 s, or every 20 Updates).
        TestTrue(*Msg(TEXT("the periodic syncs follow the clock")), Clock ? (Syncs >= 9) : (Syncs >= 4 && Syncs <= 6));
        if (Clock == 1)
        {
            TestEqual(*Msg(TEXT("no deadline sync (the clock is the sim time)")), F.SM.WpDeadlineSyncs_ForTest() - Deadline0, 0);
        }
        // A jump away (a violation of the speed bound): still kept, never culled again.
        Mover->SetActorLocation(Base + FVector(40000.f, 0.f, 0.f));
        for (int32 k = 0; k < 25; ++k)
        {
            StepUpdate(F, World);
        }
        TestTrue(*Msg(TEXT("kept at 400 m after the jump")), F.SM.IsEntityRegistered_ForTest(MC));
    }
    return true;
}

// The same for one ISM instance moved by UpdateInstanceTransform (the component's transform never changes): the whole
// component is promoted and created under a cap of 2 per Update; the registered instances follow the current transforms.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpMovingIsmBeforeRangeTest, "CarlaRGL.SceneSync.WpMovingIsmBeforeRange",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpMovingIsmBeforeRangeTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(69);
    FWpTestScope W(World, true);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 2);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0909), Base, 12800.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();
    TArray<FVector> Offsets;
    Offsets.Add(FVector(14500.f, 0.f, 0.f));   // instance 0 moves
    for (int32 i = 0; i < 20; ++i)
    {
        Offsets.Add(BandPoint(FVector::ZeroVector, i, 20, 10500.f, 13000.f));
    }
    AActor* O = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base, Offsets, O, EComponentMobility::Movable);
    F.Keep(O);
    if (!TestNotNull(TEXT("ISM"), Ism)) return false;
    const FTransform Comp0 = Ism->GetComponentTransform();
    FMissFrames Miss;
    int32 BadConsistency = 0, Stale = 0, Ghost = 0, OverCap = 0;
    for (int32 k = 0; k < 100; ++k)
    {
        FTransform T;
        Ism->GetInstanceTransform(0, T, /*bWorldSpace=*/true);
        T.AddToTranslation(FVector(-50.f, 0.f, 0.f));
        Ism->UpdateInstanceTransform(0, T, /*bWorldSpace=*/true, /*bMarkRenderStateDirty=*/true, /*bTeleport=*/true);
        Ism->UpdateBounds();   // what the end of the frame does; the test stays in one frame
        StepUpdate(F, World);
        const FRGLSceneAuditResult D = Diff(R0, F.Audit());
        Miss.Look(D, F.T);
        BadConsistency += F.SM.IsIsmGroupConsistent_ForTest(Ism) ? 0 : 1;
        Stale += D.StaleInstance != 0 ? 1 : 0;
        Ghost += D.Ghost != 0 ? 1 : 0;
        OverCap += (F.SM.WpAddedLastUpdate_ForTest + F.SM.WpRemovedLastUpdate_ForTest > 2) ? 1 : 0;
    }
    AddInfo(FString::Printf(TEXT("frames=%d bad=%d first=%s entities=%d"), Miss.Frames, Miss.Bad, *Miss.First, F.SM.GetIsmEntityCount_ForTest(Ism)));
    TestTrue(TEXT("the component transform never changed"), Ism->GetComponentTransform().Equals(Comp0));
    TestEqual(TEXT("no frame with an instance missing in range"), Miss.Bad, 0);
    TestEqual(TEXT("group consistent at every frame"), BadConsistency, 0);
    TestEqual(TEXT("no stale instance at any frame"), Stale, 0);
    TestEqual(TEXT("no ghost at any frame"), Ghost, 0);
    TestEqual(TEXT("never more than 2 changes per Update"), OverCap, 0);
    TestTrue(TEXT("promoted"), F.SM.IsWpPromoted_ForTest(Ism));
    TestEqual(TEXT("all 21 instances registered in the end"), F.SM.GetIsmEntityCount_ForTest(Ism), 21);
    return true;
}

// Two sensors: the part is still; sensor A comes at 5 m/s, sensor B 200 m to the side also moves. A part reaches the
// scene through either sphere; a part inside both is one entity; removing B drops only what B alone held; a part
// between R_load and R_unload stays (hysteresis) and leaves beyond R_unload.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpMovingSensorAndUnionTest, "CarlaRGL.SceneSync.WpMovingSensorAndUnion",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpMovingSensorAndUnionTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(70);
    FWpTestScope W(World, true);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A090A), Base, 12900.0);
    const void* SensorB = reinterpret_cast<const void*>(0x5A090B);
    FVector PosB = Base + FVector(0.f, 20000.f, 0.f);
    auto RefreshB = [&]() { F.SM.RegisterSensor(SensorB, PosB, kRegistrationCm, F.T); F.SM.SetSensorAuditRange(SensorB, kRangeCm); };
    RefreshB();
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();
    AStaticMeshActor* Ahead = F.Keep(SpawnPart(World, Cube, Base + FVector(14500.f, 0.f, 0.f)));    // A comes to it
    AStaticMeshActor* OnlyB = F.Keep(SpawnPart(World, Cube, Base + FVector(0.f, 30000.f, 0.f)));   // 100 m from B
    AStaticMeshActor* Both = F.Keep(SpawnPart(World, Cube, Base + FVector(0.f, 10000.f, 0.f)));    // 100 m from both
    AStaticMeshActor* Behind = F.Keep(SpawnPart(World, Cube, Base - FVector(12500.f, 0.f, 0.f)));  // A leaves it
    RefreshB();
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    TestTrue(TEXT("only-B part registered (B's sphere)"), F.SM.IsEntityRegistered_ForTest(OnlyB->GetStaticMeshComponent()));
    TestTrue(TEXT("part in both spheres registered"), F.SM.IsEntityRegistered_ForTest(Both->GetStaticMeshComponent()));
    TestTrue(TEXT("part behind A registered"), F.SM.IsEntityRegistered_ForTest(Behind->GetStaticMeshComponent()));
    TestFalse(TEXT("part 145 m ahead culled"), F.SM.IsEntityRegistered_ForTest(Ahead->GetStaticMeshComponent()));
    TestEqual(TEXT("three entities (no duplicate for the overlap)"), Diff(R0, R1).Registered, 3);
    FMissFrames Miss;
    int32 HysteresisChecked = 0, HysteresisBad = 0, UnloadChecked = 0, UnloadBad = 0;
    for (int32 k = 0; k < 90; ++k)
    {
        F.SensorPos += FVector(50.f, 0.f, 0.f);    // A: 5 m/s toward Ahead
        PosB += FVector(0.f, 50.f, 0.f);           // B: 5 m/s away
        RefreshB();
        const int32 Syncs = F.SM.SyncCount_ForTest;
        StepUpdate(F, World);
        Miss.Look(Diff(R0, F.Audit()), F.T);
        if (F.SM.SyncCount_ForTest != Syncs)
        {
            const float Gap = FVector::Dist(Behind->GetActorLocation(), F.SensorPos);
            const bool bReg = F.SM.IsEntityRegistered_ForTest(Behind->GetStaticMeshComponent());
            if (Gap > kLoadCm + 100.f && Gap < kUnloadCm - 100.f) { ++HysteresisChecked; HysteresisBad += bReg ? 0 : 1; }
            if (Gap > kUnloadCm + 100.f) { ++UnloadChecked; UnloadBad += bReg ? 1 : 0; }
        }
    }
    AddInfo(FString::Printf(TEXT("frames=%d bad=%d first=%s hysteresis_checked=%d unload_checked=%d"), Miss.Frames, Miss.Bad, *Miss.First,
                            HysteresisChecked, UnloadChecked));
    TestEqual(TEXT("no frame with a part missing in range"), Miss.Bad, 0);
    TestTrue(TEXT("Ahead registered before A reached it"), F.SM.IsEntityRegistered_ForTest(Ahead->GetStaticMeshComponent()));
    TestTrue(TEXT("a sync saw Behind between R_load and R_unload"), HysteresisChecked > 0);
    TestEqual(TEXT("kept between R_load and R_unload"), HysteresisBad, 0);
    TestTrue(TEXT("a sync saw Behind beyond R_unload"), UnloadChecked > 0);
    TestEqual(TEXT("gone beyond R_unload"), UnloadBad, 0);
    F.SM.UnregisterSensor(SensorB);
    F.Sync();
    TestFalse(TEXT("B removed: the only-B part leaves"), F.SM.IsEntityRegistered_ForTest(OnlyB->GetStaticMeshComponent()));
    TestTrue(TEXT("B removed: Ahead (A's) stays"), F.SM.IsEntityRegistered_ForTest(Ahead->GetStaticMeshComponent()));
    return true;
}

// A mesh that already has live entities does not let 2,000 parts in at once: cap 2, no ms cap. The sync's scan creates
// nothing itself; every Update (sync Updates included) adds at most 2; the oldest wait keeps its age across syncs.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpCachedEntitiesRespectBudgetTest, "CarlaRGL.SceneSync.WpCachedEntitiesRespectBudget",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpCachedEntitiesRespectBudgetTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(71);
    FWpTestScope W(World, true);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 2);
    SetFloatCVar(TEXT("rgl.Scene.MovableEntityBudgetMs"), 0.0f);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A090C), Base, 13000.0);
    F.Keep(SpawnMeshActor(World, Cube, Base + FVector(2000.f, 0.f, 0.f), EComponentMobility::Static));   // keeps the cube live
    F.Sync();
    TestTrue(TEXT("the cube mesh is live"), F.SM.GetMeshRefCount_ForTest(Cube) > 0);
    for (int32 i = 0; i < 2000; ++i)
    {
        F.Keep(SpawnPart(World, Cube, BandPoint(Base, i, 2000, 10500.f, 13000.f)));
    }
    int32 MaxPerUpdate = 0, Syncs = 0, MaxEntityStep = 0;
    double AgeBeforeSecondSync = -1.0, AgeAfterSecondSync = -1.0;
    for (int32 k = 0; k < 45; ++k)
    {
        const int32 E0 = F.SM.GetSceneCounts_ForTest().Entities;
        const int32 S0 = F.SM.SyncCount_ForTest;
        const double Age0 = F.SM.GetWpQueueAgeMax_ForTest();
        StepUpdate(F, World);
        MaxPerUpdate = FMath::Max(MaxPerUpdate, F.SM.WpAddedLastUpdate_ForTest);
        MaxEntityStep = FMath::Max(MaxEntityStep, F.SM.GetSceneCounts_ForTest().Entities - E0);
        if (F.SM.SyncCount_ForTest != S0 && ++Syncs == 2)
        {
            AgeBeforeSecondSync = Age0;
            AgeAfterSecondSync = F.SM.GetWpQueueAgeMax_ForTest();
        }
    }
    AddInfo(FString::Printf(TEXT("syncs=%d max_added_per_update=%d max_entity_step=%d age_before=%.2f age_after=%.2f pending=%d"), Syncs,
                            MaxPerUpdate, MaxEntityStep, AgeBeforeSecondSync, AgeAfterSecondSync, F.SM.GetWpPendingJobs_ForTest()));
    TestTrue(TEXT("two syncs ran"), Syncs >= 2);
    TestEqual(TEXT("at most 2 added per Update"), MaxPerUpdate, 2);
    TestTrue(TEXT("the scene grows by at most 2 per Update (sync Updates included)"), MaxEntityStep <= 2);
    TestTrue(TEXT("work remains"), F.SM.GetWpPendingJobs_ForTest() > 0);
    TestTrue(TEXT("the oldest wait is not reset by the sync"), AgeAfterSecondSync >= AgeBeforeSecondSync && AgeBeforeSecondSync > 1.0);
    return true;
}

// 2,000 instances of one Movable ISM come into R_load together; cap 2: the first budgeted Update adds exactly 2, no
// Update adds more, and the group stays consistent (entities, indices, mask, references). An ISM whose instances are all
// beyond R_load has no group and its mesh is never uploaded.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpLargeIsmCreatesArePartialTest, "CarlaRGL.SceneSync.WpLargeIsmCreatesArePartial",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpLargeIsmCreatesArePartialTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    TStrongObjectPtr<UStaticMesh> FarMesh(NewTransientBoxMesh(TEXT("WpFarIsm2"), 100.f));
    const FVector Base = BaseFor(72);
    FWpTestScope W(World, true);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 2);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A090D), Base, 13100.0);
    F.Keep(SpawnMeshActor(World, Cube, Base + FVector(2000.f, 0.f, 0.f), EComponentMobility::Static));
    F.Sync();
    TArray<FVector> Offsets;
    for (int32 i = 0; i < 2000; ++i)
    {
        Offsets.Add(BandPoint(FVector::ZeroVector, i, 2000, 10500.f, 13000.f));
    }
    AActor* O = nullptr; AActor* O2 = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base, Offsets, O, EComponentMobility::Movable);
    F.Keep(O);
    UInstancedStaticMeshComponent* FarIsm = SpawnIsm(World, FarMesh.Get(), Base, { FVector(30000.f, 0.f, 0.f) }, O2, EComponentMobility::Movable);
    F.Keep(O2);
    if (!TestTrue(TEXT("ISMs"), Ism && FarIsm)) return false;
    int32 First = -1, OverCap = 0, Inconsistent = 0;
    for (int32 k = 0; k < 30; ++k)
    {
        const int32 N0 = F.SM.GetIsmEntityCount_ForTest(Ism);
        StepUpdate(F, World);
        const int32 Step = F.SM.GetIsmEntityCount_ForTest(Ism) - N0;
        if (First < 0 && Step > 0) First = Step;
        OverCap += Step > 2 ? 1 : 0;
        Inconsistent += F.SM.IsIsmGroupConsistent_ForTest(Ism) ? 0 : 1;
    }
    AddInfo(FString::Printf(TEXT("first=%d entities=%d pending=%d"), First, F.SM.GetIsmEntityCount_ForTest(Ism), F.SM.GetWpPendingJobs_ForTest()));
    TestEqual(TEXT("the first budgeted Update adds exactly 2"), First, 2);
    TestEqual(TEXT("no Update adds more than 2"), OverCap, 0);
    TestEqual(TEXT("the partial group stays consistent"), Inconsistent, 0);
    TestTrue(TEXT("partial: fewer than 2000"), F.SM.GetIsmEntityCount_ForTest(Ism) < 2000);
    TestEqual(TEXT("far ISM: no group"), F.SM.GetIsmEntityCount_ForTest(FarIsm), 0);
    TestEqual(TEXT("far ISM: its mesh never uploaded"), F.SM.GetMeshRefCount_ForTest(FarMesh.Get()), 0);
    return true;
}

// A tiny ms budget (and no count cap): at least one change per Update, work left after it; with a sync on every Update
// (interval 0.1 s, dt 0.1 s) the queue still makes progress. No wall-time upper bound is asserted.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpEntityMsCapAndNoStarvationTest, "CarlaRGL.SceneSync.WpEntityMsCapAndNoStarvation",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpEntityMsCapAndNoStarvationTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(73);
    FWpTestScope W(World, true);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 0);
    SetFloatCVar(TEXT("rgl.Scene.MovableEntityBudgetMs"), 1.0e-6f);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A090E), Base, 13200.0);
    F.Sync();
    TArray<UStaticMeshComponent*> Parts;
    for (int32 i = 0; i < 40; ++i)
    {
        Parts.Add(F.Keep(SpawnPart(World, Cube, BandPoint(Base, i, 40, 10200.f, 11800.f)))->GetStaticMeshComponent());
    }
    auto CountRegistered = [&]() { int32 N = 0; for (UStaticMeshComponent* C : Parts) N += F.SM.IsEntityRegistered_ForTest(C) ? 1 : 0; return N; };
    // Until the interval's sync has found them, then three more Updates.
    int32 Guard = 0;
    while (F.SM.GetWpPendingJobs_ForTest() == 0 && CountRegistered() == 0 && Guard++ < 40) StepUpdate(F, World);
    const int32 N1 = CountRegistered();
    TestTrue(TEXT("the sync Update made progress"), N1 >= 1);
    TestTrue(TEXT("work left after it"), N1 < Parts.Num());
    StepUpdate(F, World);
    TestTrue(TEXT("next Update: at least one more"), CountRegistered() > N1);
    // A sync on every Update.
    F.SM.SetSyncInterval(0.1f);
    F.SM.WorldDtOverride_ForTest = 0.1f;
    int32 NoProgress = 0;
    for (int32 k = 0; k < 6 && CountRegistered() < Parts.Num(); ++k)
    {
        const int32 Before = CountRegistered();
        const int32 S0 = F.SM.SyncCount_ForTest;
        StepUpdate(F, World);
        TestEqual(TEXT("a sync on this Update"), F.SM.SyncCount_ForTest - S0, 1);
        NoProgress += CountRegistered() > Before ? 0 : 1;
    }
    AddInfo(FString::Printf(TEXT("registered=%d of %d"), CountRegistered(), Parts.Num()));
    TestEqual(TEXT("progress on every sync Update"), NoProgress, 0);
    return true;
}

// Changes under a partial group: an instance added, one removed, the mesh swapped, the component unregistered and
// registered again, hidden, destroyed. After each: the old jobs are dropped (no index of the old layout is used), no
// duplicate, ghost, stale instance or mesh mismatch, and the mesh references come back to where they were.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpMutationAndStreamOutTest, "CarlaRGL.SceneSync.WpMutationAndStreamOut",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpMutationAndStreamOutTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    UStaticMesh* Sphere = LoadMeshReady(kSphere); if (!TestNotNull(TEXT("sphere"), Sphere)) return false;
    const FVector Base = BaseFor(74);
    FWpTestScope W(World, true);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 2);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A090F), Base, 13300.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();
    const int32 CubeRefs0 = F.SM.GetMeshRefCount_ForTest(Cube);
    const int32 SphereRefs0 = F.SM.GetMeshRefCount_ForTest(Sphere);
    TArray<FVector> Offsets;
    for (int32 i = 0; i < 10; ++i) Offsets.Add(BandPoint(FVector::ZeroVector, i, 10, 10500.f, 13000.f));
    AActor* O = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base, Offsets, O, EComponentMobility::Movable);
    F.Keep(O);
    if (!TestNotNull(TEXT("ISM"), Ism)) return false;
    auto Run = [&](int32 N) { for (int32 k = 0; k < N; ++k) StepUpdate(F, World); };
    auto CheckClean = [&](const TCHAR* What)
    {
        const FRGLSceneAuditResult D = Diff(R0, F.Audit());
        TestEqual(FString::Printf(TEXT("%s: no ghost"), What), D.Ghost, 0);
        TestEqual(FString::Printf(TEXT("%s: no stale instance"), What), D.StaleInstance, 0);
        TestEqual(FString::Printf(TEXT("%s: no mesh mismatch"), What), D.MeshMismatch, 0);
        TestTrue(FString::Printf(TEXT("%s: group consistent"), What), F.SM.IsIsmGroupConsistent_ForTest(Ism));
        TestTrue(FString::Printf(TEXT("%s: no more entities than instances"), What), F.SM.GetIsmEntityCount_ForTest(Ism) <= Ism->GetInstanceCount());
    };
    Run(21);   // the interval's sync found it; 2 per Update after it
    TestTrue(TEXT("partial or complete group"), F.SM.GetIsmEntityCount_ForTest(Ism) > 0);
    CheckClean(TEXT("start"));
    Ism->AddInstance(FTransform(FVector(11000.f, -500.f, 0.f)));
    Run(21);
    CheckClean(TEXT("instance added"));
    TestTrue(TEXT("instance removed"), Ism->RemoveInstance(0));
    Run(21);
    CheckClean(TEXT("instance removed"));
    Ism->SetStaticMesh(Sphere);
    Run(21);
    CheckClean(TEXT("mesh swapped"));
    TestEqual(TEXT("mesh swapped: no cube reference left"), F.SM.GetMeshRefCount_ForTest(Cube), CubeRefs0);
    Ism->UnregisterComponent();
    Ism->RegisterComponent();
    Run(21);
    CheckClean(TEXT("re-registered"));
    Ism->SetVisibility(false);
    Run(21);
    TestEqual(TEXT("hidden: no entity"), F.SM.GetIsmEntityCount_ForTest(Ism), 0);
    CheckClean(TEXT("hidden"));
    Ism->SetVisibility(true);
    Run(21);
    O->Destroy();
    Run(2);
    const FRGLSceneAuditResult D = Diff(R0, F.Audit());
    TestEqual(TEXT("destroyed: no ghost"), D.Ghost, 0);
    TestEqual(TEXT("destroyed: registered back to the start"), D.Registered, 0);
    TestEqual(TEXT("destroyed: cube references back"), F.SM.GetMeshRefCount_ForTest(Cube), CubeRefs0);
    TestEqual(TEXT("destroyed: sphere references back"), F.SM.GetMeshRefCount_ForTest(Sphere), SphereRefs0);
    return true;
}

// Motion from rest: parts 300 m away, culled, with no velocity; then moved 1 m (SetActorLocation, velocity still zero),
// turned, rescaled, carried by their parent's root, or given a component velocity: the next sync promotes and registers
// them. An unchanged part stays culled.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpMotionPromotionFromRestTest, "CarlaRGL.SceneSync.WpMotionPromotionFromRest",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpMotionPromotionFromRestTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(75);
    FWpTestScope W(World, true);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0910), Base, 13400.0);
    F.Sync();
    AStaticMeshActor* Moved = F.Keep(SpawnPart(World, Cube, Base + FVector(30000.f, 0.f, 0.f)));
    AStaticMeshActor* Turned = F.Keep(SpawnPart(World, Cube, Base + FVector(30000.f, 500.f, 0.f)));
    AStaticMeshActor* Scaled = F.Keep(SpawnPart(World, Cube, Base + FVector(30000.f, 1000.f, 0.f)));
    AStaticMeshActor* Pushed = F.Keep(SpawnPart(World, Cube, Base + FVector(30000.f, 1500.f, 0.f)));
    AStaticMeshActor* Still = F.Keep(SpawnPart(World, Cube, Base + FVector(30000.f, 2000.f, 0.f)));
    AActor* Carrier = F.Keep(World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity));
    USceneComponent* Root = NewObject<USceneComponent>(Carrier);
    Root->SetMobility(EComponentMobility::Movable);
    Carrier->SetRootComponent(Root);
    Root->SetRelativeLocation_Direct(Base + FVector(30000.f, 2500.f, 0.f));
    Root->RegisterComponent();
    UStaticMeshComponent* Child = NewObject<UStaticMeshComponent>(Carrier);
    Child->SetMobility(EComponentMobility::Movable);
    Child->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Child->SetStaticMesh(Cube);
    Child->SetupAttachment(Root);
    Child->RegisterComponent();
    F.Sync();
    for (AStaticMeshActor* A : { Moved, Turned, Scaled, Pushed, Still })
    {
        TestFalse(FString::Printf(TEXT("%s culled at first"), *A->GetName()), F.SM.IsEntityRegistered_ForTest(A->GetStaticMeshComponent()));
    }
    TestFalse(TEXT("child culled at first"), F.SM.IsEntityRegistered_ForTest(Child));
    Moved->SetActorLocation(Moved->GetActorLocation() + FVector(100.f, 0.f, 0.f));
    Turned->SetActorRotation(FRotator(0.f, 30.f, 0.f));
    Scaled->SetActorScale3D(FVector(1.5f));
    Pushed->GetStaticMeshComponent()->ComponentVelocity = FVector(100.f, 0.f, 0.f);
    Carrier->SetActorLocation(Carrier->GetActorLocation() + FVector(0.f, 100.f, 0.f));
    TestTrue(TEXT("velocity of the pushed part"), Pushed->GetStaticMeshComponent()->GetComponentVelocity().Size() > 1.0);
    F.Sync();
    for (AStaticMeshActor* A : { Moved, Turned, Scaled, Pushed })
    {
        TestTrue(FString::Printf(TEXT("%s promoted"), *A->GetName()), F.SM.IsWpPromoted_ForTest(A->GetStaticMeshComponent()));
        TestTrue(FString::Printf(TEXT("%s registered"), *A->GetName()), F.SM.IsEntityRegistered_ForTest(A->GetStaticMeshComponent()));
    }
    TestTrue(TEXT("child carried by its parent: promoted"), F.SM.IsWpPromoted_ForTest(Child));
    TestTrue(TEXT("child registered"), F.SM.IsEntityRegistered_ForTest(Child));
    TestFalse(TEXT("unchanged part not promoted"), F.SM.IsWpPromoted_ForTest(Still->GetStaticMeshComponent()));
    TestFalse(TEXT("unchanged part stays culled"), F.SM.IsEntityRegistered_ForTest(Still->GetStaticMeshComponent()));
    Pushed->GetStaticMeshComponent()->ComponentVelocity = FVector::ZeroVector;
    return true;
}

// Each contract broken on its own is logged: an Update gap above U, a pose refresh gap above U, a sensor faster than its
// bound (and a sync forced by it), a culled part faster than its bound, a wait above L_create (whose age the next sync
// does not reset).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpCullContractsAreCheckedTest, "CarlaRGL.SceneSync.WpCullContractsAreChecked",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpCullContractsAreCheckedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(76);
    FWpTestScope W(World, true);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0911), Base, 13500.0);
    F.Sync();
    StepUpdate(F, World);
    StepUpdate(F, World);
    auto V = [&F](const TCHAR* K) { return F.SM.GetWpViolations_ForTest(K); };

    // U: the scene Update and the pose refresh 0.3 s after the previous ones.
    const int32 Gap0 = V(TEXT("update_gap")), Pose0 = V(TEXT("pose_gap"));
    StepUpdate(F, World, 0.3);
    TestEqual(TEXT("update_gap +1"), V(TEXT("update_gap")) - Gap0, 1);
    TestEqual(TEXT("pose_gap +1"), V(TEXT("pose_gap")) - Pose0, 1);

    // Sensor speed: 10 m in 0.1 s (100 m/s, bound 5): logged, and the Update syncs (wp_reset).
    const int32 Speed0 = V(TEXT("sensor_speed"));
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    F.SensorPos += FVector(1000.f, 0.f, 0.f);
    StepUpdate(F, World);
    TestEqual(TEXT("sensor_speed +1"), V(TEXT("sensor_speed")) - Speed0, 1);
    TestEqual(TEXT("a sync forced"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("trigger=wp_reset"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("wp_reset")));

    // A culled part faster than its bound: 50 m between two syncs.
    AStaticMeshActor* Far = F.Keep(SpawnPart(World, Cube, F.SensorPos + FVector(40000.f, 0.f, 0.f)));
    F.Sync();
    const int32 Obj0 = V(TEXT("object_speed"));
    Far->SetActorLocation(Far->GetActorLocation() + FVector(5000.f, 0.f, 0.f));
    F.Sync();
    TestEqual(TEXT("object_speed +1"), V(TEXT("object_speed")) - Obj0, 1);
    TestTrue(TEXT("and promoted"), F.SM.IsWpPromoted_ForTest(Far->GetStaticMeshComponent()));

    // L_create: 0.2 s, one change per Update, 60 parts. The knob change forces a sync at the next Update, which finds
    // them (new parts: not covered yet); the next interval sync (2 s later) queues them as known parts, older than 0.2 s.
    SetFloatCVar(TEXT("rgl.Scene.MovableCreateLatencySeconds"), 0.2f);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 1);
    for (int32 i = 0; i < 60; ++i)
    {
        F.Keep(SpawnPart(World, Cube, BandPoint(F.SensorPos, i, 60, 10500.f, 13000.f)));
    }
    const int32 Age0 = V(TEXT("queue_age"));
    const int32 SyncsQ = F.SM.SyncCount_ForTest;
    int32 Guard = 0;
    while (F.SM.SyncCount_ForTest - SyncsQ < 2 && Guard++ < 45) StepUpdate(F, World);
    StepUpdate(F, World);
    AddInfo(FString::Printf(TEXT("queue_age syncs=%d pending=%d age=%.2f"), F.SM.SyncCount_ForTest - SyncsQ,
                            F.SM.GetWpPendingJobs_ForTest(), F.SM.GetWpQueueAgeMax_ForTest()));
    TestTrue(TEXT("queue_age logged"), V(TEXT("queue_age")) - Age0 >= 1);
    // A budgeted sync (a knob change forces it) rebuilds the queue: the waits keep their start.
    const double AgeBefore = F.SM.GetWpQueueAgeMax_ForTest();
    const int32 S1 = F.SM.SyncCount_ForTest;
    SetFloatCVar(TEXT("rgl.Scene.MovableMinMarginM"), 21.0f);
    StepUpdate(F, World);
    AddInfo(FString::Printf(TEXT("age_before=%.2f age_after=%.2f pending=%d trigger=%s"), AgeBefore, F.SM.GetWpQueueAgeMax_ForTest(),
                            F.SM.GetWpPendingJobs_ForTest(), *F.SM.LastSyncTrigger_ForTest));
    TestEqual(TEXT("the knob change forced a sync"), F.SM.SyncCount_ForTest - S1, 1);
    TestTrue(TEXT("still waiting after the sync"), F.SM.GetWpPendingJobs_ForTest() > 0);
    TestTrue(TEXT("age not reset by the sync"), F.SM.GetWpQueueAgeMax_ForTest() >= AgeBefore);
    return true;
}

// The culling's state belongs to one world's manager: another world's manager starts with none (no candidate, no
// promotion, no seam), and decides with its own world (no World Partition: the original rule). Turning the seam off for
// the first manager drops its state at the next sync.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpCullStateDoesNotCrossWorldsTest, "CarlaRGL.SceneSync.WpCullStateDoesNotCrossWorlds",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpCullStateDoesNotCrossWorldsTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(78);
    UStaticMeshComponent* FarComp = nullptr;
    {
        FWpTestScope W(World, true);
        FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0912), Base, 13600.0);
        F.Sync();
        AStaticMeshActor* Far = F.Keep(SpawnPart(World, Cube, Base + FVector(30000.f, 0.f, 0.f)));
        FarComp = Far->GetStaticMeshComponent();
        F.Sync();
        Far->SetActorLocation(Far->GetActorLocation() + FVector(100.f, 0.f, 0.f));
        F.Sync();
        TestTrue(TEXT("world 1: a promoted candidate"), F.SM.IsWpPromoted_ForTest(FarComp));
        TestTrue(TEXT("world 1: candidates"), F.SM.GetWpCandidateCount_ForTest() > 0);

        UWorld* Other = UWorld::CreateWorld(EWorldType::Inactive, /*bInformEngineOfWorld=*/false);
        if (TestNotNull(TEXT("second world"), Other))
        {
            FRGLSceneManager& SM2 = FRGLSceneManager::GetInstance(Other);
            TestTrue(TEXT("another manager"), &SM2 != &F.SM);
            TestFalse(TEXT("world 2: no seam"), SM2.WpMapOverride_ForTest.IsSet());
            TestEqual(TEXT("world 2: no candidate"), SM2.GetWpCandidateCount_ForTest(), 0);
            TestFalse(TEXT("world 2: no promotion of world 1's part"), SM2.IsWpPromoted_ForTest(FarComp));
            TestEqual(TEXT("world 2: its own world decides (no World Partition)"), SM2.UsesWpNonStaticCulling(Other),
                      Other->GetWorldPartition() != nullptr);
            FRGLSceneManager::DestroyInstance(Other);
            Other->DestroyWorld(/*bInformEngineOfWorld=*/false);
        }
    }
    // The seam is gone for world 1 (EmptyMap: no World Partition): the next sync drops the culling's state.
    FAuditFixture F2(World, reinterpret_cast<const void*>(0x5A0913), Base, 13700.0);
    F2.Sync();
    TestEqual(TEXT("world 1 without the seam: no candidate left"), F2.SM.GetWpCandidateCount_ForTest(), 0);
    TestFalse(TEXT("world 1 without the seam: no promotion left"), F2.SM.IsWpPromoted_ForTest(FarComp));
    return true;
}

// R5: the dt shrinks after the culling sync (0.05 -> 0.005 s) and the periodic sync would come 20 s later; a culled part
// 420.9 m away (R_load 420 m with the default 50 m/s bounds) comes at 50 m/s and would reach the range at 6.4 s. The
// deadline on the sim clock syncs in time (trigger wp_deadline); nothing is missing at any Update.
// On both sync clocks: the above is the Update clock. On the simulation clock (rgl.Scene.SyncClock = 1) the periodic
// sync counts simulation time, so a shrinking dt does not stretch it: R_load 320 m (T_sync_max 1.1 s, H 2.2 s), the part
// starts 320.9 m away, a sync every 1 s finds it, and no deadline sync is needed.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpDtShrinkForcesSyncTest, "CarlaRGL.SceneSync.WpDtShrinkForcesSync",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpDtShrinkForcesSyncTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(79);
    for (int32 Clock = 0; Clock <= 1; ++Clock)
    {
        const TCHAR* ClockName = Clock ? TEXT("sim clock") : TEXT("update clock");
        auto Msg = [ClockName](const TCHAR* What) { return FString::Printf(TEXT("%s: %s"), ClockName, What); };
        FWpTestScope W(World, true, 0.05f, /*SpeedMps=*/50.f, Clock);
        FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0914), Base, 13800.0 + 50.0 * Clock);
        F.Sync();
        const float LoadCm = F.SM.GetWpLoadRadiusCm_ForTest(F.Sensor);
        AddInfo(FString::Printf(TEXT("%s: R_load=%.1f m"), ClockName, LoadCm / 100.f));
        TestTrue(*Msg(Clock ? TEXT("R_load 320 m") : TEXT("R_load 420 m")), FMath::IsNearlyEqual(LoadCm, Clock ? 32000.f : 42000.f, 1.f));
        const FRGLSceneAuditResult R0 = F.Audit();
        AStaticMeshActor* A = F.Keep(SpawnPart(World, Cube, Base + FVector(LoadCm + 90.f, 0.f, 0.f)));
        if (!TestNotNull(*Msg(TEXT("part")), A)) return false;
        UStaticMeshComponent* AC = A->GetStaticMeshComponent();
        TestTrue(*Msg(TEXT("beyond R_load + its radius")), FVector::Dist(AC->Bounds.Origin, Base) > LoadCm + AC->Bounds.SphereRadius);
        F.Sync();
        TestFalse(*Msg(TEXT("culled")), F.SM.IsEntityRegistered_ForTest(AC));
        F.SM.WorldDtOverride_ForTest = 0.005f;
        const int32 Deadline0 = F.SM.WpDeadlineSyncs_ForTest();
        const int32 Syncs0 = F.SM.SyncCount_ForTest;
        FMissFrames Miss;
        double RegisteredAtGapCm = -1.0;
        bool bSawDeadline = false;
        for (int32 k = 0; k < 66; ++k)
        {
            A->SetActorLocation(A->GetActorLocation() - FVector(500.f, 0.f, 0.f));   // 50 m/s
            StepUpdate(F, World);
            bSawDeadline |= (F.SM.LastSyncTrigger_ForTest == TEXT("wp_deadline"));
            Miss.Look(Diff(R0, F.Audit()), F.T);
            if (RegisteredAtGapCm < 0.0 && F.SM.IsEntityRegistered_ForTest(AC))
            {
                RegisteredAtGapCm = FVector::Dist(AC->Bounds.Origin, Base) - AC->Bounds.SphereRadius - kRangeCm;
            }
        }
        const int32 Syncs = F.SM.SyncCount_ForTest - Syncs0;
        AddInfo(FString::Printf(TEXT("%s: frames=%d bad=%d first=%s syncs=%d deadline_syncs=%d registered_at_gap_m=%.1f"), ClockName,
                                Miss.Frames, Miss.Bad, *Miss.First, Syncs, F.SM.WpDeadlineSyncs_ForTest() - Deadline0,
                                RegisteredAtGapCm / 100.0));
        if (Clock == 0)
        {
            TestTrue(*Msg(TEXT("a wp_deadline sync ran")), bSawDeadline && F.SM.WpDeadlineSyncs_ForTest() - Deadline0 >= 1);
        }
        else
        {
            TestEqual(*Msg(TEXT("no deadline sync: the periodic sync counts sim time")), F.SM.WpDeadlineSyncs_ForTest() - Deadline0, 0);
            TestTrue(*Msg(TEXT("a periodic sync every 1 s of sim time (6.6 s)")), Syncs >= 5);
        }
        TestEqual(*Msg(TEXT("no frame with a part missing in range")), Miss.Bad, 0);
        TestTrue(*Msg(TEXT("registered before its bounds reached the range")), RegisteredAtGapCm > 0.0);
        F.SM.WorldDtOverride_ForTest = 0.05f;
    }
    return true;
}

// The horizon (pure): I = 1, U = 0.1, L = 1: Update clock with dt 0.05 / 0.025 -> T_sync_max 2.1 / 4.1 (the periodic sync
// comes every 20 / 41 Updates), simulation clock -> 1.1, I = 2.5 with dt 0.025 -> 10.1; H = T_sync_max + L + U; the margin
// grows with the speeds, L, I and U; R_load = range + M covers the closing distance (V_s + V_o) * H.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpCullHorizonMatchesClockTest, "CarlaRGL.SceneSync.WpCullHorizonMatchesClock",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpCullHorizonMatchesClockTest::RunTest(const FString& Parameters)
{
    using C = FRGLSceneManager::EWpCullClock;
    auto H = [](C Clock, double I, float Dt, double U, double L) { return FRGLSceneManager::ComputeWpCullHorizon(Clock, I, Dt, U, L); };
    TestTrue(TEXT("dt 0.05: T 2.1"), FMath::IsNearlyEqual(H(C::UpdateClock, 1.0, 0.05f, 0.1, 1.0).SyncBoundSeconds, 2.1, 1e-6));
    TestTrue(TEXT("dt 0.05: H 3.2"), FMath::IsNearlyEqual(H(C::UpdateClock, 1.0, 0.05f, 0.1, 1.0).HorizonSeconds, 3.2, 1e-6));
    TestTrue(TEXT("dt 0.025: T 4.1"), FMath::IsNearlyEqual(H(C::UpdateClock, 1.0, 0.025f, 0.1, 1.0).SyncBoundSeconds, 4.1, 1e-6));
    TestTrue(TEXT("sim clock: T 1.1"), FMath::IsNearlyEqual(H(C::SimClock, 1.0, 0.05f, 0.1, 1.0).SyncBoundSeconds, 1.1, 1e-6));
    TestTrue(TEXT("sim clock: H 2.2"), FMath::IsNearlyEqual(H(C::SimClock, 1.0, 0.05f, 0.1, 1.0).HorizonSeconds, 2.2, 1e-6));
    TestTrue(TEXT("legacy I 2.5, dt 0.025: T 10.1"), FMath::IsNearlyEqual(H(C::UpdateClock, 2.5, 0.025f, 0.1, 1.0).SyncBoundSeconds, 10.1, 1e-6));
    TestTrue(TEXT("dt above U: d per Update"), FMath::IsNearlyEqual(H(C::UpdateClock, 1.0, 0.2f, 0.1, 1.0).SyncBoundSeconds, 1.1, 1e-5));
    TestTrue(TEXT("dt 0 is taken as U"), FMath::IsNearlyEqual(H(C::UpdateClock, 1.0, 0.0f, 0.1, 1.0).SyncBoundSeconds, 1.1, 1e-6));
    const double H0 = H(C::UpdateClock, 1.0, 0.05f, 0.1, 1.0).HorizonSeconds;
    TestTrue(TEXT("grows with L"), H(C::UpdateClock, 1.0, 0.05f, 0.1, 2.0).HorizonSeconds > H0);
    TestTrue(TEXT("grows with I"), H(C::UpdateClock, 2.0, 0.05f, 0.1, 1.0).HorizonSeconds > H0);
    TestTrue(TEXT("grows with U"), H(C::UpdateClock, 1.0, 0.05f, 0.2, 1.0).HorizonSeconds > H0);
    const double M = FRGLSceneManager::ComputeWpMarginM(H0, 50.0, 50.0, 20.0);
    TestTrue(TEXT("M = 320 m at 50 + 50 m/s"), FMath::IsNearlyEqual(M, 320.0, 1e-6));
    TestTrue(TEXT("grows with the speeds"), FRGLSceneManager::ComputeWpMarginM(H0, 60.0, 50.0, 20.0) > M);
    TestTrue(TEXT("never below the floor"), FMath::IsNearlyEqual(FRGLSceneManager::ComputeWpMarginM(H0, 1.0, 1.0, 20.0), 20.0, 1e-9));
    TestTrue(TEXT("R_load = range + M covers (Vs + Vo) * H"), 100.0 + M >= 100.0 + (50.0 + 50.0) * H0 - 1e-9);
    // The distance trigger is no input: T_sync_max is the same whatever it would be.
    TestTrue(TEXT("no distance input"), H(C::UpdateClock, 1.0, 0.05f, 0.1, 1.0).SyncBoundSeconds == H(C::UpdateClock, 1.0, 0.05f, 0.1, 1.0).SyncBoundSeconds);

    // The manager's own horizon follows the sync clock and interval knobs (rgl.Scene.SyncClock, SyncIntervalSeconds): the
    // init sync of the fixture's sensor (range 100 m) with FWpTestScope's U 0.1 s, L 1 s and 5 + 5 m/s. Update clock, I 1 s,
    // dt 0.05 -> T 2.1, M 32 m, R_load 132 m; Update clock, I 2.5 s, dt 0.025 -> T 10.1, H 11.2, M 112 m, R_load 212 m;
    // simulation clock, I 1 s -> T 1.1, H 2.2, M 22 m, R_load 122 m whatever the dt (0.05 or 0.025); I 2.5 s -> T 2.6, R_load 137 m.
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    struct FCase { int32 Clock; float Interval; float Dt; double Bound; float LoadCm; };
    const FCase Cases[] = { { 0, 1.0f, 0.05f, 2.1, 13200.f }, { 0, 2.5f, 0.025f, 10.1, 21200.f },
                            { 1, 1.0f, 0.05f, 1.1, 12200.f }, { 1, 1.0f, 0.025f, 1.1, 12200.f }, { 1, 2.5f, 0.025f, 2.6, 13700.f } };
    int32 Index = 0;
    for (const FCase& Case : Cases)
    {
        FWpTestScope W(World, true, Case.Dt, 5.0f, Case.Clock);
        FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0A10), BaseFor(84), 15400.0 + 10.0 * Index++);
        F.SM.SetSyncInterval(Case.Interval);
        StepUpdate(F, World);   // the sensor's init sync
        const FString Line = F.SM.LastSyncLine_ForTest;
        AddInfo(Line);
        const FString What = FString::Printf(TEXT("clock %d, I %.1f, dt %.3f"), Case.Clock, Case.Interval, Case.Dt);
        TestEqual(What + TEXT(": an init sync ran"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("init")));
        TestTrue(What + FString::Printf(TEXT(": wp_sync_bound_s=%.2f"), Case.Bound),
                 Line.Contains(FString::Printf(TEXT(" wp_sync_bound_s=%.2f "), Case.Bound)));
        TestTrue(What + FString::Printf(TEXT(": R_load %.0f m"), Case.LoadCm / 100.f),
                 FMath::IsNearlyEqual(F.SM.GetWpLoadRadiusCm_ForTest(F.Sensor), Case.LoadCm, 1.f));
    }
    return true;
}

// The knobs: every new switch sets its CVar; an invalid value falls back to the default with a warning (a negative count
// is never "no cap"); 0 ms = no ms cap; the rgl_scene_config line shows every effective value and whether the world culls.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpSceneConfigShowsKnobsTest, "CarlaRGL.SceneSync.WpSceneConfigShowsKnobs",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpSceneConfigShowsKnobsTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    FWpTestScope W(World, true);
    FRGLSceneManager& SM = W.SM;
    FRGLSceneManager::ApplySceneSwitches(
        TEXT("-rgl-scene-wp-movable-culling=1 -rgl-scene-movable-min-margin-m=30 -rgl-scene-movable-sensor-speed-bound-mps=40 "
             "-rgl-scene-movable-object-speed-bound-mps=45 -rgl-scene-movable-max-update-gap-seconds=0.2 "
             "-rgl-scene-movable-create-latency-seconds=1.5 -rgl-scene-max-movable-entity-changes-per-update=256 "
             "-rgl-scene-movable-entity-budget-ms=0"), ECVF_SetByCode);
    auto F = [](const TCHAR* N) { return IConsoleManager::Get().FindConsoleVariable(N)->GetFloat(); };
    auto I = [](const TCHAR* N) { return IConsoleManager::Get().FindConsoleVariable(N)->GetInt(); };
    TestEqual(TEXT("min margin"), F(TEXT("rgl.Scene.MovableMinMarginM")), 30.f);
    TestEqual(TEXT("sensor speed"), F(TEXT("rgl.Scene.MovableSensorSpeedBoundMps")), 40.f);
    TestEqual(TEXT("object speed"), F(TEXT("rgl.Scene.MovableObjectSpeedBoundMps")), 45.f);
    TestEqual(TEXT("update gap"), F(TEXT("rgl.Scene.MovableMaxUpdateGapSeconds")), 0.2f);
    TestEqual(TEXT("create latency"), F(TEXT("rgl.Scene.MovableCreateLatencySeconds")), 1.5f);
    TestEqual(TEXT("max changes"), I(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate")), 256);
    TestEqual(TEXT("budget ms"), F(TEXT("rgl.Scene.MovableEntityBudgetMs")), 0.f);
    TestEqual(TEXT("enable"), I(TEXT("rgl.Scene.WpMovableCulling")), 1);
    SM.Update_ForTest(World, 14000.0);
    FString Line = SM.LastConfigLine_ForTest;
    AddInfo(Line);
    for (const TCHAR* Key : { TEXT(" wp_culling=1 "), TEXT(" wp_movable_culling=1 "), TEXT(" movable_min_margin_m=30.0 "),
                              TEXT(" movable_sensor_speed_bound_mps=40.0 "), TEXT(" movable_object_speed_bound_mps=45.0 "),
                              TEXT(" movable_max_update_gap_seconds=0.200 "), TEXT(" movable_create_latency_seconds=1.500 "),
                              TEXT(" max_movable_entity_changes_per_update=256 ") })
    {
        TestTrue(FString::Printf(TEXT("config line has '%s'"), Key), Line.Contains(Key));
    }
    TestTrue(TEXT("config line ends with the budget"), Line.EndsWith(TEXT(" movable_entity_budget_ms=0.00")));
    TestTrue(TEXT("config line has the sync clock (FWpTestScope: the Update clock, 1 s)"),
             Line.Contains(TEXT(" sync_clock=update sync_interval_s=1.00 ")));
    // Invalid values: the defaults are used (and said once).
    FRGLSceneManager::ApplySceneSwitches(TEXT("-rgl-scene-movable-sensor-speed-bound-mps=-5 -rgl-scene-max-movable-entity-changes-per-update=-1 "
                                                "-rgl-scene-movable-min-margin-m=5 -rgl-scene-wp-movable-culling=7"), ECVF_SetByCode);
    SM.Update_ForTest(World, 14000.1);
    Line = SM.LastConfigLine_ForTest;
    AddInfo(Line);
    TestTrue(TEXT("negative speed -> 50"), Line.Contains(TEXT(" movable_sensor_speed_bound_mps=50.0 ")));
    TestTrue(TEXT("negative count -> 512, not 'no cap'"), Line.Contains(TEXT(" max_movable_entity_changes_per_update=512 ")));
    TestTrue(TEXT("margin below 20 -> 20"), Line.Contains(TEXT(" movable_min_margin_m=20.0 ")));
    TestTrue(TEXT("enable 7 -> 1"), Line.Contains(TEXT(" wp_movable_culling=1 ")));
    return true;
}

// wait_ms= / wait_api=: the probe times the first call after Arm only (a fake clock: 10 ms, the second call is not timed),
// none -> 0 / none. On real sync lines: a new mesh's upload, an entity on a live mesh, an idle mesh destroyed, nothing.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSyncFirstRglCallTimingTest, "CarlaRGL.SceneSync.SyncFirstRglCallTiming",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSyncFirstRglCallTimingTest::RunTest(const FString& Parameters)
{
    static double FakeNow = 0.0;
    FRGLSceneManager::FRGLFirstCallProbe P;
    P.Clock = +[]() -> double { return FakeNow; };
    P.Arm();
    const int32 R1 = P.Call(TEXT("first"), [&]() { FakeNow += 0.010; return 7; });
    const int32 R2 = P.Call(TEXT("second"), [&]() { FakeNow += 0.050; return 8; });
    P.Disarm();
    TestEqual(TEXT("results passed through"), R1 + R2, 15);
    TestTrue(TEXT("first call: 10 ms"), FMath::IsNearlyEqual(P.Ms, 10.0, 1e-6));
    TestEqual(TEXT("first call's name"), FString(P.Api), FString(TEXT("first")));
    P.Arm();
    P.Disarm();
    TestEqual(TEXT("no call: 0"), P.Ms, 0.0);
    TestEqual(TEXT("no call: none"), FString(P.Api), FString(TEXT("none")));

    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    TStrongObjectPtr<UStaticMesh> Fresh(NewTransientBoxMesh(TEXT("WpProbeMesh"), 100.f));
    const FVector Base = BaseFor(61);
    FWpTestScope W(World, false);
    FCVarGuard Idle(TEXT("rgl.Scene.IdleMeshCacheMax"));
    SetCVar(TEXT("rgl.Scene.IdleMeshCacheMax"), 4);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0915), Base, 14100.0);
    F.Sync();
    auto DistanceSync = [&]() { F.SensorPos += FVector(0.f, (F.SensorPos.Y > Base.Y) ? -3000.f : 3000.f, 0.f); StepUpdate(F, World); return F.SM.LastSyncLine_ForTest; };
    auto Api = [](const FString& Line) { FString V; FParse::Value(*Line, TEXT("wait_api="), V); return V; };
    auto Ms = [](const FString& Line) { float V = -1.f; FParse::Value(*Line, TEXT("wait_ms="), V); return V; };
    DistanceSync();   // settle
    AStaticMeshActor* A = F.Keep(SpawnMeshActor(World, Fresh.Get(), Base + FVector(4000.f, 0.f, 0.f), EComponentMobility::Static));
    FString Line = DistanceSync();
    AddInfo(Line);
    TestEqual(TEXT("new mesh: rgl_mesh_create first"), Api(Line), FString(TEXT("rgl_mesh_create")));
    TestTrue(TEXT("wait_ms >= 0"), Ms(Line) >= 0.f);
    TestTrue(TEXT("older keys kept, in order"), Line.Find(TEXT(" scanned=")) < Line.Find(TEXT(" entities=")) &&
                                                Line.Find(TEXT(" skel_ms=")) < Line.Find(TEXT(" wait_ms=")));
    AStaticMeshActor* B = F.Keep(SpawnMeshActor(World, Fresh.Get(), Base + FVector(4000.f, 500.f, 0.f), EComponentMobility::Static));
    Line = DistanceSync();
    AddInfo(Line);
    TestEqual(TEXT("live mesh: rgl_entity_create first"), Api(Line), FString(TEXT("rgl_entity_create")));
    A->Destroy();
    B->Destroy();
    StepUpdate(F, World);   // the entities leave (UpdateTransforms); the mesh is idle now
    SetCVar(TEXT("rgl.Scene.IdleMeshCacheMax"), 0);
    Line = DistanceSync();
    AddInfo(Line);
    TestEqual(TEXT("idle mesh destroyed first"), Api(Line), FString(TEXT("rgl_mesh_destroy")));
    Line = DistanceSync();
    AddInfo(Line);
    TestEqual(TEXT("nothing to do: none"), Api(Line), FString(TEXT("none")));
    TestEqual(TEXT("nothing to do: 0.00"), Ms(Line), 0.f);
    return true;
}

// The late-creation contract covers what the margin protects: a part that was outside the load sphere at the previous
// look. R6 makes parts first found inside the range immediately, without late_create. A part culled 300 m away
// and then put inside the range (faster than its bound) is created inside the range: late_create.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpLateCreateOnlyForCulledPartsTest, "CarlaRGL.SceneSync.WpLateCreateOnlyForCulledParts",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpLateCreateOnlyForCulledPartsTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(80);
    FWpTestScope W(World, true);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 1);
    SetFloatCVar(TEXT("rgl.Scene.MovableCreateLatencySeconds"), 5.0f);   // no queue_age in this test
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0916), Base, 14400.0);
    F.Sync();
    auto Late = [&F]() { return F.SM.GetWpViolations_ForTest(TEXT("late_create")); };
    TArray<UStaticMeshComponent*> InRange;
    for (int32 i = 0; i < 25; ++i)
    {
        InRange.Add(F.Keep(SpawnPart(World, Cube, BandPoint(Base, i, 25, 5000.f, 9000.f)))->GetStaticMeshComponent());
    }
    AStaticMeshActor* Far = F.Keep(SpawnPart(World, Cube, Base + FVector(30000.f, 0.f, 0.f)));
    const int32 Late0 = Late();
    const int32 S0 = F.SM.SyncCount_ForTest;
    // The interval's sync finds them (in range: wanted at once; Far: culled); 1 per Update until the next sync.
    int32 Guard = 0;
    while (F.SM.SyncCount_ForTest - S0 < 1 && Guard++ < 30) StepUpdate(F, World);
    TestFalse(TEXT("Far culled"), F.SM.IsEntityRegistered_ForTest(Far->GetStaticMeshComponent()));
    while (F.SM.SyncCount_ForTest - S0 < 2 && Guard++ < 60) StepUpdate(F, World);
    int32 Waiting = 0;
    for (UStaticMeshComponent* C : InRange) Waiting += F.SM.IsEntityRegistered_ForTest(C) ? 0 : 1;
    AddInfo(FString::Printf(TEXT("in-range parts still waiting at the second sync: %d"), Waiting));
    TestEqual(TEXT("R6: no found-in-range part waits at the second sync"), Waiting, 0);
    for (int32 k = 0; k < 10; ++k) StepUpdate(F, World);
    int32 Registered = 0;
    for (UStaticMeshComponent* C : InRange) Registered += F.SM.IsEntityRegistered_ForTest(C) ? 1 : 0;
    TestEqual(TEXT("all found-in-range parts made"), Registered, InRange.Num());
    TestEqual(TEXT("no late_create for parts found inside the range"), Late() - Late0, 0);
    // A culled part put inside the range: promoted at the next sync and created inside the range -> late_create.
    Far->SetActorLocation(Base + FVector(6000.f, 0.f, 0.f));
    const int32 S1 = F.SM.SyncCount_ForTest;
    while (F.SM.SyncCount_ForTest - S1 < 1 && Guard++ < 120) StepUpdate(F, World);
    for (int32 k = 0; k < 3; ++k) StepUpdate(F, World);
    TestTrue(TEXT("Far made"), F.SM.IsEntityRegistered_ForTest(Far->GetStaticMeshComponent()));
    TestEqual(TEXT("late_create for the culled part"), Late() - Late0, 1);
    return true;
}

// R6: a loaded cell's in-range burst bypasses BOTH entity limits, including a cold mesh, while the prefetch band
// still has a budget. Sensor B alone sees the burst; A must not define the union by itself.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpInRangeDiscoveryBurstTest, "CarlaRGL.SceneSync.WpInRangeDiscoveryBurst",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpInRangeDiscoveryBurstTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    TStrongObjectPtr<UStaticMesh> Cold(NewTransientBoxMesh(TEXT("WpBurstCold"), 100.f));
    const FVector Base = BaseFor(82);
    FWpTestScope W(World, true);
    SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 1);
    SetFloatCVar(TEXT("rgl.Scene.MovableEntityBudgetMs"), 1.0e-6f);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0918), Base, 14600.0);
    const void* B = reinterpret_cast<const void*>(0x5A0919);
    const FVector BPos = Base + FVector(50000.f, 0.f, 0.f);
    F.SM.RegisterSensor(B, BPos, kRegistrationCm, F.T);
    F.SM.SetSensorAuditRange(B, kRangeCm);
    StepUpdate(F, World);
    F.SM.RegisterSensor(B, BPos, kRegistrationCm, F.T);
    TArray<UStaticMeshComponent*> Parts;
    TArray<FVector> Offsets;
    for (int32 i = 0; i < 2000; ++i)
    {
        Parts.Add(F.Keep(SpawnPart(World, Cube, BandPoint(BPos, i, 2000, 3000.f, 7000.f)))->GetStaticMeshComponent());
        Offsets.Add(BandPoint(FVector::ZeroVector, i, 2000, 3000.f, 7000.f));
    }
    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cold.Get(), BPos, Offsets, Owner, EComponentMobility::Movable);
    F.Keep(Owner); if (!TestNotNull(TEXT("burst ISM"), Ism)) { F.SM.UnregisterSensor(B); return false; }
    for (int32 i = 0; i < 20; ++i)
        F.Keep(SpawnPart(World, Cube, BandPoint(Base, i, 20, 11000.f, 12000.f)));
    const int32 S0 = F.SM.SyncCount_ForTest;
    int32 Guard = 0;
    while (F.SM.SyncCount_ForTest == S0 && Guard++ < 30)
    {
        F.SM.RegisterSensor(B, BPos, kRegistrationCm, F.T + 0.1);
        StepUpdate(F, World);
    }
    TestTrue(TEXT("discovery sync ran"), F.SM.SyncCount_ForTest > S0);
    int32 Registered = 0;
    for (UStaticMeshComponent* C : Parts) Registered += F.SM.IsEntityRegistered_ForTest(C) ? 1 : 0;
    TestEqual(TEXT("R6: all 2000 regular parts in the discovery Update"), Registered, Parts.Num());
    TestEqual(TEXT("R6: all 2000 cold-mesh instances in the discovery Update"), F.SM.GetIsmEntityCount_ForTest(Ism), 2000);
    TestTrue(TEXT("ISM consistent after the burst"), F.SM.IsIsmGroupConsistent_ForTest(Ism));
    TestTrue(TEXT("prefetch band still waiting"), F.SM.GetWpPendingJobs_ForTest() > 0);
    TestTrue(TEXT("R6: sync reports burst count"), F.SM.LastSyncLine_ForTest.Contains(TEXT("wp_inrange_added=4000")));
    TestTrue(TEXT("R6: sync reports burst time"), F.SM.LastSyncLine_ForTest.Contains(TEXT("wp_inrange_ms=")));
    TestEqual(TEXT("R6: no WP candidate missing in raw range"), F.Audit().MissingWpCulledInRange, 0);
    F.SM.UnregisterSensor(B);
    return true;
}

// R7 boundary: first 0.15 s interval (U=.1, dt=.05), then dt shrinks with prefetch work waiting. A culled object
// approaches a stationary sensor at the bound; the deadline and budget finish it BEFORE raw range, on every frame.
// Only the first <=U+dt gap is exempt; second gaps, late sensors' second gaps, and oversized first gaps still violate.
// On both sync clocks: on the simulation clock (R_load 320 m) the periodic sync counts sim time, so the shrinking dt
// needs no deadline sync; everything else is the same.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncWpStartupGapBoundaryTest, "CarlaRGL.SceneSync.WpStartupGapBoundary",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncWpStartupGapBoundaryTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(83);
    for (int32 Clock = 0; Clock <= 1; ++Clock)
    {
        const TCHAR* ClockName = Clock ? TEXT("sim clock") : TEXT("update clock");
        auto Msg = [ClockName](const TCHAR* What) { return FString::Printf(TEXT("%s: %s"), ClockName, What); };
        FWpTestScope W(World, true, 0.05f, 50.f, Clock);
        SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 1);
        SetFloatCVar(TEXT("rgl.Scene.MovableEntityBudgetMs"), 0.f);
        FAuditFixture F(World, reinterpret_cast<const void*>(0x5A091A), Base, 14700.0 + 50.0 * Clock);
        AStaticMeshActor* A = F.Keep(SpawnPart(World, Cube, Base + FVector(42100.f, 0.f, 0.f)));
        if (!TestNotNull(*Msg(TEXT("part")), A)) return false;
        UStaticMeshComponent* C = A->GetStaticMeshComponent();
        auto V = [&F](const TCHAR* K) { return F.SM.GetWpViolations_ForTest(K); };
        const int32 U0 = V(TEXT("update_gap")), P0 = V(TEXT("pose_gap"));
        const int32 Age0 = V(TEXT("queue_age")), Late0 = V(TEXT("late_create"));
        StepUpdate(F, World);
        TestFalse(*Msg(TEXT("approaching part initially culled")), F.SM.IsEntityRegistered_ForTest(C));
        const FRGLSceneAuditResult R0 = F.Audit();
        A->SetActorLocation(A->GetActorLocation() - FVector(750.f, 0.f, 0.f));
        // R10: only A was active at the interval start. Seven sensors join at its END, before the next init sync.
        // Their uninitialized state must not invalidate A's first scene-wide interval (nor excuse a later interval).
        TArray<const void*> JoinedSensors;
        for (UPTRINT i = 0; i < 7; ++i)
        {
            const void* Sensor = reinterpret_cast<const void*>(0x5A0920 + i);
            JoinedSensors.Add(Sensor);
            F.SM.RegisterSensor(Sensor, Base, kRegistrationCm, F.T + 0.15);
        }
        StepUpdate(F, World, 0.15);
        TestEqual(*Msg(TEXT("R10: seven joining sensors do not invalidate the first update interval")), V(TEXT("update_gap")) - U0, 0);
        TestEqual(*Msg(TEXT("R7: first pose gap <= U+dt exempt")), V(TEXT("pose_gap")) - P0, 0);
        for (const void* Sensor : JoinedSensors) F.SM.UnregisterSensor(Sensor);
        for (int32 i = 0; i < 8; ++i) F.Keep(SpawnPart(World, Cube, BandPoint(Base, i, 8, 20000.f, 22000.f)));
        F.SM.WorldDtOverride_ForTest = 0.005f;
        // R5 must force the scan before the old horizon expires, including the first extra dt (Update clock).
        const int32 Deadline0 = F.SM.WpDeadlineSyncs_ForTest();
        int32 PendingMax = 0, Missing = 0;
        double RegisteredGap = -1.0;
        for (int32 k = 0; k < 66; ++k)
        {
            A->SetActorLocation(A->GetActorLocation() - FVector(500.f, 0.f, 0.f));
            StepUpdate(F, World);
            PendingMax = FMath::Max(PendingMax, F.SM.GetWpPendingJobs_ForTest());
            Missing += Diff(R0, F.Audit()).MissingInRange > 0 ? 1 : 0;
            if (RegisteredGap < 0.0 && F.SM.IsEntityRegistered_ForTest(C))
                RegisteredGap = FVector::Dist(C->Bounds.Origin, Base) - C->Bounds.SphereRadius - kRangeCm;
        }
        AddInfo(FString::Printf(TEXT("%s: pending_max=%d missing_frames=%d deadline_syncs=%d registered_gap_m=%.1f"), ClockName,
                                PendingMax, Missing, F.SM.WpDeadlineSyncs_ForTest() - Deadline0, RegisteredGap / 100.0));
        TestTrue(*Msg(TEXT("creation work really waited")), PendingMax > 0);
        if (Clock == 0)
        {
            TestTrue(*Msg(TEXT("deadline forced a sync after dt shrink")), F.SM.WpDeadlineSyncs_ForTest() > Deadline0);
        }
        else
        {
            TestEqual(*Msg(TEXT("no deadline sync: the periodic sync counts sim time")), F.SM.WpDeadlineSyncs_ForTest() - Deadline0, 0);
        }
        TestEqual(*Msg(TEXT("every frame complete inside raw range")), Missing, 0);
        TestTrue(*Msg(TEXT("registered strictly before range")), RegisteredGap > 0.0);
        TestEqual(*Msg(TEXT("all creation waits met L_create")), V(TEXT("queue_age")) - Age0, 0);
        TestEqual(*Msg(TEXT("no late creation")), V(TEXT("late_create")) - Late0, 0);
        TestEqual(*Msg(TEXT("normal later update gaps")), V(TEXT("update_gap")) - U0, 0);
        TestEqual(*Msg(TEXT("normal later pose gaps")), V(TEXT("pose_gap")) - P0, 0);
        F.SM.WorldDtOverride_ForTest = 0.05f;
        StepUpdate(F, World, 0.15);
        TestEqual(*Msg(TEXT("second update gap is a violation")), V(TEXT("update_gap")) - U0, 1);
        TestEqual(*Msg(TEXT("second pose gap is a violation")), V(TEXT("pose_gap")) - P0, 1);
        const void* Late = reinterpret_cast<const void*>(0x5A091B);
        F.SM.RegisterSensor(Late, Base, kRegistrationCm, F.T);
        StepUpdate(F, World);
        F.SM.RegisterSensor(Late, Base, kRegistrationCm, F.T); // first pose's origin is the init sync
        F.T += 0.15; F.Refresh(); F.SM.RegisterSensor(Late, Base, kRegistrationCm, F.T); F.SM.Update_ForTest(World, F.T);
        const int32 P1 = V(TEXT("pose_gap"));
        F.T += 0.15; F.Refresh(); F.SM.RegisterSensor(Late, Base, kRegistrationCm, F.T); F.SM.Update_ForTest(World, F.T);
        TestEqual(*Msg(TEXT("late sensor second gap also violates (both sensors)")), V(TEXT("pose_gap")) - P1, 2);
        F.SM.UnregisterSensor(Late);
        // A fresh sensor with an oversized FIRST interval (>U+dt) cannot claim start-up.
        F.SM.UnregisterSensor(F.Sensor);
        F.Refresh(); F.SM.Update_ForTest(World, F.T);
        const int32 U1 = V(TEXT("update_gap")), P2 = V(TEXT("pose_gap"));
        StepUpdate(F, World, 0.20);
        TestEqual(*Msg(TEXT("oversized first update gap violates")), V(TEXT("update_gap")) - U1, 1);
        TestEqual(*Msg(TEXT("oversized first pose gap violates")), V(TEXT("pose_gap")) - P2, 1);
    }
    return true;
}

// ============================================================================
// P8 part 1: the clock and the interval of the periodic sync (rgl.Scene.SyncClock, rgl.Scene.SyncIntervalSeconds), the
// shared parser of the -rgl-scene-*= switches, the rgl_scene_config line and the per-sensor rgl_scene_sensor line
// ============================================================================

// The default clock is simulation time: with the sensor still and a world dt of 0.01 s per Update, Updates 0.1 ... 0.9 s
// (sim) after a sync do not sync, the Update 1.0 s after it does (trigger time). The original timer would have counted
// 0.1 s by then. Simulation time going back (earlier than the last sync) syncs at once instead of waiting to catch up.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSyncTimerCountsSimTimeTest, "CarlaRGL.SceneSync.SyncTimerCountsSimTime",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSyncTimerCountsSimTimeTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    TestEqual(TEXT("the default clock is simulation time (rgl.Scene.SyncClock = 1)"),
              IConsoleManager::Get().FindConsoleVariable(TEXT("rgl.Scene.SyncClock"))->GetInt(), 1);
    FRGLSceneManager::FWpScope_ForTest Dt(FRGLSceneManager::GetInstance(World), /*bWp=*/false, 0.01f);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0A01), BaseFor(44), 15000.0);
    F.SM.SetSyncInterval(1.0f);
    F.Sync();
    const double TS = F.T;
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    for (int32 k = 1; k <= 9; ++k)
    {
        F.T = TS + 0.1 * k;
        F.Refresh();
        F.SM.Update_ForTest(World, F.T);
    }
    TestEqual(TEXT("+0.1 ... +0.9 s: no sync"), F.SM.SyncCount_ForTest - Syncs0, 0);
    F.T = TS + 1.0;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("+1.0 s: one sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("+1.0 s: trigger=time"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("time")));
    F.T = TS + 1.5;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("+1.5 s: no sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    F.T = TS + 0.5;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("back to +0.5 s (before the last sync): a sync"), F.SM.SyncCount_ForTest - Syncs0, 2);
    TestEqual(TEXT("back to +0.5 s: trigger=time"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("time")));
    return true;
}

// rgl.Scene.SyncClock = 0, the original timer: only the world dt of the scene Updates counts. With a world dt of 0.125 s,
// seven Updates 0.2 s (sim) apart (1.4 s of simulation time, 0.875 s of Update dt) do not sync; the eighth (1.0 s) does.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSyncClockLegacyCountsUpdateFramesTest, "CarlaRGL.SceneSync.SyncClockLegacyCountsUpdateFrames",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSyncClockLegacyCountsUpdateFramesTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    FCVarGuard ClockGuard(TEXT("rgl.Scene.SyncClock"));
    SetCVar(TEXT("rgl.Scene.SyncClock"), 0);
    FRGLSceneManager::FWpScope_ForTest Dt(FRGLSceneManager::GetInstance(World), /*bWp=*/false, 0.125f);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0A02), BaseFor(44), 15100.0);
    F.SM.SetSyncInterval(1.0f);
    F.Sync();
    const double TS = F.T;
    const int32 Syncs0 = F.SM.SyncCount_ForTest;
    for (int32 k = 1; k <= 7; ++k)
    {
        F.T = TS + 0.2 * k;
        F.Refresh();
        F.SM.Update_ForTest(World, F.T);
    }
    TestEqual(TEXT("7 Updates (1.4 s sim, 0.875 s of Update dt): no sync"), F.SM.SyncCount_ForTest - Syncs0, 0);
    F.T = TS + 1.6;
    F.Refresh();
    F.SM.Update_ForTest(World, F.T);
    TestEqual(TEXT("the 8th Update (1.0 s of Update dt): one sync"), F.SM.SyncCount_ForTest - Syncs0, 1);
    TestEqual(TEXT("trigger=time"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("time")));
    return true;
}

// ApplySceneSwitches reads every -rgl-scene-*= switch (and the hit dump's) from the string it is given and sets the CVar
// with the given priority; a switch that is absent leaves its CVar alone. The guards put every value back.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSceneSwitchesParseTest, "CarlaRGL.SceneSync.SceneSwitchesParse",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSceneSwitchesParseTest::RunTest(const FString& Parameters)
{
    FCVarGuard G1(TEXT("rgl.Scene.MaxMeshCreatesPerUpdate"));
    FFloatCVarGuard G2(TEXT("rgl.Scene.CreateBudgetMs"));
    FFloatCVarGuard G3(TEXT("rgl.Scene.UnloadDistanceFactor"));
    FCVarGuard G4(TEXT("rgl.Scene.IdleMeshCacheMax"));
    FCVarGuard G5(TEXT("rgl.Scene.MaxIsmInstanceChangesPerUpdate"));
    FCVarGuard G6(TEXT("rgl.Scene.Audit"));
    FStringCVarGuard G7(TEXT("rgl.Debug.HitDumpDir"));
    FCVarGuard G8(TEXT("rgl.Debug.HitDumpEvery"));
    FCVarGuard G9(TEXT("rgl.Scene.WpMovableCulling"));
    FFloatCVarGuard G10(TEXT("rgl.Scene.MovableMinMarginM"));
    FFloatCVarGuard G11(TEXT("rgl.Scene.MovableSensorSpeedBoundMps"));
    FFloatCVarGuard G12(TEXT("rgl.Scene.MovableObjectSpeedBoundMps"));
    FFloatCVarGuard G13(TEXT("rgl.Scene.MovableMaxUpdateGapSeconds"));
    FFloatCVarGuard G14(TEXT("rgl.Scene.MovableCreateLatencySeconds"));
    FCVarGuard G15(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"));
    FFloatCVarGuard G16(TEXT("rgl.Scene.MovableEntityBudgetMs"));
    FCVarGuard G17(TEXT("rgl.Scene.SyncClock"));
    FFloatCVarGuard G18(TEXT("rgl.Scene.SyncIntervalSeconds"));
    FRGLSceneManager::ApplySceneSwitches(
        TEXT("-rgl-scene-max-mesh-creates-per-update=3 -rgl-scene-create-budget-ms=2.5 -rgl-scene-unload-distance-factor=1.5 "
             "-rgl-scene-idle-mesh-cache-max=7 -rgl-scene-max-ism-instance-changes-per-update=64 -rgl-scene-audit=5 "
             "-rgl-debug-hit-dump=/nonexistent/rgl-switch-parse -rgl-debug-hit-dump-every=4 -rgl-scene-wp-movable-culling=0 "
             "-rgl-scene-movable-min-margin-m=25 -rgl-scene-movable-sensor-speed-bound-mps=30 "
             "-rgl-scene-movable-object-speed-bound-mps=35 -rgl-scene-movable-max-update-gap-seconds=0.25 "
             "-rgl-scene-movable-create-latency-seconds=2 -rgl-scene-max-movable-entity-changes-per-update=128 "
             "-rgl-scene-movable-entity-budget-ms=6 -rgl-scene-sync-clock=0 -rgl-scene-sync-interval-seconds=2.5"),
        ECVF_SetByCode);
    auto F = [](const TCHAR* N) { return IConsoleManager::Get().FindConsoleVariable(N)->GetFloat(); };
    auto I = [](const TCHAR* N) { return IConsoleManager::Get().FindConsoleVariable(N)->GetInt(); };
    auto S = [](const TCHAR* N) { return IConsoleManager::Get().FindConsoleVariable(N)->GetString(); };
    TestEqual(TEXT("max mesh creates"), I(TEXT("rgl.Scene.MaxMeshCreatesPerUpdate")), 3);
    TestEqual(TEXT("create budget ms"), F(TEXT("rgl.Scene.CreateBudgetMs")), 2.5f);
    TestEqual(TEXT("unload distance factor"), F(TEXT("rgl.Scene.UnloadDistanceFactor")), 1.5f);
    TestEqual(TEXT("idle mesh cache max"), I(TEXT("rgl.Scene.IdleMeshCacheMax")), 7);
    TestEqual(TEXT("max ISM instance changes"), I(TEXT("rgl.Scene.MaxIsmInstanceChangesPerUpdate")), 64);
    TestEqual(TEXT("audit"), I(TEXT("rgl.Scene.Audit")), 5);
    TestEqual(TEXT("hit dump dir"), S(TEXT("rgl.Debug.HitDumpDir")), FString(TEXT("/nonexistent/rgl-switch-parse")));
    TestEqual(TEXT("hit dump every"), I(TEXT("rgl.Debug.HitDumpEvery")), 4);
    TestEqual(TEXT("WP movable culling"), I(TEXT("rgl.Scene.WpMovableCulling")), 0);
    TestEqual(TEXT("min margin"), F(TEXT("rgl.Scene.MovableMinMarginM")), 25.f);
    TestEqual(TEXT("sensor speed"), F(TEXT("rgl.Scene.MovableSensorSpeedBoundMps")), 30.f);
    TestEqual(TEXT("object speed"), F(TEXT("rgl.Scene.MovableObjectSpeedBoundMps")), 35.f);
    TestEqual(TEXT("update gap"), F(TEXT("rgl.Scene.MovableMaxUpdateGapSeconds")), 0.25f);
    TestEqual(TEXT("create latency"), F(TEXT("rgl.Scene.MovableCreateLatencySeconds")), 2.f);
    TestEqual(TEXT("max movable changes"), I(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate")), 128);
    TestEqual(TEXT("movable budget ms"), F(TEXT("rgl.Scene.MovableEntityBudgetMs")), 6.f);
    TestEqual(TEXT("sync clock"), I(TEXT("rgl.Scene.SyncClock")), 0);
    TestEqual(TEXT("sync interval"), F(TEXT("rgl.Scene.SyncIntervalSeconds")), 2.5f);
    // Only the interval: the clock keeps its value.
    FRGLSceneManager::ApplySceneSwitches(TEXT("-rgl-scene-sync-interval-seconds=3"), ECVF_SetByCode);
    TestEqual(TEXT("absent switch: the clock is left alone"), I(TEXT("rgl.Scene.SyncClock")), 0);
    TestEqual(TEXT("the interval alone: 3"), F(TEXT("rgl.Scene.SyncIntervalSeconds")), 3.f);
    return true;
}

// The rgl_scene_config line from the knobs: the defaults carry sync_clock=sim sync_interval_s=1.00; every key the line
// carried before (the create queue, the ISM cap, the unload margin, the World Partition culling) is still on it, in its
// order; other values and invalid ones (a clock other than 0/1 -> sim, an interval below 0.1 s -> 0.1 s) are shown.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSceneConfigLineShowsKnobsTest, "CarlaRGL.SceneSync.SceneConfigLineShowsKnobs",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSceneConfigLineShowsKnobsTest::RunTest(const FString& Parameters)
{
    const FString Line = FRGLSceneManager::FormatSceneConfigLine(1.0, FRGLSceneManager::ReadSceneKnobs());
    AddInfo(Line);
    TestTrue(TEXT("the line starts with its name and the time"), Line.StartsWith(TEXT("rgl_scene_config t=1.000 ")));
    TestTrue(TEXT("defaults: sync_clock=sim sync_interval_s=1.00"), Line.Contains(TEXT(" sync_clock=sim sync_interval_s=1.00 ")));
    const TCHAR* const Keys[] = {
        TEXT(" max_mesh_creates_per_update="), TEXT(" create_budget_ms="), TEXT(" unload_distance_factor="),
        TEXT(" idle_mesh_cache_max="), TEXT(" max_ism_instance_changes_per_update="), TEXT(" unload_margin_cm="),
        TEXT(" sync_clock="), TEXT(" sync_interval_s="), TEXT(" wp_culling="), TEXT(" wp_movable_culling="),
        TEXT(" movable_min_margin_m="), TEXT(" movable_sensor_speed_bound_mps="), TEXT(" movable_object_speed_bound_mps="),
        TEXT(" movable_max_update_gap_seconds="), TEXT(" movable_create_latency_seconds="),
        TEXT(" max_movable_entity_changes_per_update="), TEXT(" movable_entity_budget_ms=") };
    int32 From = 0;
    for (const TCHAR* Key : Keys)
    {
        const int32 At = Line.Find(Key, ESearchCase::CaseSensitive, ESearchDir::FromStart, From);
        TestTrue(FString::Printf(TEXT("'%s' on the line, after the keys before it"), Key), At != INDEX_NONE);
        From = (At != INDEX_NONE) ? At + 1 : From;
    }
    {
        FCVarGuard Clock(TEXT("rgl.Scene.SyncClock"));
        FFloatCVarGuard Interval(TEXT("rgl.Scene.SyncIntervalSeconds"));
        SetCVar(TEXT("rgl.Scene.SyncClock"), 0);
        SetFloatCVar(TEXT("rgl.Scene.SyncIntervalSeconds"), 2.5f);
        const FString L2 = FRGLSceneManager::FormatSceneConfigLine(2.0, FRGLSceneManager::ReadSceneKnobs());
        AddInfo(L2);
        TestTrue(TEXT("clock 0, interval 2.5"), L2.Contains(TEXT(" sync_clock=update sync_interval_s=2.50 ")));
        SetCVar(TEXT("rgl.Scene.SyncClock"), 7);
        SetFloatCVar(TEXT("rgl.Scene.SyncIntervalSeconds"), 0.01f);
        const FString L3 = FRGLSceneManager::FormatSceneConfigLine(3.0, FRGLSceneManager::ReadSceneKnobs());
        AddInfo(L3);
        TestTrue(TEXT("clock 7 -> sim, interval 0.01 -> 0.10"), L3.Contains(TEXT(" sync_clock=sim sync_interval_s=0.10 ")));
    }
    return true;
}

// The first sync of a sensor prints its effective numbers once: rgl_scene_sensor t= sensor= registration_m= range_m=
// trigger_m= (the distance trigger: 0.2 R). The fixture's sensor: R 120 m, range 100 m -> trigger 24 m. A later sync of
// the same sensor (a 30 m move) prints none.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncSceneSensorLineAtFirstSyncTest, "CarlaRGL.SceneSync.SceneSensorLineAtFirstSync",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncSceneSensorLineAtFirstSyncTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0A03), BaseFor(44), 15200.0);   // registered, never synced
    StepUpdate(F, World);
    TestEqual(TEXT("the sensor's first sync"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("init")));
    const FString Expected = FString::Printf(
        TEXT("rgl_scene_sensor t=%.3f sensor=0x%llx registration_m=120.0 range_m=100.0 trigger_m=24.0"), F.T,
        static_cast<unsigned long long>(reinterpret_cast<UPTRINT>(F.Sensor)));
    for (const FString& L : F.SM.LastSensorLines_ForTest) AddInfo(L);
    TestTrue(TEXT("the sensor line of the first sync"), F.SM.LastSensorLines_ForTest.Contains(Expected));
    F.SensorPos += FVector(3000.f, 0.f, 0.f);
    StepUpdate(F, World);
    TestEqual(TEXT("a later sync"), F.SM.LastSyncTrigger_ForTest, FString(TEXT("distance")));
    TestEqual(TEXT("no sensor line at a later sync"), F.SM.LastSensorLines_ForTest.Num(), 0);
    return true;
}
}
#endif
