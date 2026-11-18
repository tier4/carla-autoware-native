// Copyright (c) 2026 RGL Integration for CARLA.
// Scene synchronization of FRGLSceneManager on the game world of EmptyMap, judged with the scene audit.
// Shared fixture and helpers: RGLSceneTestUtil.h (each test works around its own BaseFor slot).
// First the audit counts the World Partition fixes are judged by: missing_steady (the miss that the last sync
// should already have registered), the ISM part of the ghosts, and spline mesh components in the scene.

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/World.h"
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
// to register) and around the sensor now (what is still needed). The sensor moves 20 m (below the 24 m distance
// trigger) with no Update, then two cubes appear: A 110 m behind the last sync point (inside the last sync's R,
// 130 m from the sensor = outside its current R) and B 50 m ahead (inside both).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneSyncMissingSteadyNeedsBothSpheresTest, "CarlaRGL.SceneSync.MissingSteadyNeedsBothSpheres",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneSyncMissingSteadyNeedsBothSpheresTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadMeshReady(kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(31);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0020), Base, 8100.0);
    F.Sync();                                    // the last sync: R = 120 m around P0 = Base

    const float MoveCm = 2000.f;
    TestTrue(TEXT("the move stays below the distance trigger (0.2 R)"), MoveCm < 0.2f * kRegistrationCm);
    F.SensorPos = Base + FVector(MoveCm, 0.f, 0.f);
    F.Refresh();                                 // no Update: the last sync's sphere stays at P0
    const FRGLSceneAuditResult R0 = F.Audit();

    TestNotNull(TEXT("cube A"), F.Keep(SpawnMeshActor(World, Cube, Base - FVector(11000.f, 0.f, 0.f), EComponentMobility::Static)));
    TestNotNull(TEXT("cube B"), F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static)));
    const FRGLSceneAuditResult R1 = F.Audit();
    AddInfo(FRGLSceneManager::FormatAuditLine(31, F.T, R1));
    const FRGLSceneAuditResult D = Diff(R0, R1);
    TestEqual(TEXT("A and B inside the last sync's R: missing_sync +2"), D.MissingSync, 2);
    TestEqual(TEXT("only B inside the current R as well: missing_steady +1"), D.MissingSteady, 1);
    TestEqual(TEXT("only B inside the range: missing_in_range +1"), D.MissingInRange, 1);
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
}
#endif
