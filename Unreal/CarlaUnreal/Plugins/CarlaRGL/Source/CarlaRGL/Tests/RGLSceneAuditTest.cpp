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

namespace RGLSceneAuditTests
{
static const TCHAR* kCube   = TEXT("/Engine/BasicShapes/Cube.Cube");
static const TCHAR* kSphere = TEXT("/Engine/BasicShapes/Sphere.Sphere");
static const float  kZ = 30000.f;               // 300 m up: clear of the map
static const float  kRegistrationCm = 12000.f;  // RegisterSensor's distance: R = 120 m
static const float  kRangeCm = 10000.f;         // SetSensorAuditRange: 100 m (R - 20 m, as for a 100 m lidar)

static UWorld* GetGameWorld()
{
    for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
        if (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE) return Ctx.World();
    return nullptr;
}
// Local copies (unity build: this file's own namespace) of the CVar helpers of the skeletal tests.
static void SetCVar(const TCHAR* Name, int32 V) { IConsoleManager::Get().FindConsoleVariable(Name)->Set(V, ECVF_SetByCode); }
struct FCVarGuard { FString Name; int32 Old; FCVarGuard(const TCHAR* N) : Name(N), Old(IConsoleManager::Get().FindConsoleVariable(N)->GetInt()) {} ~FCVarGuard() { SetCVar(*Name, Old); } };
struct FStringCVarGuard
{
    FString Name; FString Old;
    FStringCVarGuard(const TCHAR* N) : Name(N), Old(IConsoleManager::Get().FindConsoleVariable(N)->GetString()) {}
    ~FStringCVarGuard() { IConsoleManager::Get().FindConsoleVariable(*Name)->Set(*Old, ECVF_SetByCode); }
};

/// Base point of a test: 2 km west, (Slot + 1) km south, 300 m up (the skeletal tests stay near x = 0).
static FVector BaseFor(int32 Slot) { return FVector(-200000.f, -100000.f * static_cast<float>(Slot + 1), kZ); }

/// A static mesh actor with the mesh and mobility set while its component is unregistered: once the world has
/// begun play, UStaticMeshComponent::SetStaticMesh refuses a registered Static component, and the native
/// component is registered in PostSpawnInitialize even for a deferred spawn. Mesh may be nullptr.
static AStaticMeshActor* SpawnMeshActor(UWorld* World, UStaticMesh* Mesh, const FVector& Loc, EComponentMobility::Type Mobility,
                                        FName Name = NAME_None, const FVector& Scale = FVector::OneVector)
{
    FActorSpawnParameters Params;
    Params.Name = Name;
    Params.NameMode = FActorSpawnParameters::ESpawnActorNameMode::Requested;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    Params.bDeferConstruction = true;
    const FTransform Tf(FRotator::ZeroRotator, Loc, Scale);
    AStaticMeshActor* Actor = World->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), Tf, Params);
    if (!Actor) return nullptr;
    UStaticMeshComponent* Comp = Actor->GetStaticMeshComponent();
    const bool bWasRegistered = Comp->IsRegistered();
    if (bWasRegistered) Comp->UnregisterComponent();
    Comp->SetMobility(Mobility);
    if (Mesh) Comp->SetStaticMesh(Mesh);
    if (bWasRegistered) Comp->RegisterComponent();
    Actor->FinishSpawning(Tf);
    if (Mesh && Comp->GetStaticMesh() != Mesh)
    {
        UE_LOG(LogTemp, Error, TEXT("SpawnMeshActor: mesh %s was not applied to %s"), *GetNameSafe(Mesh), *Actor->GetName());
    }
    return Actor;
}

/// A Static ISMC (no collision: instance moves must not touch static bodies) at Loc with one instance per offset.
static UInstancedStaticMeshComponent* SpawnIsm(UWorld* World, UStaticMesh* Mesh, const FVector& Loc,
                                               const TArray<FVector>& LocalOffsets, AActor*& OutOwner)
{
    OutOwner = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity);
    if (!OutOwner) return nullptr;
    UInstancedStaticMeshComponent* Comp = NewObject<UInstancedStaticMeshComponent>(OutOwner);
    Comp->SetMobility(EComponentMobility::Static);
    Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Comp->SetStaticMesh(Mesh);
    OutOwner->SetRootComponent(Comp);
    Comp->SetRelativeLocation_Direct(Loc);
    for (const FVector& Offset : LocalOffsets) Comp->AddInstance(FTransform(Offset));
    Comp->RegisterComponent();
    return Comp;
}

/// After - Before on every counter (the timings and the stream fields are copied from After).
static FRGLSceneAuditResult Diff(const FRGLSceneAuditResult& Before, const FRGLSceneAuditResult& After)
{
    FRGLSceneAuditResult D = After;
    D.Ghost             -= Before.Ghost;
    D.GhostInRange      -= Before.GhostInRange;
    D.MissingSync       -= Before.MissingSync;
    D.MissingInRange    -= Before.MissingInRange;
    D.MeshMismatch      -= Before.MeshMismatch;
    D.StaleInstance     -= Before.StaleInstance;
    D.ExcludedByName    -= Before.ExcludedByName;
    D.Unextractable     -= Before.Unextractable;
    D.NonSmCollision    -= Before.NonSmCollision;
    D.HlodRegistered    -= Before.HlodRegistered;
    D.Registered        -= Before.Registered;
    D.Pending           -= Before.Pending;
    D.MovableRegistered -= Before.MovableRegistered;
    D.LevelsAdded       -= Before.LevelsAdded;
    D.MissingIsmSync    -= Before.MissingIsmSync;
    D.MissingIsmInRange -= Before.MissingIsmInRange;
    return D;
}

/// Every defect counter of the difference D is 0.
static void TestNoDefects(FAutomationTestBase& Test, const FString& What, const FRGLSceneAuditResult& D)
{
    Test.TestEqual(What + TEXT(": ghost"), D.Ghost, 0);
    Test.TestEqual(What + TEXT(": ghost_in_range"), D.GhostInRange, 0);
    Test.TestEqual(What + TEXT(": missing_sync"), D.MissingSync, 0);
    Test.TestEqual(What + TEXT(": missing_in_range"), D.MissingInRange, 0);
    Test.TestEqual(What + TEXT(": mesh_mismatch"), D.MeshMismatch, 0);
    Test.TestEqual(What + TEXT(": stale_instance"), D.StaleInstance, 0);
    Test.TestEqual(What + TEXT(": excluded_by_name"), D.ExcludedByName, 0);
    Test.TestEqual(What + TEXT(": unextractable"), D.Unextractable, 0);
    Test.TestEqual(What + TEXT(": non_sm_collision"), D.NonSmCollision, 0);
    Test.TestEqual(What + TEXT(": hlod_registered"), D.HlodRegistered, 0);
}

/// One sensor (R = 120 m, audit range 100 m) at SensorPos, scene time T; spawned actors are destroyed and the
/// shared scene is synced clean again when the fixture goes.
struct FAuditFixture
{
    UWorld* World;
    FRGLSceneManager& SM;
    const void* Sensor;
    FVector SensorPos;
    double T;
    TArray<AActor*> Spawned;

    FAuditFixture(UWorld* InWorld, const void* InSensor, const FVector& InSensorPos, double InT)
        : World(InWorld), SM(FRGLSceneManager::GetInstance(InWorld)), Sensor(InSensor), SensorPos(InSensorPos), T(InT)
    {
        Refresh();
    }
    ~FAuditFixture()
    {
        for (AActor* A : Spawned) if (IsValid(A)) A->Destroy();
        SM.UnregisterSensor(Sensor);
        SM.SyncStaticNow_ForTest(World, T + 0.05);   // drop this test's entities from the shared scene
    }
    void Refresh()
    {
        SM.RegisterSensor(Sensor, SensorPos, kRegistrationCm, T);
        SM.SetSensorAuditRange(Sensor, kRangeCm);
    }
    void Sync() { T += 0.05; Refresh(); SM.SyncStaticNow_ForTest(World, T); }
    FRGLSceneAuditResult Audit() const { return SM.AuditScene(World); }
    template <class TActor> TActor* Keep(TActor* Actor) { if (Actor) Spawned.Add(Actor); return Actor; }
};

// ============================================================================
// Task 3: AuditScene
// ============================================================================

// A static cube in range: registered by a sync, and no defect counter moves.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditCleanTest, "CarlaRGL.SceneAudit.Clean",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditCleanTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
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
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
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
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
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

// P8: range 100 m, R = 120 m, distance trigger 0.2 R = 24 m. A cube whose bounds start at 122 m is outside R
// at the sync; a 23 m move brings it inside the range without triggering a sync: missing in range, while the
// registration rule (the sphere at the last sync) had nothing to register.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditRangeEdgeGapTest, "CarlaRGL.SceneAudit.RangeEdgeGap",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditRangeEdgeGapTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
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
    TestTrue(TEXT("the move stays below the distance trigger (0.2 R)"), MoveCm < 0.2f * kRegistrationCm);
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
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, kSphere); if (!TestNotNull(TEXT("sphere"), Sphere)) return false;
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

// P3: the instances of a registered ISM are chosen once. An instance outside every sensor sphere at registration
// stays out after the sensor comes close and a sync runs; the audit counts it as an ISM-instance miss.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditIsmNotReevaluatedTest, "CarlaRGL.SceneAudit.IsmNotReevaluated",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditIsmNotReevaluatedTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(16);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0011), Base, 8000.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    // instance 0 at 40 m (inside R = 120 m), instance 1 at 200 m (outside R and outside the range)
    AActor* Owner = nullptr;
    UInstancedStaticMeshComponent* Ism = SpawnIsm(World, Cube, Base + FVector(4000.f, 0.f, 0.f),
                                                  { FVector::ZeroVector, FVector(16000.f, 0.f, 0.f) }, Owner);
    F.Keep(Owner);
    if (!TestNotNull(TEXT("ISMC"), Ism)) return false;
    F.Sync();
    const FRGLSceneAuditResult R1 = F.Audit();
    TestEqual(TEXT("only instance 0 registered"), Diff(R0, R1).Registered, 1);
    TestEqual(TEXT("instance 1 is out of every sphere: not missing yet"), Diff(R0, R1).MissingInRange, 0);

    // the sensor moves 150 m toward instance 1 (now 50 m away, instance 0 is 110 m behind) and a sync runs
    F.SensorPos = Base + FVector(15000.f, 0.f, 0.f);
    F.Sync();
    const FRGLSceneAuditResult D = Diff(R1, F.Audit());
    TestEqual(TEXT("instance 1 inside the range but never added: missing_in_range +1"), D.MissingInRange, 1);
    TestEqual(TEXT("... and it is an ISM instance: missing_ism_in_range +1"), D.MissingIsmInRange, 1);
    TestEqual(TEXT("not a ghost"), D.Ghost, 0);
    return true;
}

// An ISM instance moved after registration: the entity stays where the instance was.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditStaleInstanceTest, "CarlaRGL.SceneAudit.StaleInstance",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditStaleInstanceTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
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

// The "Sky" name rule drops a building called Skyscraper_01: counted as excluded, not as missing.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSceneAuditExcludedByNameTest, "CarlaRGL.SceneAudit.ExcludedByName",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FSceneAuditExcludedByNameTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    const FVector Base = BaseFor(6);
    FAuditFixture F(World, reinterpret_cast<const void*>(0x5A0007), Base, 5600.0);
    F.Sync();
    const FRGLSceneAuditResult R0 = F.Audit();

    AStaticMeshActor* A = F.Keep(SpawnMeshActor(World, Cube, Base + FVector(5000.f, 0.f, 0.f), EComponentMobility::Static,
                                                FName(TEXT("Skyscraper_01"))));
    if (!TestNotNull(TEXT("building"), A)) return false;
    TestTrue(FString::Printf(TEXT("owner name carries Sky (%s)"), *A->GetName()), A->GetName().Contains(TEXT("Sky")));
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
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
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
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
    UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, kSphere); if (!TestNotNull(TEXT("sphere"), Sphere)) return false;
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
    if (Ball) Ball->Destroy();   // its mesh becomes an idle cache entry (FindCachedMesh would destroy a stale one)
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
    const FString Line = FRGLSceneManager::FormatAuditLine(42, 1.5, R);
    const FString Expected = TEXT(
        "rgl_scene_audit frame=42 sim_t=1.500 ghost=1 ghost_in_range=0 missing_sync=0 missing_in_range=0 "
        "mesh_mismatch=0 stale_instance=0 excluded_by_name=0 unextractable=0 non_sm_collision=0 hlod_registered=0 "
        "registered=0 pending=0 movable_registered=0 levels_added=0 since_sync_s=0.00 stream_covered=-1 "
        "parent_stream_radius_m=-1.0 frame_ms=0.00 audit_ms=0.000 missing_ism_sync=0 missing_ism_in_range=0");
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
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;
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

// Off unless asked for: no line and no LevelAddedToWorld binding.
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
    TestFalse(TEXT("no LevelAddedToWorld binding while off"), FWorldDelegates::LevelAddedToWorld.IsBoundToObject(&SM));
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
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, kCube); if (!TestNotNull(TEXT("cube"), Cube)) return false;

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
}
#endif
