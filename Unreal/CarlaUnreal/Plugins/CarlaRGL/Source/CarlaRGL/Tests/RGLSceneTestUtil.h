// Copyright (c) 2026 RGL Integration for CARLA.
// Shared fixture and helpers of the scene tests (CarlaRGL.SceneAudit.*, CarlaRGL.SceneSync.*) on the game world of
// EmptyMap. The scene manager of a world is shared by every test, so counts are compared as differences around each
// step (Diff), and every test works around its own base point (BaseFor), 1 km from the others.
// BaseFor slots in use: 0-9, 16, 20, 77, 81 (SceneAudit), 10-15, 17-19, 21-27, 31-38, 40, 44-51, 60, 61-76, 78-80,
// 82-84 (SceneSync; 12 by the two level-sync tests, 44 by the sync-clock tests, 45 by the three in-range create tests,
// 48 by RangeEdgeGapClosed, 49-50 by the ISM re-evaluation when every Update syncs, 51 by the live clock change).
// Free: 28-30, 39, 41-43, 52-59, 85 and up. Give each new test a slot nobody else uses.

#pragma once

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
#include "UObject/Package.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/AggregateGeom.h"
#if WITH_EDITOR
#include "StaticMeshCompiler.h"
#endif
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLSceneManager.h"

namespace RGLSceneTestUtil
{
inline const TCHAR* const kCube   = TEXT("/Engine/BasicShapes/Cube.Cube");
inline const TCHAR* const kSphere = TEXT("/Engine/BasicShapes/Sphere.Sphere");
inline const TCHAR* const kCylinder = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");   // used by one test only (no live mesh)
inline constexpr float kZ = 30000.f;               // 300 m up: clear of the map
inline constexpr float kRegistrationCm = 12000.f;  // RegisterSensor's distance: R = 120 m
inline constexpr float kRangeCm = 10000.f;         // SetSensorAuditRange: 100 m (R - 20 m, as for a 100 m lidar)

inline UWorld* GetGameWorld()
{
    for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
        if (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE) return Ctx.World();
    return nullptr;
}

// CVar helpers: set with ECVF_SetByCode, restored by the guards when the test leaves.
inline void SetCVar(const TCHAR* Name, int32 V) { IConsoleManager::Get().FindConsoleVariable(Name)->Set(V, ECVF_SetByCode); }
inline void SetFloatCVar(const TCHAR* Name, float V) { IConsoleManager::Get().FindConsoleVariable(Name)->Set(V, ECVF_SetByCode); }
struct FCVarGuard { FString Name; int32 Old; FCVarGuard(const TCHAR* N) : Name(N), Old(IConsoleManager::Get().FindConsoleVariable(N)->GetInt()) {} ~FCVarGuard() { SetCVar(*Name, Old); } };
struct FFloatCVarGuard
{
    FString Name; float Old;
    FFloatCVarGuard(const TCHAR* N) : Name(N), Old(IConsoleManager::Get().FindConsoleVariable(N)->GetFloat()) {}
    ~FFloatCVarGuard() { SetFloatCVar(*Name, Old); }
};
struct FStringCVarGuard
{
    FString Name; FString Old;
    FStringCVarGuard(const TCHAR* N) : Name(N), Old(IConsoleManager::Get().FindConsoleVariable(N)->GetString()) {}
    ~FStringCVarGuard() { IConsoleManager::Get().FindConsoleVariable(*Name)->Set(*Old, ECVF_SetByCode); }
};

/// LoadObject<UStaticMesh> that returns the mesh only after the editor's asynchronous static mesh build is done. The
/// tests run in the editor binary (-game): the first load of an asset in a process leaves it compiling, and an ISM
/// registered on it in that frame was not registered by the next sync (the first test of a fresh process failed, the
/// same test after other tests passed). Cooked builds never compile (no-op). bOutWasCompiling: for the test's log.
inline UStaticMesh* LoadMeshReady(const TCHAR* Path, bool* bOutWasCompiling = nullptr)
{
    UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, Path);
    bool bCompiling = false;
#if WITH_EDITOR
    bCompiling = Mesh && Mesh->IsCompiling();
    if (bCompiling)
    {
        FStaticMeshCompilingManager::Get().FinishCompilation({ Mesh });
    }
#endif
    if (bOutWasCompiling) *bOutWasCompiling = bCompiling;
    return Mesh;
}

/// An empty transient static mesh whose name starts with BaseName (made unique in the transient package), for the
/// name-rule tests: they must not depend on an engine asset being there (a missing asset would make them check
/// nothing). It has no geometry and no render data; only a mesh the name rule leaves out may use it (never uploaded).
inline UStaticMesh* NewNamedTransientMesh(const TCHAR* BaseName)
{
    const FName Name = MakeUniqueObjectName(GetTransientPackage(), UStaticMesh::StaticClass(), FName(BaseName));
    return NewObject<UStaticMesh>(GetTransientPackage(), Name, RF_Transient);
}

/// A transient static mesh (NewNamedTransientMesh) that the scene manager can upload without render data: its simple
/// collision is one box of BoxCm, which the extraction's last fallback reads. Unlike a DuplicateObject copy of an engine
/// mesh (its body setup is duplicatetransient, so only render data would be left, read back from a null RHI), it needs
/// no build and no GPU. Not standalone: the next garbage collection takes it once nothing uses it.
inline void AddBoxCollision(UStaticMesh* Mesh, float BoxCm)
{
    Mesh->CreateBodySetup();
    if (UBodySetup* Body = Mesh->GetBodySetup())
    {
        Body->AggGeom.BoxElems.Add(FKBoxElem(BoxCm));
    }
}
inline UStaticMesh* NewTransientBoxMesh(const TCHAR* BaseName, float BoxCm)
{
    UStaticMesh* Mesh = NewNamedTransientMesh(BaseName);
    AddBoxCollision(Mesh, BoxCm);
    return Mesh;
}

/// Base point of a test: 2 km west, (Slot + 1) km south, 300 m up (the skeletal tests stay near x = 0).
inline FVector BaseFor(int32 Slot) { return FVector(-200000.f, -100000.f * static_cast<float>(Slot + 1), kZ); }

/// A static mesh actor with the mesh and mobility set while its component is unregistered: once the world has
/// begun play, UStaticMeshComponent::SetStaticMesh refuses a registered Static component, and the native
/// component is registered in PostSpawnInitialize even for a deferred spawn. Mesh may be nullptr.
inline AStaticMeshActor* SpawnMeshActor(UWorld* World, UStaticMesh* Mesh, const FVector& Loc, EComponentMobility::Type Mobility,
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
/// Mobility: another mobility for the World Partition culling tests (Movable, Stationary).
inline UInstancedStaticMeshComponent* SpawnIsm(UWorld* World, UStaticMesh* Mesh, const FVector& Loc,
                                               const TArray<FVector>& LocalOffsets, AActor*& OutOwner,
                                               EComponentMobility::Type Mobility = EComponentMobility::Static)
{
    OutOwner = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity);
    if (!OutOwner) return nullptr;
    UInstancedStaticMeshComponent* Comp = NewObject<UInstancedStaticMeshComponent>(OutOwner);
    Comp->SetMobility(Mobility);
    Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Comp->SetStaticMesh(Mesh);
    OutOwner->SetRootComponent(Comp);
    Comp->SetRelativeLocation_Direct(Loc);
    for (const FVector& Offset : LocalOffsets) Comp->AddInstance(FTransform(Offset));
    Comp->RegisterComponent();
    return Comp;
}

/// After - Before on every int counter (the timings and the stream fields are copied from After).
/// A field added to FRGLSceneAuditResult must be added here too.
inline FRGLSceneAuditResult Diff(const FRGLSceneAuditResult& Before, const FRGLSceneAuditResult& After)
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
    D.GhostIsm          -= Before.GhostIsm;
    D.GhostIsmInRange   -= Before.GhostIsmInRange;
    D.MissingSteady     -= Before.MissingSteady;
    D.SplineRegistered  -= Before.SplineRegistered;
    D.CacheStale        -= Before.CacheStale;
    D.PendingIsm        -= Before.PendingIsm;
    D.MissingWpCulledInRange -= Before.MissingWpCulledInRange;
    D.WpPending         -= Before.WpPending;
    return D;
}

/// The World Partition culling of non-Static parts for one test: the map is taken as a World Partition one (or not),
/// the world dt is fixed, and every knob of the culling is put back when the test leaves. Declare it BEFORE the
/// FAuditFixture, so that the fixture's clean-up sync still runs with it and the next test starts from the original rule.
/// The defaults it sets: speed bounds 5 m/s, U 0.1 s, L_create 1 s, margin floor 20 m, budget 512 / 4 ms, and the sync
/// clock SyncClock (default 0, the Update clock, which the numbers below assume), so that with I = 1 s and dt = 0.05 s:
/// T_sync_max = 2.1 s, H = 3.2 s, M = max(20, 10 * 3.2) = 32 m, R_load = 100 + 32 = 132 m (the fixture's sensor: R 120 m,
/// range 100 m), R_unload = 132 + 20 = 152 m. With SyncClock = 1 (simulation time): T_sync_max = 1.1 s, H = 2.2 s,
/// M = 22 m, R_load = 122 m, R_unload = 142 m.
struct FWpTestScope
{
    FRGLSceneManager& SM;
    FRGLSceneManager::FWpScope_ForTest Scope;
    FCVarGuard Enable{TEXT("rgl.Scene.WpMovableCulling")};
    FFloatCVarGuard MinMargin{TEXT("rgl.Scene.MovableMinMarginM")};
    FFloatCVarGuard SensorSpeed{TEXT("rgl.Scene.MovableSensorSpeedBoundMps")};
    FFloatCVarGuard ObjectSpeed{TEXT("rgl.Scene.MovableObjectSpeedBoundMps")};
    FFloatCVarGuard Gap{TEXT("rgl.Scene.MovableMaxUpdateGapSeconds")};
    FFloatCVarGuard Latency{TEXT("rgl.Scene.MovableCreateLatencySeconds")};
    FCVarGuard MaxChanges{TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate")};
    FFloatCVarGuard BudgetMs{TEXT("rgl.Scene.MovableEntityBudgetMs")};
    FFloatCVarGuard Unload{TEXT("rgl.Scene.UnloadDistanceFactor")};
    FCVarGuard Clock{TEXT("rgl.Scene.SyncClock")};
    FFloatCVarGuard Interval{TEXT("rgl.Scene.SyncIntervalSeconds")};
    FWpTestScope(UWorld* World, bool bWp, float Dt = 0.05f, float SpeedMps = 5.0f, int32 SyncClock = 0)
        : SM(FRGLSceneManager::GetInstance(World)), Scope(SM, bWp, Dt)
    {
        SetCVar(TEXT("rgl.Scene.SyncClock"), SyncClock);
        SetFloatCVar(TEXT("rgl.Scene.SyncIntervalSeconds"), 1.0f);
        SetCVar(TEXT("rgl.Scene.WpMovableCulling"), 1);
        SetFloatCVar(TEXT("rgl.Scene.MovableMinMarginM"), 20.0f);
        SetFloatCVar(TEXT("rgl.Scene.MovableSensorSpeedBoundMps"), SpeedMps);
        SetFloatCVar(TEXT("rgl.Scene.MovableObjectSpeedBoundMps"), SpeedMps);
        SetFloatCVar(TEXT("rgl.Scene.MovableMaxUpdateGapSeconds"), 0.1f);
        SetFloatCVar(TEXT("rgl.Scene.MovableCreateLatencySeconds"), 1.0f);
        SetCVar(TEXT("rgl.Scene.MaxMovableEntityChangesPerUpdate"), 512);
        SetFloatCVar(TEXT("rgl.Scene.MovableEntityBudgetMs"), 4.0f);
        SetFloatCVar(TEXT("rgl.Scene.UnloadDistanceFactor"), 1.0f);
        SM.ResetWpUpdateClock_ForTest();   // the previous test's clock is no time going back
    }
};

/// A Movable (or Stationary) static mesh actor for the culling tests (SpawnMeshActor with another mobility).
inline AStaticMeshActor* SpawnPart(UWorld* World, UStaticMesh* Mesh, const FVector& Loc,
                                   EComponentMobility::Type Mobility = EComponentMobility::Movable)
{
    return SpawnMeshActor(World, Mesh, Loc, Mobility);
}

/// Every defect counter of the difference D is 0.
inline void TestNoDefects(FAutomationTestBase& Test, const FString& What, const FRGLSceneAuditResult& D)
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
    Test.TestEqual(What + TEXT(": ghost_ism"), D.GhostIsm, 0);
    Test.TestEqual(What + TEXT(": ghost_ism_in_range"), D.GhostIsmInRange, 0);
    Test.TestEqual(What + TEXT(": missing_steady"), D.MissingSteady, 0);
    Test.TestEqual(What + TEXT(": cache_stale"), D.CacheStale, 0);
}

/// One sensor (R = 120 m, audit range 100 m) at SensorPos, scene time T; spawned actors are destroyed and the
/// shared scene is synced clean again when the fixture goes. The sync clock and interval (rgl.Scene.SyncClock,
/// rgl.Scene.SyncIntervalSeconds: process-wide CVars, SetSyncInterval writes the second) are put back to the values the
/// fixture found, after its clean-up sync: a test that stretches the interval must not leak it into the tests after it.
struct FAuditFixture
{
    UWorld* World;
    FRGLSceneManager& SM;
    const void* Sensor;
    FVector SensorPos;
    double T;
    TArray<AActor*> Spawned;
    FCVarGuard ClockGuard{TEXT("rgl.Scene.SyncClock")};
    FFloatCVarGuard IntervalGuard{TEXT("rgl.Scene.SyncIntervalSeconds")};

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
}   // namespace RGLSceneTestUtil
#endif
