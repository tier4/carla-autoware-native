// Copyright (c) 2026 RGL Integration for CARLA.
// Shared fixture and helpers of the scene tests (CarlaRGL.SceneAudit.*, CarlaRGL.SceneSync.*) on the game world of
// EmptyMap. The scene manager of a world is shared by every test, so counts are compared as differences around each
// step (Diff), and every test works around its own base point (BaseFor), 1 km from the others.
// BaseFor slots in use: 0-9, 16, 20 (SceneAudit), 10-15, 17-19, 31-33, 40, 46, 47 (SceneSync; 12 by the two level-sync
// tests).
// Give each new test a slot nobody else uses.

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
inline UInstancedStaticMeshComponent* SpawnIsm(UWorld* World, UStaticMesh* Mesh, const FVector& Loc,
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
    return D;
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
/// shared scene is synced clean again when the fixture goes. The sync interval is put back to its default
/// (1 s) too: a test that stretches it must not leak the value into the tests that run after it.
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
        SM.SetSyncInterval(1.0f);                    // the default
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
