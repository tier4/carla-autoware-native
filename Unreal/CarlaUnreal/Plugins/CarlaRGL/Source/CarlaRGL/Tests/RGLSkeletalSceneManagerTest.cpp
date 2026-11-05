#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Pawn.h"
#include "Components/SkeletalMeshComponent.h"
#include "HAL/IConsoleManager.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLSceneManager.h"

namespace RGLSkeletalTestsScene
{
static const TCHAR* kWalkerSK = TEXT("/Game/Carla/Static/Pedestrian/EuroG02_G2/SK_EuroG02_A_G2.SK_EuroG02_A_G2");
static const TCHAR* kAudiSK   = TEXT("/Game/Carla/Static/Car/4Wheeled/AudiTT/SK_AudiTT.SK_AudiTT");
static const float  kZ = 30000.f;   // 300 m up

static UWorld* GetGameWorld()
{
    for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
        if (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE) return Ctx.World();
    return nullptr;
}
// Pawn-owned skeletal component with an explicit, non-default tick policy so restoration is observable.
static USkeletalMeshComponent* SpawnPawn(UWorld* World, USkeletalMesh* Mesh, const FVector& Loc, APawn*& OutPawn)
{
    OutPawn = World->SpawnActor<APawn>(APawn::StaticClass(), FTransform::Identity);
    USkeletalMeshComponent* Comp = NewObject<USkeletalMeshComponent>(OutPawn);
    OutPawn->SetRootComponent(Comp);
    Comp->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::OnlyTickPoseWhenRendered;   // engine default is AlwaysTick...
    Comp->bEnableUpdateRateOptimizations = true;
    Comp->SetSkeletalMeshAsset(Mesh);
    Comp->RegisterComponent();
    OutPawn->SetActorLocation(Loc);
    Comp->RefreshBoneTransforms();
    return Comp;
}
static void SetCVar(const TCHAR* Name, int32 V) { IConsoleManager::Get().FindConsoleVariable(Name)->Set(V, ECVF_SetByCode); }
struct FCVarGuard { FString Name; int32 Old; FCVarGuard(const TCHAR* N) : Name(N), Old(IConsoleManager::Get().FindConsoleVariable(N)->GetInt()) {} ~FCVarGuard() { SetCVar(*Name, Old); } };

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLifecycleTest, "CarlaRGL.Skeletal.Scene.Lifecycle",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FLifecycleTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, kWalkerSK); if (!TestNotNull(TEXT("walker SK"), Mesh)) return false;
    USkeletalMesh* Other = LoadObject<USkeletalMesh>(nullptr, kAudiSK);  if (!TestNotNull(TEXT("audi SK"), Other)) return false;
    FCVarGuard G1(TEXT("rgl.SkeletalMesh.Enable")), G2(TEXT("rgl.SkeletalMesh.AlwaysTickPose")), G3(TEXT("rgl.SkeletalMesh.MaxEntities"));
    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 1); SetCVar(TEXT("rgl.SkeletalMesh.AlwaysTickPose"), 1);

    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    const void* Sensor = reinterpret_cast<const void*>(0xC0FFEE);
    const FVector SensorPos(0.f, 0.f, kZ);
    double T = 100.0;
    auto Step = [&](bool bSync) { SM.RegisterSensor(Sensor, SensorPos, 10000.f, T); if (bSync) SM.SyncSkeletalNow_ForTest(World, T); SM.UpdateSkeletalPosesNow_ForTest(T); T += 0.05; };

    const int32 Before = SM.GetSkeletalRegisteredCount_ForTest();
    TArray<APawn*> Pawns; TArray<USkeletalMeshComponent*> Comps;
    for (int32 i = 0; i < 5; ++i) { APawn* P; Comps.Add(SpawnPawn(World, Mesh, FVector(500.f * (i + 1), 0.f, kZ), P)); Pawns.Add(P); }

    Step(true);
    TestEqual(TEXT("5 registered"), SM.GetSkeletalRegisteredCount_ForTest() - Before, 5);
    TestEqual(TEXT("5 live (entity created with first pose in the same step)"), SM.GetSkeletalLiveEntityCount_ForTest(), 5);
    TestEqual(TEXT("1 cached mesh"), SM.GetSkeletalMeshCacheCount_ForTest(), 1);
    TestEqual(TEXT("policy applied"), (int)Comps[0]->VisibilityBasedAnimTickOption, (int)EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones);
    TestFalse(TEXT("URO disabled"), (bool)Comps[0]->bEnableUpdateRateOptimizations);

    // live CVar 1 -> 0 restores the values we set before registration
    SetCVar(TEXT("rgl.SkeletalMesh.AlwaysTickPose"), 0); Step(true);
    TestEqual(TEXT("tick option restored"), (int)Comps[0]->VisibilityBasedAnimTickOption, (int)EVisibilityBasedAnimTickOption::OnlyTickPoseWhenRendered);
    TestTrue(TEXT("URO restored"), (bool)Comps[0]->bEnableUpdateRateOptimizations);
    SetCVar(TEXT("rgl.SkeletalMesh.AlwaysTickPose"), 1); Step(true);

    // asset swap: detected in the pose update (unregister), re-registered at the NEXT sync
    Comps[0]->SetSkeletalMeshAsset(Other); Comps[0]->RefreshBoneTransforms();
    Step(false);
    TestFalse(TEXT("swapped comp unregistered by pose update"), SM.IsSkeletalRegistered_ForTest(Comps[0]));
    Step(true);
    TestTrue(TEXT("swapped comp re-registered by sync"), SM.IsSkeletalRegistered_ForTest(Comps[0]));
    TestEqual(TEXT("2 cached meshes"), SM.GetSkeletalMeshCacheCount_ForTest(), 2);

    // budget: MaxEntities=3 must keep the 3 NEAREST (x=500,1000,1500) and drop the rest
    SetCVar(TEXT("rgl.SkeletalMesh.MaxEntities"), 3); Step(true);
    TestEqual(TEXT("budget enforced"), SM.GetSkeletalRegisteredCount_ForTest() - Before, 3);
    TestTrue(TEXT("nearest kept"), SM.IsSkeletalRegistered_ForTest(Comps[0]) && SM.IsSkeletalRegistered_ForTest(Comps[1]) && SM.IsSkeletalRegistered_ForTest(Comps[2]));
    TestFalse(TEXT("farthest dropped"), SM.IsSkeletalRegistered_ForTest(Comps[4]));
    SetCVar(TEXT("rgl.SkeletalMesh.MaxEntities"), 256); Step(true);

    // pose failure -> pending (entity destroyed) -> recovery
    SetCVar(TEXT("rgl.SkeletalMesh.Debug.ForcePoseFailure"), 1); Step(false);
    TestEqual(TEXT("all pending while pose fails"), SM.GetSkeletalLiveEntityCount_ForTest(), 0);
    TestEqual(TEXT("still registered"), SM.GetSkeletalRegisteredCount_ForTest() - Before, 5);
    SetCVar(TEXT("rgl.SkeletalMesh.Debug.ForcePoseFailure"), 0); Step(false);
    TestEqual(TEXT("recovered"), SM.GetSkeletalLiveEntityCount_ForTest(), 5);

    for (APawn* P : Pawns) P->Destroy();
    Step(true);
    TestEqual(TEXT("none registered"), SM.GetSkeletalRegisteredCount_ForTest() - Before, 0);
    TestEqual(TEXT("cache empty"), SM.GetSkeletalMeshCacheCount_ForTest(), 0);
    SM.UnregisterSensor(Sensor);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FKillSwitchTest, "CarlaRGL.Skeletal.Scene.KillSwitchAndApiUnavailable",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FKillSwitchTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, kWalkerSK); if (!TestNotNull(TEXT("walker SK"), Mesh)) return false;
    FCVarGuard G1(TEXT("rgl.SkeletalMesh.Enable")), G2(TEXT("rgl.SkeletalMesh.Debug.ForceApiUnavailable"));
    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    const void* Sensor = reinterpret_cast<const void*>(0xBEEF); double T = 200.0;
    auto Step = [&]() { SM.RegisterSensor(Sensor, FVector(0.f, 0.f, kZ), 10000.f, T); SM.SyncSkeletalNow_ForTest(World, T); SM.UpdateSkeletalPosesNow_ForTest(T); T += 0.05; };
    APawn* P; USkeletalMeshComponent* Comp = SpawnPawn(World, Mesh, FVector(600.f, 0.f, kZ), P);

    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 0); Step();
    TestFalse(TEXT("disabled: not registered"), SM.IsSkeletalRegistered_ForTest(Comp));
    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 1); Step();
    TestTrue(TEXT("enabled: registered"), SM.IsSkeletalRegistered_ForTest(Comp));
    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 0); Step();
    TestFalse(TEXT("disabled again: torn down"), SM.IsSkeletalRegistered_ForTest(Comp));
    TestEqual(TEXT("policy restored"), (int)Comp->VisibilityBasedAnimTickOption, (int)EVisibilityBasedAnimTickOption::OnlyTickPoseWhenRendered);
    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 1); Step();
    TestTrue(TEXT("re-enabled: registered again"), SM.IsSkeletalRegistered_ForTest(Comp));

    SetCVar(TEXT("rgl.SkeletalMesh.Debug.ForceApiUnavailable"), 1); Step();
    TestFalse(TEXT("api unavailable behaves like disabled"), SM.IsSkeletalRegistered_ForTest(Comp));
    SetCVar(TEXT("rgl.SkeletalMesh.Debug.ForceApiUnavailable"), 0);
    P->Destroy(); Step(); SM.UnregisterSensor(Sensor);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTeardownTest, "CarlaRGL.Skeletal.Scene.Teardown",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FTeardownTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, kWalkerSK); if (!TestNotNull(TEXT("walker SK"), Mesh)) return false;
    FCVarGuard G1(TEXT("rgl.SkeletalMesh.Enable")); SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 1);
    const void* A = reinterpret_cast<const void*>(0xA11CE); double T = 300.0;

    // (e) last sensor
    {
        FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
        APawn* P; USkeletalMeshComponent* Comp = SpawnPawn(World, Mesh, FVector(700.f, 0.f, kZ), P);
        SM.RegisterSensor(A, FVector(0.f, 0.f, kZ), 10000.f, T); SM.SyncSkeletalNow_ForTest(World, T); SM.UpdateSkeletalPosesNow_ForTest(T);
        TestTrue(TEXT("registered"), SM.IsSkeletalRegistered_ForTest(Comp));
        SM.UnregisterSensor(A);
        TestFalse(TEXT("torn down on last sensor"), SM.IsSkeletalRegistered_ForTest(Comp));
        TestEqual(TEXT("policy restored"), (int)Comp->VisibilityBasedAnimTickOption, (int)EVisibilityBasedAnimTickOption::OnlyTickPoseWhenRendered);
        TestEqual(TEXT("cache empty"), SM.GetSkeletalMeshCacheCount_ForTest(), 0);
        P->Destroy();
    }
    // (f) DestroyInstance restores components too; a fresh instance is created on next GetInstance
    {
        FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
        APawn* P; USkeletalMeshComponent* Comp = SpawnPawn(World, Mesh, FVector(800.f, 0.f, kZ), P);
        SM.RegisterSensor(A, FVector(0.f, 0.f, kZ), 10000.f, T); SM.SyncSkeletalNow_ForTest(World, T); SM.UpdateSkeletalPosesNow_ForTest(T);
        TestTrue(TEXT("registered"), SM.IsSkeletalRegistered_ForTest(Comp));
        FRGLSceneManager::DestroyInstance(World);
        TestEqual(TEXT("policy restored by DestroyInstance"), (int)Comp->VisibilityBasedAnimTickOption, (int)EVisibilityBasedAnimTickOption::OnlyTickPoseWhenRendered);
        FRGLSceneManager& Fresh = FRGLSceneManager::GetInstance(World);
        TestEqual(TEXT("fresh instance has no skeletal entities"), Fresh.GetSkeletalRegisteredCount_ForTest(), 0);
        P->Destroy();
    }
    return true;
}
}
#endif
