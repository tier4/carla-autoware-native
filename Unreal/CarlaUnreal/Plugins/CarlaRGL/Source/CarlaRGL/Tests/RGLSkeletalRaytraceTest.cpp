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
#include "../RGLSkeletalMeshExtractor.h"
#include "../RGLCoordinateUtils.h"
#include <rgl/api/core.h>

namespace RGLSkeletalTestsRaytrace
{
static const TCHAR* kWalkerSK = TEXT("/Game/Carla/Static/Pedestrian/AfroF01_G2/SK_AfroF01_A_G2.SK_AfroF01_A_G2");
static const float kZ = 30000.f;
static UWorld* GetGameWorld()
{
    for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
        if (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE) return Ctx.World();
    return nullptr;
}
// Local copies (unity build: this file's own namespace) of the CVar helpers in
// RGLSkeletalSceneManagerTest.cpp — a test must not leak CVar state to the next one.
static void SetCVar(const TCHAR* Name, int32 V) { IConsoleManager::Get().FindConsoleVariable(Name)->Set(V, ECVF_SetByCode); }
struct FCVarGuard { FString Name; int32 Old; FCVarGuard(const TCHAR* N) : Name(N), Old(IConsoleManager::Get().FindConsoleVariable(N)->GetInt()) {} ~FCVarGuard() { SetCVar(*Name, Old); } };

// Ray grid from (0,-2000cm,kZ) looking +Y, covering the walker placed at (0,0,kZ). Returns hit points (RGL frame, m).
static bool CastGrid(TArray<FVector>& OutHits, FString& Err)
{
    const int32 NX = 200, NZ = 200;                 // 1 cm spacing over 2 m x 2 m at 20 m: ~0.03 deg
    TArray<rgl_mat3x4f> Rays; Rays.Reserve(NX * NZ);
    for (int32 ix = 0; ix < NX; ++ix) for (int32 iz = 0; iz < NZ; ++iz)
    {
        const float X = -100.f + 200.f * ix / (NX - 1), Z = kZ - 50.f + 200.f * iz / (NZ - 1);   // cm
        // Ray = transform whose local +Z axis is the ray direction (RGL convention: rays_from_mat3x4f, direction = local Z).
        const FTransform T(FRotator(-90.f, 0.f, 0.f) /* local Z -> world +Y? verify below */, FVector(X, -2000.f, Z));
        Rays.Add(RGLCoord::ToRGL(T));
    }
    // NOTE: RGL ray direction is the transform's local +Z. UE FRotator(-90,0,0) rotates +Z (up) to +Y? No: pitch rotates
    // around Y. Use an explicit matrix: columns X=(1,0,0), Y=(0,0,1), Z=(0,1,0) i.e. local Z -> world +Y, local Y -> world +Z.
    for (int32 i = 0; i < Rays.Num(); ++i)
    {
        rgl_mat3x4f& R = Rays[i];
        R.value[0][0] = 1.f; R.value[0][1] = 0.f; R.value[0][2] = 0.f;
        R.value[1][0] = 0.f; R.value[1][1] = 0.f; R.value[1][2] = 1.f;
        R.value[2][0] = 0.f; R.value[2][1] = 1.f; R.value[2][2] = 0.f;
    }
    rgl_node_t RaysN = nullptr, RtN = nullptr, CompactN = nullptr, YieldN = nullptr;
    bool bRtLinked = false, bCompactLinked = false, bYieldLinked = false;
    const rgl_field_t Fields[] = { RGL_FIELD_XYZ_VEC3_F32 };
    // rgl_graph_destroy(RaysN) only frees the nodes reachable from RaysN, so a node that was
    // created but not yet linked into that graph has to be destroyed on its own or it leaks.
    auto Fail = [&](const TCHAR* Api)
    {
        const char* E = nullptr; rgl_get_last_error_string(&E);
        Err = FString::Printf(TEXT("%s: %s"), Api, E ? *FString(UTF8_TO_TCHAR(E)) : TEXT("?"));
        if (YieldN   && !bYieldLinked)   rgl_graph_destroy(YieldN);
        if (CompactN && !bCompactLinked) rgl_graph_destroy(CompactN);
        if (RtN      && !bRtLinked)      rgl_graph_destroy(RtN);
        if (RaysN) rgl_graph_destroy(RaysN);
        return false;
    };
    if (rgl_node_rays_from_mat3x4f(&RaysN, Rays.GetData(), Rays.Num()) != RGL_SUCCESS) return Fail(TEXT("rays_from_mat3x4f"));
    if (rgl_node_raytrace(&RtN, nullptr) != RGL_SUCCESS) return Fail(TEXT("raytrace"));
    if (rgl_node_points_compact_by_field(&CompactN, RGL_FIELD_IS_HIT_I32) != RGL_SUCCESS) return Fail(TEXT("compact"));
    if (rgl_node_points_yield(&YieldN, Fields, 1) != RGL_SUCCESS) return Fail(TEXT("yield"));
    if (rgl_graph_node_add_child(RaysN, RtN) != RGL_SUCCESS) return Fail(TEXT("add_child(rays->raytrace)"));
    bRtLinked = true;
    if (rgl_graph_node_add_child(RtN, CompactN) != RGL_SUCCESS) return Fail(TEXT("add_child(raytrace->compact)"));
    bCompactLinked = true;
    if (rgl_graph_node_add_child(CompactN, YieldN) != RGL_SUCCESS) return Fail(TEXT("add_child(compact->yield)"));
    bYieldLinked = true;
    if (rgl_graph_run(RaysN) != RGL_SUCCESS) return Fail(TEXT("graph_run"));
    int32_t Count = 0, Size = 0;
    if (rgl_graph_get_result_size(YieldN, RGL_FIELD_XYZ_VEC3_F32, &Count, &Size) != RGL_SUCCESS) return Fail(TEXT("get_result_size"));
    TArray<rgl_vec3f> Pts; Pts.SetNumUninitialized(Count);
    if (Count > 0 && rgl_graph_get_result_data(YieldN, RGL_FIELD_XYZ_VEC3_F32, Pts.GetData()) != RGL_SUCCESS) return Fail(TEXT("get_result_data"));
    rgl_graph_destroy(RaysN);
    OutHits.Reset(Count); for (const rgl_vec3f& P : Pts) OutHits.Add(FVector(P.value[0], P.value[1], P.value[2]));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FContainmentTest, "CarlaRGL.Skeletal.Raytrace.Containment",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FContainmentTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, kWalkerSK); if (!TestNotNull(TEXT("walker SK"), Mesh)) return false;
    FCVarGuard EnableGuard(TEXT("rgl.SkeletalMesh.Enable"));
    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 1);

    APawn* P = World->SpawnActor<APawn>(APawn::StaticClass(), FTransform::Identity);
    USkeletalMeshComponent* Comp = NewObject<USkeletalMeshComponent>(P);
    P->SetRootComponent(Comp); Comp->SetSkeletalMeshAsset(Mesh); Comp->RegisterComponent();
    P->SetActorLocation(FVector(0.f, 0.f, kZ)); Comp->RefreshBoneTransforms();

    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    const void* Sensor = reinterpret_cast<const void*>(0x5EED); const double T = 400.0;
    SM.RegisterSensor(Sensor, FVector(0.f, -2000.f, kZ), 10000.f, T);
    SM.SyncSkeletalNow_ForTest(World, T); SM.UpdateSkeletalPosesNow_ForTest(T);
    TestTrue(TEXT("walker registered+live"), SM.IsSkeletalRegistered_ForTest(Comp) && SM.GetSkeletalLiveEntityCount_ForTest() >= 1);

    // Expected geometry: CPU replay of our own skinning chain -> world AABB (m), inflated 5 cm.
    FString Reason; FRGLSkeletalMeshData Data;
    if (!TestTrue(FString::Printf(TEXT("extract: %s"), *Reason), RGLSkeletal::ExtractSkeletalMesh(Mesh, Data, Reason)))
    {
        P->Destroy(); SM.UnregisterSensor(Sensor);
        return false;
    }
    TArray<rgl_mat3x4f> Pose;
    if (!TestTrue(TEXT("BuildWorldPose"), RGLSkeletal::BuildWorldPose(Comp, Data.RawBoneNum, Pose)))
    {
        P->Destroy(); SM.UnregisterSensor(Sensor);
        return false;   // Pose is untouched on failure; indexing it below would read garbage.
    }
    FBox Box(ForceInit);
    for (int32 v = 0; v < Data.Vertices.Num(); ++v)
    {
        FVector Q(0.0); const rgl_bone_weights_t& W = Data.Weights[v];
        for (int k = 0; k < 4; ++k) if (W.weights[k] > 0.f)
        {
            const rgl_mat3x4f A = RGLCoord::MulRGL(Pose[W.bone_indexes[k]], Data.RestposesInv[W.bone_indexes[k]]);
            const rgl_vec3f& V = Data.Vertices[v];
            for (int r = 0; r < 3; ++r) Q[r] += W.weights[k] * (A.value[r][0] * V.value[0] + A.value[r][1] * V.value[1] + A.value[r][2] * V.value[2] + A.value[r][3]);
        }
        Box += Q;
    }
    Box = Box.ExpandBy(0.05);

    TArray<FVector> Hits; FString Err;
    if (!TestTrue(FString::Printf(TEXT("raytrace: %s"), *Err), CastGrid(Hits, Err))) { P->Destroy(); SM.UnregisterSensor(Sensor); return false; }
    int32 Inside = 0; for (const FVector& H : Hits) Inside += Box.IsInside(H) ? 1 : 0;
    AddInfo(FString::Printf(TEXT("hits=%d inside=%d box=%s"), Hits.Num(), Inside, *Box.ToString()));
    TestTrue(TEXT(">= 100 hits on the walker"), Hits.Num() >= 100);
    TestTrue(TEXT(">= 95% of hits inside CPU-replay AABB (+5 cm)"), Hits.Num() > 0 && Inside >= FMath::CeilToInt(0.95f * Hits.Num()));

    P->Destroy(); SM.UnregisterSensor(Sensor);
    return true;
}
}
#endif
