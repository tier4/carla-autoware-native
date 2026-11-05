#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/SkeletalMesh.h"
#include "GameFramework/Actor.h"
#include "Components/PoseableMeshComponent.h"
#include "Rendering/SkeletalMeshRenderData.h"
#include "Rendering/SkeletalMeshLODRenderData.h"
#include "Rendering/SkinWeightVertexBuffer.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLSkeletalMeshExtractor.h"
#include "../RGLCoordinateUtils.h"

namespace RGLSkeletalTestsOracle
{
static const TCHAR* kAssets[] = {
    TEXT("/Game/Carla/Static/Pedestrian/EuroG02_G2/SK_EuroG02_A_G2.SK_EuroG02_A_G2"),
    TEXT("/Game/Carla/Static/Pedestrian/AfroF01_G2/SK_AfroF01_A_G2.SK_AfroF01_A_G2"),
    TEXT("/Game/Carla/Static/Car/4Wheeled/AudiTT/SK_AudiTT.SK_AudiTT"),
};
static const FTransform kActorTf(FRotator(0.f, 33.f, 0.f), FVector(1234.f, -567.f, 30000.f), FVector(1.f));   // 300 m up: clear of Town10

static UWorld* GetGameWorld()
{
    for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
        if (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE) return Ctx.World();
    return nullptr;
}

// Poseable component with a deterministic non-trivial pose; actor transform applied explicitly
// (SetRootComponent does NOT transfer the spawn transform).
static UPoseableMeshComponent* SpawnPosed(UWorld* World, USkeletalMesh* Mesh, AActor*& OutActor)
{
    OutActor = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity);
    UPoseableMeshComponent* Comp = NewObject<UPoseableMeshComponent>(OutActor);
    OutActor->SetRootComponent(Comp);
    Comp->SetSkinnedAssetAndUpdate(Mesh, true);
    Comp->RegisterComponent();
    OutActor->SetActorTransform(kActorTf);
    const FReferenceSkeleton& Ref = Mesh->GetRefSkeleton();
    FRandomStream Rng(20260910);
    for (int32 b = 1; b < Ref.GetRawBoneNum(); b += 3)
    {
        const FName Name = Ref.GetBoneName(b);
        const FRotator Cur = Comp->GetBoneRotationByName(Name, EBoneSpaces::ComponentSpace);
        Comp->SetBoneRotationByName(Name, Cur + FRotator(Rng.FRandRange(-25.f, 25.f), Rng.FRandRange(-25.f, 25.f), Rng.FRandRange(-25.f, 25.f)), EBoneSpaces::ComponentSpace);
    }
    Comp->RefreshBoneTransforms();   // synchronous: FillComponentSpaceTransforms + bounds
    return Comp;
}

static FVector3f ApplyRGL(const rgl_mat3x4f& M, const rgl_vec3f& V)
{
    FVector3f O;
    for (int r = 0; r < 3; ++r) O[r] = M.value[r][0] * V.value[0] + M.value[r][1] * V.value[1] + M.value[r][2] * V.value[2] + M.value[r][3];
    return O;
}

// 1a(ii): pose[b]·restInv[b] == Phi(RefToLocal[b] * C2W)  for every raw bone
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMatrixIdentityTest, "CarlaRGL.Skeletal.Oracle.MatrixIdentity",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FMatrixIdentityTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    for (const TCHAR* Path : kAssets)
    {
        USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, Path); if (!TestNotNull(Path, Mesh)) continue;
        FString Reason; FRGLSkeletalMeshData Data;
        if (!TestTrue(FString::Printf(TEXT("extract: %s"), *Reason), RGLSkeletal::ExtractSkeletalMesh(Mesh, Data, Reason))) continue;
        AActor* Actor = nullptr; UPoseableMeshComponent* Comp = SpawnPosed(World, Mesh, Actor);
        TestTrue(TEXT("actor transform applied"), Comp->GetComponentTransform().Equals(kActorTf, 1e-2f));

        TArray<rgl_mat3x4f> Pose;
        if (!TestTrue(FString::Printf(TEXT("%s BuildWorldPose"), Path), RGLSkeletal::BuildWorldPose(Comp, Data.RawBoneNum, Pose))) { Actor->Destroy(); continue; }
        TArray<FMatrix44f> RefToLocal; Comp->CacheRefToLocalMatrices(RefToLocal);   // RefInv[b]*CST[b] (UE row-vector)
        const FMatrix C2W = Comp->GetComponentTransform().ToMatrixWithScale();
        float MaxErr = 0.f;
        for (int32 b = 0; b < Data.RawBoneNum; ++b)
        {
            const rgl_mat3x4f Ours = RGLCoord::MulRGL(Pose[b], Data.RestposesInv[b]);
            const rgl_mat3x4f Exp  = RGLCoord::ToRGLMat(FMatrix(RefToLocal[b]) * C2W);
            for (int r = 0; r < 3; ++r) for (int c = 0; c < 4; ++c) MaxErr = FMath::Max(MaxErr, FMath::Abs(Ours.value[r][c] - Exp.value[r][c]));
        }
        TestTrue(FString::Printf(TEXT("%s matrix identity max err %g"), Path, MaxErr), MaxErr < 1e-4f);
        Actor->Destroy();
    }
    return true;
}

// 1a(iii) exact all-influence replay + 1b 4-influence approximation, both vs ComputeSkinnedPositions
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVertexReplayTest, "CarlaRGL.Skeletal.Oracle.VertexReplay",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FVertexReplayTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    for (const TCHAR* Path : kAssets)
    {
        USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, Path); if (!TestNotNull(Path, Mesh)) continue;
        FString Reason; FRGLSkeletalMeshData Data;
        if (!TestTrue(FString::Printf(TEXT("extract: %s"), *Reason), RGLSkeletal::ExtractSkeletalMesh(Mesh, Data, Reason))) continue;
        AActor* Actor = nullptr; UPoseableMeshComponent* Comp = SpawnPosed(World, Mesh, Actor);

        // Oracle: UE CPU skinning (component space, cm, ALL influences). The cache MUST be filled first.
        const FSkeletalMeshLODRenderData& LOD = Mesh->GetResourceForRendering()->LODRenderData[Data.LODIndex];
        const FSkinWeightVertexBuffer* SW = LOD.GetSkinWeightVertexBuffer();
        TArray<FMatrix44f> RefToLocal; Comp->CacheRefToLocalMatrices(RefToLocal);
        TArray<FVector3f> Oracle;
        USkinnedMeshComponent::ComputeSkinnedPositions(Comp, Oracle, RefToLocal, LOD, *SW);
        if (!TestEqual(TEXT("oracle vertex count"), Oracle.Num(), Data.Vertices.Num())) { Actor->Destroy(); continue; }

        TArray<rgl_mat3x4f> Pose;
        if (!TestTrue(FString::Printf(TEXT("%s BuildWorldPose"), Path), RGLSkeletal::BuildWorldPose(Comp, Data.RawBoneNum, Pose))) { Actor->Destroy(); continue; }
        TArray<rgl_mat3x4f> Anim; Anim.SetNum(Data.RawBoneNum);
        for (int32 b = 0; b < Data.RawBoneNum; ++b) Anim[b] = RGLCoord::MulRGL(Pose[b], Data.RestposesInv[b]);
        const FTransform C2W = Comp->GetComponentTransform();
        const int32 MaxInf = static_cast<int32>(SW->GetMaxBoneInfluences());

        // Section lookup for raw bone remap of the oracle's own weights (all influences, /65535 like UE).
        TArray<const FSkelMeshRenderSection*> SectionOf; SectionOf.SetNumZeroed(Oracle.Num());
        for (const FSkelMeshRenderSection& S : LOD.RenderSections) if (!S.bDisabled)
            for (uint32 v = S.BaseVertexIndex; v < S.BaseVertexIndex + S.NumVertices; ++v) SectionOf[v] = &S;

        float ExactMax = 0.f; int32 ExactN = 0;
        TArray<float> ApproxErr; ApproxErr.Reserve(Oracle.Num());
        for (int32 v = 0; v < Oracle.Num(); ++v)
        {
            const FVector Ow = C2W.TransformPosition(FVector(Oracle[v])) * RGLCoord::UE_TO_RGL;   // oracle -> world m
            // (iii) exact: all influences, UE weight semantics (w/65535, no per-vertex renormalisation)
            if (SectionOf[v])
            {
                const FSkinWeightInfo Info = SW->GetVertexSkinWeights(v);
                FVector3f P(0.f);
                for (int32 k = 0; k < MaxInf; ++k)
                    if (Info.InfluenceWeights[k] > 0)
                        P += ApplyRGL(Anim[SectionOf[v]->BoneMap[Info.InfluenceBones[k]]], Data.Vertices[v]) * (Info.InfluenceWeights[k] / 65535.f);
                ExactMax = FMath::Max(ExactMax, (float)(FVector(P) - Ow).Size()); ++ExactN;

                // (1b) approx: our reduced 4 influences. Gated by the same SectionOf[v] check as the
                // exact branch: outside every enabled section, Data.Weights[v] is the extractor's
                // fabricated rigid bone-0 fallback (not a real skinned position), so comparing it
                // against the oracle (which skins across ALL sections, including disabled ones) would
                // inflate p99/max with a meaningless error.
                FVector3f Q(0.f); const rgl_bone_weights_t& W = Data.Weights[v];
                for (int k = 0; k < 4; ++k) if (W.weights[k] > 0.f) Q += ApplyRGL(Anim[W.bone_indexes[k]], Data.Vertices[v]) * W.weights[k];
                ApproxErr.Add((FVector(Q) - Ow).Size());
            }
        }
        if (!TestTrue(FString::Printf(TEXT("%s at least one section-covered vertex for approx stats"), Path), ApproxErr.Num() > 0)) { Actor->Destroy(); continue; }
        ApproxErr.Sort();
        const float P99 = ApproxErr[FMath::Clamp(int32(ApproxErr.Num() * 0.99f), 0, ApproxErr.Num() - 1)];
        AddInfo(FString::Printf(TEXT("%s exact(all-influence) max=%.6f m over %d verts; approx(4) p99=%.4f max=%.4f m"), Path, ExactMax, ExactN, P99, ApproxErr.Last()));
        TestTrue(FString::Printf(TEXT("%s exact replay < 1e-4 m"), Path), ExactMax < 1e-4f);   // matrices/BoneMap/uint16 correct
        TestTrue(FString::Printf(TEXT("%s approx p99 < 1 cm"), Path), P99 < 0.01f);            // asset fidelity of top-4 reduction
        TestTrue(FString::Printf(TEXT("%s approx max < 5 cm"), Path), ApproxErr.Last() < 0.05f);
        Actor->Destroy();
    }
    return true;
}
}
#endif
