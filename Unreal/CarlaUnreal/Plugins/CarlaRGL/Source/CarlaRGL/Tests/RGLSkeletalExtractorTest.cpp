#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/SkeletalMesh.h"
#include "Rendering/SkeletalMeshRenderData.h"
#include "Rendering/SkeletalMeshLODRenderData.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLSkeletalMeshExtractor.h"
#include "../RGLCoordinateUtils.h"

namespace RGLSkeletalTestsExtractor
{
static const TCHAR* kAssets[] = {
    TEXT("/Game/Carla/Static/Pedestrian/EuroG02_G2/SK_EuroG02_A_G2.SK_EuroG02_A_G2"),
    TEXT("/Game/Carla/Static/Pedestrian/AfroF01_G2/SK_AfroF01_A_G2.SK_AfroF01_A_G2"),   // walker.pedestrian.0016 family
    TEXT("/Game/Carla/Static/Car/4Wheeled/AudiTT/SK_AudiTT.SK_AudiTT"),                  // Door=0 vehicle
    TEXT("/Game/Carla/Static/Car/4Wheeled/LincolnMKZ/SK_LincolnMKZ.SK_LincolnMKZ"),      // 32-bit index candidates
    TEXT("/Game/Carla/Static/Car/4Wheeled/MiniCooper/SK_MiniCooper.SK_MiniCooper"),
};

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FReduceInfluencesTest, "CarlaRGL.Skeletal.Extractor.ReduceInfluences",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FReduceInfluencesTest::RunTest(const FString& Parameters)
{
    rgl_bone_weights_t Out;
    {   // 6 influences -> 4 largest, renormalised, raw ids preserved
        const int32 Bones[6] = { 10, 11, 12, 13, 14, 15 };
        const uint16 W16[6]  = { 6553, 32767, 655, 13107, 9830, 2621 };
        TestTrue(TEXT("ok"), RGLSkeletal::ReduceInfluences(Bones, W16, 6, Out));
        float Sum = 0.f; for (float w : Out.weights) Sum += w;
        TestTrue(TEXT("sum==1"), FMath::IsNearlyEqual(Sum, 1.f, 1e-5f));
        TestEqual(TEXT("1st"), Out.bone_indexes[0], 11); TestEqual(TEXT("2nd"), Out.bone_indexes[1], 13);
        TestEqual(TEXT("3rd"), Out.bone_indexes[2], 14); TestEqual(TEXT("4th"), Out.bone_indexes[3], 10);
        TestTrue(TEXT("renormalised"), FMath::IsNearlyEqual(Out.weights[0], 32767.f / (32767.f + 13107.f + 9830.f + 6553.f), 1e-5f));
    }
    {   // 2 influences -> unused slots weight 0 with a VALID index
        const int32 Bones[2] = { 7, 3 }; const uint16 W16[2] = { 49151, 16384 };
        TestTrue(TEXT("ok"), RGLSkeletal::ReduceInfluences(Bones, W16, 2, Out));
        TestEqual(TEXT("w2"), Out.weights[2], 0.f); TestEqual(TEXT("w3"), Out.weights[3], 0.f);
        TestEqual(TEXT("i2"), Out.bone_indexes[2], 7); TestEqual(TEXT("i3"), Out.bone_indexes[3], 7);
    }
    {   const int32 Bones[3] = { 1, 2, 3 }; const uint16 W16[3] = { 0, 0, 0 };
        TestFalse(TEXT("zero total rejected"), RGLSkeletal::ReduceInfluences(Bones, W16, 3, Out)); }
    TestFalse(TEXT("empty rejected"), RGLSkeletal::ReduceInfluences(nullptr, nullptr, 0, Out));
    return true;
}

// Stage-0 probe + 1a(i) round-trip + structural invariants + 32-bit index fixture
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FExtractAssetsTest, "CarlaRGL.Skeletal.Extractor.ExtractAssets",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FExtractAssetsTest::RunTest(const FString& Parameters)
{
    bool bSaw32Bit = false;
    for (const TCHAR* Path : kAssets)
    {
        USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, Path);
        if (!TestNotNull(FString::Printf(TEXT("load %s"), Path), Mesh)) continue;

        // Stage-0 probe: log the 6 readiness values for LOD0 regardless of outcome.
        FRGLSkeletalLODReadiness R;
        const bool bLod0 = RGLSkeletal::IsLODReadable(Mesh, 0, R);
        AddInfo(FString::Printf(TEXT("PROBE %s LOD0 readable=%d %s"), Path, bLod0, *R.ToString()));

        FString Reason; FRGLSkeletalMeshData Data;
        if (!TestTrue(FString::Printf(TEXT("extract %s: %s"), Path, *Reason), RGLSkeletal::ExtractSkeletalMesh(Mesh, Data, Reason))) continue;
        AddInfo(FString::Printf(TEXT("%s: LOD=%d verts=%d tris=%d rawBones=%d"), Path, Data.LODIndex, Data.Vertices.Num(), Data.Indices.Num(), Data.RawBoneNum));
        TestTrue(TEXT("LOD selected"), Data.LODIndex >= 0);
        TestEqual(TEXT("weights per vertex"), Data.Weights.Num(), Data.Vertices.Num());
        TestEqual(TEXT("restposes per raw bone"), Data.RestposesInv.Num(), Data.RawBoneNum);
        TestEqual(TEXT("raw bones == ref skeleton"), Data.RawBoneNum, Mesh->GetRefSkeleton().GetRawBoneNum());

        const FSkeletalMeshLODRenderData& LOD = Mesh->GetResourceForRendering()->LODRenderData[Data.LODIndex];
        const FPositionVertexBuffer& VB = LOD.StaticVertexBuffers.PositionVertexBuffer;
        TestEqual(TEXT("vertex count"), (uint32)Data.Vertices.Num(), VB.GetNumVertices());
        float MaxErrCm = 0.f;
        for (int32 i = 0; i < Data.Vertices.Num(); ++i)
        {
            const FVector3f P = VB.VertexPosition(i);
            for (int k = 0; k < 3; ++k) MaxErrCm = FMath::Max(MaxErrCm, FMath::Abs(Data.Vertices[i].value[k] * 100.f - P[k]));
        }
        // spec §7 stage 1a(i): 1e-4 cm. float32 cm -> m -> cm is exact to well under that.
        TestTrue(FString::Printf(TEXT("vertex round-trip max err %f cm"), MaxErrCm), MaxErrCm < 1e-4f);

        bool bIdxOk = true, bBoneOk = true, bSumOk = true;
        for (const rgl_vec3i& T : Data.Indices) for (int k = 0; k < 3; ++k) bIdxOk &= (T.value[k] >= 0 && T.value[k] < Data.Vertices.Num());
        for (const rgl_bone_weights_t& W : Data.Weights)
        {
            float S = 0.f;
            for (int k = 0; k < 4; ++k) { bBoneOk &= (W.bone_indexes[k] >= 0 && W.bone_indexes[k] < Data.RawBoneNum); S += W.weights[k]; }
            bSumOk &= FMath::IsNearlyEqual(S, 1.f, 1e-4f);
        }
        TestTrue(TEXT("indices in range"), bIdxOk); TestTrue(TEXT("bone indices in range"), bBoneOk); TestTrue(TEXT("weights normalised"), bSumOk);
        bSaw32Bit |= (LOD.MultiSizeIndexContainer.GetDataTypeSize() == sizeof(uint32));
    }
    TestTrue(TEXT("at least one 32-bit-index (>65535 vertex) fixture exercised the wide path"), bSaw32Bit);
    return true;
}
}
#endif
