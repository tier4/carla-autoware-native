#ifdef WITH_RGL
#include "RGLSkeletalMeshExtractor.h"
#include "RGLCoordinateUtils.h"

#include <util/ue-header-guard-begin.h>
#include "Engine/SkeletalMesh.h"
#include "ReferenceSkeleton.h"
#include "Rendering/SkeletalMeshRenderData.h"
#include "Rendering/SkeletalMeshLODRenderData.h"
#include "Rendering/SkinWeightVertexBuffer.h"
#include "Rendering/MultiSizeIndexContainer.h"
#include "Rendering/PositionVertexBuffer.h"
#include "Components/SkinnedMeshComponent.h"
#include <util/ue-header-guard-end.h>

namespace RGLSkeletal
{

bool ReduceInfluences(const int32* RawBones, const uint16* Weights16, int32 Count, rgl_bone_weights_t& Out)
{
    if (Count <= 0 || !RawBones || !Weights16) return false;
    struct FInf { int32 Bone; uint16 W; };
    TArray<FInf, TInlineAllocator<16>> Infs;
    for (int32 i = 0; i < Count; ++i) if (Weights16[i] > 0) Infs.Add({ RawBones[i], Weights16[i] });
    if (Infs.Num() == 0) return false;
    Infs.Sort([](const FInf& A, const FInf& B) { return A.W > B.W; });
    const int32 N = FMath::Min(4, Infs.Num());
    float Sum = 0.f; for (int32 i = 0; i < N; ++i) Sum += static_cast<float>(Infs[i].W);
    if (!(Sum > 0.f) || !FMath::IsFinite(Sum)) return false;
    for (int32 i = 0; i < 4; ++i)
    {
        if (i < N) { Out.weights[i] = static_cast<float>(Infs[i].W) / Sum; Out.bone_indexes[i] = Infs[i].Bone; }
        else       { Out.weights[i] = 0.f; Out.bone_indexes[i] = Infs[0].Bone; }   // RGL dereferences all 4 indices
    }
    return true;
}

bool IsLODReadable(const USkeletalMesh* Mesh, int32 LODIndex, FRGLSkeletalLODReadiness& R)
{
    R = FRGLSkeletalLODReadiness();
    if (!Mesh) { R.FailReason = TEXT("null mesh"); return false; }
    const FSkeletalMeshRenderData* RD = Mesh->GetResourceForRendering();
    if (!RD) { R.FailReason = TEXT("no render data"); return false; }
    if (LODIndex < 0 || LODIndex >= RD->LODRenderData.Num()) { R.FailReason = TEXT("LOD index out of range"); return false; }
    const FSkeletalMeshLODRenderData& LOD = RD->LODRenderData[LODIndex];

    const FPositionVertexBuffer& VB = LOD.StaticVertexBuffers.PositionVertexBuffer;
    R.NumVertices = VB.GetNumVertices(); R.bPositionsCPU = VB.GetAllowCPUAccess();
    if (R.NumVertices == 0) { R.FailReason = TEXT("0 vertices"); return false; }
    if (!R.bPositionsCPU) { R.FailReason = TEXT("positions have no CPU copy"); return false; }

    if (!LOD.MultiSizeIndexContainer.IsIndexBufferValid()) { R.FailReason = TEXT("no index buffer"); return false; }
    const FRawStaticIndexBuffer16or32Interface* IB = LOD.MultiSizeIndexContainer.GetIndexBuffer();
    R.IndexCount = IB->Num(); R.IndexBytes = IB->GetResourceDataSize();
    if (R.IndexCount <= 0 || R.IndexBytes <= 0) { R.FailReason = TEXT("index buffer has no CPU payload"); return false; }

    const FSkinWeightVertexBuffer* SW = LOD.GetSkinWeightVertexBuffer();
    R.WeightVertices = SW ? SW->GetNumVertices() : 0;
    R.bWeightData = SW && SW->GetDataVertexBuffer() && SW->GetDataVertexBuffer()->GetWeightData();
    if (!SW || R.WeightVertices != R.NumVertices) { R.FailReason = TEXT("skin weight vertex count mismatch"); return false; }
    if (!R.bWeightData) { R.FailReason = TEXT("skin weight data not resident"); return false; }
    R.bVariableBones = SW->GetVariableBonesPerVertex();
    R.bLookupData = SW->GetLookupVertexBuffer() && SW->GetLookupVertexBuffer()->GetLookupData();
    if (R.bVariableBones && !R.bLookupData) { R.FailReason = TEXT("skin weight lookup buffer not resident"); return false; }

    R.RawBoneNum = Mesh->GetRefSkeleton().GetRawBoneNum();
    R.InvBindCount = Mesh->GetRefBasesInvMatrix().Num();
    if (R.RawBoneNum <= 0) { R.FailReason = TEXT("0 raw bones"); return false; }
    if (R.InvBindCount != R.RawBoneNum) { R.FailReason = TEXT("RefBasesInvMatrix size != raw bone count"); return false; }

    R.Sections = LOD.RenderSections.Num();
    for (const FSkelMeshRenderSection& S : LOD.RenderSections)
    {
        if (S.bDisabled) continue;
        if (S.BaseVertexIndex + S.NumVertices > R.NumVertices) { R.FailReason = TEXT("section vertex range out of bounds"); return false; }
        if (static_cast<int64>(S.BaseIndex) + static_cast<int64>(S.NumTriangles) * 3 > R.IndexCount) { R.FailReason = TEXT("section index range out of bounds"); return false; }
        for (FBoneIndexType B : S.BoneMap) if (static_cast<int32>(B) >= R.RawBoneNum) { R.FailReason = TEXT("BoneMap entry >= raw bone count"); return false; }
    }
    return true;
}

int32 SelectReadableLOD(const USkeletalMesh* Mesh, FString& OutReason, int32 MinLOD)
{
    const FSkeletalMeshRenderData* RD = Mesh ? Mesh->GetResourceForRendering() : nullptr;
    if (!RD) { OutReason = TEXT("no render data"); return -1; }
    const int32 NumLODs = RD->LODRenderData.Num();
    if (NumLODs <= 0) { OutReason = TEXT("NoResidentLOD (no LOD render data)"); return -1; }
    // MinLOD floors the scan; clamping keeps an over-large CVar value from disabling every asset.
    const int32 First = FMath::Clamp(MinLOD, 0, NumLODs - 1);
    FString Reasons;
    for (int32 L = First; L < NumLODs; ++L)
    {
        FRGLSkeletalLODReadiness R;
        if (IsLODReadable(Mesh, L, R)) return L;
        Reasons += FString::Printf(TEXT("[LOD%d: %s] "), L, *R.FailReason);
    }
    OutReason = FString::Printf(TEXT("NoResidentLOD (from LOD%d) "), First) + Reasons;
    return -1;
}

bool ExtractSkeletalMesh(const USkeletalMesh* Mesh, FRGLSkeletalMeshData& Out, FString& OutReason, int32 MinLOD)
{
    Out = FRGLSkeletalMeshData();
    const int32 L = SelectReadableLOD(Mesh, OutReason, MinLOD);
    if (L < 0) return false;

    const FSkeletalMeshLODRenderData& LOD = Mesh->GetResourceForRendering()->LODRenderData[L];
    const FPositionVertexBuffer& VB = LOD.StaticVertexBuffers.PositionVertexBuffer;
    const FRawStaticIndexBuffer16or32Interface* IB = LOD.MultiSizeIndexContainer.GetIndexBuffer();
    const FSkinWeightVertexBuffer* SW = LOD.GetSkinWeightVertexBuffer();
    const uint32 NumVerts = VB.GetNumVertices();
    const int32 RawBoneNum = Mesh->GetRefSkeleton().GetRawBoneNum();

    Out.Vertices.SetNumUninitialized(NumVerts);
    for (uint32 i = 0; i < NumVerts; ++i)
    {
        const FVector3f P = VB.VertexPosition(i);
        Out.Vertices[i] = { { P.X * RGLCoord::UE_TO_RGL, P.Y * RGLCoord::UE_TO_RGL, P.Z * RGLCoord::UE_TO_RGL } };
    }

    Out.Weights.SetNumZeroed(NumVerts);
    TBitArray<> VertexDone(false, NumVerts);
    // "Unlimited bone influences" assets report > MAX_TOTAL_INFLUENCES, but FSkinWeightInfo only
    // holds MAX_TOTAL_INFLUENCES entries and so do the per-vertex stack arrays below. Truncating
    // is safe: ReduceInfluences keeps the 4 largest of what it is given and renormalises.
    const int32 MaxInf = FMath::Min<int32>(static_cast<int32>(SW->GetMaxBoneInfluences()), MAX_TOTAL_INFLUENCES);
    for (const FSkelMeshRenderSection& S : LOD.RenderSections)
    {
        if (S.bDisabled) continue;
        const uint32 VBegin = S.BaseVertexIndex, VEnd = S.BaseVertexIndex + S.NumVertices;
        for (uint32 t = 0; t < S.NumTriangles; ++t)
        {
            const uint32 b = S.BaseIndex + t * 3;
            const uint32 I0 = IB->Get(b), I1 = IB->Get(b + 1), I2 = IB->Get(b + 2);
            // Triangles of a section must reference that section's vertex range (BoneMap is per section).
            if (I0 < VBegin || I0 >= VEnd || I1 < VBegin || I1 >= VEnd || I2 < VBegin || I2 >= VEnd)
            { OutReason = FString::Printf(TEXT("triangle %u of a section references vertices outside its range"), t); return false; }
            Out.Indices.Add({ { static_cast<int32>(I0), static_cast<int32>(I1), static_cast<int32>(I2) } });
        }
        for (uint32 v = VBegin; v < VEnd; ++v)
        {
            if (VertexDone[v]) continue;
            const FSkinWeightInfo Info = SW->GetVertexSkinWeights(v);
            int32 RawBones[MAX_TOTAL_INFLUENCES]; uint16 W16[MAX_TOTAL_INFLUENCES];
            for (int32 k = 0; k < MaxInf; ++k)
            {
                const int32 Local = static_cast<int32>(Info.InfluenceBones[k]);
                if (Local < 0 || Local >= S.BoneMap.Num())
                { OutReason = FString::Printf(TEXT("vertex %u influence %d: local bone %d outside BoneMap(%d)"), v, k, Local, S.BoneMap.Num()); return false; }
                RawBones[k] = static_cast<int32>(S.BoneMap[Local]);   // section-local -> raw skeleton index
                W16[k] = Info.InfluenceWeights[k];                     // uint16; 8-bit stored weights are pre-expanded by UE
            }
            if (!ReduceInfluences(RawBones, W16, MaxInf, Out.Weights[v]))
            { OutReason = FString::Printf(TEXT("vertex %u has zero/invalid total weight"), v); return false; }
            VertexDone[v] = true;
        }
    }
    // Vertices outside every enabled section are never referenced by an emitted triangle (checked above);
    // give them valid rigid bone-0 weights so RGL's 4-index dereference is always in range.
    for (uint32 v = 0; v < NumVerts; ++v)
        if (!VertexDone[v]) { Out.Weights[v] = rgl_bone_weights_t{ { 1.f, 0.f, 0.f, 0.f }, { 0, 0, 0, 0 } }; }

    const TArray<FMatrix44f>& Inv = Mesh->GetRefBasesInvMatrix();
    Out.RestposesInv.SetNumUninitialized(RawBoneNum);
    for (int32 b = 0; b < RawBoneNum; ++b) Out.RestposesInv[b] = RGLCoord::ToRGLMat(FMatrix(Inv[b]));
    Out.RawBoneNum = RawBoneNum;
    Out.LODIndex = L;
    return true;
}

bool BuildWorldPose(const USkinnedMeshComponent* Comp, int32 RawBoneNum, TArray<rgl_mat3x4f>& Out)
{
    if (!Comp || RawBoneNum <= 0) return false;
    const TArray<FTransform>& CST = Comp->GetComponentSpaceTransforms();
    if (CST.Num() < RawBoneNum) return false;
    const FMatrix C2W = Comp->GetComponentTransform().ToMatrixWithScale();
    Out.SetNumUninitialized(RawBoneNum);
    for (int32 b = 0; b < RawBoneNum; ++b) Out[b] = RGLCoord::ToRGLMat(CST[b].ToMatrixWithScale() * C2W);
    return true;
}

} // namespace RGLSkeletal
#endif // WITH_RGL
