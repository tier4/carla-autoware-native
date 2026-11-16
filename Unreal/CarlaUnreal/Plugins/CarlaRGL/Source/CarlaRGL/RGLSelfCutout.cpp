#ifdef WITH_RGL
#include "RGLSelfCutout.h"
#include "RGLCoordinateUtils.h"

#include <cmath>

namespace RGLSelfCutout
{

FVector ClosestPointOnTriangle(const FVector& P, const FVector& A, const FVector& B, const FVector& C)
{
    const FVector AB = B - A, AC = C - A, AP = P - A;
    const double D1 = AB | AP, D2 = AC | AP;
    if (D1 <= 0.0 && D2 <= 0.0) return A;                                   // vertex region A

    const FVector BP = P - B;
    const double D3 = AB | BP, D4 = AC | BP;
    if (D3 >= 0.0 && D4 <= D3) return B;                                    // vertex region B

    const double VC = D1 * D4 - D3 * D2;
    if (VC <= 0.0 && D1 >= 0.0 && D3 <= 0.0)                                // edge region AB
    {
        const double Den = D1 - D3;
        return (Den > 0.0) ? A + AB * (D1 / Den) : A;
    }

    const FVector CP = P - C;
    const double D5 = AB | CP, D6 = AC | CP;
    if (D6 >= 0.0 && D5 <= D6) return C;                                    // vertex region C

    const double VB = D5 * D2 - D1 * D6;
    if (VB <= 0.0 && D2 >= 0.0 && D6 <= 0.0)                                // edge region AC
    {
        const double Den = D2 - D6;
        return (Den > 0.0) ? A + AC * (D2 / Den) : A;
    }

    const double VA = D3 * D6 - D5 * D4;
    if (VA <= 0.0 && (D4 - D3) >= 0.0 && (D5 - D6) >= 0.0)                  // edge region BC
    {
        const double Den = (D4 - D3) + (D5 - D6);
        return (Den > 0.0) ? B + (C - B) * ((D4 - D3) / Den) : B;
    }

    const double Denom = VA + VB + VC;                                      // inside the face
    if (!(FMath::Abs(Denom) > 0.0)) return A;                               // degenerate triangle
    const double V = VB / Denom, W = VC / Denom;
    return A + AB * V + AC * W;
}

double ElevationDeg(const FVector& Local)
{
    return FMath::RadiansToDegrees(std::atan2(Local.Z, std::sqrt(Local.X * Local.X + Local.Y * Local.Y)));
}

bool TouchesElevationBand(const FVector& A, const FVector& B, const FVector& C, double LoDeg, double HiDeg)
{
    if (LoDeg > HiDeg) return false;
    const double EA = ElevationDeg(A), EB = ElevationDeg(B), EC = ElevationDeg(C);

    // Elevation is continuous on the (connected) triangle, so the triangle touches the band unless it
    // lies entirely above HiDeg or entirely below LoDeg.
    //  - {elevation > Hi} is a convex cone for 0 <= Hi < 90 and {elevation < Lo} one for -90 < Lo <= 0:
    //    a triangle lies inside a convex set iff its three vertices do, so the vertices decide exactly.
    //  - On the other side (Hi < 0 or Lo > 0) the set is not convex. Use the bound around the centroid:
    //    every point of the triangle is within Rt of the centroid, so its direction (and elevation)
    //    differs from the centroid's by at most asin(Rt / |centroid|). No bound when the triangle can
    //    reach the sensor origin; then never claim "entirely outside".
    const FVector Cn = (A + B + C) / 3.0;
    const double Dc = Cn.Size();
    const double Rt = FMath::Max3(FVector::Dist(A, Cn), FVector::Dist(B, Cn), FVector::Dist(C, Cn));
    const bool bBounded = Dc > Rt;
    const double Ec = bBounded ? ElevationDeg(Cn) : 0.0;
    const double Delta = bBounded ? FMath::RadiansToDegrees(std::asin(Rt / Dc)) : 180.0;   // Rt < Dc, so the argument is in [0, 1)

    bool bAbove = false;
    if (HiDeg >= 90.0)     bAbove = false;
    else if (HiDeg >= 0.0) bAbove = EA > HiDeg && EB > HiDeg && EC > HiDeg;
    else                   bAbove = bBounded && (Ec - Delta > HiDeg);

    bool bBelow = false;
    if (LoDeg <= -90.0)    bBelow = false;
    else if (LoDeg <= 0.0) bBelow = EA < LoDeg && EB < LoDeg && EC < LoDeg;
    else                   bBelow = bBounded && (Ec + Delta < LoDeg);

    return !bAbove && !bBelow;
}

bool IsNear(const FVector& A, const FVector& B, const FVector& C, const FSensor& S)
{
    if (!(S.RadiusM > 0.0)) return false;
    const FVector Closest = ClosestPointOnTriangle(S.OriginM, A, B, C);
    if (FVector::DistSquared(Closest, S.OriginM) >= S.RadiusM * S.RadiusM) return false;
    const FVector LA = S.Rotation.UnrotateVector(A - S.OriginM);
    const FVector LB = S.Rotation.UnrotateVector(B - S.OriginM);
    const FVector LC = S.Rotation.UnrotateVector(C - S.OriginM);
    return TouchesElevationBand(LA, LB, LC, S.MinElevationDeg, S.MaxElevationDeg);
}

void ClassifyTriangles(const TArray<FVector>& VerticesM, const TArray<rgl_vec3i>& Indices,
                       const TArray<FSensor>& Sensors, TArray<int32>& OutOwner)
{
    OutOwner.Init(OwnerRest, Indices.Num());
    if (Sensors.Num() == 0) return;
    const int32 NumVerts = VerticesM.Num();
    for (int32 t = 0; t < Indices.Num(); ++t)
    {
        const rgl_vec3i& I = Indices[t];
        if (I.value[0] < 0 || I.value[0] >= NumVerts || I.value[1] < 0 || I.value[1] >= NumVerts ||
            I.value[2] < 0 || I.value[2] >= NumVerts)
        {
            continue;
        }
        const FVector& A = VerticesM[I.value[0]];
        const FVector& B = VerticesM[I.value[1]];
        const FVector& C = VerticesM[I.value[2]];
        const FVector Cn = (A + B + C) / 3.0;
        const double Rt = FMath::Max3(FVector::Dist(A, Cn), FVector::Dist(B, Cn), FVector::Dist(C, Cn));

        int32 Owner = OwnerRest;
        for (int32 s = 0; s < Sensors.Num(); ++s)
        {
            const FSensor& S = Sensors[s];
            // Cheap reject: every point of the triangle is at least |centroid - origin| - Rt away.
            if (FVector::Dist(Cn, S.OriginM) - Rt >= S.RadiusM) continue;
            if (!IsNear(A, B, C, S)) continue;
            if (Owner != OwnerRest) { Owner = OwnerShared; break; }
            Owner = s;
        }
        OutOwner[t] = Owner;
    }
}

bool SkinVertices(const FRGLSkeletalMeshData& Mesh, const TArray<rgl_mat3x4f>& WorldPose, TArray<FVector>& OutVerticesM)
{
    const int32 NumBones = Mesh.RawBoneNum;
    if (NumBones <= 0 || WorldPose.Num() < NumBones || Mesh.RestposesInv.Num() < NumBones ||
        Mesh.Weights.Num() != Mesh.Vertices.Num())
    {
        return false;
    }
    // Same chain as RGL's animator and the raytrace test's CPU replay: Pose[b] * RestposeInv[b].
    TArray<rgl_mat3x4f> Skin;
    Skin.SetNumUninitialized(NumBones);
    for (int32 b = 0; b < NumBones; ++b) Skin[b] = RGLCoord::MulRGL(WorldPose[b], Mesh.RestposesInv[b]);

    OutVerticesM.SetNumUninitialized(Mesh.Vertices.Num());
    for (int32 v = 0; v < Mesh.Vertices.Num(); ++v)
    {
        const rgl_vec3f& P = Mesh.Vertices[v];
        const rgl_bone_weights_t& W = Mesh.Weights[v];
        double Q[3] = { 0.0, 0.0, 0.0 };
        for (int32 k = 0; k < 4; ++k)
        {
            const float Wk = W.weights[k];
            const int32 Bk = W.bone_indexes[k];
            if (!(Wk > 0.0f) || Bk < 0 || Bk >= NumBones) continue;
            const rgl_mat3x4f& M = Skin[Bk];
            for (int32 r = 0; r < 3; ++r)
            {
                Q[r] += static_cast<double>(Wk) *
                        (M.value[r][0] * P.value[0] + M.value[r][1] * P.value[1] + M.value[r][2] * P.value[2] + M.value[r][3]);
            }
        }
        OutVerticesM[v] = FVector(Q[0], Q[1], Q[2]);
    }
    return true;
}

int32 ExtractPart(const FRGLSkeletalMeshData& Mesh, const TArray<int32>& Owner, int32 WantedOwner,
                  TArray<rgl_vec3f>& OutVertices, TArray<rgl_vec3i>& OutIndices, TArray<rgl_bone_weights_t>& OutWeights)
{
    OutVertices.Reset();
    OutIndices.Reset();
    OutWeights.Reset();
    const int32 NumVerts = Mesh.Vertices.Num();
    if (Mesh.Weights.Num() != NumVerts) return 0;
    TArray<int32> Remap;
    Remap.Init(INDEX_NONE, NumVerts);
    const int32 NumTris = FMath::Min(Mesh.Indices.Num(), Owner.Num());
    for (int32 t = 0; t < NumTris; ++t)
    {
        if (Owner[t] != WantedOwner) continue;
        const rgl_vec3i& In = Mesh.Indices[t];
        if (In.value[0] < 0 || In.value[0] >= NumVerts || In.value[1] < 0 || In.value[1] >= NumVerts ||
            In.value[2] < 0 || In.value[2] >= NumVerts)
        {
            continue;
        }
        rgl_vec3i Out;
        for (int32 k = 0; k < 3; ++k)
        {
            int32& R = Remap[In.value[k]];
            if (R == INDEX_NONE)
            {
                R = OutVertices.Num();
                OutVertices.Add(Mesh.Vertices[In.value[k]]);
                OutWeights.Add(Mesh.Weights[In.value[k]]);
            }
            Out.value[k] = R;
        }
        OutIndices.Add(Out);
    }
    return OutIndices.Num();
}

void SortConfig(TArray<FSensorConfig>& Config)
{
    Config.Sort([](const FSensorConfig& A, const FSensorConfig& B) { return A.Key < B.Key; });
}

bool SameConfig(const TArray<FSensorConfig>& A, const TArray<FSensorConfig>& B)
{
    if (A.Num() != B.Num()) return false;
    constexpr double LocationTolCm = 1.0;
    const double RotationTolRad = FMath::DegreesToRadians(0.1);
    for (int32 i = 0; i < A.Num(); ++i)
    {
        const FSensorConfig& X = A[i];
        const FSensorConfig& Y = B[i];
        if (X.Key != Y.Key) return false;
        if (!FMath::IsNearlyEqual(X.RadiusCm, Y.RadiusCm, 1e-3f)) return false;
        if (!FMath::IsNearlyEqual(X.MinElevationDeg, Y.MinElevationDeg, 1e-3f)) return false;
        if (!FMath::IsNearlyEqual(X.MaxElevationDeg, Y.MaxElevationDeg, 1e-3f)) return false;
        if (FVector::Dist(X.SensorToOwner.GetLocation(), Y.SensorToOwner.GetLocation()) > LocationTolCm) return false;
        if (X.SensorToOwner.GetRotation().AngularDistance(Y.SensorToOwner.GetRotation()) > RotationTolRad) return false;
    }
    return true;
}

} // namespace RGLSelfCutout
#endif // WITH_RGL
