#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLSelfCutout.h"
#include "../RGLCoordinateUtils.h"
#include <cmath>

// Pure-function tests of the lidar self cutout: no world, no GPU.
namespace RGLSelfCutoutTestsPure
{
namespace SC = RGLSelfCutout;

static bool IsClose(const FVector& A, const FVector& B, double Tol = 1e-9) { return FVector::Dist(A, B) <= Tol; }

// Small upright triangle (about 10 cm) centred at C, facing +-X.
static void AddSmallTri(TArray<FVector>& V, TArray<rgl_vec3i>& I, const FVector& C)
{
    const int32 Base = V.Num();
    V.Add(C + FVector(0.0, -0.05, -0.05)); V.Add(C + FVector(0.0, 0.05, -0.05)); V.Add(C + FVector(0.0, 0.0, 0.05));
    rgl_vec3i T; T.value[0] = Base; T.value[1] = Base + 1; T.value[2] = Base + 2; I.Add(T);
}

static SC::FSensor MakeSensor(const FVector& Origin, double RadiusM, const FRotator& Rot = FRotator::ZeroRotator)
{
    SC::FSensor S;
    S.OriginM = Origin; S.Rotation = Rot.Quaternion(); S.RadiusM = RadiusM;
    S.MinElevationDeg = -15.0 - SC::ElevationMarginDeg; S.MaxElevationDeg = 15.0 + SC::ElevationMarginDeg;
    return S;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FClosestPointTest, "CarlaRGL.Skeletal.Cutout.ClosestPoint",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FClosestPointTest::RunTest(const FString& Parameters)
{
    const FVector A(0, 0, 0), B(1, 0, 0), C(0, 1, 0);
    TestTrue(TEXT("face"),         IsClose(SC::ClosestPointOnTriangle(FVector(0.2, 0.2, 1.0), A, B, C), FVector(0.2, 0.2, 0.0)));
    TestTrue(TEXT("vertex A"),     IsClose(SC::ClosestPointOnTriangle(FVector(-1, -1, 0), A, B, C), A));
    TestTrue(TEXT("vertex B"),     IsClose(SC::ClosestPointOnTriangle(FVector(2, -1, 0), A, B, C), B));
    TestTrue(TEXT("vertex C"),     IsClose(SC::ClosestPointOnTriangle(FVector(-1, 2, 0), A, B, C), C));
    TestTrue(TEXT("edge AB"),      IsClose(SC::ClosestPointOnTriangle(FVector(0.5, -1, 0), A, B, C), FVector(0.5, 0, 0)));
    TestTrue(TEXT("edge AC"),      IsClose(SC::ClosestPointOnTriangle(FVector(-1, 0.5, 0), A, B, C), FVector(0, 0.5, 0)));
    TestTrue(TEXT("edge BC"),      IsClose(SC::ClosestPointOnTriangle(FVector(1, 1, 0), A, B, C), FVector(0.5, 0.5, 0)));
    const FVector D = SC::ClosestPointOnTriangle(FVector(3, 3, 3), A, A, A);
    TestTrue(TEXT("degenerate triangle stays finite"), !D.ContainsNaN() && IsClose(D, A));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FElevationBandTest, "CarlaRGL.Skeletal.Cutout.ElevationBand",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FElevationBandTest::RunTest(const FString& Parameters)
{
    TestTrue(TEXT("elevation of +Z"), FMath::IsNearlyEqual(SC::ElevationDeg(FVector(0, 0, 1)), 90.0, 1e-9));
    TestTrue(TEXT("elevation of 45 deg"), FMath::IsNearlyEqual(SC::ElevationDeg(FVector(1, 0, 1)), 45.0, 1e-9));
    // band straddling 0: exact vertex rule
    TestFalse(TEXT("all vertices above"), SC::TouchesElevationBand(FVector(1, 0, 1), FVector(1, 0.1, 1), FVector(1, -0.1, 1), -10, 10));
    TestTrue(TEXT("one vertex inside"), SC::TouchesElevationBand(FVector(1, 0, 0), FVector(1, 0.1, 1), FVector(1, -0.1, 1), -10, 10));
    TestFalse(TEXT("all vertices below"), SC::TouchesElevationBand(FVector(1, 0, -1), FVector(1, 0.1, -1), FVector(1, -0.1, -1), -10, 10));
    TestTrue(TEXT("spans the band without a vertex inside"), SC::TouchesElevationBand(FVector(1, 0, 1), FVector(1, 0.1, -1), FVector(1, -0.1, 1), -10, 10));
    // band entirely below the horizon: a wide triangle under the sensor has every vertex above the
    // band but its interior right below the sensor crosses it -> must not be rejected
    TestTrue(TEXT("interior crosses a negative band"),
             SC::TouchesElevationBand(FVector(5, 0, -1), FVector(-5, 5, -1), FVector(-5, -5, -1), -30, -20));
    // band entirely above the horizon: a small far triangle at 10 deg is safely below it
    const double Z10 = 10.0 * std::tan(FMath::DegreesToRadians(10.0));
    TestFalse(TEXT("small triangle under a positive band"),
              SC::TouchesElevationBand(FVector(10, 0, Z10), FVector(10, 0.01, Z10), FVector(10, -0.01, Z10 + 0.01), 20, 40));
    TestTrue(TEXT("triangle through the origin touches any band"),
             SC::TouchesElevationBand(FVector(-1, -1, 0), FVector(1, -1, 0), FVector(0, 1, 0), 20, 40));
    TestFalse(TEXT("empty band"), SC::TouchesElevationBand(FVector(1, 0, 0), FVector(1, 1, 0), FVector(1, 0, 1), 5, -5));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FClassifyTest, "CarlaRGL.Skeletal.Cutout.Classify",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FClassifyTest::RunTest(const FString& Parameters)
{
    TArray<FVector> V; TArray<rgl_vec3i> I;
    AddSmallTri(V, I, FVector(0.3, 0.0, 0.0));    // T0: 0.3 m in front of S0, in its band
    AddSmallTri(V, I, FVector(3.7, 0.0, 0.0));    // T1: 0.3 m behind S1, in its band
    AddSmallTri(V, I, FVector(1.5, 0.0, 0.0));    // T2: farther than the radius from everyone
    {   // T3: 0.3 m right above S0 (elevation ~80 deg): within the radius but outside the band
        const int32 B0 = V.Num();
        V.Add(FVector(-0.05, -0.05, 0.3)); V.Add(FVector(0.05, -0.05, 0.3)); V.Add(FVector(0.0, 0.05, 0.3));
        rgl_vec3i T; T.value[0] = B0; T.value[1] = B0 + 1; T.value[2] = B0 + 2; I.Add(T);
    }
    AddSmallTri(V, I, FVector(10.3, 0.0, 0.0));   // T4: between S2 and S3, near both
    {   // T5: wide triangle whose vertices are far but whose face passes 0.2 m above S0
        const int32 B0 = V.Num();
        V.Add(FVector(-5, -5, 0.2)); V.Add(FVector(5, -5, 0.2)); V.Add(FVector(0, 5, 0.2));
        rgl_vec3i T; T.value[0] = B0; T.value[1] = B0 + 1; T.value[2] = B0 + 2; I.Add(T);
    }
    {   // T6: out-of-range vertex index stays in the rest
        rgl_vec3i T; T.value[0] = 0; T.value[1] = 1; T.value[2] = V.Num(); I.Add(T);
    }

    TArray<SC::FSensor> S;
    S.Add(MakeSensor(FVector(0, 0, 0), 0.5));
    S.Add(MakeSensor(FVector(4, 0, 0), 0.5));
    S.Add(MakeSensor(FVector(10, 0, 0), 0.5));
    S.Add(MakeSensor(FVector(10.6, 0, 0), 0.5));

    TArray<int32> Owner;
    SC::ClassifyTriangles(V, I, S, Owner);
    if (!TestEqual(TEXT("one owner per triangle"), Owner.Num(), I.Num())) return false;
    TestEqual(TEXT("T0 near S0"), Owner[0], 0);
    TestEqual(TEXT("T1 near S1"), Owner[1], 1);
    TestEqual(TEXT("T2 far -> rest"), Owner[2], SC::OwnerRest);
    TestEqual(TEXT("T3 outside the band -> rest"), Owner[3], SC::OwnerRest);
    TestEqual(TEXT("T4 near two sensors -> shared"), Owner[4], SC::OwnerShared);
    TestEqual(TEXT("T5 face within the radius -> near S0"), Owner[5], 0);
    TestEqual(TEXT("T6 bad index -> rest"), Owner[6], SC::OwnerRest);

    // The band is in the sensor frame: S0 pitched 90 deg up sees T3 at elevation 0 and T0 below it.
    S[0] = MakeSensor(FVector(0, 0, 0), 0.5, FRotator(90.0, 0.0, 0.0));
    SC::ClassifyTriangles(V, I, S, Owner);
    TestEqual(TEXT("pitched S0: T0 now outside the band"), Owner[0], SC::OwnerRest);
    TestEqual(TEXT("pitched S0: T3 now near"), Owner[3], 0);

    // No sensors / zero radius: everything stays in the rest
    SC::ClassifyTriangles(V, I, TArray<SC::FSensor>(), Owner);
    bool bAllRest = true; for (int32 O : Owner) bAllRest &= (O == SC::OwnerRest);
    TestTrue(TEXT("no sensors -> all rest"), bAllRest);
    TArray<SC::FSensor> Zero; Zero.Add(MakeSensor(FVector(0, 0, 0), 0.0));
    SC::ClassifyTriangles(V, I, Zero, Owner);
    bAllRest = true; for (int32 O : Owner) bAllRest &= (O == SC::OwnerRest);
    TestTrue(TEXT("zero radius -> all rest"), bAllRest);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FConfigTest, "CarlaRGL.Skeletal.Cutout.Config",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FConfigTest::RunTest(const FString& Parameters)
{
    auto Make = [](UPTRINT Key, const FVector& LocCm, const FRotator& Rot)
    {
        SC::FSensorConfig C; C.Key = Key; C.RadiusCm = 30.f; C.MinElevationDeg = -15.f; C.MaxElevationDeg = 15.f;
        C.SensorToOwner = FTransform(Rot, LocCm); return C;
    };
    TArray<SC::FSensorConfig> A; A.Add(Make(0x20, FVector(100, 0, 200), FRotator(0, 0, 0))); A.Add(Make(0x10, FVector(-100, 0, 200), FRotator(0, 180, 0)));
    TArray<SC::FSensorConfig> B = A;
    SC::SortConfig(A);
    TestFalse(TEXT("order matters before sorting"), SC::SameConfig(A, B));
    SC::SortConfig(B);
    TestTrue(TEXT("same after sorting"), SC::SameConfig(A, B));

    TArray<SC::FSensorConfig> C = B; C[0].SensorToOwner.AddToTranslation(FVector(0.5, 0, 0));
    TestTrue(TEXT("0.5 cm jitter ignored"), SC::SameConfig(A, C));
    C = B; C[0].SensorToOwner.AddToTranslation(FVector(2.0, 0, 0));
    TestFalse(TEXT("2 cm move detected"), SC::SameConfig(A, C));
    C = B; C[0].SensorToOwner.SetRotation(C[0].SensorToOwner.GetRotation() * FRotator(0, 0.05, 0).Quaternion());
    TestTrue(TEXT("0.05 deg jitter ignored"), SC::SameConfig(A, C));
    C = B; C[0].SensorToOwner.SetRotation(C[0].SensorToOwner.GetRotation() * FRotator(0, 1.0, 0).Quaternion());
    TestFalse(TEXT("1 deg turn detected"), SC::SameConfig(A, C));
    C = B; C[1].RadiusCm = 40.f;
    TestFalse(TEXT("radius change detected"), SC::SameConfig(A, C));
    C = B; C[1].MaxElevationDeg = 20.f;
    TestFalse(TEXT("field of view change detected"), SC::SameConfig(A, C));
    C = B; C[1].Key = 0x30;
    TestFalse(TEXT("another sensor detected"), SC::SameConfig(A, C));
    C = B; C.SetNum(1);
    TestFalse(TEXT("sensor removed detected"), SC::SameConfig(A, C));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPartsTest, "CarlaRGL.Skeletal.Cutout.Parts",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FPartsTest::RunTest(const FString& Parameters)
{
    // Two bones; bone 1 carries vertices 3 and 4.
    FRGLSkeletalMeshData M;
    M.RawBoneNum = 2; M.LODIndex = 0;
    for (int32 v = 0; v < 5; ++v) M.Vertices.Add(rgl_vec3f{ { float(v), 0.f, 0.f } });
    for (int32 v = 0; v < 5; ++v)
    {
        const int32 Bone = (v >= 3) ? 1 : 0;
        M.Weights.Add(rgl_bone_weights_t{ { 1.f, 0.f, 0.f, 0.f }, { Bone, Bone, Bone, Bone } });
    }
    auto Tri = [](int32 a, int32 b, int32 c) { rgl_vec3i T; T.value[0] = a; T.value[1] = b; T.value[2] = c; return T; };
    M.Indices.Add(Tri(0, 1, 2)); M.Indices.Add(Tri(1, 2, 3)); M.Indices.Add(Tri(2, 3, 4));
    M.RestposesInv.Add(RGLCoord::Identity()); M.RestposesInv.Add(RGLCoord::Identity());

    const TArray<int32> Owner = { 0, SC::OwnerRest, 0 };
    TArray<rgl_vec3f> PV; TArray<rgl_vec3i> PIdx; TArray<rgl_bone_weights_t> PW;
    TestEqual(TEXT("part 0 has 2 triangles"), SC::ExtractPart(M, Owner, 0, PV, PIdx, PW), 2);
    TestEqual(TEXT("part 0 compacted to the 5 referenced vertices"), PV.Num(), 5);
    TestEqual(TEXT("weights follow vertices"), PW.Num(), PV.Num());
    bool bSame = true;   // every remapped corner still points at the original position and weight
    for (int32 t = 0, k2 = 0; t < M.Indices.Num(); ++t)
    {
        if (Owner[t] != 0) continue;
        for (int32 k = 0; k < 3; ++k)
        {
            const int32 Src = M.Indices[t].value[k], Dst = PIdx[k2].value[k];
            bSame &= (PV[Dst].value[0] == M.Vertices[Src].value[0]) && (PW[Dst].bone_indexes[0] == M.Weights[Src].bone_indexes[0]);
        }
        ++k2;
    }
    TestTrue(TEXT("remap keeps positions and weights"), bSame);
    TestEqual(TEXT("rest part has 1 triangle"), SC::ExtractPart(M, Owner, SC::OwnerRest, PV, PIdx, PW), 1);
    TestEqual(TEXT("rest part has 3 vertices"), PV.Num(), 3);
    TestEqual(TEXT("absent owner -> empty part"), SC::ExtractPart(M, Owner, 7, PV, PIdx, PW), 0);

    // CPU skinning: bone 0 identity, bone 1 translated by (1, 2, 3) m.
    TArray<rgl_mat3x4f> Pose; Pose.Add(RGLCoord::Identity()); Pose.Add(RGLCoord::FromTranslation(1.f, 2.f, 3.f));
    TArray<FVector> W;
    if (!TestTrue(TEXT("skin ok"), SC::SkinVertices(M, Pose, W))) return false;
    TestTrue(TEXT("bone 0 vertex unchanged"), IsClose(W[1], FVector(1, 0, 0), 1e-6));
    TestTrue(TEXT("bone 1 vertex moved"), IsClose(W[4], FVector(5, 2, 3), 1e-6));
    Pose.SetNum(1);
    TestFalse(TEXT("short pose rejected"), SC::SkinVertices(M, Pose, W));
    return true;
}
}
#endif
