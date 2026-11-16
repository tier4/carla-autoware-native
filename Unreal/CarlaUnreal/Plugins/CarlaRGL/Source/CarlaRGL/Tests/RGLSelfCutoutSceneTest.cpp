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
#include "../RGLSelfCutout.h"
#include "../RGLCoordinateUtils.h"
#include <rgl/api/core.h>

// Lidar self cutout through the scene manager (game world + GPU, like the other skeletal scene tests).
namespace RGLSelfCutoutTestsScene
{
static const TCHAR* kCarSK    = TEXT("/Game/Carla/Static/Car/4Wheeled/AudiTT/SK_AudiTT.SK_AudiTT");
static const TCHAR* kWalkerSK = TEXT("/Game/Carla/Static/Pedestrian/EuroG02_G2/SK_EuroG02_A_G2.SK_EuroG02_A_G2");
static const float  kZ = 30000.f;   // 300 m up: clear of the map

static UWorld* GetGameWorld()
{
    for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
        if (Ctx.WorldType == EWorldType::Game || Ctx.WorldType == EWorldType::PIE) return Ctx.World();
    return nullptr;
}
// Local copies (unity build: this file's own namespace) of the helpers of RGLSkeletalSceneManagerTest.cpp.
static void SetCVar(const TCHAR* Name, int32 V) { IConsoleManager::Get().FindConsoleVariable(Name)->Set(V, ECVF_SetByCode); }
struct FCVarGuard { FString Name; int32 Old; FCVarGuard(const TCHAR* N) : Name(N), Old(IConsoleManager::Get().FindConsoleVariable(N)->GetInt()) {} ~FCVarGuard() { SetCVar(*Name, Old); } };

static USkeletalMeshComponent* SpawnPawn(UWorld* World, USkeletalMesh* Mesh, const FVector& Loc, APawn*& OutPawn)
{
    OutPawn = World->SpawnActor<APawn>(APawn::StaticClass(), FTransform::Identity);
    USkeletalMeshComponent* Comp = NewObject<USkeletalMeshComponent>(OutPawn);
    OutPawn->SetRootComponent(Comp);
    Comp->SetSkeletalMeshAsset(Mesh);
    Comp->RegisterComponent();
    OutPawn->SetActorLocation(Loc);
    Comp->RefreshBoneTransforms();
    return Comp;
}

// World-space vertices (m) of Comp under its current pose: our own extraction + CPU skinning replay.
static bool SkinnedVerticesM(USkeletalMeshComponent* Comp, TArray<FVector>& Out)
{
    FRGLSkeletalMeshData Data; FString Reason;
    if (!RGLSkeletal::ExtractSkeletalMesh(Comp->GetSkeletalMeshAsset(), Data, Reason)) return false;
    TArray<rgl_mat3x4f> Pose;
    if (!RGLSkeletal::BuildWorldPose(Comp, Data.RawBoneNum, Pose)) return false;
    return RGLSelfCutout::SkinVertices(Data, Pose, Out);
}

// A point 10 cm outside the outermost vertex of the car's side (Side = +1: +Y, -1: -Y), at mid length
// and 45 % of the height (m). The window grows until it holds vertices.
static bool SideSensorPointM(const TArray<FVector>& V, double Side, FVector& Out)
{
    if (V.Num() == 0) return false;
    FBox Box(ForceInit);
    for (const FVector& P : V) Box += P;
    const double Cx = Box.GetCenter().X, Zc = Box.Min.Z + 0.45 * (Box.Max.Z - Box.Min.Z);
    for (double Win = 0.15; Win <= 0.61; Win *= 2.0)
    {
        bool bFound = false; double Edge = 0.0;
        for (const FVector& P : V)
        {
            if (FMath::Abs(P.X - Cx) > Win || FMath::Abs(P.Z - Zc) > Win) continue;
            const double Y = P.Y * Side;
            if (!bFound || Y > Edge) { Edge = Y; bFound = true; }
        }
        if (bFound) { Out = FVector(Cx, (Edge + 0.10) * Side, Zc); return true; }
    }
    return false;
}

// Fan of rays from OriginM (m): yaw [Yaw0, Yaw1] x pitch [Pitch0, Pitch1] (degrees, world frame), on a
// plain raytrace graph (no sensor id). Hit points in m.
static bool CastFan(const FVector& OriginM, float Yaw0, float Yaw1, float Pitch0, float Pitch1, TArray<FVector>& OutHits, FString& Err)
{
    const int32 NY = 41, NP = 17;
    TArray<rgl_mat3x4f> Rays; Rays.Reserve(NY * NP);
    for (int32 iy = 0; iy < NY; ++iy) for (int32 ip = 0; ip < NP; ++ip)
    {
        const float Yaw = Yaw0 + (Yaw1 - Yaw0) * iy / (NY - 1);
        const float Pitch = Pitch0 + (Pitch1 - Pitch0) * ip / (NP - 1);
        rgl_mat3x4f R = RGLCoord::FromPitchYaw(Pitch, Yaw);   // ray direction = column 2
        R.value[0][3] = static_cast<float>(OriginM.X); R.value[1][3] = static_cast<float>(OriginM.Y); R.value[2][3] = static_cast<float>(OriginM.Z);
        Rays.Add(R);
    }
    rgl_node_t RaysN = nullptr, RtN = nullptr, CompactN = nullptr, YieldN = nullptr;
    bool bRtLinked = false, bCompactLinked = false, bYieldLinked = false;
    const rgl_field_t Fields[] = { RGL_FIELD_XYZ_VEC3_F32 };
    // rgl_graph_destroy(RaysN) only frees the nodes reachable from RaysN: unlinked nodes go on their own.
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

static double MinDistance(const TArray<FVector>& Hits, const FVector& OriginM)
{
    double D = TNumericLimits<double>::Max(); for (const FVector& H : Hits) D = FMath::Min(D, FVector::Dist(H, OriginM)); return D;
}
// Hits closer than RadiusM to OriginM AND inside its vertical band (|elevation| < BandDeg, identity
// rotation): exactly the points that lie on triangles the cutout removes.
static int32 CountNearInBand(const TArray<FVector>& Hits, const FVector& OriginM, double RadiusM, double BandDeg)
{
    int32 N = 0;
    for (const FVector& H : Hits)
        N += (FVector::Dist(H, OriginM) < RadiusM && FMath::Abs(RGLSelfCutout::ElevationDeg(H - OriginM)) < BandDeg) ? 1 : 0;
    return N;
}
// Hits on the car (inside its box) at least RadiusM away from OriginM.
static int32 CountOnCarBeyond(const TArray<FVector>& Hits, const FBox& CarBoxM, const FVector& OriginM, double RadiusM)
{
    int32 N = 0;
    for (const FVector& H : Hits) N += (CarBoxM.IsInside(H) && FVector::Dist(H, OriginM) >= RadiusM) ? 1 : 0;
    return N;
}

// The default (no rgl_self_cutout_radius) keeps the shared path: same registrations, same shared mesh
// cache, no cutout mesh. A cutout on another vehicle leaves this one alone, and removing the cutout
// sensor returns its vehicle to the shared path without leftovers.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDefaultOffTest, "CarlaRGL.Skeletal.Cutout.DefaultOff",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FDefaultOffTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    USkeletalMesh* CarSK = LoadObject<USkeletalMesh>(nullptr, kCarSK);       if (!TestNotNull(TEXT("car SK"), CarSK)) return false;
    USkeletalMesh* WalkerSK = LoadObject<USkeletalMesh>(nullptr, kWalkerSK); if (!TestNotNull(TEXT("walker SK"), WalkerSK)) return false;
    FCVarGuard G1(TEXT("rgl.SkeletalMesh.Enable")), G2(TEXT("rgl.SkeletalMesh.MaxEntities"));
    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 1); SetCVar(TEXT("rgl.SkeletalMesh.MaxEntities"), 256);

    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    const void* Plain  = reinterpret_cast<const void*>(0xD0F0);
    const void* Cutout = reinterpret_cast<const void*>(0xD0F1);
    const FVector Base(0.f, -3000.f, kZ);
    double T = 600.0;
    const int32 RegBefore = SM.GetSkeletalRegisteredCount_ForTest(), LiveBefore = SM.GetSkeletalLiveEntityCount_ForTest();
    const int32 CacheBefore = SM.GetSkeletalMeshCacheCount_ForTest(), BuildsBefore = SM.GetSkeletalCutoutBuildCount_ForTest();

    APawn* Car = nullptr;    USkeletalMeshComponent* CarComp = SpawnPawn(World, CarSK, Base, Car);
    APawn* Walker = nullptr; USkeletalMeshComponent* WalkerComp = SpawnPawn(World, WalkerSK, Base + FVector(600.f, 0.f, 0.f), Walker);
    auto Step = [&]() { SM.RegisterSensor(Plain, Base + FVector(0.f, 0.f, 300.f), 10000.f, T); SM.SyncSkeletalNow_ForTest(World, T); SM.UpdateSkeletalPosesNow_ForTest(T); T += 0.05; };

    // 1) No sensor asks for a cutout
    Step();
    TestEqual(TEXT("both registered"), SM.GetSkeletalRegisteredCount_ForTest() - RegBefore, 2);
    TestEqual(TEXT("both live on the shared path"), SM.GetSkeletalLiveEntityCount_ForTest() - LiveBefore, 2);
    TestEqual(TEXT("2 shared meshes cached"), SM.GetSkeletalMeshCacheCount_ForTest() - CacheBefore, 2);
    TestFalse(TEXT("car not on the cutout path"), SM.IsSkeletalCutout_ForTest(CarComp));
    TestFalse(TEXT("walker not on the cutout path"), SM.IsSkeletalCutout_ForTest(WalkerComp));
    TestEqual(TEXT("no cutout mesh built"), SM.GetSkeletalCutoutBuildCount_ForTest(), BuildsBefore);

    // 2) Radius 0 is off
    const FTransform CarSensorTf(FQuat::Identity, Base + FVector(0.f, 0.f, 200.f));
    SM.RegisterSensor(Cutout, CarSensorTf.GetLocation(), 10000.f, T);
    SM.SetSensorSelfCutout(Cutout, Car, 0.f, CarSensorTf, -15.f, 15.f);
    Step();
    TestFalse(TEXT("radius 0: car stays shared"), SM.IsSkeletalCutout_ForTest(CarComp));
    TestEqual(TEXT("radius 0: no cutout mesh built"), SM.GetSkeletalCutoutBuildCount_ForTest(), BuildsBefore);
    TestEqual(TEXT("radius 0: cache unchanged"), SM.GetSkeletalMeshCacheCount_ForTest() - CacheBefore, 2);

    // 3) A cutout on the walker leaves the car on the shared path
    const FTransform WalkerSensorTf(FQuat::Identity, Base + FVector(630.f, 0.f, 0.f));
    SM.RegisterSensor(Cutout, WalkerSensorTf.GetLocation(), 10000.f, T);
    SM.SetSensorSelfCutout(Cutout, Walker, 30.f, WalkerSensorTf, -15.f, 15.f);
    Step();
    TestFalse(TEXT("car still shared"), SM.IsSkeletalCutout_ForTest(CarComp));
    TestTrue(TEXT("walker on the cutout path"), SM.IsSkeletalCutout_ForTest(WalkerComp));
    TestEqual(TEXT("walker's cutout mesh built once"), SM.GetSkeletalCutoutBuildCount_ForTest() - BuildsBefore, 1);
    TestTrue(TEXT("walker keeps triangles"), SM.GetSkeletalCutoutKeptTriangles_ForTest(WalkerComp) > 0);
    TestEqual(TEXT("walker's shared mesh released, car's kept"), SM.GetSkeletalMeshCacheCount_ForTest() - CacheBefore, 1);
    TestEqual(TEXT("both live (car shared, walker on its kept mesh)"), SM.GetSkeletalLiveEntityCount_ForTest() - LiveBefore, 2);

    // 4) Removing the cutout sensor returns the walker to the shared path in the same update
    SM.UnregisterSensor(Cutout);
    Step();
    TestFalse(TEXT("walker back on the shared path"), SM.IsSkeletalCutout_ForTest(WalkerComp));
    TestEqual(TEXT("no cutout mesh left"), SM.GetSkeletalCutoutKeptTriangles_ForTest(WalkerComp), -1);
    TestEqual(TEXT("both live again"), SM.GetSkeletalLiveEntityCount_ForTest() - LiveBefore, 2);
    TestEqual(TEXT("2 shared meshes cached again"), SM.GetSkeletalMeshCacheCount_ForTest() - CacheBefore, 2);

    Car->Destroy(); Walker->Destroy();
    Step();
    TestEqual(TEXT("none registered"), SM.GetSkeletalRegisteredCount_ForTest() - RegBefore, 0);
    SM.UnregisterSensor(Plain);
    return true;
}

// A cutout lidar 10 cm outside a car door: with R > 0 its rays reach the surfaces behind the near ones
// (no hit closer than R) and the car beyond R is still hit; a plain graph from elsewhere aimed at the
// same spot no longer hits it either (a hole for every sensor); with R = 0 the door is hit as before.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRaytraceTest, "CarlaRGL.Skeletal.Cutout.Raytrace",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FRaytraceTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    USkeletalMesh* CarSK = LoadObject<USkeletalMesh>(nullptr, kCarSK); if (!TestNotNull(TEXT("car SK"), CarSK)) return false;
    FCVarGuard G1(TEXT("rgl.SkeletalMesh.Enable")), G2(TEXT("rgl.SkeletalMesh.MaxEntities"));
    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 1); SetCVar(TEXT("rgl.SkeletalMesh.MaxEntities"), 256);

    APawn* Car = nullptr; USkeletalMeshComponent* Comp = SpawnPawn(World, CarSK, FVector(0.f, 6000.f, kZ), Car);
    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    const void* Sensor = reinterpret_cast<const void*>(0xC07);
    const float kRadiusCm = 30.f;
    const double RadiusM = kRadiusCm * RGLCoord::UE_TO_RGL;
    const double BandDeg = 10.0 + RGLSelfCutout::ElevationMarginDeg - 0.5;   // inside the removed band
    double T = 700.0;

    TArray<FVector> V; FVector OriginM;
    if (!TestTrue(TEXT("skinned vertices"), SkinnedVerticesM(Comp, V)) || !TestTrue(TEXT("side point"), SideSensorPointM(V, +1.0, OriginM)))
    {
        Car->Destroy(); return false;
    }
    FBox CarBoxM(ForceInit);
    for (const FVector& P : V) CarBoxM += P;
    CarBoxM = CarBoxM.ExpandBy(0.05);
    // Another graph 0.9 m farther out on the same line, aimed narrowly at the spot in front of the sensor.
    const FVector OtherM = OriginM + FVector(0.0, 0.9, 0.0);
    const FTransform SensorTf(FQuat::Identity, OriginM * RGLCoord::RGL_TO_UE);
    auto Step = [&]() { SM.RegisterSensor(Sensor, SensorTf.GetLocation(), 10000.f, T); SM.SyncSkeletalNow_ForTest(World, T); SM.UpdateSkeletalPosesNow_ForTest(T); T += 0.05; };
    auto Cast = [&](const TCHAR* What, TArray<FVector>& Own, TArray<FVector>& Other) -> bool
    {
        FString Err;
        const bool bOwn = CastFan(OriginM, -110.f, -70.f, -8.f, 8.f, Own, Err);   // the lidar's own rays, inside its band
        TestTrue(FString::Printf(TEXT("%s: cast from the sensor (%s)"), What, *Err), bOwn);
        const bool bOther = CastFan(OtherM, -93.f, -87.f, -0.5f, 0.5f, Other, Err);
        TestTrue(FString::Printf(TEXT("%s: cast from elsewhere (%s)"), What, *Err), bOther);
        AddInfo(FString::Printf(TEXT("%s: own_hits=%d own_min=%.3f m own_near=%d own_on_car_beyond=%d other_hits=%d other_near=%d"), What,
            Own.Num(), MinDistance(Own, OriginM), CountNearInBand(Own, OriginM, RadiusM, BandDeg), CountOnCarBeyond(Own, CarBoxM, OriginM, RadiusM),
            Other.Num(), CountNearInBand(Other, OriginM, RadiusM, BandDeg)));
        return bOwn && bOther;
    };

    // 1) R = 0: the shared path, the door is hit from both origins
    SM.RegisterSensor(Sensor, SensorTf.GetLocation(), 10000.f, T);
    SM.SetSensorSelfCutout(Sensor, Car, 0.f, SensorTf, -10.f, 10.f);
    Step();
    TestFalse(TEXT("R = 0: shared path"), SM.IsSkeletalCutout_ForTest(Comp));
    TArray<FVector> Own0, Other0;
    if (Cast(TEXT("R=0"), Own0, Other0))
    {
        TestTrue(TEXT("R = 0: the sensor hits the near surfaces"), CountNearInBand(Own0, OriginM, RadiusM, BandDeg) > 0);
        TestTrue(TEXT("R = 0: another graph hits them too"), CountNearInBand(Other0, OriginM, RadiusM, BandDeg) > 0);
    }

    // 2) R = 30 cm: the near triangles are out of the scene
    SM.SetSensorSelfCutout(Sensor, Car, kRadiusCm, SensorTf, -10.f, 10.f);
    Step();
    TestTrue(TEXT("cutout path"), SM.IsSkeletalCutout_ForTest(Comp));
    TestTrue(TEXT("triangles removed"), SM.GetSkeletalCutoutRemovedTriangles_ForTest(Comp) > 0);
    TestTrue(TEXT("triangles kept"), SM.GetSkeletalCutoutKeptTriangles_ForTest(Comp) > 0);
    TArray<FVector> Own, Other;
    if (Cast(TEXT("R=30cm"), Own, Other))
    {
        TestTrue(TEXT("the sensor hits what lies behind the near surfaces"), Own.Num() > 0);
        TestEqual(TEXT("the sensor: no hit closer than R"), CountNearInBand(Own, OriginM, RadiusM - 0.01, BandDeg), 0);
        TestTrue(TEXT("the sensor still hits the car beyond R"), CountOnCarBeyond(Own, CarBoxM, OriginM, RadiusM) > 0);
        TestEqual(TEXT("another graph: the near surfaces are a hole for it too"), CountNearInBand(Other, OriginM, RadiusM - 0.01, BandDeg), 0);
        TestTrue(TEXT("another graph still hits the car behind the hole"), CountOnCarBeyond(Other, CarBoxM, OriginM, RadiusM) > 0);
    }

    // 3) Back to R = 0 (sensor re-registered without a cutout): the door is hit again
    SM.UnregisterSensor(Sensor);
    SM.RegisterSensor(Sensor, SensorTf.GetLocation(), 10000.f, T);
    Step();
    TestFalse(TEXT("back on the shared path"), SM.IsSkeletalCutout_ForTest(Comp));
    TArray<FVector> Own1, Other1;
    if (Cast(TEXT("R=0 again"), Own1, Other1))
    {
        TestTrue(TEXT("R = 0 again: the door is hit"), CountNearInBand(Own1, OriginM, RadiusM, BandDeg) > 0);
        TestTrue(TEXT("R = 0 again: another graph hits it too"), CountNearInBand(Other1, OriginM, RadiusM, BandDeg) > 0);
    }

    Car->Destroy(); Step();
    TestFalse(TEXT("car unregistered"), SM.IsSkeletalRegistered_ForTest(Comp));
    SM.UnregisterSensor(Sensor);
    return true;
}

// The kept mesh follows the configuration: built once per change (not per tick), rebuilt on a new
// sensor, a radius change, a moved sensor, a removed sensor; jitter below 1 cm is ignored; a lost pose
// only drops the entity; the last cutout sensor gone returns the car to the shared path; nothing is
// left behind.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FResyncTest, "CarlaRGL.Skeletal.Cutout.Resync",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FResyncTest::RunTest(const FString& Parameters)
{
    UWorld* World = GetGameWorld(); if (!TestNotNull(TEXT("game world"), World)) return false;
    USkeletalMesh* CarSK = LoadObject<USkeletalMesh>(nullptr, kCarSK); if (!TestNotNull(TEXT("car SK"), CarSK)) return false;
    FCVarGuard G1(TEXT("rgl.SkeletalMesh.Enable")), G2(TEXT("rgl.SkeletalMesh.MaxEntities")), G3(TEXT("rgl.SkeletalMesh.Debug.ForcePoseFailure"));
    SetCVar(TEXT("rgl.SkeletalMesh.Enable"), 1); SetCVar(TEXT("rgl.SkeletalMesh.MaxEntities"), 256);

    FRGLSceneManager& SM = FRGLSceneManager::GetInstance(World);
    const int32 Live0 = SM.GetSkeletalLiveEntityCount_ForTest(), B0 = SM.GetSkeletalCutoutBuildCount_ForTest();
    const FVector CarLoc(0.f, 12000.f, kZ);
    APawn* Car = nullptr; USkeletalMeshComponent* Comp = SpawnPawn(World, CarSK, CarLoc, Car);
    const void* Plain = reinterpret_cast<const void*>(0xA0);
    const void* A = reinterpret_cast<const void*>(0xA1);
    const void* B = reinterpret_cast<const void*>(0xA2);
    double T = 800.0;

    TArray<FVector> V; FVector PA, PB;
    if (!TestTrue(TEXT("skinned vertices"), SkinnedVerticesM(Comp, V)) ||
        !TestTrue(TEXT("right side point"), SideSensorPointM(V, +1.0, PA)) || !TestTrue(TEXT("left side point"), SideSensorPointM(V, -1.0, PB)))
    {
        Car->Destroy(); return false;
    }
    FTransform TfA(FQuat::Identity, PA * RGLCoord::RGL_TO_UE), TfB(FQuat::Identity, PB * RGLCoord::RGL_TO_UE);
    bool bA = true, bB = false;
    auto Step = [&](bool bSync)
    {
        SM.RegisterSensor(Plain, CarLoc + FVector(0.f, 0.f, 300.f), 10000.f, T);
        if (bA) SM.RegisterSensor(A, TfA.GetLocation(), 10000.f, T);
        if (bB) SM.RegisterSensor(B, TfB.GetLocation(), 10000.f, T);
        if (bSync) SM.SyncSkeletalNow_ForTest(World, T);
        SM.UpdateSkeletalPosesNow_ForTest(T);
        T += 0.05;
    };
    auto Removed = [&]() { return SM.GetSkeletalCutoutRemovedTriangles_ForTest(Comp); };

    // 1) First cutout sensor
    SM.RegisterSensor(A, TfA.GetLocation(), 10000.f, T);
    SM.SetSensorSelfCutout(A, Car, 30.f, TfA, -15.f, 15.f);
    Step(true);
    TestTrue(TEXT("car on the cutout path"), SM.IsSkeletalCutout_ForTest(Comp));
    TestEqual(TEXT("built once"), SM.GetSkeletalCutoutBuildCount_ForTest() - B0, 1);
    TestEqual(TEXT("live on its kept mesh"), SM.GetSkeletalLiveEntityCount_ForTest() - Live0, 1);
    const int32 RemovedA30 = Removed();
    TestTrue(TEXT("near A removed"), RemovedA30 > 0);

    // 2) Unchanged configuration: no rebuild per tick
    Step(false); Step(true); Step(false);
    TestEqual(TEXT("stable: not rebuilt"), SM.GetSkeletalCutoutBuildCount_ForTest() - B0, 1);
    TestEqual(TEXT("stable: same removal"), Removed(), RemovedA30);

    // 3) Second cutout sensor on the other side
    bB = true;
    SM.RegisterSensor(B, TfB.GetLocation(), 10000.f, T);
    SM.SetSensorSelfCutout(B, Car, 30.f, TfB, -15.f, 15.f);
    Step(false);
    TestEqual(TEXT("sensor added: rebuilt"), SM.GetSkeletalCutoutBuildCount_ForTest() - B0, 2);
    const int32 RemovedAB = Removed();
    TestTrue(TEXT("near B removed too"), RemovedAB > RemovedA30);

    // 4) Pose lost -> entity dropped, kept mesh stays; pose back -> entity again, no rebuild
    SetCVar(TEXT("rgl.SkeletalMesh.Debug.ForcePoseFailure"), 1); Step(false);
    TestEqual(TEXT("pose lost: entity dropped"), SM.GetSkeletalLiveEntityCount_ForTest() - Live0, 0);
    TestTrue(TEXT("pose lost: still on the cutout path"), SM.IsSkeletalCutout_ForTest(Comp));
    SetCVar(TEXT("rgl.SkeletalMesh.Debug.ForcePoseFailure"), 0); Step(false);
    TestEqual(TEXT("pose back: live again"), SM.GetSkeletalLiveEntityCount_ForTest() - Live0, 1);
    TestEqual(TEXT("pose back: not rebuilt"), SM.GetSkeletalCutoutBuildCount_ForTest() - B0, 2);

    // 5) Larger radius for A: more triangles removed
    SM.SetSensorSelfCutout(A, Car, 50.f, TfA, -15.f, 15.f);
    Step(false);
    TestEqual(TEXT("radius changed: rebuilt"), SM.GetSkeletalCutoutBuildCount_ForTest() - B0, 3);
    TestTrue(TEXT("radius 50 cm removes more than 30 cm"), Removed() > RemovedAB);

    // 6) B moved by 5 cm: rebuilt. A jittered by 3 mm: not rebuilt.
    TfB.AddToTranslation(FVector(5.f, 0.f, 0.f));
    SM.SetSensorSelfCutout(B, Car, 30.f, TfB, -15.f, 15.f);
    Step(false);
    TestEqual(TEXT("sensor moved: rebuilt"), SM.GetSkeletalCutoutBuildCount_ForTest() - B0, 4);
    SM.SetSensorSelfCutout(A, Car, 50.f, FTransform(FQuat::Identity, TfA.GetLocation() + FVector(0.3f, 0.f, 0.f)), -15.f, 15.f);
    Step(false);
    TestEqual(TEXT("jitter: not rebuilt"), SM.GetSkeletalCutoutBuildCount_ForTest() - B0, 4);

    // 7) A removed: only B's neighbourhood stays out
    const int32 RemovedBeforeDrop = Removed();
    bA = false; SM.UnregisterSensor(A);
    Step(true);
    TestEqual(TEXT("sensor removed: rebuilt"), SM.GetSkeletalCutoutBuildCount_ForTest() - B0, 5);
    TestTrue(TEXT("fewer triangles removed"), Removed() > 0 && Removed() < RemovedBeforeDrop);

    // 8) Last cutout sensor removed: back on the shared path, no cutout mesh left
    bB = false; SM.UnregisterSensor(B);
    Step(true);
    TestFalse(TEXT("back on the shared path"), SM.IsSkeletalCutout_ForTest(Comp));
    TestEqual(TEXT("no cutout mesh left"), Removed(), -1);
    TestEqual(TEXT("live on the shared path"), SM.GetSkeletalLiveEntityCount_ForTest() - Live0, 1);

    // 9) Cutout again, then the car destroyed: nothing left behind
    bA = true;
    SM.RegisterSensor(A, TfA.GetLocation(), 10000.f, T);
    SM.SetSensorSelfCutout(A, Car, 30.f, TfA, -15.f, 15.f);
    Step(true);
    TestTrue(TEXT("cutout again"), SM.IsSkeletalCutout_ForTest(Comp) && Removed() > 0);
    Car->Destroy();
    Step(true);
    TestFalse(TEXT("car unregistered"), SM.IsSkeletalRegistered_ForTest(Comp));
    TestEqual(TEXT("no entity left"), SM.GetSkeletalLiveEntityCount_ForTest() - Live0, 0);
    SM.UnregisterSensor(A); SM.UnregisterSensor(Plain);
    return true;
}
}
#endif
