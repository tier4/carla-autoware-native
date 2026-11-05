// Copyright (c) 2026 RGL Integration for CARLA.
// Implementation of FRGLSceneManager: UE5 <-> RGL scene synchronization.

#ifdef WITH_RGL

#include "RGLSceneManager.h"
#include "RGLCoordinateUtils.h"
#include "CarlaRGLModule.h"
#include "RGLSkeletalMeshExtractor.h"
#include "RGLDynLoader.h"

#include <cstdio>   // Shipping breadcrumbs: NO_LOGGING compiles UE_LOG out (precedent: RGLDynLoader.cpp)

#include <util/ue-header-guard-begin.h>
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/SkeletalMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Pawn.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Rendering/PositionVertexBuffer.h"
#include "StaticMeshResources.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "EngineUtils.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include <util/ue-header-guard-end.h>

#include <util/disable-ue4-macros.h>
#include <carla/Logging.h>
#include <util/enable-ue4-macros.h>

// RGL_CHECK macro is defined in RGLCoordinateUtils.h

// [DEBUG] Global counters for mesh extraction path statistics (Development only)
#if !UE_BUILD_SHIPPING
int32 GRGLExtractRender = 0, GRGLExtractGPU = 0, GRGLExtractChaos = 0, GRGLExtractFailed = 0;
int64 GRGLTotalVertices = 0, GRGLTotalTriangles = 0;
#endif

// ============================================================================
// Static singleton storage
// ============================================================================

// ---- Skeletal mesh path CVars (spec §2.3) ----
static TAutoConsoleVariable<int32> CVarRGLSkeletalEnable(TEXT("rgl.SkeletalMesh.Enable"), 1,
    TEXT("Register pawn-owned USkeletalMeshComponents in the RGL scene. 0 = legacy static-mesh-only (live: tears down)."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalAlwaysTickPose(TEXT("rgl.SkeletalMesh.AlwaysTickPose"), 1,
    TEXT("Force AlwaysTickPoseAndRefreshBones and disable URO on registered skeletal components (live: re-applied/restored at next sync)."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalScope(TEXT("rgl.SkeletalMesh.Scope"), 0,
    TEXT("0 = APawn-owned components only. 1 = every USkeletalMeshComponent."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalMaxEntities(TEXT("rgl.SkeletalMesh.MaxEntities"), 64,
    TEXT("Budget of simultaneously registered skeletal components; ranked by distance to the nearest sensor. "
         "Device memory is roughly 24 B per vertex per registered entity (RGL SkeletonAnimator + GAS), "
         "so a 350k-vertex LOD0 vehicle costs ~8 MB; lower this (or raise MinLOD) on VRAM-tight setups."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalMinLOD(TEXT("rgl.SkeletalMesh.MinLOD"), 0,
    TEXT("Lowest LOD index the extractor may use; raise to trade fidelity for VRAM/skinning cost."), ECVF_Default);
#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarRGLSkeletalDebugForceApiUnavailable(TEXT("rgl.SkeletalMesh.Debug.ForceApiUnavailable"), 0,
    TEXT("Test only: pretend the RGL skeleton API did not resolve."), ECVF_Default);
static TAutoConsoleVariable<int32> CVarRGLSkeletalDebugForcePoseFailure(TEXT("rgl.SkeletalMesh.Debug.ForcePoseFailure"), 0,
    TEXT("Test only: make every skeletal pose build fail (exercises destroy->pending->recover)."), ECVF_Default);
#endif

// Shipping-safe rebuild-free kill switch (spec §6.4). A packaged Shipping server cannot
// take `-ExecCmds=` (UnrealEngine.cpp: #if !UE_BUILD_SHIPPING) nor `-ini:` overrides
// (ALLOW_INI_OVERRIDE_FROM_COMMANDLINE = UE_SERVER || !UE_BUILD_SHIPPING), so the CVars
// above would be unreachable from the command line. FParse::Value is compiled in for every
// configuration, so these switches work everywhere; they only *set* the CVars, which remain
// the single source of truth. ECVF_SetByCommandline ranks below ECVF_SetByCode, so console
// commands and the Automation tests can still override them at runtime.
static void ApplySkeletalCommandLineOverrides()
{
    static bool bApplied = false; if (bApplied) return; bApplied = true;
    int32 V = 0;
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-enable="), V))
    {
        CVarRGLSkeletalEnable->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: Enable=%d"), V);
    }
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-always-tick-pose="), V))
    {
        CVarRGLSkeletalAlwaysTickPose->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: AlwaysTickPose=%d"), V);
    }
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-scope="), V))
    {
        CVarRGLSkeletalScope->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: Scope=%d"), V);
    }
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-max-entities="), V))
    {
        CVarRGLSkeletalMaxEntities->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: MaxEntities=%d"), V);
    }
    if (FParse::Value(FCommandLine::Get(), TEXT("-rgl-skeletal-mesh-min-lod="), V))
    {
        CVarRGLSkeletalMinLOD->Set(V, ECVF_SetByCommandline);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] command line: MinLOD=%d"), V);
    }
}

TMap<UWorld*, FRGLSceneManager*> FRGLSceneManager::Instances;

FRGLSceneManager::FRGLSceneManager()
{
    // Apply the Shipping-safe command-line switches before anything reads the CVars.
    ApplySkeletalCommandLineOverrides();

    // Use the default scene (nullptr).
    // RGL treats nullptr as the implicit default scene.
    Scene = nullptr;

    bSkeletalApiAvailable = RGLDynLoader::IsSkeletalApiAvailable();
    if (!bSkeletalApiAvailable)
    {
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager: RGL skeleton API unavailable; skeletal meshes will not be registered."));
    }
}

FRGLSceneManager::~FRGLSceneManager()
{
    TeardownSkeletal();

    // Destroy ground plane
    if (GroundEntity)
    {
        rgl_entity_destroy(GroundEntity);
        GroundEntity = nullptr;
    }
    if (GroundMesh)
    {
        rgl_mesh_destroy(GroundMesh);
        GroundMesh = nullptr;
    }

    // Destroy all entities first (they reference meshes)
    for (auto& Pair : EntityMap)
    {
        if (Pair.Value.Entity)
        {
            rgl_entity_destroy(Pair.Value.Entity);
            Pair.Value.Entity = nullptr;
        }
    }
    EntityMap.Empty();

    // Destroy all ISMC entities
    for (auto& Pair : ISMCEntityMap)
    {
        for (rgl_entity_t Entity : Pair.Value.Entities)
        {
            if (Entity)
            {
                rgl_entity_destroy(Entity);
            }
        }
    }
    ISMCEntityMap.Empty();

    // Destroy all cached meshes
    for (auto& Pair : MeshCache)
    {
        if (Pair.Value)
        {
            rgl_mesh_destroy(Pair.Value);
        }
    }
    MeshCache.Empty();
    MeshRefCounts.Empty();
}

FRGLSceneManager& FRGLSceneManager::GetInstance(UWorld* World)
{
    // Clean up stale entries for worlds that are no longer valid
    TArray<UWorld*> StaleWorlds;
    for (auto& Pair : Instances)
    {
        if (!IsValid(Pair.Key))
        {
            StaleWorlds.Add(Pair.Key);
        }
    }
    for (UWorld* Stale : StaleWorlds)
    {
        RGLLog::Info("RGLSceneManager: Cleaning up stale instance for destroyed world");
        delete Instances[Stale];
        Instances.Remove(Stale);
    }

    FRGLSceneManager** Found = Instances.Find(World);
    if (Found && *Found)
    {
        return **Found;
    }

    FRGLSceneManager* NewInstance = new FRGLSceneManager();
    Instances.Add(World, NewInstance);
    return *NewInstance;
}

void FRGLSceneManager::DestroyInstance(UWorld* World)
{
    FRGLSceneManager** Found = Instances.Find(World);
    if (Found && *Found)
    {
        delete *Found;
        Instances.Remove(World);
    }
}

void FRGLSceneManager::DestroyAllInstances()
{
    for (auto& Pair : Instances) delete Pair.Value;
    Instances.Empty();
}

// ============================================================================
// Update: called each frame before raytrace
// ============================================================================

void FRGLSceneManager::Update(UWorld* World, double SimulationTime)
{
    if (!World)
    {
        return;
    }

    // Frame skip: avoid redundant updates if already called this frame
    const uint64 CurrentFrame = GFrameCounter;
    if (bInitialized && CurrentFrame == LastUpdateFrame)
    {
        return;
    }
    LastUpdateFrame = CurrentFrame;

    if (!bInitialized)
    {
        InitializeFromWorld(World);
        bInitialized = true;
    }

    // Track sim time for stale-sensor eviction (used inside SyncWorldComponents).
    CurrentSimTime = (SimulationTime >= 0.0) ? SimulationTime : World->GetTimeSeconds();

    // Sync on a time interval OR when any sensor has moved far enough that newly
    // relevant geometry could be missed before the next periodic sync
    // (Phase 1: high-speed safety).
    TimeSinceLastSync += World->GetDeltaSeconds();
    bool bShouldSync = (TimeSinceLastSync >= SyncIntervalSeconds);
    if (!bShouldSync)
    {
        for (const auto& Pair : RegisteredSensors)
        {
            const FSensorRegistration& S = Pair.Value;
            const float TriggerCm = 0.2f * S.RegistrationDistanceCm;
            if (!S.bLastSyncValid || FVector::Dist(S.Position, S.LastSyncPosition) > TriggerCm)
            {
                bShouldSync = true;
                break;
            }
        }
    }
    if (bShouldSync)
    {
        SyncWorldComponents(World);
        TimeSinceLastSync = 0.0f;
        for (auto& Pair : RegisteredSensors)
        {
            Pair.Value.LastSyncPosition = Pair.Value.Position;
            Pair.Value.bLastSyncValid = true;
        }
    }

    // Update scene time for velocity computation and ROS2 timestamp synchronization.
    // Use SimulationTime (elapsed since episode start) if provided, otherwise fall back
    // to World->GetTimeSeconds() which includes UE5 engine startup time (~7s offset).
    const double SceneTime = (SimulationTime >= 0.0) ? SimulationTime : World->GetTimeSeconds();
    const uint64 TimeNs = static_cast<uint64>(SceneTime * 1e9);
    RGL_CHECK(rgl_scene_set_time(Scene, TimeNs));

    // Update only dynamic entity transforms (static entities skip after first set)
    UpdateTransforms();

    // Re-pose every registered skeletal entity for this scene time (no skip optimization).
    if (SkeletalEnabled())
    {
        UpdateSkeletalPoses();
    }
}

// ============================================================================
// InitializeFromWorld: first-time scan
// ============================================================================

void FRGLSceneManager::InitializeFromWorld(UWorld* World)
{
    RGLLog::Info("RGLSceneManager: Initializing from world...");
    RGLLog::Info("RGLSceneManager: Initializing with registered sensors=",
                 RegisteredSensors.Num());

    int32 EntityCount = 0;
    int32 SkippedCount = 0;
    int32 RangeSkippedCount = 0;
    int32 ActorCount = 0;
    int32 CompCount = 0;

    // [DEBUG] Diagnose what types of collision-bearing actors exist in the world
#if !UE_BUILD_SHIPPING
    {
        int32 LandscapeCount = 0;
        int32 StaticMeshActorCount = 0;
        int32 CollisionCompCount = 0;
        int32 NoStaticMeshCompCount = 0;

        for (TActorIterator<AActor> It(World); It; ++It)
        {
            AActor* Actor = *It;
            if (!IsValid(Actor)) continue;

            FString ClassName = Actor->GetClass()->GetName();

            if (ClassName.Contains(TEXT("Landscape")))
            {
                ++LandscapeCount;
            }

            TArray<UStaticMeshComponent*> SMComps;
            Actor->GetComponents<UStaticMeshComponent>(SMComps);
            if (SMComps.Num() > 0)
            {
                ++StaticMeshActorCount;
            }
            else
            {
                // Check if this actor has any primitive component with collision
                TArray<UPrimitiveComponent*> PrimComps;
                Actor->GetComponents<UPrimitiveComponent>(PrimComps);
                for (UPrimitiveComponent* Prim : PrimComps)
                {
                    if (Prim && Prim->IsCollisionEnabled())
                    {
                        ++CollisionCompCount;
                        // Log first few non-StaticMesh collision actors
                        static int32 sNonSMLog = 0;
                        if (sNonSMLog < 5)
                        {
                            RGLLog::Info("[DEBUG] Non-SM collision actor:",
                                         TCHAR_TO_UTF8(*Actor->GetName()),
                                         "class:", TCHAR_TO_UTF8(*ClassName),
                                         "comp:", TCHAR_TO_UTF8(*Prim->GetClass()->GetName()));
                            ++sNonSMLog;
                        }
                        break;
                    }
                }
                ++NoStaticMeshCompCount;
            }
        }

        RGLLog::Info("[DEBUG] World actor types: Landscape=", LandscapeCount,
                     "WithStaticMesh=", StaticMeshActorCount,
                     "WithoutStaticMesh=", NoStaticMeshCompCount,
                     "NonSM-Collision=", CollisionCompCount);
    }
#endif

    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!IsValid(Actor))
        {
            continue;
        }
        ++ActorCount;

        TArray<UStaticMeshComponent*> Components;
        Actor->GetComponents<UStaticMeshComponent>(Components);

        for (UStaticMeshComponent* Comp : Components)
        {
            if (IsValid(Comp) && Comp->GetStaticMesh())
            {
                ++CompCount;
            }

            bool bOutOfRange = false;
            if (!ShouldRegisterComponent(Comp, bOutOfRange))
            {
                if (bOutOfRange)
                {
                    ++RangeSkippedCount;
                }
                continue;
            }

            if (RegisterComponent(Comp))
            {
                ++EntityCount;
            }
            else
            {
                ++SkippedCount;
            }
        }
    }

    if (SkeletalEnabled())
    {
        SyncSkeletalComponents(World);
    }

    const int32 MeshCount = MeshCache.Num();
    RGLLog::Info("RGLSceneManager: Scanned", ActorCount, "actors,", CompCount,
                 "mesh components. Uploaded", MeshCount, "unique meshes, created",
                 EntityCount, "entities (skipped", SkippedCount,
                 ", range-skipped", RangeSkippedCount, ")");

    const int32 ISMCCompCount = ISMCEntityMap.Num();
    int32 ISMCEntityTotal = 0;
    for (const auto& Pair : ISMCEntityMap)
    {
        ISMCEntityTotal += Pair.Value.Entities.Num();
    }
    RGLLog::Info("RGLSceneManager: ISMC components=", ISMCCompCount,
                 " total ISMC instances=", ISMCEntityTotal);

#if !UE_BUILD_SHIPPING
    // Print extraction path statistics
    {
        extern int32 GRGLExtractRender, GRGLExtractGPU, GRGLExtractChaos, GRGLExtractFailed;
        extern int64 GRGLTotalVertices, GRGLTotalTriangles;
        RGLLog::Info("[DEBUG] Mesh extraction paths: RenderData=", GRGLExtractRender,
                     "GPUBuffer=", GRGLExtractGPU,
                     "ChaosCollision=", GRGLExtractChaos, "Failed=", GRGLExtractFailed);
        RGLLog::Info("[DEBUG] Total scene geometry: vertices=", GRGLTotalVertices,
                     "triangles=", GRGLTotalTriangles);

        int32 StaticCount = 0, DynamicCount = 0;
        for (const auto& Pair : EntityMap)
        {
            if (Pair.Value.bIsStatic) ++StaticCount; else ++DynamicCount;
        }
        RGLLog::Info("[DEBUG] Entity mobility: Static=", StaticCount, "Dynamic=", DynamicCount);
        RGLLog::Info("[DEBUG] ISMC entity map: components=", ISMCEntityMap.Num());
    }
#endif
}

// ============================================================================
// Mesh upload: extract vertex/index data from UStaticMesh
// ============================================================================

/// Try to extract geometry from Chaos TriMeshes (complex collision, full triangle detail).
/// This is the primary path — preserves concave geometry unlike ConvexElems.
static bool ExtractFromChaosTriMeshes(UStaticMesh* StaticMesh,
                                      TArray<rgl_vec3f>& OutVertices,
                                      TArray<rgl_vec3i>& OutIndices)
{
    UBodySetup* BodySetup = StaticMesh->GetBodySetup();
    if (!BodySetup)
    {
        return false;
    }

    if (BodySetup->ChaosTriMeshes.Num() > 0)
    {
        // Merge all tri-meshes into one vertex/index set
        int32 VertexOffset = 0;

        for (const auto& TriMeshPtr : BodySetup->ChaosTriMeshes)
        {
            if (!TriMeshPtr.IsValid())
            {
                continue;
            }

            const Chaos::FTriangleMeshImplicitObject& TriMesh = *TriMeshPtr;
            const auto& Particles = TriMesh.Particles();
            const auto& Elements = TriMesh.Elements();
            const int32 NumVerts = static_cast<int32>(Particles.Size());
            const int32 NumTris = Elements.GetNumTriangles();

            if (NumVerts == 0 || NumTris == 0)
            {
                continue;
            }

            // Append vertices (in local space, cm -> m)
            const int32 PrevVertCount = OutVertices.Num();
            OutVertices.AddUninitialized(NumVerts);
            for (int32 i = 0; i < NumVerts; ++i)
            {
                const Chaos::FVec3 P = Particles.GetX(i);
                OutVertices[PrevVertCount + i] = { {
                    static_cast<float>(P[0]) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(P[1]) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(P[2]) * RGLCoord::UE_TO_RGL
                } };
            }

            // Append indices (offset by accumulated vertex count)
            const int32 PrevTriCount = OutIndices.Num();
            OutIndices.AddUninitialized(NumTris);

            if (Elements.RequiresLargeIndices())
            {
                const auto& IdxBuf = Elements.GetLargeIndexBuffer();
                for (int32 i = 0; i < NumTris; ++i)
                {
                    OutIndices[PrevTriCount + i] = { {
                        static_cast<int32>(IdxBuf[i][0]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][1]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][2]) + VertexOffset
                    } };
                }
            }
            else
            {
                const auto& IdxBuf = Elements.GetSmallIndexBuffer();
                for (int32 i = 0; i < NumTris; ++i)
                {
                    OutIndices[PrevTriCount + i] = { {
                        static_cast<int32>(IdxBuf[i][0]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][1]) + VertexOffset,
                        static_cast<int32>(IdxBuf[i][2]) + VertexOffset
                    } };
                }
            }

            VertexOffset += NumVerts;
        }

        return OutVertices.Num() > 0 && OutIndices.Num() > 0;
    }

    return false;
}

/// Fallback: extract from ConvexElems or BoxElems (simplified convex geometry).
/// Only used as last resort — convex hulls lose concave features.
static bool ExtractFromChaosConvexOrBox(UStaticMesh* StaticMesh,
                                        TArray<rgl_vec3f>& OutVertices,
                                        TArray<rgl_vec3i>& OutIndices)
{
    UBodySetup* BodySetup = StaticMesh->GetBodySetup();
    if (!BodySetup)
    {
        return false;
    }

    const FKAggregateGeom& AggGeom = BodySetup->AggGeom;
    if (AggGeom.ConvexElems.Num() > 0)
    {
        int32 VertexOffset = 0;
        for (const FKConvexElem& Convex : AggGeom.ConvexElems)
        {
            const int32 NumVerts = Convex.VertexData.Num();
            const int32 NumIdx = Convex.IndexData.Num();
            const int32 NumTris = NumIdx / 3;
            if (NumVerts == 0 || NumTris == 0)
            {
                continue;
            }

            const int32 PrevVertCount = OutVertices.Num();
            OutVertices.AddUninitialized(NumVerts);
            for (int32 i = 0; i < NumVerts; ++i)
            {
                const FVector& V = Convex.VertexData[i];
                OutVertices[PrevVertCount + i] = { {
                    static_cast<float>(V.X) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(V.Y) * RGLCoord::UE_TO_RGL,
                    static_cast<float>(V.Z) * RGLCoord::UE_TO_RGL
                } };
            }

            const int32 PrevTriCount = OutIndices.Num();
            OutIndices.AddUninitialized(NumTris);
            for (int32 i = 0; i < NumTris; ++i)
            {
                OutIndices[PrevTriCount + i] = { {
                    static_cast<int32>(Convex.IndexData[i * 3 + 0]) + VertexOffset,
                    static_cast<int32>(Convex.IndexData[i * 3 + 1]) + VertexOffset,
                    static_cast<int32>(Convex.IndexData[i * 3 + 2]) + VertexOffset
                } };
            }

            VertexOffset += NumVerts;
        }

        return OutVertices.Num() > 0 && OutIndices.Num() > 0;
    }

    // Fallback: try box collision elements (common for walls, floors, building shells)
    if (AggGeom.BoxElems.Num() > 0)
    {
        int32 VertexOffset = 0;
        for (const FKBoxElem& Box : AggGeom.BoxElems)
        {
            // Generate 8 vertices and 12 triangles for each box
            const float HX = static_cast<float>(Box.X) * 0.5f * RGLCoord::UE_TO_RGL; // half-extent in meters
            const float HY = static_cast<float>(Box.Y) * 0.5f * RGLCoord::UE_TO_RGL;
            const float HZ = static_cast<float>(Box.Z) * 0.5f * RGLCoord::UE_TO_RGL;

            // Box center offset (in meters)
            const FVector C = Box.Center;
            const float CX = static_cast<float>(C.X) * RGLCoord::UE_TO_RGL;
            const float CY = static_cast<float>(C.Y) * RGLCoord::UE_TO_RGL;
            const float CZ = static_cast<float>(C.Z) * RGLCoord::UE_TO_RGL;

            const int32 PrevVert = OutVertices.Num();
            OutVertices.AddUninitialized(8);
            OutVertices[PrevVert + 0] = {{ CX - HX, CY - HY, CZ - HZ }};
            OutVertices[PrevVert + 1] = {{ CX + HX, CY - HY, CZ - HZ }};
            OutVertices[PrevVert + 2] = {{ CX + HX, CY + HY, CZ - HZ }};
            OutVertices[PrevVert + 3] = {{ CX - HX, CY + HY, CZ - HZ }};
            OutVertices[PrevVert + 4] = {{ CX - HX, CY - HY, CZ + HZ }};
            OutVertices[PrevVert + 5] = {{ CX + HX, CY - HY, CZ + HZ }};
            OutVertices[PrevVert + 6] = {{ CX + HX, CY + HY, CZ + HZ }};
            OutVertices[PrevVert + 7] = {{ CX - HX, CY + HY, CZ + HZ }};

            const int32 V = VertexOffset;
            const int32 PrevTri = OutIndices.Num();
            OutIndices.AddUninitialized(12);
            // Bottom face
            OutIndices[PrevTri + 0]  = {{ V+0, V+2, V+1 }};
            OutIndices[PrevTri + 1]  = {{ V+0, V+3, V+2 }};
            // Top face
            OutIndices[PrevTri + 2]  = {{ V+4, V+5, V+6 }};
            OutIndices[PrevTri + 3]  = {{ V+4, V+6, V+7 }};
            // Front face
            OutIndices[PrevTri + 4]  = {{ V+0, V+1, V+5 }};
            OutIndices[PrevTri + 5]  = {{ V+0, V+5, V+4 }};
            // Back face
            OutIndices[PrevTri + 6]  = {{ V+2, V+3, V+7 }};
            OutIndices[PrevTri + 7]  = {{ V+2, V+7, V+6 }};
            // Left face
            OutIndices[PrevTri + 8]  = {{ V+0, V+4, V+7 }};
            OutIndices[PrevTri + 9]  = {{ V+0, V+7, V+3 }};
            // Right face
            OutIndices[PrevTri + 10] = {{ V+1, V+2, V+6 }};
            OutIndices[PrevTri + 11] = {{ V+1, V+6, V+5 }};

            VertexOffset += 8;
        }

        return OutVertices.Num() > 0 && OutIndices.Num() > 0;
    }

    return false;
}

/// Try to extract geometry from render vertex/index buffers (requires CPU access).
static bool ExtractFromRenderData(UStaticMesh* StaticMesh,
                                  TArray<rgl_vec3f>& OutVertices,
                                  TArray<rgl_vec3i>& OutIndices)
{
    const FStaticMeshRenderData* RenderData = StaticMesh->GetRenderData();
    if (!RenderData || RenderData->LODResources.Num() == 0)
    {
        return false;
    }

    const FStaticMeshLODResources& LOD = RenderData->LODResources[0];
    const FPositionVertexBuffer& VB = LOD.VertexBuffers.PositionVertexBuffer;
    const uint32 NumVertices = VB.GetNumVertices();

    if (NumVertices == 0 || VB.GetStride() == 0)
    {
        return false;
    }

    // CPU data must be accessible (otherwise VertexPosition() dereferences null -> SIGSEGV).
    // This check is required in ALL build configurations — even Development packaged builds
    // release CPU-side vertex data after GPU upload.
    if (!StaticMesh->bAllowCPUAccess && !VB.GetAllowCPUAccess())
    {
        return false;
    }

    OutVertices.SetNum(NumVertices);
    for (uint32 i = 0; i < NumVertices; ++i)
    {
        const FVector3f& Pos = VB.VertexPosition(i);
        OutVertices[i] = { {
            Pos.X * RGLCoord::UE_TO_RGL,
            Pos.Y * RGLCoord::UE_TO_RGL,
            Pos.Z * RGLCoord::UE_TO_RGL
        } };
    }

    FIndexArrayView IndexView = LOD.IndexBuffer.GetArrayView();
    const int32 NumTriangles = IndexView.Num() / 3;
    if (NumTriangles == 0)
    {
        OutVertices.Empty();
        return false;
    }

    OutIndices.SetNum(NumTriangles);
    for (int32 i = 0; i < NumTriangles; ++i)
    {
        const int32 Idx0 = static_cast<int32>(IndexView[i * 3 + 0]);
        const int32 Idx1 = static_cast<int32>(IndexView[i * 3 + 1]);
        const int32 Idx2 = static_cast<int32>(IndexView[i * 3 + 2]);

        if (Idx0 >= static_cast<int32>(NumVertices) ||
            Idx1 >= static_cast<int32>(NumVertices) ||
            Idx2 >= static_cast<int32>(NumVertices))
        {
            OutVertices.Empty();
            OutIndices.Empty();
            return false;
        }

        OutIndices[i] = { { Idx0, Idx1, Idx2 } };
    }

    return true;
}

/// Extract geometry by reading GPU vertex/index buffers via RHI readback.
/// Works in ALL build configurations (Shipping included) since render data is always on GPU.
/// Must be called from game thread (uses FlushRenderingCommands).
/// Uses the coarsest available LOD to minimize polygon count for raytrace performance.
static bool ExtractFromGPUBuffer(UStaticMesh* StaticMesh,
                                  TArray<rgl_vec3f>& OutVertices,
                                  TArray<rgl_vec3i>& OutIndices)
{
    const FStaticMeshRenderData* RenderData = StaticMesh->GetRenderData();
    if (!RenderData || RenderData->LODResources.Num() == 0)
    {
        return false;
    }

    const FStaticMeshLODResources& LOD = RenderData->LODResources[0];

    // --- Vertex buffer readback ---
    const FPositionVertexBuffer& VB = LOD.VertexBuffers.PositionVertexBuffer;
    const uint32 NumVertices = VB.GetNumVertices();
    if (NumVertices == 0)
    {
        return false;
    }

    const FBufferRHIRef& VBRef = VB.VertexBufferRHI;
    if (!VBRef.IsValid())
    {
        return false;
    }

    // Flush rendering to ensure the buffer is up to date
    FlushRenderingCommands();

    const uint32 VBSize = VBRef->GetSize();
    const uint32 Stride = VBSize / NumVertices;
    if (Stride < sizeof(FVector3f))
    {
        return false;
    }

    // Lock the GPU buffer for reading
    void* LockedVB = nullptr;
    ENQUEUE_RENDER_COMMAND(LockVertexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        LockedVB = RHICmdList.LockBuffer(VBRef, 0, VBSize, RLM_ReadOnly);
    });
    FlushRenderingCommands();

    if (!LockedVB)
    {
        return false;
    }

    OutVertices.SetNum(NumVertices);
    for (uint32 i = 0; i < NumVertices; ++i)
    {
        const FVector3f* Pos = reinterpret_cast<const FVector3f*>(
            static_cast<const uint8*>(LockedVB) + i * Stride);
        OutVertices[i] = { {
            Pos->X * RGLCoord::UE_TO_RGL,
            Pos->Y * RGLCoord::UE_TO_RGL,
            Pos->Z * RGLCoord::UE_TO_RGL
        } };
    }

    ENQUEUE_RENDER_COMMAND(UnlockVertexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        RHICmdList.UnlockBuffer(VBRef);
    });
    FlushRenderingCommands();

    // --- Index buffer readback ---
    const FRawStaticIndexBuffer& IB = LOD.IndexBuffer;
    const FBufferRHIRef& IBRef = IB.IndexBufferRHI;
    if (!IBRef.IsValid())
    {
        OutVertices.Empty();
        return false;
    }

    const uint32 IBSize = IBRef->GetSize();
    const bool bIs32Bit = (IB.Is32Bit());
    const uint32 IndexStride = bIs32Bit ? 4 : 2;
    const uint32 NumIndices = IBSize / IndexStride;
    const uint32 NumTriangles = NumIndices / 3;

    if (NumTriangles == 0)
    {
        OutVertices.Empty();
        return false;
    }

    void* LockedIB = nullptr;
    ENQUEUE_RENDER_COMMAND(LockIndexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        LockedIB = RHICmdList.LockBuffer(IBRef, 0, IBSize, RLM_ReadOnly);
    });
    FlushRenderingCommands();

    if (!LockedIB)
    {
        OutVertices.Empty();
        return false;
    }

    OutIndices.SetNum(NumTriangles);
    for (uint32 i = 0; i < NumTriangles; ++i)
    {
        int32 Idx0, Idx1, Idx2;
        if (bIs32Bit)
        {
            const uint32* Indices32 = static_cast<const uint32*>(LockedIB);
            Idx0 = static_cast<int32>(Indices32[i * 3 + 0]);
            Idx1 = static_cast<int32>(Indices32[i * 3 + 1]);
            Idx2 = static_cast<int32>(Indices32[i * 3 + 2]);
        }
        else
        {
            const uint16* Indices16 = static_cast<const uint16*>(LockedIB);
            Idx0 = static_cast<int32>(Indices16[i * 3 + 0]);
            Idx1 = static_cast<int32>(Indices16[i * 3 + 1]);
            Idx2 = static_cast<int32>(Indices16[i * 3 + 2]);
        }

        if (Idx0 >= static_cast<int32>(NumVertices) ||
            Idx1 >= static_cast<int32>(NumVertices) ||
            Idx2 >= static_cast<int32>(NumVertices))
        {
            OutVertices.Empty();
            OutIndices.Empty();
            ENQUEUE_RENDER_COMMAND(UnlockIndexBuffer)([&](FRHICommandListImmediate& RHICmdList)
            {
                RHICmdList.UnlockBuffer(IBRef);
            });
            FlushRenderingCommands();
            return false;
        }

        OutIndices[i] = { { Idx0, Idx1, Idx2 } };
    }

    ENQUEUE_RENDER_COMMAND(UnlockIndexBuffer)([&](FRHICommandListImmediate& RHICmdList)
    {
        RHICmdList.UnlockBuffer(IBRef);
    });
    FlushRenderingCommands();

    return true;
}

rgl_mesh_t FRGLSceneManager::UploadMesh(UStaticMesh* StaticMesh)
{
    if (!StaticMesh)
    {
        return nullptr;
    }

    // Check cache first
    rgl_mesh_t* CachedMesh = MeshCache.Find(StaticMesh);
    if (CachedMesh)
    {
        return *CachedMesh;
    }

    TArray<rgl_vec3f> Vertices;
    TArray<rgl_vec3i> Indices;

    // Extraction priority:
    // 1. Chaos TriMeshes — complex collision with full triangle detail (fast raytrace)
    // 2. Render data (CPU access) — if bAllowCPUAccess is set
    // 3. GPU buffer readback — render mesh from GPU (full fidelity, heavy)
    // 4. Chaos ConvexElems/BoxElems — simplified convex hulls (last resort)
    //
    // Note: AWSIM uses render meshes or direct collision trimeshes, never convex hulls.
    // ConvexElems lose concave features (e.g., spiral towers become simple polyhedra).
    // GPU readback preserves full visual fidelity, so it is preferred over ConvexElems.
    if (ExtractFromChaosTriMeshes(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractChaos;
#endif
    }
    else if (ExtractFromRenderData(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractRender;
#endif
    }
    else if (ExtractFromGPUBuffer(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractGPU;
#endif
    }
    else if (ExtractFromChaosConvexOrBox(StaticMesh, Vertices, Indices))
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractChaos;  // Still count as Chaos path
#endif
    }
    else
    {
#if !UE_BUILD_SHIPPING
        ++GRGLExtractFailed;
#endif
        return nullptr;
    }

    if (Vertices.Num() == 0 || Indices.Num() == 0)
    {
        return nullptr;
    }

#if !UE_BUILD_SHIPPING
    // [DEBUG] Log first mesh's vertex bounds
    {
        static int32 sMeshLogCount = 0;
        if (sMeshLogCount < 2)
        {
            float minX=1e9, minY=1e9, minZ=1e9, maxX=-1e9, maxY=-1e9, maxZ=-1e9;
            for (int32 i = 0; i < Vertices.Num(); ++i)
            {
                minX = FMath::Min(minX, Vertices[i].value[0]);
                minY = FMath::Min(minY, Vertices[i].value[1]);
                minZ = FMath::Min(minZ, Vertices[i].value[2]);
                maxX = FMath::Max(maxX, Vertices[i].value[0]);
                maxY = FMath::Max(maxY, Vertices[i].value[1]);
                maxZ = FMath::Max(maxZ, Vertices[i].value[2]);
            }
            RGLLog::Info("[DEBUG] Mesh", sMeshLogCount,
                         "name:", TCHAR_TO_UTF8(*StaticMesh->GetName()),
                         "verts:", Vertices.Num(), "tris:", Indices.Num(),
                         "bounds(m): min(", minX, minY, minZ, ") max(", maxX, maxY, maxZ, ")");
            ++sMeshLogCount;
        }
    }
#endif

#if !UE_BUILD_SHIPPING
    // Track total scene geometry
    extern int64 GRGLTotalVertices, GRGLTotalTriangles;
    GRGLTotalVertices += Vertices.Num();
    GRGLTotalTriangles += Indices.Num();
#endif

    // Upload to RGL
    rgl_mesh_t RGLMesh = nullptr;
    rgl_status_t Status = rgl_mesh_create(
        &RGLMesh,
        Vertices.GetData(),
        static_cast<int32_t>(Vertices.Num()),
        Indices.GetData(),
        static_cast<int32_t>(Indices.Num()));

    if (Status != RGL_SUCCESS || !RGLMesh)
    {
        const char* ErrMsg = nullptr;
        rgl_get_last_error_string(&ErrMsg);
        UE_LOG(LogCarlaRGL, Warning,
               TEXT("RGLSceneManager: Failed to upload mesh '%s' (%d verts, %d tris): %s"),
               *StaticMesh->GetName(),
               Vertices.Num(), Indices.Num(),
               ErrMsg ? *FString(UTF8_TO_TCHAR(ErrMsg)) : TEXT("unknown"));
        return nullptr;
    }

    // Cache the mesh
    MeshCache.Add(StaticMesh, RGLMesh);

    return RGLMesh;
}

void FRGLSceneManager::AddMeshReference(UStaticMesh* StaticMesh, int32 Count)
{
    if (!StaticMesh || Count <= 0)
    {
        return;
    }

    int32& RefCount = MeshRefCounts.FindOrAdd(StaticMesh);
    RefCount += Count;
}

void FRGLSceneManager::ReleaseMeshReference(UStaticMesh* StaticMesh, int32 Count)
{
    if (!StaticMesh || Count <= 0)
    {
        return;
    }

    int32* RefCount = MeshRefCounts.Find(StaticMesh);
    if (!RefCount)
    {
        return;
    }

    *RefCount -= Count;
    if (*RefCount > 0)
    {
        return;
    }

    MeshRefCounts.Remove(StaticMesh);

    rgl_mesh_t* RGLMesh = MeshCache.Find(StaticMesh);
    if (RGLMesh && *RGLMesh)
    {
        rgl_mesh_destroy(*RGLMesh);
    }
    MeshCache.Remove(StaticMesh);
}

// ============================================================================
// Entity management
// ============================================================================

bool FRGLSceneManager::IsWithinAnySensor(const FVector& CenterCm, float BoundsRadiusCm, float ExtraCm) const
{
    for (const auto& Pair : RegisteredSensors)
    {
        const FSensorRegistration& S = Pair.Value;
        const float EffectiveDistanceCm = S.RegistrationDistanceCm + BoundsRadiusCm + ExtraCm;
        if (FVector::DistSquared(CenterCm, S.Position) <= FMath::Square(EffectiveDistanceCm))
        {
            return true;
        }
    }
    return false;
}

bool FRGLSceneManager::IsComponentWithinAnySensor(UStaticMeshComponent* Component, float ExtraCm) const
{
    if (!IsValid(Component))
    {
        return false;
    }
    return IsWithinAnySensor(Component->Bounds.Origin, Component->Bounds.SphereRadius, ExtraCm);
}

bool FRGLSceneManager::IsInstanceWithinAnySensor(
    UInstancedStaticMeshComponent* Component,
    UStaticMesh* StaticMesh,
    const FTransform& InstanceTransform,
    float ExtraCm) const
{
    if (!IsValid(Component) || !StaticMesh)
    {
        return false;
    }
    const FVector Scale = InstanceTransform.GetScale3D();
    const float MaxScale = FMath::Max(FMath::Max(FMath::Abs(Scale.X), FMath::Abs(Scale.Y)), FMath::Abs(Scale.Z));
    const float BoundsRadiusCm = StaticMesh->GetBounds().SphereRadius * MaxScale;
    return IsWithinAnySensor(InstanceTransform.GetLocation(), BoundsRadiusCm, ExtraCm);
}

bool FRGLSceneManager::ShouldRegisterComponent(UStaticMeshComponent* Component, bool& bOutOfRange) const
{
    bOutOfRange = false;

    if (!IsValid(Component) || !Component->GetStaticMesh() || !Component->IsVisible())
    {
        return false;
    }

    const FTransform WorldTransform = Component->GetComponentTransform();
    const FVector Scale = WorldTransform.GetScale3D();
    if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
    {
        return false;
    }

    AActor* Owner = Component->GetOwner();
    const FString OwnerName = Owner ? Owner->GetName() : FString();
    const FString ComponentName = Component->GetName();
    const FString MeshName = Component->GetStaticMesh()->GetName();

    // Sky domes are valid static meshes but should never participate in LiDAR ray tracing.
    if (OwnerName.Contains(TEXT("Sky"), ESearchCase::IgnoreCase) ||
        ComponentName.Contains(TEXT("Sky"), ESearchCase::IgnoreCase) ||
        MeshName.Contains(TEXT("Sky"), ESearchCase::IgnoreCase))
    {
        return false;
    }

    // Movable actors are kept even if no sensor has registered yet.
    if (Component->Mobility != EComponentMobility::Static)
    {
        return true;
    }

    // Static geometry on large maps is large enough to exhaust OptiX if uploaded wholesale.
    // Register only meshes whose bounds intersect the union of all sensor spheres.
    if (RegisteredSensors.Num() == 0)
    {
        bOutOfRange = true;
        return false;
    }

    if (!IsComponentWithinAnySensor(Component, 0.0f))
    {
        bOutOfRange = true;
        return false;
    }

    return true;
}

bool FRGLSceneManager::ShouldKeepRegisteredComponent(UStaticMeshComponent* Component) const
{
    bool bOutOfRange = false;
    if (ShouldRegisterComponent(Component, bOutOfRange))
    {
        return true;
    }

    if (!bOutOfRange || !IsValid(Component))
    {
        return false;
    }

    // Keep a wider unload radius than the load radius to prevent repeated
    // destroy/recreate near the boundary while sensors move.
    return IsComponentWithinAnySensor(Component, UnregistrationHysteresisCm);
}

bool FRGLSceneManager::RegisterComponent(UStaticMeshComponent* Component)
{
    if (!IsValid(Component) || EntityMap.Contains(Component))
    {
        return false;
    }

    // ISMC branch: expand all instances into separate RGL entities
    if (UInstancedStaticMeshComponent* ISMComp = Cast<UInstancedStaticMeshComponent>(Component))
    {
        if (ISMCEntityMap.Contains(ISMComp))
        {
            return false;
        }
        return RegisterISMComponent(ISMComp);
    }

    UStaticMesh* StaticMesh = Component->GetStaticMesh();
    if (!StaticMesh)
    {
        return false;
    }

    rgl_mesh_t RGLMesh = UploadMesh(StaticMesh);
    if (!RGLMesh)
    {
        return false;
    }
    AddMeshReference(StaticMesh);

    rgl_entity_t Entity = nullptr;
    rgl_status_t Status = rgl_entity_create(&Entity, Scene, RGLMesh);
    if (Status != RGL_SUCCESS || !Entity)
    {
        ReleaseMeshReference(StaticMesh);
        const char* ErrMsg = nullptr;
        rgl_get_last_error_string(&ErrMsg);
        UE_LOG(LogCarlaRGL, Warning,
               TEXT("RGLSceneManager: Failed to create entity for '%s': %s"),
               *Component->GetName(),
               ErrMsg ? *FString(UTF8_TO_TCHAR(ErrMsg)) : TEXT("unknown"));
        return false;
    }

    // Set initial transform
    const FTransform WorldTransform = Component->GetComponentTransform();
    rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(WorldTransform);
    RGL_CHECK(rgl_entity_set_transform(Entity, &RGLTransform));

#if !UE_BUILD_SHIPPING
    // [DEBUG] Log first 3 entities' positions
    {
        static int32 sEntityLogCount = 0;
        if (sEntityLogCount < 3)
        {
            const FVector Pos = WorldTransform.GetLocation();
            RGLLog::Info("[DEBUG] Entity", sEntityLogCount,
                         "name:", TCHAR_TO_UTF8(*Component->GetOwner()->GetName()),
                         "UE5pos(cm):", Pos.X, Pos.Y, Pos.Z,
                         "RGLpos(m):", RGLTransform.value[0][3], RGLTransform.value[1][3], RGLTransform.value[2][3]);
            ++sEntityLogCount;
        }
    }
#endif

    // Store mapping with static/dynamic classification
    FEntityInfo Info;
    Info.Entity = Entity;
    Info.Component = Component;
    Info.StaticMesh = StaticMesh;
    Info.bIsStatic = (Component->Mobility == EComponentMobility::Static);
    Info.bTransformInitialized = true;
    Info.LastTransform = WorldTransform;
    EntityMap.Add(Component, Info);

    return true;
}

void FRGLSceneManager::UnregisterComponent(UStaticMeshComponent* Component)
{
    FEntityInfo* Info = EntityMap.Find(Component);
    if (!Info)
    {
        return;
    }

    if (Info->Entity)
    {
        rgl_entity_destroy(Info->Entity);
        Info->Entity = nullptr;
    }
    ReleaseMeshReference(Info->StaticMesh);
    EntityMap.Remove(Component);
}

bool FRGLSceneManager::RegisterISMComponent(UInstancedStaticMeshComponent* Component)
{
    if (!IsValid(Component))
    {
        return false;
    }

    UStaticMesh* StaticMesh = Component->GetStaticMesh();
    if (!StaticMesh)
    {
        return false;
    }

    rgl_mesh_t RGLMesh = UploadMesh(StaticMesh);
    if (!RGLMesh)
    {
        return false;
    }
    AddMeshReference(StaticMesh); // Temporary reference until at least one entity owns the mesh.

    const int32 NumInstances = Component->GetInstanceCount();
    if (NumInstances <= 0)
    {
        ReleaseMeshReference(StaticMesh);
        return false;
    }

    FISMCEntityGroup Group;
    Group.Component = Component;
    Group.StaticMesh = StaticMesh;
    Group.OriginalInstanceCount = NumInstances;
    Group.Entities.Reserve(NumInstances);

    int32 SuccessCount = 0;
    int32 RangeSkippedCount = 0;
    for (int32 i = 0; i < NumInstances; ++i)
    {
        FTransform InstanceTransform;
        if (!Component->GetInstanceTransform(i, InstanceTransform, /*bWorldSpace=*/true))
        {
            continue;
        }

        // Skip instances with zero/near-zero scale
        const FVector Scale = InstanceTransform.GetScale3D();
        if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
        {
            continue;
        }

        if (Component->Mobility == EComponentMobility::Static &&
            !IsInstanceWithinAnySensor(Component, StaticMesh, InstanceTransform, 0.0f))
        {
            ++RangeSkippedCount;
            continue;
        }

        rgl_entity_t Entity = nullptr;
        rgl_status_t Status = rgl_entity_create(&Entity, Scene, RGLMesh);
        if (Status != RGL_SUCCESS || !Entity)
        {
            continue;
        }

        rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(InstanceTransform);
        RGL_CHECK(rgl_entity_set_transform(Entity, &RGLTransform));

        Group.Entities.Add(Entity);
        ++SuccessCount;
    }

    if (SuccessCount == 0)
    {
        ReleaseMeshReference(StaticMesh);
        return false;
    }

    Group.MeshRefCount = SuccessCount;
    ISMCEntityMap.Add(Component, MoveTemp(Group));
    AddMeshReference(StaticMesh, SuccessCount);
    ReleaseMeshReference(StaticMesh); // Drop the temporary upload reference.

    RGLLog::Info("RGLSceneManager: Registered ISMC '",
                 TCHAR_TO_UTF8(*Component->GetName()),
                 "' instances=", NumInstances,
                 " entities=", SuccessCount,
                 " range-skipped=", RangeSkippedCount,
                 " mesh='", TCHAR_TO_UTF8(*StaticMesh->GetName()), "'");

    return true;
}

void FRGLSceneManager::UnregisterISMComponent(UInstancedStaticMeshComponent* Component)
{
    FISMCEntityGroup* Group = ISMCEntityMap.Find(Component);
    if (!Group)
    {
        return;
    }

    for (rgl_entity_t Entity : Group->Entities)
    {
        if (Entity)
        {
            rgl_entity_destroy(Entity);
        }
    }
    ReleaseMeshReference(Group->StaticMesh, Group->MeshRefCount);
    ISMCEntityMap.Remove(Component);
}

// ============================================================================
// Transform update
// ============================================================================

void FRGLSceneManager::UpdateTransforms()
{
    TArray<UStaticMeshComponent*> ToRemove;

    for (auto& Pair : EntityMap)
    {
        FEntityInfo& Info = Pair.Value;

        // Use weak pointer to safely detect destroyed components
        if (!Info.Component.IsValid())
        {
            ToRemove.Add(Pair.Key);
            continue;
        }

        UStaticMeshComponent* Comp = Info.Component.Get();
        if (!IsValid(Comp) || !Comp->IsRegistered())
        {
            ToRemove.Add(Pair.Key);
            continue;
        }

        // Optimization: skip static entities after initial transform is set
        if (Info.bIsStatic && Info.bTransformInitialized)
        {
            continue;
        }

        const FTransform WorldTransform = Comp->GetComponentTransform();

        // Skip entities with zero/near-zero scale to avoid NaN in transforms
        const FVector Scale = WorldTransform.GetScale3D();
        if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z))
        {
            continue;
        }

        // Optimization: skip dynamic entities whose transform hasn't changed
        if (Info.bTransformInitialized && WorldTransform.Equals(Info.LastTransform, 0.01f))
        {
            continue;
        }

        rgl_mat3x4f RGLTransform = RGLCoord::ToRGL(WorldTransform);
        RGL_CHECK(rgl_entity_set_transform(Info.Entity, &RGLTransform));

        Info.LastTransform = WorldTransform;
        Info.bTransformInitialized = true;
    }

    for (UStaticMeshComponent* Comp : ToRemove)
    {
        UnregisterComponent(Comp);
    }
}

// ============================================================================
// Periodic world sync: detect new/removed components
// ============================================================================

void FRGLSceneManager::SyncWorldComponents(UWorld* World)
{
    // Evict sensors that stopped refreshing (paused but not destroyed) so their
    // surrounding geometry can be released and stop holding VRAM.
    {
        TArray<const void*> StaleSensors;
        for (const auto& Pair : RegisteredSensors)
        {
            if (CurrentSimTime - Pair.Value.LastUpdateTime > SensorStaleTimeoutSeconds)
            {
                StaleSensors.Add(Pair.Key);
            }
        }
        for (const void* Id : StaleSensors)
        {
            RegisteredSensors.Remove(Id);
        }
    }

    TSet<UStaticMeshComponent*> CurrentComponents;
    TSet<UInstancedStaticMeshComponent*> CurrentISMComponents;
    int32 AddedRegular = 0;
    int32 AddedISM = 0;
    int32 RemovedRegular = 0;
    int32 RemovedISM = 0;
    int32 RangeSkipped = 0;

    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!IsValid(Actor))
        {
            continue;
        }

        TArray<UStaticMeshComponent*> Components;
        Actor->GetComponents<UStaticMeshComponent>(Components);

        for (UStaticMeshComponent* Comp : Components)
        {
            bool bOutOfRange = false;
            if (!ShouldRegisterComponent(Comp, bOutOfRange))
            {
                if (bOutOfRange)
                {
                    ++RangeSkipped;
                }
                continue;
            }

            // Classify: ISMC or regular
            if (UInstancedStaticMeshComponent* ISMComp = Cast<UInstancedStaticMeshComponent>(Comp))
            {
                CurrentISMComponents.Add(ISMComp);
                if (!ISMCEntityMap.Contains(ISMComp))
                {
                    if (RegisterISMComponent(ISMComp))
                    {
                        ++AddedISM;
                    }
                }
            }
            else
            {
                CurrentComponents.Add(Comp);
                if (!EntityMap.Contains(Comp))
                {
                    if (RegisterComponent(Comp))
                    {
                        ++AddedRegular;
                    }
                }
            }
        }
    }

    // Remove regular entities for components that no longer exist or moved out
    // of the LiDAR-relevant window.
    TArray<UStaticMeshComponent*> ToRemove;
    for (auto& Pair : EntityMap)
    {
        // A destroyed component leaves a dangling raw key in this non-UObject map
        // (GC does not null it: e.g. LargeMap tile stream-out, actor destruction).
        // Detect it via the weak pointer and remove without dereferencing the raw
        // key through ShouldKeepRegisteredComponent. Mirrors UpdateTransforms's guard.
        if (!Pair.Value.Component.IsValid())
        {
            ToRemove.Add(Pair.Key);
            continue;
        }
        if (!CurrentComponents.Contains(Pair.Key) && !ShouldKeepRegisteredComponent(Pair.Key))
        {
            ToRemove.Add(Pair.Key);
        }
    }
    for (UStaticMeshComponent* Comp : ToRemove)
    {
        UnregisterComponent(Comp);
        ++RemovedRegular;
    }

    // Remove ISMC entities for components that no longer exist or moved out
    // of the LiDAR-relevant window.
    TArray<UInstancedStaticMeshComponent*> ISMCToRemove;
    for (auto& Pair : ISMCEntityMap)
    {
        // Same dangling-raw-key guard as the regular-entity loop above.
        if (!Pair.Value.Component.IsValid())
        {
            ISMCToRemove.Add(Pair.Key);
            continue;
        }
        if (!CurrentISMComponents.Contains(Pair.Key) && !ShouldKeepRegisteredComponent(Pair.Key))
        {
            ISMCToRemove.Add(Pair.Key);
        }
    }
    for (UInstancedStaticMeshComponent* Comp : ISMCToRemove)
    {
        UnregisterISMComponent(Comp);
        ++RemovedISM;
    }

    // Skeletal components are tracked in a separate map with their own budget /
    // hysteresis rules; a live kill-switch flip tears the whole skeletal path down.
    if (SkeletalEnabled())
    {
        SyncSkeletalComponents(World);
    }
    else if (SkeletalEntityMap.Num() > 0)
    {
        TeardownSkeletal();
    }

    if (AddedRegular || AddedISM || RemovedRegular || RemovedISM)
    {
        RGLLog::Info("RGLSceneManager: Dynamic sync active sensors=", RegisteredSensors.Num(),
                     " regular=", EntityMap.Num(),
                     " ISMC=", ISMCEntityMap.Num(),
                     " cachedMeshes=", MeshCache.Num(),
                     " added=", AddedRegular, "+", AddedISM,
                     " removed=", RemovedRegular, "+", RemovedISM,
                     " range-skipped=", RangeSkipped);
    }
}

// ============================================================================
// Ground plane: a large flat mesh at Z=0 to represent road surface
// ============================================================================

void FRGLSceneManager::CreateGroundPlane()
{
    // Create a 2km × 2km quad centered at the origin, at Z=0.
    // This covers the typical LiDAR range (100m) around any sensor position.
    // The actual ground height varies, but Z=0 is a reasonable approximation
    // for flat urban roads in CARLA.
    static constexpr float HALF_SIZE = 1000.0f; // 1000m = 1km half-extent

    rgl_vec3f Vertices[4] = {
        {{ -HALF_SIZE, -HALF_SIZE, 0.0f }},
        {{  HALF_SIZE, -HALF_SIZE, 0.0f }},
        {{  HALF_SIZE,  HALF_SIZE, 0.0f }},
        {{ -HALF_SIZE,  HALF_SIZE, 0.0f }}
    };

    // Two triangles forming the quad (both face upward)
    rgl_vec3i Indices[2] = {
        {{ 0, 1, 2 }},
        {{ 0, 2, 3 }}
    };

    rgl_status_t Status = rgl_mesh_create(&GroundMesh, Vertices, 4, Indices, 2);
    if (Status != RGL_SUCCESS || !GroundMesh)
    {
        RGLLog::Info("RGLSceneManager: Failed to create ground plane mesh");
        return;
    }

    Status = rgl_entity_create(&GroundEntity, Scene, GroundMesh);
    if (Status != RGL_SUCCESS || !GroundEntity)
    {
        RGLLog::Info("RGLSceneManager: Failed to create ground plane entity");
        rgl_mesh_destroy(GroundMesh);
        GroundMesh = nullptr;
        return;
    }

    // Place at origin with identity rotation (already in meters, Z=0).
    rgl_mat3x4f GroundTf = RGLCoord::Identity();
    RGL_CHECK(rgl_entity_set_transform(GroundEntity, &GroundTf));

    RGLLog::Info("RGLSceneManager: Ground plane created (2km x 2km)");
}

void FRGLSceneManager::UpdateGroundPlane(UWorld* World, const FTransform& SensorTransform)
{
    if (!GroundEntity || !World)
    {
        return;
    }

    // Trace a ray straight down from the sensor to find the actual ground level.
    const FVector SensorPos = SensorTransform.GetLocation();
    const FVector TraceStart = SensorPos;
    const FVector TraceEnd = SensorPos - FVector(0, 0, 10000.0f); // 100m downward

    FHitResult HitResult;
    FCollisionQueryParams TraceParams;
    TraceParams.bTraceComplex = false;

    float GroundZ;
    if (World->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd,
                                          ECC_WorldStatic, TraceParams))
    {
        GroundZ = static_cast<float>(HitResult.ImpactPoint.Z) * RGLCoord::UE_TO_RGL;
    }
    else
    {
        // Fallback: assume ground is 2m below sensor
        GroundZ = static_cast<float>(SensorPos.Z - 200.0f) * RGLCoord::UE_TO_RGL;
    }

    rgl_mat3x4f GroundTf = RGLCoord::Identity();
    GroundTf.value[0][3] = static_cast<float>(SensorPos.X) * RGLCoord::UE_TO_RGL;
    GroundTf.value[1][3] = static_cast<float>(SensorPos.Y) * RGLCoord::UE_TO_RGL;
    GroundTf.value[2][3] = GroundZ;

    RGL_CHECK(rgl_entity_set_transform(GroundEntity, &GroundTf));
}

// ============================================================================
// Skeletal mesh path
// ============================================================================

void FRGLSceneManager::RglDestroyChecked(rgl_status_t Status, const TCHAR* Api)
{
    if (Status != RGL_SUCCESS)
    {
        const char* Err = nullptr; rgl_get_last_error_string(&Err);
        UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] %s failed (status %d): %s"), Api, (int)Status, Err ? *FString(UTF8_TO_TCHAR(Err)) : TEXT("?"));
    }
}

void FRGLSceneManager::UnregisterSensor(const void* SensorId)
{
    RegisteredSensors.Remove(SensorId);
    // Update() only runs from sensor ticks; with no sensor left nothing would restore tick policies or free entities.
    if (RegisteredSensors.Num() == 0 && SkeletalEntityMap.Num() > 0) TeardownSkeletal();
}

bool FRGLSceneManager::SkeletalEnabled() const
{
    bool bApi = bSkeletalApiAvailable;
#if !UE_BUILD_SHIPPING
    if (CVarRGLSkeletalDebugForceApiUnavailable.GetValueOnGameThread() != 0) bApi = false;
#endif
    return bApi && CVarRGLSkeletalEnable.GetValueOnGameThread() != 0;
}

void FRGLSceneManager::WarnSkeletalOnce(const USkeletalMeshComponent* Comp, const USkeletalMesh* Mesh, const FString& Msg)
{
    // (component, asset, reason) identity. FObjectKey carries the object's serial number, so an
    // address recycled by GC starts with a clean slate; the reason hash lets a *different* later
    // failure for the same pair (e.g. permanent skip after "not readable yet") still be logged once.
    const FSkeletalWarnKey Key = MakeTuple(FObjectKey(Comp), FObjectKey(Mesh), GetTypeHash(Msg));
    if (SkeletalWarned.Contains(Key)) return;
    SkeletalWarned.Add(Key);
    UE_LOG(LogCarlaRGL, Warning, TEXT("RGLSceneManager[skeletal] %s (%s / %s)"), *Msg, Comp ? *Comp->GetPathName() : TEXT("?"), Mesh ? *Mesh->GetName() : TEXT("?"));
}

// Non-distance disqualifiers (return false with bOutOfRange=false) are applied without hysteresis.
bool FRGLSceneManager::ShouldRegisterSkeletalComponent(USkeletalMeshComponent* Comp, bool& bOutOfRange, bool& bRetryLater, FString& OutReason) const
{
    bOutOfRange = false; bRetryLater = false;
    if (!IsValid(Comp) || !Comp->IsRegistered() || !Comp->GetSkeletalMeshAsset() || !Comp->IsVisible() || Comp->bRenderStatic) return false;
    const FVector Scale = Comp->GetComponentTransform().GetScale3D();
    if (FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) || FMath::IsNearlyZero(Scale.Z)) return false;
    if (CVarRGLSkeletalScope.GetValueOnGameThread() == 0) { const AActor* O = Comp->GetOwner(); if (!O || !O->IsA<APawn>()) return false; }
    if (Comp->bNoSkeletonUpdate) return false;
    const FBoxSphereBounds B = Comp->Bounds;
    if (RegisteredSensors.Num() == 0 || !IsWithinAnySensor(B.Origin, B.SphereRadius, 0.0f)) { bOutOfRange = true; return false; }
    // Permanently skipped after a non-recoverable extraction/upload failure (spec §6.1);
    // hard disqualifier, so no hysteresis and no repeated extraction attempts.
    if (SkeletalSkipped.Contains(FSkeletalSkipKey(FObjectKey(Comp), FObjectKey(Comp->GetSkeletalMeshAsset())))) return false;
    if (RGLSkeletal::SelectReadableLOD(Comp->GetSkeletalMeshAsset(), OutReason, CVarRGLSkeletalMinLOD.GetValueOnGameThread()) < 0) { bRetryLater = true; return false; }
    return true;
}

rgl_mesh_t FRGLSceneManager::GetOrUploadSkeletalMesh(USkeletalMesh* Mesh, int32& OutLOD, int32& OutRawBoneNum, FString& OutReason)
{
    if (!Mesh) { OutReason = TEXT("null skeletal mesh"); return nullptr; }
    // Resolve the cache key WITHOUT extracting: SelectReadableLOD only inspects buffer
    // sizes, whereas ExtractSkeletalMesh walks every vertex. N pawns sharing one asset
    // must therefore cost one extraction, not N (spec §4.2).
    // MinLOD is part of the selection, and the cache key carries the LOD actually used, so a live
    // MinLOD change simply produces new cache entries when components are (re-)registered.
    const int32 MinLOD = CVarRGLSkeletalMinLOD.GetValueOnGameThread();
    const int32 LOD = RGLSkeletal::SelectReadableLOD(Mesh, OutReason, MinLOD);
    if (LOD < 0) return nullptr;   // §3.1 readiness not met: retry later, OutLOD stays < 0
    // From here on OutLOD >= 0, which tells the caller readiness passed and any later
    // failure is non-recoverable and must be skipped permanently (spec §6.1).
    OutLOD = LOD;
    const FSkeletalMeshKey Key{ Mesh, LOD };
    if (rgl_mesh_t* Cached = SkeletalMeshCache.Find(Key))
    {
        OutRawBoneNum = Mesh->GetRefSkeleton().GetRawBoneNum();
        return *Cached;
    }

    FRGLSkeletalMeshData Data;
    if (!RGLSkeletal::ExtractSkeletalMesh(Mesh, Data, OutReason, MinLOD)) return nullptr;
    // ExtractSkeletalMesh selects the same lowest readable LOD; a divergence would put
    // the entity on a mesh cached under the wrong key.
    if (Data.LODIndex != LOD) { OutReason = FString::Printf(TEXT("LOD mismatch: SelectReadableLOD=%d ExtractSkeletalMesh=%d"), LOD, Data.LODIndex); return nullptr; }
    OutRawBoneNum = Data.RawBoneNum;
    rgl_mesh_t M = nullptr;
    if (rgl_mesh_create(&M, Data.Vertices.GetData(), Data.Vertices.Num(), Data.Indices.GetData(), Data.Indices.Num()) != RGL_SUCCESS || !M) { OutReason = TEXT("rgl_mesh_create failed"); return nullptr; }
    if (rgl_mesh_set_bone_weights(M, Data.Weights.GetData(), Data.Weights.Num()) != RGL_SUCCESS) { RglDestroyChecked(rgl_mesh_destroy(M), TEXT("rgl_mesh_destroy")); OutReason = TEXT("rgl_mesh_set_bone_weights failed"); return nullptr; }
    if (rgl_mesh_set_restposes(M, Data.RestposesInv.GetData(), Data.RestposesInv.Num()) != RGL_SUCCESS) { RglDestroyChecked(rgl_mesh_destroy(M), TEXT("rgl_mesh_destroy")); OutReason = TEXT("rgl_mesh_set_restposes failed"); return nullptr; }
    SkeletalMeshCache.Add(Key, M);
    UE_LOG(LogCarlaRGL, Log, TEXT("RGLSceneManager[skeletal] uploaded %s LOD=%d verts=%d tris=%d rawBones=%d"), *Mesh->GetName(), Data.LODIndex, Data.Vertices.Num(), Data.Indices.Num(), Data.RawBoneNum);
    // Shipping has NO_LOGGING, so the line above vanishes there. One stderr breadcrumb per process
    // is enough to tell an operator that the skeletal path is actually live in a packaged server.
    static bool bFirstSkeletalUploadLogged = false;
    if (!bFirstSkeletalUploadLogged)
    {
        bFirstSkeletalUploadLogged = true;
        fprintf(stderr, "CarlaRGL[skeletal]: first skeletal mesh uploaded (%s LOD=%d verts=%d bones=%d); skeletal path active\n",
                TCHAR_TO_UTF8(*Mesh->GetName()), Data.LODIndex, Data.Vertices.Num(), Data.RawBoneNum);
    }
    return M;
}

void FRGLSceneManager::ReleaseSkeletalMeshReference(const FSkeletalMeshKey& Key)
{
    int32* Count = SkeletalMeshRefCounts.Find(Key); if (!Count) return;
    if (--(*Count) <= 0)
    {
        if (rgl_mesh_t* M = SkeletalMeshCache.Find(Key)) { if (*M) RglDestroyChecked(rgl_mesh_destroy(*M), TEXT("rgl_mesh_destroy")); SkeletalMeshCache.Remove(Key); }
        SkeletalMeshRefCounts.Remove(Key);
    }
}

void FRGLSceneManager::ApplyTickPolicy(USkeletalMeshComponent* Comp, FSkeletalEntityInfo& Info, bool bEnable)
{
    if (!IsValid(Comp)) return;
    if (bEnable && !Info.bTickPolicyApplied)
    {
        Info.SavedTickOption = Comp->VisibilityBasedAnimTickOption; Info.bSavedURO = Comp->bEnableUpdateRateOptimizations;
        Comp->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones;
        Comp->bEnableUpdateRateOptimizations = false; Info.bTickPolicyApplied = true;
    }
    else if (!bEnable && Info.bTickPolicyApplied)
    {
        Comp->VisibilityBasedAnimTickOption = Info.SavedTickOption; Comp->bEnableUpdateRateOptimizations = Info.bSavedURO;
        Info.bTickPolicyApplied = false;
    }
}

bool FRGLSceneManager::RegisterSkeletalComponent(USkeletalMeshComponent* Comp)
{
    if (!IsValid(Comp) || SkeletalEntityMap.Contains(Comp)) return false;
    USkeletalMesh* Mesh = Comp->GetSkeletalMeshAsset();
    FString Reason; int32 LOD = -1, RawBoneNum = 0;
    rgl_mesh_t M = GetOrUploadSkeletalMesh(Mesh, LOD, RawBoneNum, Reason);
    if (!M)
    {
        // LOD >= 0 means readiness passed and extraction/upload itself failed: not a
        // §3.1 condition, so never retry this (component, asset) pair (spec §6.1).
        // Readiness failures cannot reach here (ShouldRegisterSkeletalComponent returns bRetryLater).
        if (LOD >= 0)
        {
            SkeletalSkipped.Add(FSkeletalSkipKey(FObjectKey(Comp), FObjectKey(Mesh)));
            // Shipping has no log: a permanent skip silently removes an actor from the point cloud,
            // so leave a stderr breadcrumb (LOD >= 0 implies Mesh != nullptr; Comp is IsValid here).
            fprintf(stderr, "CarlaRGL[skeletal]: permanently skipping %s (%s): %s\n",
                    TCHAR_TO_UTF8(*Comp->GetPathName()), TCHAR_TO_UTF8(*Mesh->GetName()), TCHAR_TO_UTF8(*Reason));
        }
        WarnSkeletalOnce(Comp, Mesh, TEXT("skipped: ") + Reason);
        return false;
    }
    FSkeletalEntityInfo Info;
    Info.Component = Comp; Info.SkeletalMesh = Mesh; Info.LODIndex = LOD; Info.RawBoneNum = RawBoneNum;
    Info.PoseScratch.SetNumUninitialized(RawBoneNum);
    ApplyTickPolicy(Comp, Info, CVarRGLSkeletalAlwaysTickPose.GetValueOnGameThread() != 0);
    SkeletalMeshRefCounts.FindOrAdd(FSkeletalMeshKey{ Mesh, LOD })++;
    SkeletalEntityMap.Add(Comp, MoveTemp(Info));   // Entity stays nullptr until the first valid pose (spec §4.3/§4.5)
    return true;
}

void FRGLSceneManager::UnregisterSkeletalComponent(USkeletalMeshComponent* Key)
{
    FSkeletalEntityInfo* Info = SkeletalEntityMap.Find(Key); if (!Info) return;
    if (Info->Entity) { RglDestroyChecked(rgl_entity_destroy(Info->Entity), TEXT("rgl_entity_destroy")); Info->Entity = nullptr; }
    ReleaseSkeletalMeshReference(FSkeletalMeshKey{ Info->SkeletalMesh, Info->LODIndex });
    if (USkeletalMeshComponent* Live = Info->Component.Get()) ApplyTickPolicy(Live, *Info, false);   // never through the raw key
    SkeletalEntityMap.Remove(Key);
}

void FRGLSceneManager::TeardownSkeletal()
{
    TArray<USkeletalMeshComponent*> Keys; SkeletalEntityMap.GetKeys(Keys);
    for (USkeletalMeshComponent* K : Keys) UnregisterSkeletalComponent(K);
    for (auto& P : SkeletalMeshCache) if (P.Value) RglDestroyChecked(rgl_mesh_destroy(P.Value), TEXT("rgl_mesh_destroy"));
    SkeletalMeshCache.Empty(); SkeletalMeshRefCounts.Empty();
    SkeletalSkipped.Empty(); SkeletalWarned.Empty();
}

void FRGLSceneManager::UpdateSkeletalPoses()
{
    TArray<USkeletalMeshComponent*> ToRemove;
    for (auto& Pair : SkeletalEntityMap)
    {
        FSkeletalEntityInfo& Info = Pair.Value;
        USkeletalMeshComponent* Comp = Info.Component.Get();
        if (!Comp || !IsValid(Comp) || !Comp->IsRegistered()) { ToRemove.Add(Pair.Key); continue; }
        if (Comp->GetSkeletalMeshAsset() != Info.SkeletalMesh) { ToRemove.Add(Pair.Key); continue; }   // swapped: re-register at next sync
        bool bPose = RGLSkeletal::BuildWorldPose(Comp, Info.RawBoneNum, Info.PoseScratch);
#if !UE_BUILD_SHIPPING
        if (CVarRGLSkeletalDebugForcePoseFailure.GetValueOnGameThread() != 0) bPose = false;
#endif
        if (!bPose)
        {
            if (Info.Entity) { RglDestroyChecked(rgl_entity_destroy(Info.Entity), TEXT("rgl_entity_destroy")); Info.Entity = nullptr; }
            WarnSkeletalOnce(Comp, Info.SkeletalMesh, TEXT("no complete pose this tick; pending"));
            continue;
        }
        if (!Info.Entity)
        {
            rgl_mesh_t* M = SkeletalMeshCache.Find(FSkeletalMeshKey{ Info.SkeletalMesh, Info.LODIndex });
            if (!M || !*M) { ToRemove.Add(Pair.Key); continue; }
            rgl_entity_t E = nullptr;
            if (rgl_entity_create(&E, Scene, *M) != RGL_SUCCESS || !E) { WarnSkeletalOnce(Comp, Info.SkeletalMesh, TEXT("rgl_entity_create failed; pending")); continue; }
            Info.Entity = E;   // identity transform; world-space poses follow. Never call rgl_entity_set_transform.
        }
        if (rgl_entity_set_pose_world(Info.Entity, Info.PoseScratch.GetData(), Info.RawBoneNum) != RGL_SUCCESS)
        {
            RglDestroyChecked(rgl_entity_destroy(Info.Entity), TEXT("rgl_entity_destroy")); Info.Entity = nullptr;
            WarnSkeletalOnce(Comp, Info.SkeletalMesh, TEXT("rgl_entity_set_pose_world failed; pending"));
        }
    }
    for (USkeletalMeshComponent* K : ToRemove) UnregisterSkeletalComponent(K);
}

void FRGLSceneManager::SyncSkeletalComponents(UWorld* World)
{
    const bool bTickPolicy = CVarRGLSkeletalAlwaysTickPose.GetValueOnGameThread() != 0;
    const int32 MaxEntities = FMath::Max(0, CVarRGLSkeletalMaxEntities.GetValueOnGameThread());
    for (auto& Pair : SkeletalEntityMap) if (USkeletalMeshComponent* Live = Pair.Value.Component.Get()) ApplyTickPolicy(Live, Pair.Value, bTickPolicy);

    // 1) Evaluate every skeletal component: eligible set (with distance), hard-disqualified set, retry set.
    struct FCand { USkeletalMeshComponent* Comp; float DistSq; };
    TArray<FCand> Eligible; TSet<USkeletalMeshComponent*> Disqualified; int32 RangeSkipped = 0, RetryLater = 0;
    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It; if (!IsValid(Actor)) continue;
        TArray<USkeletalMeshComponent*> Comps; Actor->GetComponents<USkeletalMeshComponent>(Comps);
        for (USkeletalMeshComponent* C : Comps)
        {
            bool bOut = false, bRetry = false; FString Reason;
            if (!ShouldRegisterSkeletalComponent(C, bOut, bRetry, Reason))
            {
                if (bRetry) { ++RetryLater; WarnSkeletalOnce(C, C->GetSkeletalMeshAsset(), TEXT("not readable yet, will retry: ") + Reason); }
                else if (bOut) ++RangeSkipped;
                else Disqualified.Add(C);          // scope/visibility/scale/bNoSkeletonUpdate/bRenderStatic: no hysteresis
                continue;
            }
            float Best = TNumericLimits<float>::Max();
            for (const auto& S : RegisteredSensors) Best = FMath::Min(Best, (float)FVector::DistSquared(S.Value.Position, C->Bounds.Origin));
            Eligible.Add({ C, Best });
        }
    }
    // 2) Unregister: dead, hard-disqualified, or out of range beyond hysteresis. Registrations that
    //    survive this step but are NOT eligible ("retained" by hysteresis) still occupy VRAM, so
    //    they are ranked alongside the eligible candidates in step 3 (spec §4.2-5).
    TSet<USkeletalMeshComponent*> EligibleSet; for (const FCand& C : Eligible) EligibleSet.Add(C.Comp);
    TArray<USkeletalMeshComponent*> ToRemove; TArray<FCand> Retained;
    for (auto& Pair : SkeletalEntityMap)
    {
        USkeletalMeshComponent* Live = Pair.Value.Component.Get();
        if (!Live || !IsValid(Live) || Disqualified.Contains(Pair.Key)) { ToRemove.Add(Pair.Key); continue; }
        if (EligibleSet.Contains(Pair.Key)) continue;   // already ranked as an eligible candidate
        const FBoxSphereBounds B = Live->Bounds;
        if (!IsWithinAnySensor(B.Origin, B.SphereRadius, UnregistrationHysteresisCm)) { ToRemove.Add(Pair.Key); continue; }
        float Best = TNumericLimits<float>::Max();
        for (const auto& S : RegisteredSensors) Best = FMath::Min(Best, (float)FVector::DistSquared(S.Value.Position, B.Origin));
        Retained.Add({ Pair.Key, Best });
    }
    for (USkeletalMeshComponent* K : ToRemove) UnregisterSkeletalComponent(K);
    // 3) Budget over the UNION of eligible candidates and hysteresis-retained registrations,
    //    nearest first. Everything past MaxEntities is unregistered FIRST (so MaxEntities=0 really
    //    empties the set), then the in-budget newcomers are registered.
    TArray<FCand> Ranked = MoveTemp(Eligible);
    Ranked.Append(Retained);
    Ranked.Sort([](const FCand& A, const FCand& B) { return A.DistSq < B.DistSq; });
    TArray<USkeletalMeshComponent*> OverBudget;
    for (int32 i = MaxEntities; i < Ranked.Num(); ++i)
        if (SkeletalEntityMap.Contains(Ranked[i].Comp)) OverBudget.Add(Ranked[i].Comp);
    for (USkeletalMeshComponent* K : OverBudget) UnregisterSkeletalComponent(K);
    int32 Added = 0;
    for (int32 i = 0, N = FMath::Min(MaxEntities, Ranked.Num()); i < N; ++i)
    {
        USkeletalMeshComponent* C = Ranked[i].Comp;
        if (!SkeletalEntityMap.Contains(C) && RegisterSkeletalComponent(C)) ++Added;
    }
    if (Added || ToRemove.Num() || OverBudget.Num())
        RGLLog::Info("RGLSceneManager[skeletal]: registered=", SkeletalEntityMap.Num(), " cachedMeshes=", SkeletalMeshCache.Num(),
                     " added=", Added, " removed=", ToRemove.Num() + OverBudget.Num(), " range-skipped=", RangeSkipped, " retry-later=", RetryLater);
}

#if !UE_BUILD_SHIPPING
void FRGLSceneManager::SyncSkeletalNow_ForTest(UWorld* World, double SimTime)
{
    CurrentSimTime = SimTime;
    if (SkeletalEnabled()) SyncSkeletalComponents(World); else if (SkeletalEntityMap.Num() > 0) TeardownSkeletal();
}
void FRGLSceneManager::UpdateSkeletalPosesNow_ForTest(double SimTime)
{
    CurrentSimTime = SimTime;
    const uint64 TimeNs = static_cast<uint64>(SimTime * 1e9);
    RGL_CHECK(rgl_scene_set_time(Scene, TimeNs));      // mirrors Update(): time before poses
    if (SkeletalEnabled()) UpdateSkeletalPoses();
}
int32 FRGLSceneManager::GetSkeletalLiveEntityCount_ForTest() const { int32 N = 0; for (const auto& P : SkeletalEntityMap) N += (P.Value.Entity != nullptr); return N; }
#endif

#endif // WITH_RGL
