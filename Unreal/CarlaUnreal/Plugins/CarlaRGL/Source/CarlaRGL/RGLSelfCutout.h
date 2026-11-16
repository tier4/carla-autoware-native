#pragma once
// Lidar self cutout (rgl_self_cutout_radius): pure geometry helpers. No RGL calls, no scene state.
//
// A lidar mounted inside a housing that the vehicle mesh models hits that housing with most of its
// rays. For such a vehicle the scene manager registers only "the rest" of its skeletal mesh: the
// triangles near any of its cutout lidars (closest point within the radius AND touching that lidar's
// vertical field of view) are left out of the RGL scene, so no lidar in the scene sees them (not even
// lidars on other vehicles). The radius should stay about the size of the housing. The helpers below
// decide which triangles are near.
#ifdef WITH_RGL
#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>
#include <rgl/api/core.h>

#include "RGLSkeletalMeshExtractor.h"   // FRGLSkeletalMeshData

namespace RGLSelfCutout
{
    /// Triangle owner codes of ClassifyTriangles; values >= 0 are indexes into the sensor array.
    constexpr int32 OwnerRest = -1;     // not near any sensor: kept in the scene
    constexpr int32 OwnerShared = -2;   // near several sensors (removed like any near triangle, counted apart)

    /// Degrees added on both sides of a sensor's vertical field of view (angular noise, divergence).
    constexpr float ElevationMarginDeg = 2.0f;

    /// One sensor in the frame of the vertices (meters).
    struct FSensor
    {
        FVector OriginM = FVector::ZeroVector;
        FQuat   Rotation = FQuat::Identity;      // sensor local -> vertex frame
        double  RadiusM = 0.0;
        double  MinElevationDeg = 0.0;           // sensor frame; margin already included
        double  MaxElevationDeg = 0.0;
    };

    /// Closest point to P on triangle ABC (Ericson, Real-Time Collision Detection 5.1.5).
    FVector ClosestPointOnTriangle(const FVector& P, const FVector& A, const FVector& B, const FVector& C);

    /// Elevation (degrees) of a point in the sensor frame: atan2(z, sqrt(x^2 + y^2)).
    double ElevationDeg(const FVector& Local);

    /// Whether triangle ABC, given in the sensor frame, has a point whose elevation lies in
    /// [LoDeg, HiDeg]. Exact when LoDeg <= 0 <= HiDeg; conservative (may answer true for a
    /// triangle just outside the band) otherwise.
    bool TouchesElevationBand(const FVector& A, const FVector& B, const FVector& C, double LoDeg, double HiDeg);

    /// Near = closest point closer than the radius AND touching the vertical field of view.
    bool IsNear(const FVector& A, const FVector& B, const FVector& C, const FSensor& S);

    /// OutOwner[t] = OwnerRest, OwnerShared or the index of the only sensor triangle t is near.
    /// Triangles with an out-of-range vertex index stay OwnerRest.
    void ClassifyTriangles(const TArray<FVector>& VerticesM, const TArray<rgl_vec3i>& Indices,
                           const TArray<FSensor>& Sensors, TArray<int32>& OutOwner);

    /// CPU replay of RGL's linear blend skinning: world-space vertex positions (meters) of Mesh
    /// under WorldPose (one rgl_entity_set_pose_world matrix per raw bone). False if the inputs
    /// do not fit together.
    bool SkinVertices(const FRGLSkeletalMeshData& Mesh, const TArray<rgl_mat3x4f>& WorldPose, TArray<FVector>& OutVerticesM);

    /// Compacted copy of the triangles of Mesh whose owner is WantedOwner, with their vertices and
    /// bone weights (restposes are shared and stay those of Mesh). Returns the triangle count.
    int32 ExtractPart(const FRGLSkeletalMeshData& Mesh, const TArray<int32>& Owner, int32 WantedOwner,
                      TArray<rgl_vec3f>& OutVertices, TArray<rgl_vec3i>& OutIndices, TArray<rgl_bone_weights_t>& OutWeights);

    /// One cutout sensor as seen by one vehicle; the kept mesh is rebuilt when this changes.
    struct FSensorConfig
    {
        UPTRINT    Key = 0;                  // scene manager's sensor key (stable order)
        float      RadiusCm = 0.0f;
        float      MinElevationDeg = 0.0f;   // vertical field of view, without margin
        float      MaxElevationDeg = 0.0f;
        FTransform SensorToOwner = FTransform::Identity;   // sensor pose relative to the vehicle actor (cm)
    };

    /// Sort by key so that the comparison does not depend on map iteration order.
    void SortConfig(TArray<FSensorConfig>& Config);

    /// Same sensors with the same settings; poses equal within 1 cm and 0.1 degree, so float
    /// jitter of an attached sensor does not trigger a rebuild.
    bool SameConfig(const TArray<FSensorConfig>& A, const TArray<FSensorConfig>& B);
}
#endif // WITH_RGL
