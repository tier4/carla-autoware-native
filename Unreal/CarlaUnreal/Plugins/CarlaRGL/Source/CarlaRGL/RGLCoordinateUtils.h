// Copyright (c) 2026 RGL Integration for CARLA.
// Coordinate conversion utilities between UE5 and RGL.
//
// UE5 FMatrix convention:
//   - Row-major storage: M.M[Row][Col]
//   - Row-vector multiplication: V' = V * M
//   - Layout: Row0=X-axis(Forward), Row1=Y-axis(Right), Row2=Z-axis(Up), Row3=Translation
//   - M[0..2][3] = 0, M[3][3] = 1
//   - Units: centimeters
//
// RGL rgl_mat3x4f convention:
//   - Row-major storage: value[Row][Col]
//   - Column-vector multiplication: V' = M * V
//   - Layout: Columns 0..2 = rotation axes, Column 3 = translation
//   - Ray direction = Column 2 (local Z-axis of the ray pose)
//   - Units: meters
//
// Conversion: M_rgl = transpose(M_ue5_3x3) for rotation,
//             translation from UE5 Row 3 to RGL Column 3, scaled cm->m.

#pragma once

// Log helper: writes directly to std::cout.
// Uses "INFO_RGL:" prefix to distinguish from other log systems.
// In Shipping builds, Info() is a no-op to avoid debug overhead.
#include <iostream>

namespace RGLLog
{
#if UE_BUILD_SHIPPING
    template <typename... Args>
    inline void Info(Args&&...) {}  // No-op in Shipping
#else
    template <typename... Args>
    inline void Info(Args&&... args)
    {
        std::cout << "INFO_RGL:";
        ((std::cout << ' ' << std::forward<Args>(args)), ...);
        std::cout << std::endl;
    }
#endif
} // namespace RGLLog

#ifdef WITH_RGL

#include "CarlaRGLModule.h"

#include <util/disable-ue4-macros.h>
#include <rgl/api/core.h>
#include <util/enable-ue4-macros.h>

// Shared helper macro: check RGL return status and log on error.
// Guarded to avoid redefinition in unity builds.
#ifndef RGL_CHECK
#define RGL_CHECK(call)                                                       \
    do {                                                                      \
        rgl_status_t _s = (call);                                             \
        if (_s != RGL_SUCCESS) {                                              \
            const char* _err = nullptr;                                       \
            rgl_get_last_error_string(&_err);                                 \
            UE_LOG(LogCarlaRGL, Error,                                           \
                   TEXT("RGL error in %s: %s"),                               \
                   TEXT(#call),                                               \
                   _err ? *FString(UTF8_TO_TCHAR(_err)) : TEXT("unknown"));   \
        }                                                                     \
    } while (0)
#endif

#include <util/ue-header-guard-begin.h>
#include "Math/Vector.h"
#include "Math/Rotator.h"
#include "Math/Transform.h"
#include "Math/Matrix.h"
#include "Math/UnrealMathUtility.h"
#include <util/ue-header-guard-end.h>

namespace RGLCoord
{
    // UE5 centimeters -> meters
    constexpr float UE_TO_RGL = 0.01f;
    // meters -> UE5 centimeters
    constexpr float RGL_TO_UE = 100.0f;

    // Convert UE5 FTransform to RGL 3x4 matrix.
    // Transposes the 3x3 rotation (row-vector → column-vector convention)
    // and extracts translation from UE5's Row 3, scaled cm → m.
    inline rgl_mat3x4f ToRGL(const FTransform& Transform)
    {
        const FMatrix M = Transform.ToMatrixWithScale();
        rgl_mat3x4f Out;
        // Transpose the 3x3 rotation part:
        //   RGL column j = UE5 row j (axis j)
        // Translation: UE5 M[3][0..2] → RGL Out[0..2][3]
        for (int Row = 0; Row < 3; ++Row)
        {
            Out.value[Row][0] = static_cast<float>(M.M[0][Row]); // Column 0 = UE5 X-axis (Forward)
            Out.value[Row][1] = static_cast<float>(M.M[1][Row]); // Column 1 = UE5 Y-axis (Right)
            Out.value[Row][2] = static_cast<float>(M.M[2][Row]); // Column 2 = UE5 Z-axis (Up)
            Out.value[Row][3] = static_cast<float>(M.M[3][Row]) * UE_TO_RGL; // Translation
        }
        return Out;
    }

    // Build a 3x4 identity matrix.
    inline rgl_mat3x4f Identity()
    {
        rgl_mat3x4f Out = {};
        Out.value[0][0] = 1.0f;
        Out.value[1][1] = 1.0f;
        Out.value[2][2] = 1.0f;
        return Out;
    }

    // Build a ray-pose matrix for RGL from pitch/yaw angles (degrees).
    //
    // RGL uses Column 2 of the ray matrix as the ray direction.
    // This function constructs an orthonormal basis with the desired
    // ray direction as Column 2.
    //
    // The ray direction in UE5 coordinates:
    //   Dir = (cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), sin(pitch))
    inline rgl_mat3x4f FromPitchYaw(float PitchDeg, float YawDeg)
    {
        const float P = FMath::DegreesToRadians(PitchDeg);
        const float Y = FMath::DegreesToRadians(YawDeg);

        // Desired ray direction in UE5 coordinates (X=forward, Y=right, Z=up)
        FVector Dir(
            FMath::Cos(P) * FMath::Cos(Y),
            FMath::Cos(P) * FMath::Sin(Y),
            FMath::Sin(P));
        Dir.Normalize();

        // Build orthonormal basis with Dir as Column 2 (RGL ray direction)
        FVector WorldUp(0.0f, 0.0f, 1.0f);
        FVector Right = FVector::CrossProduct(WorldUp, Dir);
        if (Right.IsNearlyZero())
        {
            // Dir is nearly vertical; use a different reference
            Right = FVector(0.0f, 1.0f, 0.0f);
        }
        Right.Normalize();
        FVector Up = FVector::CrossProduct(Dir, Right);
        Up.Normalize();

        // RGL matrix (column-vector convention):
        //   Column 0 = Right, Column 1 = Up, Column 2 = Dir (ray direction)
        rgl_mat3x4f Out;
        Out.value[0][0] = static_cast<float>(Right.X);
        Out.value[0][1] = static_cast<float>(Up.X);
        Out.value[0][2] = static_cast<float>(Dir.X);
        Out.value[0][3] = 0.0f;

        Out.value[1][0] = static_cast<float>(Right.Y);
        Out.value[1][1] = static_cast<float>(Up.Y);
        Out.value[1][2] = static_cast<float>(Dir.Y);
        Out.value[1][3] = 0.0f;

        Out.value[2][0] = static_cast<float>(Right.Z);
        Out.value[2][1] = static_cast<float>(Up.Z);
        Out.value[2][2] = static_cast<float>(Dir.Z);
        Out.value[2][3] = 0.0f;

        return Out;
    }

    // Build a 3x4 translation-only matrix (position in meters).
    inline rgl_mat3x4f FromTranslation(float X, float Y, float Z)
    {
        rgl_mat3x4f Out = Identity();
        Out.value[0][3] = X;
        Out.value[1][3] = Y;
        Out.value[2][3] = Z;
        return Out;
    }

    // Convert UE5 FVector (cm) to rgl_vec3f (m).
    inline rgl_vec3f ToRGLVec(const FVector& V)
    {
        return { {
            static_cast<float>(V.X) * UE_TO_RGL,
            static_cast<float>(V.Y) * UE_TO_RGL,
            static_cast<float>(V.Z) * UE_TO_RGL
        } };
    }

    // Convert rgl_vec3f (m) to UE5 FVector (cm).
    inline FVector ToUEVec(const rgl_vec3f& V)
    {
        return FVector(
            V.value[0] * RGL_TO_UE,
            V.value[1] * RGL_TO_UE,
            V.value[2] * RGL_TO_UE
        );
    }

    // Compute the inverse of a sensor transform for converting world->local.
    inline rgl_mat3x4f InverseTransform(const FTransform& Transform)
    {
        const FTransform Inv = Transform.Inverse();
        return ToRGL(Inv);
    }

} // namespace RGLCoord

#endif // WITH_RGL
