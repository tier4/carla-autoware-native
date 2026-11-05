#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLCoordinateUtils.h"

namespace RGLSkeletalTestsMath
{
static bool NearlyEqualMat(const rgl_mat3x4f& A, const rgl_mat3x4f& B, float Tol, FString& Why)
{
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 4; ++c)
        if (!FMath::IsNearlyEqual(A.value[r][c], B.value[r][c], Tol))
        { Why = FString::Printf(TEXT("[%d][%d] %f != %f"), r, c, A.value[r][c], B.value[r][c]); return false; }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FToRGLMatMatchesToRGLTest, "CarlaRGL.Skeletal.Math.ToRGLMatMatchesToRGL",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FToRGLMatMatchesToRGLTest::RunTest(const FString& Parameters)
{
    const FTransform T(FRotator(12.f, -37.f, 71.f), FVector(123.f, -456.f, 78.9f), FVector(1.5f, 0.5f, 2.f));
    FString Why;
    TestTrue(FString::Printf(TEXT("ToRGLMat == ToRGL (%s)"), *Why),
             NearlyEqualMat(RGLCoord::ToRGL(T), RGLCoord::ToRGLMat(T.ToMatrixWithScale()), 1e-5f, Why));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FToRGLMatContravariantTest, "CarlaRGL.Skeletal.Math.ToRGLMatIsContravariant",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FToRGLMatContravariantTest::RunTest(const FString& Parameters)
{
    // UE row-vector composition Child*Parent maps to RGL Phi(Parent).Phi(Child)  (spec §5)
    const FMatrix A = FTransform(FRotator(5.f, 10.f, 15.f), FVector(10.f, 20.f, 30.f)).ToMatrixWithScale();
    const FMatrix B = FTransform(FRotator(-20.f, 40.f, 3.f), FVector(-7.f, 8.f, 900.f), FVector(2.f)).ToMatrixWithScale();
    FString Why;
    TestTrue(FString::Printf(TEXT("Phi(A*B) == Phi(B).Phi(A) (%s)"), *Why),
             NearlyEqualMat(RGLCoord::ToRGLMat(A * B), RGLCoord::MulRGL(RGLCoord::ToRGLMat(B), RGLCoord::ToRGLMat(A)), 1e-3f, Why));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FToRGLMatShearTest, "CarlaRGL.Skeletal.Math.ToRGLMatKeepsShear",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FToRGLMatShearTest::RunTest(const FString& Parameters)
{
    FMatrix S = FMatrix::Identity;
    S.M[0][1] = 0.25f;   // shear: UE row 0 (X axis) has a Y component
    S.M[3][2] = 100.f;   // translation Z = 100 cm
    const rgl_mat3x4f R = RGLCoord::ToRGLMat(S);
    TestEqual(TEXT("shear transposed to [1][0]"), R.value[1][0], 0.25f);
    TestEqual(TEXT("translation scaled to m"), R.value[2][3], 1.0f);
    return true;
}
}
#endif
