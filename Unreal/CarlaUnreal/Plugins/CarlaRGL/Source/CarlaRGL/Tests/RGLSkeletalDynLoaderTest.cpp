#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLDynLoader.h"

namespace RGLSkeletalTestsDynLoader
{
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FApiAvailableTest,
    "CarlaRGL.Skeletal.DynLoader.ApiAvailable",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FApiAvailableTest::RunTest(const FString& Parameters)
{
    // StartupModule dlopens libRobotecGPULidar.so in editor builds too (CarlaRGLModule.cpp:53-66).
    TestTrue(TEXT("library loaded"), RGLDynLoader::IsLoaded());
    TestTrue(TEXT("skeletal API resolved"), RGLDynLoader::IsSkeletalApiAvailable());
    return true;
}
}
#endif
