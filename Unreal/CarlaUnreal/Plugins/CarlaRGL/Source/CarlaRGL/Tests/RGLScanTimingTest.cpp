#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLScanTiming.h"

// Pure-function tests of the rgl_scan_timing attribute: no world, no GPU.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRGLScanTimingUdpOptionsTest, "CarlaRGL.ScanTiming.UdpOptions",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FRGLScanTimingUdpOptionsTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("same-timestamp bit is RGL's 1 << 4"),
        static_cast<uint32>(RGL_UDP_SAME_TIMESTAMP_FOR_PACKETS), 16u);
    TestEqual(TEXT("firing adds no option"),
        RGLScanTiming::UdpOptions(TEXT("firing")), static_cast<uint32>(RGL_UDP_NO_ADDITIONAL_OPTIONS));
    TestEqual(TEXT("instant stamps every packet with the capture time"),
        RGLScanTiming::UdpOptions(TEXT("instant")), static_cast<uint32>(RGL_UDP_SAME_TIMESTAMP_FOR_PACKETS));
    TestTrue(TEXT("firing is a value"), RGLScanTiming::IsValid(TEXT("firing")));
    TestTrue(TEXT("instant is a value"), RGLScanTiming::IsValid(TEXT("instant")));
    TestFalse(TEXT("an unknown value is not"), RGLScanTiming::IsValid(TEXT("bogus")));
    TestEqual(TEXT("an unknown value adds no option"),
        RGLScanTiming::UdpOptions(TEXT("bogus")), static_cast<uint32>(RGL_UDP_NO_ADDITIONAL_OPTIONS));
    return true;
}
#endif
