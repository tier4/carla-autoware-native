#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include <util/ue-header-guard-end.h>

#if WITH_DEV_AUTOMATION_TESTS && defined(WITH_RGL)
#include "../RGLScanTiming.h"
#include "../RGLVelocityDistortion.h"
#include "../RGLVehicleIgnore.h"

#include <cstring>

// Pure-function tests of the rgl_scan_timing attribute: no world, no GPU.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRGLScanTimingUdpOptionsTest, "CarlaRGL.ScanTiming.UdpOptions",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FRGLScanTimingUdpOptionsTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("firing adds no option"),
        RGLScanTiming::UdpOptions(TEXT("firing")), static_cast<uint32>(RGL_UDP_NO_ADDITIONAL_OPTIONS));
    TestEqual(TEXT("velocity_distortion retains firing packet times"),
        RGLScanTiming::UdpOptions(TEXT("velocity_distortion")), static_cast<uint32>(RGL_UDP_NO_ADDITIONAL_OPTIONS));
    TestTrue(TEXT("firing is a value"), RGLScanTiming::IsValid(TEXT("firing")));
    TestTrue(TEXT("velocity_distortion is a value"), RGLScanTiming::IsValid(TEXT("velocity_distortion")));
    TestFalse(TEXT("the removed value is rejected"), RGLScanTiming::IsValid(TEXT("instant")));
    TestFalse(TEXT("an unknown value is not"), RGLScanTiming::IsValid(TEXT("bogus")));
    TestEqual(TEXT("an unknown value adds no option"),
        RGLScanTiming::UdpOptions(TEXT("bogus")), static_cast<uint32>(RGL_UDP_NO_ADDITIONAL_OPTIONS));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRGLScanTimingVelocityTest, "CarlaRGL.ScanTiming.VelocityDistortion",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FRGLScanTimingVelocityTest::RunTest(const FString& Parameters)
{
    const auto Offsets = RGLVelocityDistortion::TimeOffsetsMs(1800, 2, 360, 10);
    if (!TestEqual(TEXT("one offset per ray"), static_cast<int32>(Offsets.size()), 3600)) return false;
    TestTrue(TEXT("first ray is one rotation in the past"), FMath::IsNearlyEqual(Offsets.front(), -100.0f));
    TestTrue(TEXT("channels in a step share the time"), Offsets[0] == Offsets[1]);
    TestTrue(TEXT("last ray precedes capture"), FMath::IsNearlyEqual(Offsets.back(), -100.0f / 1800, 1e-5f));

    const float Previous[3][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}};
    const float Current[3][4] = {{1, 0, 0, 1}, {0, 1, 0, 0}, {0, 0, 1, 0}};
    const auto Velocity = RGLVelocityDistortion::EstimateVelocity(Current, Previous, 0.1, 0.1);
    TestTrue(TEXT("history is valid"), Velocity.State == RGLVelocityDistortion::Status::Valid);
    TestTrue(TEXT("one metre in 0.1 seconds is ten m/s"), FMath::IsNearlyEqual(Velocity.Linear[0], 10.0f));
    TestTrue(TEXT("no yaw motion"), FMath::IsNearlyZero(Velocity.Angular[2]));
    TestTrue(TEXT("standard Pandar128 UDP step timing matches"),
        RGLVelocityDistortion::ValidateOutput(true, false, "HesaiPandar128E4X", 1800, 10, 360) == nullptr);
    TestTrue(TEXT("half the required UDP steps are rejected"),
        RGLVelocityDistortion::ValidateOutput(true, false, "HesaiPandar128E4X", 900, 10, 360) != nullptr);
    // Rejection reasons go to stdout as key=value tokens; the sentence is for the engine log.
    const char* PartialSweep = RGLVelocityDistortion::ValidateOutput(true, false, "HesaiPandar128E4X", 900, 10, 180);
    TestTrue(TEXT("a partial sweep is rejected with a token"),
        PartialSweep && std::strcmp(PartialSweep, "partial_sweep") == 0);
    const char* DirectRos2 = RGLVelocityDistortion::ValidateOutput(false, true, "HesaiPandar128E4X", 1800, 10, 360);
    TestTrue(TEXT("direct ROS 2 output is rejected with a token"),
        DirectRos2 && std::strcmp(DirectRos2, "direct_ros2_output_unsupported") == 0);
    TestTrue(TEXT("the engine log gets a sentence"),
        std::strchr(RGLVelocityDistortion::Describe("partial_sweep"), ' ') != nullptr);

    const float Scaled[3][4] = {{2, 0, 0, 1}, {0, 2, 0, 0}, {0, 0, 2, 0}};
    const auto Invalid = RGLVelocityDistortion::EstimateVelocity(Scaled, Previous, 0.1, 0.1);
    TestTrue(TEXT("scale two is invalid"), Invalid.State == RGLVelocityDistortion::Status::InvalidInput);
    TestTrue(TEXT("invalid pose has zero speed"), FMath::IsNearlyZero(Invalid.Linear[0]));

    const float SuddenYaw[3][4] = {{0, -1, 0, 0}, {1, 0, 0, 0}, {0, 0, 1, 0}};
    const auto Teleport = RGLVelocityDistortion::EstimateVelocity(SuddenYaw, Previous, 0.1, 0.1);
    TestTrue(TEXT("ninety degrees in 0.1 seconds is a teleport"),
        Teleport.State == RGLVelocityDistortion::Status::Teleport);
    TestTrue(TEXT("teleport has zero angular speed"), FMath::IsNearlyZero(Teleport.Angular[2]));
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRGLScanTimingVehicleIgnoreTest, "CarlaRGL.ScanTiming.VehicleIgnore",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FRGLScanTimingVehicleIgnoreTest::RunTest(const FString& Parameters)
{
    RGLVehicleIgnore::Registry<int> Registry;
    int SensorA = 0, SensorB = 0, SensorC = 0;
    const int32 VehicleId = Registry.Register(&SensorA, 1);
    TestTrue(TEXT("vehicle id is positive"), VehicleId > 0);
    TestEqual(TEXT("sensors on one vehicle share an id"), Registry.Register(&SensorB, 1), VehicleId);
    TestTrue(TEXT("another vehicle has a different id"), Registry.Register(&SensorC, 2) != VehicleId);
    const auto EntityId = RGLVehicleIgnore::FindSensorId(3,
        [](int Actor) { return std::array<int, 2>{{Actor == 3 ? 1 : 0, 0}}; },
        [&Registry](int Actor) { return Registry.Find(Actor); });
    TestEqual(TEXT("a child entity inherits the vehicle id"), EntityId, VehicleId);
    TestEqual(TEXT("a firing-only vehicle has no ignore id"), Registry.Find(4), 0);
    Registry.Remove(&SensorA);
    TestEqual(TEXT("remaining sensor keeps the id"), Registry.Find(1), VehicleId);

    // QT128C2X preset: 864000 points/s, 192 ray entries per step (two 96-firing sequences), 10 Hz.
    const std::size_t QtSteps = RGLVelocityDistortion::ScanSteps(864000, 192, 0.1, 10);
    TestEqual(TEXT("QT128C2X preset has 450 steps per rotation"), static_cast<int32>(QtSteps), 450);
    TestTrue(TEXT("QT128C2X preset with 192 ray entries is accepted"),
        RGLVelocityDistortion::ValidateOutput(true, false, "HesaiQT128C2X", QtSteps, 10, 360, 192) == nullptr);
    TestTrue(TEXT("QT128C2X with 128 ray entries is rejected"),
        RGLVelocityDistortion::ValidateOutput(true, false, "HesaiQT128C2X", QtSteps, 10, 360, 128) != nullptr);
    TestTrue(TEXT("Pandar128E4X with 128 ray entries is accepted"),
        RGLVelocityDistortion::ValidateOutput(true, false, "HesaiPandar128E4X", 1800, 10, 360, 128) == nullptr);
    TestTrue(TEXT("Pandar128E4X step is 2 x 27778 ns"),
        FMath::IsNearlyEqual(RGLVelocityDistortion::ModelStepSeconds("HesaiPandar128E4X"), 55556e-9, 1e-12));
    TestTrue(TEXT("QT128C2X step is 2 x 111110 ns"),
        FMath::IsNearlyEqual(RGLVelocityDistortion::ModelStepSeconds("HesaiQT128C2X"), 222220e-9, 1e-12));
    TestTrue(TEXT("other models have no step"), RGLVelocityDistortion::ModelStepSeconds("VelodyneVLP16") == 0.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRGLScanTimingIgnoredFacesTest, "CarlaRGL.ScanTiming.IgnoredFacesLimit",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FRGLScanTimingIgnoredFacesTest::RunTest(const FString& Parameters)
{
    using RGLVehicleIgnore::FacesLimitQuery;
    TestFalse(TEXT("a library without rgl_get_ignored_faces_limit is rejected"),
        RGLVehicleIgnore::AcceptsIgnoredFacesLimit(FacesLimitQuery::Missing, 256));
    TestFalse(TEXT("a failing rgl_get_ignored_faces_limit is rejected"),
        RGLVehicleIgnore::AcceptsIgnoredFacesLimit(FacesLimitQuery::Error, 256));
    TestFalse(TEXT("three faces (nested traces) are too few"),
        RGLVehicleIgnore::AcceptsIgnoredFacesLimit(FacesLimitQuery::Available, 3));
    TestFalse(TEXT("63 faces are too few"), RGLVehicleIgnore::AcceptsIgnoredFacesLimit(FacesLimitQuery::Available, 63));
    TestTrue(TEXT("64 faces are enough"), RGLVehicleIgnore::AcceptsIgnoredFacesLimit(FacesLimitQuery::Available, 64));
    TestTrue(TEXT("the looping library (256) is accepted"),
        RGLVehicleIgnore::AcceptsIgnoredFacesLimit(FacesLimitQuery::Available, 256));
    TestTrue(TEXT("a missing function is written as missing"),
        RGLVehicleIgnore::IgnoredFacesLimitField(FacesLimitQuery::Missing, 0) == "ignored_faces_limit=missing");
    TestTrue(TEXT("a failing call is written as error"),
        RGLVehicleIgnore::IgnoredFacesLimitField(FacesLimitQuery::Error, 0) == "ignored_faces_limit=error");
    TestTrue(TEXT("a small limit is written with its value"),
        RGLVehicleIgnore::IgnoredFacesLimitField(FacesLimitQuery::Available, 3) == "ignored_faces_limit=3");
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRGLScanTimingIgnoreRefreshTest, "CarlaRGL.ScanTiming.VehicleIgnoreRefresh",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)
bool FRGLScanTimingIgnoreRefreshTest::RunTest(const FString& Parameters)
{
    // Pure part of FRGLSceneManager::ApplyVehicleIgnore: 0 = RGL's default visibility.
    const int32 DefaultId = TNumericLimits<int32>::Min(); // RGL_SENSOR_ID_NONE
    TestFalse(TEXT("a fresh entity of a firing-only vehicle needs no call"),
        RGLVehicleIgnore::PlanEntityIgnore(0, 0, DefaultId).bNeeded);
    const auto SetCall = RGLVehicleIgnore::PlanEntityIgnore(0, 7, DefaultId);
    TestTrue(TEXT("a vehicle id is set"), SetCall.bNeeded && SetCall.SensorId == 7);
    TestFalse(TEXT("an unchanged id needs no call"), RGLVehicleIgnore::PlanEntityIgnore(7, 7, DefaultId).bNeeded);
    const auto ResetCall = RGLVehicleIgnore::PlanEntityIgnore(7, 0, DefaultId);
    TestTrue(TEXT("the last velocity sensor leaving restores the default"), ResetCall.bNeeded && ResetCall.SensorId == DefaultId);

    using RGLVehicleIgnore::RefreshStep;
    TestTrue(TEXT("firing-only scenes do not scan"),
        RGLVehicleIgnore::PlanRefresh(true, false, false, 1, 0, 0, 0) == RefreshStep::Nothing);
    TestTrue(TEXT("first refresh scans"),
        RGLVehicleIgnore::PlanRefresh(false, false, false, 1, 0, 1, 0) == RefreshStep::Scan);
    TestTrue(TEXT("other sensors in the frame reuse the scan"),
        RGLVehicleIgnore::PlanRefresh(false, true, true, 1, 1, 1, 1) == RefreshStep::Cached);
    TestTrue(TEXT("the next frame scans again"),
        RGLVehicleIgnore::PlanRefresh(false, true, true, 2, 1, 1, 1) == RefreshStep::Scan);
    TestTrue(TEXT("a registry change scans within the frame"),
        RGLVehicleIgnore::PlanRefresh(false, true, true, 1, 1, 2, 1) == RefreshStep::Scan);
    TestTrue(TEXT("ids left after the last sensor are reset once"),
        RGLVehicleIgnore::PlanRefresh(true, true, true, 1, 1, 2, 1) == RefreshStep::Scan);

    RGLVehicleIgnore::Registry<int> Registry;
    int SensorA = 0, SensorB = 0;
    Registry.Register(&SensorA, 1);
    const uint64 Revision = Registry.Revision();
    Registry.Register(&SensorA, 1);
    Registry.Register(&SensorB, 1);
    TestTrue(TEXT("sensors joining a vehicle do not change entity ids"), Registry.Revision() == Revision);
    Registry.Remove(&SensorA);
    Registry.Remove(&SensorB);
    TestTrue(TEXT("the vehicle losing its id is a change"), Registry.Revision() != Revision);
    TestTrue(TEXT("registry is empty"), Registry.IsEmpty());

    // The ancestor walk is bounded: a vehicle beyond MaxAncestors is not found.
    const auto Parent = [](int Actor) { return std::array<int, 2>{{Actor > 1 ? Actor - 1 : 0, 0}}; };
    const auto VehicleAtOne = [](int Actor) { return Actor == 1 ? 9 : 0; };
    TestEqual(TEXT("a near vehicle is found"), RGLVehicleIgnore::FindSensorId(4, Parent, VehicleAtOne), 9);
    TestEqual(TEXT("a vehicle beyond the bound is not"),
        RGLVehicleIgnore::FindSensorId(static_cast<int>(RGLVehicleIgnore::MaxAncestors) + 1, Parent, VehicleAtOne), 0);
    return true;
}
#endif
