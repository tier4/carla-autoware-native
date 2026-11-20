#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include "RGLUdpExtensionShim.h"

// Values of the rgl_scan_timing attribute of sensor.lidar.rgl. RGL captures a
// scan's points at one time, the scene time of the frame.
//   firing (default): RGL stamps each UDP packet as the real sensor would
//       fire it, over one rotation from that time. The stamps then run up to
//       one rotation ahead of the points, and a driver's motion compensation
//       moves points that did not move.
//   instant: every UDP packet carries the capture time, so the stamps match
//       the points.
namespace RGLScanTiming
{
    inline const TCHAR* Firing() { return TEXT("firing"); }
    inline const TCHAR* Instant() { return TEXT("instant"); }

    inline bool IsValid(const FString& Timing)
    {
        return Timing == Firing() || Timing == Instant();
    }

    /// The UDP option bits a scan timing adds.
    inline uint32 UdpOptions(const FString& Timing)
    {
        return Timing == Instant()
            ? static_cast<uint32>(RGL_UDP_SAME_TIMESTAMP_FOR_PACKETS)
            : static_cast<uint32>(RGL_UDP_NO_ADDITIONAL_OPTIONS);
    }
}
