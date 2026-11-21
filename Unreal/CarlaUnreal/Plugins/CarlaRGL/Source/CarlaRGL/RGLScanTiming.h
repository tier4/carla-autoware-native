#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include "RGLUdpExtensionShim.h"

// Values of the rgl_scan_timing attribute of sensor.lidar.rgl.
// firing (default) captures at frame time, with packet times advancing from it.
// velocity_distortion applies sensor motion over the preceding rotation;
// packet times advance from frame time minus one rotation. Both use firing times.
namespace RGLScanTiming
{
    inline const TCHAR* Firing() { return TEXT("firing"); }
    inline const TCHAR* VelocityDistortion() { return TEXT("velocity_distortion"); }

    inline bool IsValid(const FString& Timing)
    {
        return Timing == Firing() || Timing == VelocityDistortion();
    }

    /// The UDP option bits a scan timing adds. Neither timing adds any: both keep the model's
    /// advancing packet times (velocity_distortion moves the packet epoch through the scene time
    /// instead). The timing stays a parameter so that the session code and the UDP options test
    /// keep naming the timing they configure.
    inline uint32 UdpOptions(const FString& /*Timing*/)
    {
        return static_cast<uint32>(RGL_UDP_NO_ADDITIONAL_OPTIONS);
    }
}
