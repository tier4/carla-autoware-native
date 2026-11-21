#pragma once

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// One-line key=value results on stdout. stdout stays available when engine logging is
// compiled out (Shipping), so a LiDAR that stops producing data is still explained.
namespace RGLScanTiming
{
    // A whitespace-free value for key=value lines.
    inline std::string Token(const char* Text)
    {
        std::string Out = Text && *Text ? Text : "unknown";
        for (char& C : Out)
            if (std::isspace(static_cast<unsigned char>(C)) || C == '"') C = '_';
        return Out;
    }

    // Session creation: a rejection is written exactly once, including early RGL API failures.
    // An accepted session writes its line after its first successful capture (CaptureDiagnostics),
    // when the vehicle it is attached to is known and the graph has run.
    class SessionDiagnostics
    {
    public:
        SessionDiagnostics(bool bInEnabled, void* InSensor, const char* InName, const char* InModel)
            : bEnabled(bInEnabled), Sensor(InSensor), Name(Token(InName)), Model(Token(InModel)) {}
        ~SessionDiagnostics() { Reject("session_creation_failed"); }
        // Reason: a token (see RGLVelocityDistortion::Describe for the engine log sentence).
        // Detail: optional extra key=value field.
        void Reject(const char* Reason, const char* Detail = nullptr)
        {
            if (!bEnabled || bDone) return;
            std::printf("rgl_scan_timing_rejected sensor=%p name=%s model=%s reason=%s%s%s\n",
                Sensor, Name.c_str(), Model.c_str(), Token(Reason).c_str(), Detail ? " " : "", Detail ? Detail : "");
            std::fflush(stdout);
            bDone = true;
        }
        void Accept() { bDone = true; }
        SessionDiagnostics(const SessionDiagnostics&) = delete;
        SessionDiagnostics& operator=(const SessionDiagnostics&) = delete;
    private:
        bool bEnabled;
        bool bDone = false;
        void* Sensor;
        std::string Name;
        std::string Model;
    };

    // Capture: each line is written at most once per session; a stop once per reason.
    class CaptureDiagnostics
    {
    public:
        CaptureDiagnostics() = default;
        CaptureDiagnostics(void* InSensor, const char* InName, const char* InModel, bool bInVelocityDistortion)
            : Sensor(InSensor), Name(Token(InName)), Model(Token(InModel)), bVelocityDistortion(bInVelocityDistortion) {}

        // A capture ended early (any mode). Reason: a token. Most reasons are retried at the next
        // capture, so a later, different reason is still written; a recovery is not.
        void Stopped(const char* Reason)
        {
            const char* Raw = Reason ? Reason : "";
            for (const std::string& Done : StopReasons)
                if (Done == Raw) return; // no allocation while a capture keeps failing
            StopReasons.emplace_back(Raw);
            std::printf("rgl_scan_timing_stopped sensor=%p name=%s mode=%s reason=%s\n", Sensor, Name.c_str(),
                bVelocityDistortion ? "velocity_distortion" : "firing", Token(Raw).c_str());
            std::fflush(stdout);
        }
        // The actual ray pattern was rejected before capture. True when the line was written.
        bool Rejected(const char* Reason)
        {
            if (!bVelocityDistortion || bRejected) return false;
            bRejected = true;
            std::printf("rgl_scan_timing_rejected sensor=%p name=%s model=%s reason=%s\n", Sensor,
                Name.c_str(), Model.c_str(), Token(Reason).c_str());
            std::fflush(stdout);
            return true;
        }
        // The cached ray pattern, for the accepted line.
        void SetRayPattern(std::size_t InSteps, double InStepUs)
        {
            Steps = InSteps;
            StepUs = InStepUs;
        }
        // After each successful graph run. IgnoreId: the vehicle's raytrace id, 0 when no vehicle was found.
        // The first call writes the accepted line; a vehicle found after ignore=none is written once,
        // and a capture without a vehicle is warned about once.
        void Captured(std::int32_t IgnoreId)
        {
            if (!bVelocityDistortion) return;
            bool bWritten = false;
            if (!bAccepted)
            {
                bAccepted = true;
                bIgnoreWritten = IgnoreId != 0;
                const std::string Ignore = IgnoreId ? std::to_string(IgnoreId) : std::string("none");
                std::printf("rgl_scan_timing sensor=%p name=%s mode=velocity_distortion model=%s steps=%llu step_us=%.6f ignore=%s\n",
                    Sensor, Name.c_str(), Model.c_str(), static_cast<unsigned long long>(Steps), StepUs, Ignore.c_str());
                bWritten = true;
            }
            else if (IgnoreId != 0 && !bIgnoreWritten)
            {
                bIgnoreWritten = true;
                std::printf("rgl_scan_timing_ignore sensor=%p name=%s ignore=%d\n", Sensor, Name.c_str(), static_cast<int>(IgnoreId));
                bWritten = true;
            }
            if (IgnoreId == 0 && !bWarned)
            {
                // The LiDAR sees its own vehicle: it is not attached to one (directly or through other actors).
                bWarned = true;
                std::printf("rgl_scan_timing_warning sensor=%p name=%s reason=no_parent_vehicle\n", Sensor, Name.c_str());
                bWritten = true;
            }
            if (bWritten) std::fflush(stdout);
        }

    private:
        void* Sensor = nullptr;
        std::string Name = "unknown";
        std::string Model = "unknown";
        bool bVelocityDistortion = false;
        std::vector<std::string> StopReasons;
        bool bRejected = false;
        bool bAccepted = false;
        bool bIgnoreWritten = false;
        bool bWarned = false;
        std::size_t Steps = 0;
        double StepUs = 0;
    };

    // Quarantine a shared scene's time state; written once per scene manager.
    // World: the path name of the manager's world.
    inline void DisableSceneTime(bool& bUsable, const char* World, const char* Reason)
    {
        if (bUsable)
        {
            std::printf("rgl_scene_time_disabled world=%s reason=%s\n", Token(World).c_str(), Token(Reason).c_str());
            std::fflush(stdout);
        }
        bUsable = false;
    }
}
