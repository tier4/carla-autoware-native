#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace RGLVelocityDistortion
{
    enum class Status { Valid, NoPreviousPose, InvalidDelta, ExcessiveGap, Teleport, InvalidInput };

    struct Velocity
    {
        std::array<float, 3> Linear{};
        std::array<float, 3> Angular{};
        Status State = Status::NoPreviousPose;
    };

    // Use the same capped capture interval for session checks and ray generation.
    inline std::size_t ScanSteps(std::uint32_t PointsPerSecond, std::uint32_t Channels,
                                 double DeltaSeconds, double RotationFrequency)
    {
        if (Channels == 0 || !std::isfinite(DeltaSeconds) || DeltaSeconds <= 0 ||
            !std::isfinite(RotationFrequency) || RotationFrequency <= 0) return 0;
        const double Count = std::round(PointsPerSecond *
            std::fmin(DeltaSeconds, 1.0 / RotationFrequency) / Channels);
        if (!std::isfinite(Count) || Count < 1 ||
            Count > std::numeric_limits<int>::max() / Channels) return 0;
        return static_cast<std::size_t>(Count);
    }

    // One CARLA azimuth step, including both QT128C2X firing sequences.
    inline double ModelStepSeconds(const char* Model)
    {
        if (Model && std::strcmp(Model, "HesaiPandar128E4X") == 0) return 55556e-9;
        if (Model && std::strcmp(Model, "HesaiQT128C2X") == 0) return 2 * 111110e-9;
        return 0;
    }

    // Null means supported; otherwise a stable rejection reason, a whitespace-free token for the
    // key=value lines on stdout. Describe() gives the sentence for the engine log.
    inline const char* ValidateOutput(bool UdpEnabled, bool DirectRos2,
                                      const char* Model, std::size_t Steps,
                                      double RotationFrequency, double SweepDegrees,
                                      std::size_t Channels = 0)
    {
        if (DirectRos2)
            return "direct_ros2_output_unsupported";
        if (!UdpEnabled) return nullptr;
        const double StepSeconds = ModelStepSeconds(Model);
        if (StepSeconds == 0)
            return "unsupported_udp_model";
        if (Channels != 0 && Channels != (std::strcmp(Model, "HesaiQT128C2X") == 0 ? 192u : 128u))
            return "udp_model_ray_channel_count_mismatch";
        if (!std::isfinite(SweepDegrees) || std::abs(SweepDegrees - 360.0) > 1e-6)
            return "partial_sweep";
        if (Steps == 0 || !std::isfinite(RotationFrequency) || RotationFrequency <= 0)
            return "invalid_interval_or_steps";
        // RGL HesaiPandar128e4xPacketFormat.hpp: standard block = 2 * 27778 ns.
        // nebula pandar_128e4x.hpp: single-return block offset = -2 * 27778 ns;
        // dual returns share an azimuth step, so both modes advance by 55556 ns.
        // QT128C2X: 192 rays per CARLA step = two 96-firing blocks.
        // HesaiQt128c2xPacketFormat.hpp: each block advances by 111110 ns.
        const double RayStepSeconds = (1.0 / RotationFrequency) / Steps;
        if (std::abs(RayStepSeconds - StepSeconds) > 0.01 * StepSeconds)
            return "rotation_period_per_step_mismatch_tolerance_1_percent";
        return nullptr;
    }

    // Engine log sentence for a ValidateOutput reason; other reasons are returned unchanged.
    inline const char* Describe(const char* Reason)
    {
        static constexpr const char* Sentences[][2] = {
            {"direct_ros2_output_unsupported",
             "velocity_distortion does not support direct RGL ROS 2 output (rgl_lidar_topic_name must be empty)"},
            {"unsupported_udp_model",
             "velocity_distortion UDP supports HesaiPandar128E4X (standard mode) and HesaiQT128C2X only"},
            {"udp_model_ray_channel_count_mismatch",
             "the ray entries per azimuth step do not match the UDP model (QT128C2X 192, Pandar128E4X 128)"},
            {"partial_sweep", "velocity_distortion UDP requires a full 360 degree sweep per capture"},
            {"invalid_interval_or_steps",
             "velocity_distortion UDP requires a positive capture interval, step count and rotation frequency"},
            {"rotation_period_per_step_mismatch_tolerance_1_percent",
             "the rotation period per azimuth step differs from the model's step interval by more than 1%"},
        };
        if (!Reason) return "unknown";
        for (const auto& Entry : Sentences)
            if (std::strcmp(Reason, Entry[0]) == 0) return Entry[1];
        return Reason;
    }

    struct Timing
    {
        std::uint64_t ScanTimeNs;
        bool ZeroVelocity;
    };

    // Called after validating the rotation frequency at session creation.
    inline Timing CaptureTiming(std::uint64_t FrameTimeNs, double RotationFrequency)
    {
        const double PeriodNs = 1e9 / RotationFrequency;
        const bool Startup = static_cast<double>(FrameTimeNs) < PeriodNs;
        return {Startup ? 0 : FrameTimeNs - static_cast<std::uint64_t>(PeriodNs), Startup};
    }

    inline bool IsRigidPose(const float Pose[3][4])
    {
        if (!Pose) return false;
        for (int R = 0; R < 3; ++R)
            for (int C = 0; C < 4; ++C)
                if (!std::isfinite(Pose[R][C])) return false;
        for (int C = 0; C < 3; ++C)
        {
            double LengthSquared = 0;
            for (int R = 0; R < 3; ++R)
                LengthSquared += static_cast<double>(Pose[R][C]) * Pose[R][C];
            if (std::abs(std::sqrt(LengthSquared) - 1.0) > 1e-3) return false;
            for (int Other = C + 1; Other < 3; ++Other)
            {
                double Dot = 0;
                for (int R = 0; R < 3; ++R)
                    Dot += static_cast<double>(Pose[R][C]) * Pose[R][Other];
                if (std::abs(Dot) > 1e-3) return false;
            }
        }
        const double Det =
            static_cast<double>(Pose[0][0]) * (static_cast<double>(Pose[1][1])*Pose[2][2] - static_cast<double>(Pose[1][2])*Pose[2][1]) -
            static_cast<double>(Pose[0][1]) * (static_cast<double>(Pose[1][0])*Pose[2][2] - static_cast<double>(Pose[1][2])*Pose[2][0]) +
            static_cast<double>(Pose[0][2]) * (static_cast<double>(Pose[1][0])*Pose[2][1] - static_cast<double>(Pose[1][1])*Pose[2][0]);
        return Det > 0;
    }

    inline std::vector<float> TimeOffsetsMs(std::size_t Steps, std::size_t Channels,
                                            double HorizontalFov, double RotationFrequency,
                                            const char* UdpModel = nullptr)
    {
        if (Steps == 0 || Channels == 0 || !std::isfinite(HorizontalFov) ||
            HorizontalFov <= 0 || HorizontalFov > 360 ||
            !std::isfinite(RotationFrequency) || RotationFrequency <= 0 ||
            Steps > std::numeric_limits<std::size_t>::max() / Channels)
        {
            return {};
        }
        const double StepsPer360 = static_cast<double>(Steps) * 360.0 / HorizontalFov;
        const double PeriodMs = 1000.0 / RotationFrequency;
        if (UdpModel && ModelStepSeconds(UdpModel) == 0) return {};
        // Keep every offset in [-P, 0). ValidateOutput bounds the difference
        // between this configured interval and the model's packet interval.
        const double StepMs = PeriodMs / StepsPer360;
        std::vector<float> Offsets(Steps * Channels);
        // Azimuth-major layout: all channels in one step share the same offset.
        // Channel firing delays are intentionally omitted.
        for (std::size_t Step = 0; Step < Steps; ++Step)
        {
            const float Offset = static_cast<float>(Step * StepMs - PeriodMs);
            for (std::size_t Channel = 0; Channel < Channels; ++Channel)
            {
                Offsets[Step * Channels + Channel] = Offset;
            }
        }
        return Offsets;
    }

    // Row-major rigid sensor-to-world poses, translation in metres (rgl_mat3x4f layout).
    // Velocities are in the CURRENT sensor frame; Angular is RPY in rad/s.
    // A null Previous denotes the first capture. No engine or physics types are used.
    inline Velocity EstimateVelocity(const float Current[3][4], const float Previous[3][4],
                                     double DeltaSeconds, double PeriodSeconds)
    {
        Velocity Result;
        if (!std::isfinite(PeriodSeconds) || PeriodSeconds <= 0 ||
            !IsRigidPose(Current) || (Previous && !IsRigidPose(Previous)))
        {
            Result.State = Status::InvalidInput;
            return Result;
        }
        if (!Previous) return Result;
        if (!std::isfinite(DeltaSeconds) || DeltaSeconds <= 0)
        {
            Result.State = Status::InvalidDelta;
            return Result;
        }
        if (DeltaSeconds > 2.5 * PeriodSeconds)
        {
            Result.State = Status::ExcessiveGap;
            return Result;
        }

        double Translation[3]{};
        double Rotation[3][3]{};
        // Relative pose = inverse(Current) * Previous = TRS(-dt*v, -dt*w).
        // For a rigid pose the inverse rotation is the transpose.
        for (int R = 0; R < 3; ++R)
        {
            for (int K = 0; K < 3; ++K)
            {
                Translation[R] += static_cast<double>(Current[K][R]) *
                    (static_cast<double>(Previous[K][3]) - Current[K][3]);
                for (int C = 0; C < 3; ++C)
                    Rotation[R][C] += static_cast<double>(Current[K][R]) * Previous[K][C];
            }
        }
        double SpeedSquared = 0;
        for (double T : Translation) SpeedSquared += (T / DeltaSeconds) * (T / DeltaSeconds);
        if (SpeedSquared > 70.0 * 70.0)
        {
            Result.State = Status::Teleport;
            return Result;
        }

        // Mat3x4f::rotationRad implements Rz(yaw) * Ry(pitch) * Rx(roll).
        // Extract angles from the backward relative pose, then negate / dt.
        // atan2 avoids asin domain errors near +/- pi/2. At gimbal lock choose roll=0.
        const double CosPitch = std::hypot(Rotation[0][0], Rotation[1][0]);
        const double Pitch = std::atan2(-Rotation[2][0], CosPitch);
        const double Roll = CosPitch > 1e-6 ? std::atan2(Rotation[2][1], Rotation[2][2]) : 0;
        const double Yaw = CosPitch > 1e-6
            ? std::atan2(Rotation[1][0], Rotation[0][0])
            : std::atan2(-Rotation[0][1], Rotation[1][1]);
        const double AngularSpeedSquared = (Roll * Roll + Pitch * Pitch + Yaw * Yaw) /
            (DeltaSeconds * DeltaSeconds);
        if (AngularSpeedSquared > 3.0 * 3.0)
        {
            Result.State = Status::Teleport;
            return Result;
        }
        for (int Axis = 0; Axis < 3; ++Axis)
            Result.Linear[Axis] = static_cast<float>(-Translation[Axis] / DeltaSeconds);
        Result.Angular = {{static_cast<float>(-Roll / DeltaSeconds),
                           static_cast<float>(-Pitch / DeltaSeconds),
                           static_cast<float>(-Yaw / DeltaSeconds)}};
        Result.State = Status::Valid;
        return Result;
    }
}
