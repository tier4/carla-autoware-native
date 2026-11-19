// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/InertialMeasurementUnitHighPrecision.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/CarlaStatics.h"

// UE5 engine/physics headers: repo convention uses the (no-op) UE header guard.
#include <util/ue-header-guard-begin.h>
#include "PhysicsEngine/BodyInstance.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PBDRigidsSolver.h" // complete Chaos::FPBDRigidsSolver for *SimCallbackObject_External
#include "HAL/PlatformProcess.h" // raw sink: GetDllHandle / GetDllExport
#include <util/ue-header-guard-end.h>

// LibCarla headers: disable-ue4-macros block.
#include <util/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/QuaternionAngularVelocity.h"
#include "carla/ros2/ROS2.h"
#if PLATFORM_LINUX
// ImuMath.h uses M_PI, which only LibCarla's own targets define on MSVC; the
// raw sink is Linux only, so include it there.
#include "carla/ros2/publishers/ImuMath.h"
#endif
#include <util/enable-ue4-macros.h>

AInertialMeasurementUnitHighPrecision::AInertialMeasurementUnitHighPrecision(
    const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
}

FActorDefinition AInertialMeasurementUnitHighPrecision::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeIMUHighPrecisionDefinition();
}

void AInertialMeasurementUnitHighPrecision::Set(const FActorDescription &ActorDescription)
{
  // Base class reads noise/bias attributes.
  Super::Set(ActorDescription);

  // Read high-precision-specific attributes.
  const FString ModeStr = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
      TEXT("substep_mode"), ActorDescription.Variations, TEXT("auto"));
  if (ModeStr.Equals(TEXT("substep"), ESearchCase::IgnoreCase))
  {
    ConfiguredMode = EIMUOutputMode::Substep;
  }
  else if (ModeStr.Equals(TEXT("upsample"), ESearchCase::IgnoreCase))
  {
    ConfiguredMode = EIMUOutputMode::Upsample;
  }
  else
  {
    ConfiguredMode = EIMUOutputMode::Auto; // "auto" or unrecognised
  }

  OutputRateHz = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("output_rate_hz"), ActorDescription.Variations, 200.0f);

  RawSinkLibrary = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
      TEXT("raw_sink_library"), ActorDescription.Variations, TEXT(""));
  RawSinkConfig = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
      TEXT("raw_sink_config"), ActorDescription.Variations, TEXT(""));
}

// =========================================================================
// Raw sink: an external shared library receives every sample through the C
// interface in carla_imu_raw_sink.h. It is independent of the ROS 2 publish
// (enable_for_ros): each output can be switched on or off on its own.
// =========================================================================

namespace
{
  // Called from a sink-owned thread; UE_LOG is thread safe.
  void RawSinkLog(void *User, int Level, const char *Message)
  {
    const FString &Name = *static_cast<const FString *>(User);
    const FString Text = UTF8_TO_TCHAR(Message);
    if (Level == CARLA_IMU_RAW_SINK_LOG_ERROR)
    {
      UE_LOG(LogCarla, Error, TEXT("%s: %s"), *Name, *Text);
    }
    else if (Level == CARLA_IMU_RAW_SINK_LOG_WARNING)
    {
      UE_LOG(LogCarla, Warning, TEXT("%s: %s"), *Name, *Text);
    }
    else
    {
      UE_LOG(LogCarla, Log, TEXT("%s: %s"), *Name, *Text);
    }
  }
} // namespace

void AInertialMeasurementUnitHighPrecision::StartRawSinkOnce()
{
  if (RawSinkLibrary.IsEmpty() || bRawSinkTried)
  {
    return;
  }
  bRawSinkTried = true;

#if PLATFORM_LINUX
  RawSinkDll = FPlatformProcess::GetDllHandle(*RawSinkLibrary);
  if (RawSinkDll == nullptr)
  {
    UE_LOG(LogCarla, Error,
        TEXT("%s: could not load raw_sink_library '%s'; no raw output for this sensor."),
        *GetName(), *RawSinkLibrary);
    return;
  }
  const auto AbiVersion = reinterpret_cast<carla_imu_raw_sink_abi_version_fn>(
      FPlatformProcess::GetDllExport(RawSinkDll, TEXT("carla_imu_raw_sink_abi_version")));
  const auto Create = reinterpret_cast<carla_imu_raw_sink_create_fn>(
      FPlatformProcess::GetDllExport(RawSinkDll, TEXT("carla_imu_raw_sink_create")));
  RawSinkPush = reinterpret_cast<carla_imu_raw_sink_push_fn>(
      FPlatformProcess::GetDllExport(RawSinkDll, TEXT("carla_imu_raw_sink_push")));
  RawSinkDestroy = reinterpret_cast<carla_imu_raw_sink_destroy_fn>(
      FPlatformProcess::GetDllExport(RawSinkDll, TEXT("carla_imu_raw_sink_destroy")));
  if (AbiVersion == nullptr || Create == nullptr || RawSinkPush == nullptr ||
      RawSinkDestroy == nullptr || AbiVersion() != CARLA_IMU_RAW_SINK_ABI_VERSION)
  {
    UE_LOG(LogCarla, Error,
        TEXT("%s: '%s' is not an IMU raw sink of ABI version %d; "
             "no raw output for this sensor."),
        *GetName(), *RawSinkLibrary, CARLA_IMU_RAW_SINK_ABI_VERSION);
    StopRawSink();
    return;
  }

  RawSinkLogName = new FString(GetName());
  char ErrorText[512] = {0};
  RawSink = Create(TCHAR_TO_UTF8(*RawSinkConfig), &RawSinkLog, RawSinkLogName,
      ErrorText, sizeof(ErrorText));
  if (RawSink == nullptr)
  {
    UE_LOG(LogCarla, Error,
        TEXT("%s: raw sink '%s' did not start: %s; no raw output for this sensor."),
        *GetName(), *RawSinkLibrary, UTF8_TO_TCHAR(ErrorText));
    StopRawSink();
    return;
  }
  UE_LOG(LogCarla, Log, TEXT("%s: raw sink '%s' started"), *GetName(), *RawSinkLibrary);
#else
  UE_LOG(LogCarla, Error,
      TEXT("%s: raw_sink_library is only supported on Linux; no raw output for this sensor."),
      *GetName());
#endif
}

bool AInertialMeasurementUnitHighPrecision::IsStreamEnabledForRos()
{
  // This sensor publishes to ROS 2 itself (per substep), not through the
  // stream, so it has to look at the stream's ROS flag the way the client's
  // enable_for_ros / disable_for_ros set it (single-GPU streaming server).
  auto *GameInstance = UCarlaStatics::GetGameInstance(GetWorld());
  if (GameInstance == nullptr)
  {
    return true;
  }
  const auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
  return GameInstance->GetServer().GetStreamingServer().IsEnabledForROS(StreamId);
}

void AInertialMeasurementUnitHighPrecision::StopRawSink()
{
  if (RawSink != nullptr && RawSinkDestroy != nullptr)
  {
    // Joins the sink's threads; its log callback is not called after this.
    RawSinkDestroy(RawSink);
  }
  RawSink = nullptr;
  RawSinkPush = nullptr;
  RawSinkDestroy = nullptr;
  delete RawSinkLogName;
  RawSinkLogName = nullptr;
  if (RawSinkDll != nullptr)
  {
    FPlatformProcess::FreeDllHandle(RawSinkDll);
    RawSinkDll = nullptr;
  }
}

// =========================================================================
// Phase 1 frame-rate gyro (fallback path, used when substep is unavailable).
// =========================================================================

// Angular velocity from the finite difference of the sensor's own world
// rotation, following AWSIM's ImuSensor.cs (physics-independent). The result
// is expressed in the sensor's local frame (CARLA: X-fwd, Y-right, Z-up) so
// that the reused CarlaIMUPublisher produces ROS output identical in
// convention to the standard IMU.
carla::geom::Vector3D AInertialMeasurementUnitHighPrecision::ComputeGyroscope()
{
  const float CurrentTime = GetWorld()->GetTimeSeconds();
  const FQuat CurrentRotation =
      GetRootComponent()->GetComponentTransform().GetRotation();

  // First valid frame: seed state and report zero.
  if (!FMath::IsFinite(PrevGyroTime))
  {
    PrevGyroTime = CurrentTime;
    PrevRotation = CurrentRotation;
    return carla::geom::Vector3D{};
  }

  const float DeltaTime = CurrentTime - PrevGyroTime;
  PrevGyroTime = CurrentTime;

  if (DeltaTime <= 0.0f)
  {
    PrevRotation = CurrentRotation;
    return carla::geom::Vector3D{};
  }

  const auto Omega = carla::geom::AngularVelocityFromQuaternions(
      {PrevRotation.W, PrevRotation.X, PrevRotation.Y, PrevRotation.Z},
      {CurrentRotation.W, CurrentRotation.X, CurrentRotation.Y, CurrentRotation.Z},
      DeltaTime);
  const FVector WorldAngularVelocity(Omega[0], Omega[1], Omega[2]);
  FVector LocalAngularVelocity =
      CurrentRotation.UnrotateVector(WorldAngularVelocity);

  PrevRotation = CurrentRotation;

  // Guard against NaN from invalid rotation data.
  if (LocalAngularVelocity.ContainsNaN())
  {
    LocalAngularVelocity = FVector::ZeroVector;
  }

  // Reuse the base noise/bias model so behavior matches the standard IMU.
  return ComputeGyroscopeNoise(LocalAngularVelocity);
}

// =========================================================================
// Substep callback registration / teardown.
// =========================================================================

void AInertialMeasurementUnitHighPrecision::RegisterSubstepCallback()
{
  // In upsample mode we intentionally skip registering the substep callback.
  if (ConfiguredMode == EIMUOutputMode::Upsample)
  {
    return;
  }

  UWorld* World = GetWorld();
  if (World == nullptr) return;
  FPhysScene_Chaos* Scene = static_cast<FPhysScene_Chaos*>(World->GetPhysicsScene());
  if (Scene == nullptr) return;
  Chaos::FPhysicsSolver* Solver = Scene->GetSolver();
  if (Solver == nullptr) return;

  // Owner vehicle root primitive -> physics proxy.
  AActor* Owner = GetOwner();
  UPrimitiveComponent* Root = (Owner != nullptr)
      ? Cast<UPrimitiveComponent>(Owner->GetRootComponent()) : nullptr;
  FBodyInstance* BI = (Root != nullptr) ? Root->GetBodyInstance() : nullptr;
  VehicleProxy = (BI != nullptr)
      ? static_cast<Chaos::FSingleParticlePhysicsProxy*>(BI->GetPhysicsActorHandle())
      : nullptr;
  if (VehicleProxy == nullptr)
  {
    UE_LOG(LogCarla, Warning,
        TEXT("imu_highprecision: no vehicle physics proxy; substep unavailable, "
             "falling back to upsample ZOH path"));
    return;
  }
  SubstepCallback = Solver->CreateAndRegisterSimCallbackObject_External<FIMUSubstepCallback>();
}

void AInertialMeasurementUnitHighPrecision::UnregisterSubstepCallback()
{
  if (SubstepCallback == nullptr) return;
  UWorld* World = GetWorld();
  FPhysScene_Chaos* Scene = (World != nullptr)
      ? static_cast<FPhysScene_Chaos*>(World->GetPhysicsScene()) : nullptr;
  if (Scene != nullptr && Scene->GetSolver() != nullptr)
  {
    Scene->GetSolver()->UnregisterAndFreeSimCallbackObject_External(SubstepCallback);
  }
  SubstepCallback = nullptr;
}

void AInertialMeasurementUnitHighPrecision::EndPlay(EEndPlayReason::Type EndPlayReason)
{
  UnregisterSubstepCallback();
  StopRawSink();
  Super::EndPlay(EndPlayReason);
}

void AInertialMeasurementUnitHighPrecision::BeginDestroy()
{
  // Actors destroyed without EndPlay must not leave a sink thread running.
  StopRawSink();
  Super::BeginDestroy();
}

// =========================================================================
// Per-substep publish helper (ROS2 + streaming) with per-substep timestamp.
// =========================================================================

void AInertialMeasurementUnitHighPrecision::PublishSubstepSample(
    double TimeOffsetSeconds,
    const carla::geom::Vector3D &Accelerometer,
    const carla::geom::Vector3D &Gyroscope,
    float Compass)
{
  auto DataStream = GetDataStream(*this);

#if PLATFORM_LINUX
  if (RawSink != nullptr)
  {
    // Same CARLA -> REP-103 conversion as the ROS 2 IMU publisher, so the sink
    // receives exactly what the topic would have carried.
    const auto Accel = carla::ros2::AccelToRos(Accelerometer.x, Accelerometer.y, Accelerometer.z);
    const auto Gyro = carla::ros2::GyroToRos(Gyroscope.x, Gyroscope.y, Gyroscope.z);
    carla_imu_raw_sample RawSample;
    RawSample.sim_time_s = GetEpisode().GetElapsedGameTime() - TimeOffsetSeconds;
    for (int Axis = 0; Axis < 3; ++Axis)
    {
      RawSample.gyro_radps[Axis] = Gyro[Axis];
      RawSample.accel_mps2[Axis] = Accel[Axis];
    }
    RawSinkPush(RawSink, &RawSample);
  }
#endif

  #if defined(WITH_ROS2)
  // Independent of the raw sink: the ROS 2 publish follows this sensor's ROS
  // flag (enable_for_ros / disable_for_ros, on by default with the server's
  // default topic visibility), so either output can be on or off.
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled() && bRos2StreamEnabled)
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    AActor* ParentActor = GetAttachParentActor();
    if (ParentActor)
    {
      FTransform LocalTransformRelativeToParent =
          GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
      ROS2->ProcessDataFromIMUStamped(
          TimeOffsetSeconds, DataStream.GetSensorType(), StreamId,
          LocalTransformRelativeToParent, Accelerometer, Gyroscope, Compass, this);
    }
    else
    {
      ROS2->ProcessDataFromIMUStamped(
          TimeOffsetSeconds, DataStream.GetSensorType(), StreamId,
          DataStream.GetSensorTransform(), Accelerometer, Gyroscope, Compass, this);
    }
  }
  #endif

  {
    TRACE_CPUPROFILER_EVENT_SCOPE(AInertialMeasurementUnitHighPrecision::SerializeAndSend);
    DataStream.SerializeAndSend(*this, Accelerometer, Gyroscope, Compass);
  }
}

// =========================================================================
// ZOH upsample helper: emits N copies of a frame-rate IMU value.
// =========================================================================

static void EmitZOHSamples(
    AInertialMeasurementUnitHighPrecision* Sensor,
    float OutputRateHz,
    float DeltaTime)
{
  const int32 N = FMath::Max(1, FMath::RoundToInt(OutputRateHz * DeltaTime));
  const carla::geom::Vector3D G = Sensor->ComputeGyroscope();
  const carla::geom::Vector3D A = Sensor->ComputeAccelerometerAtTime(
      Sensor->GetWorld()->GetTimeSeconds());
  const float Compass = Sensor->ComputeCompass();
  for (int32 k = 0; k < N; ++k)
  {
    // Timestamps are evenly spaced across the frame ending at the frame stamp.
    // Sample k=0 is the oldest (furthest before the frame stamp),
    // sample k=N-1 is the most recent (at the frame stamp).
    const double tOffset =
        static_cast<double>(N - 1 - k) * static_cast<double>(DeltaTime) / N;
    Sensor->PublishSubstepSample(tOffset, A, G, Compass);
  }
}

// =========================================================================
// PostPhysTick: mode-dispatch to substep path, upsample ZOH path, or fallback.
// =========================================================================

void AInertialMeasurementUnitHighPrecision::PostPhysTick(
    UWorld* World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AInertialMeasurementUnitHighPrecision::PostPhysTick);

  StartRawSinkOnce();
  // Read once per frame: the samples of this frame are published below.
  bRos2StreamEnabled = IsStreamEnabledForRos();

  // Upsample-only mode: skip substep registration entirely; emit ZOH samples.
  if (ConfiguredMode == EIMUOutputMode::Upsample)
  {
    ResetSubstepHistory();
    EmitZOHSamples(this, OutputRateHz, DeltaTime);
    return;
  }

  // Auto / Substep mode: try to register the substep callback on first tick.
  if (SubstepCallback == nullptr && VehicleProxy == nullptr)
  {
    RegisterSubstepCallback();
  }

  // No callback registered (no valid proxy): fall back to ZOH upsample.
  // The base PrevLocation/PrevTime will be seeded by ComputeAccelerometerAtTime on
  // the first call; the base NaN guard handles the first-sample transition.
  if (SubstepCallback == nullptr)
  {
    ResetSubstepHistory();
    EmitZOHSamples(this, OutputRateHz, DeltaTime);
    return;
  }

  // Feed the current proxy to the physics thread for the next step.
  if (FIMUSubstepInput* InData = SubstepCallback->GetProducerInputData_External())
  {
    InData->Proxy = VehicleProxy;
  }

  // Drain all substep samples accumulated since the last frame.
  TArray<FIMUSubstepSample> Frame;
  while (Chaos::TSimCallbackOutputHandle<FIMUSubstepOutput> Out =
             SubstepCallback->PopFutureOutputData_External())
  {
    Frame.Append(Out->Samples);
  }

  const int32 N = Frame.Num();

  // No samples this frame (e.g. first frame before the physics thread has fired,
  // or a brief transition out of substep). The base PrevLocation/PrevTime become
  // stale when we were in the substep path last frame, which would cause a
  // one-frame spike in the base accelerometer. Fall back to ZOH upsample;
  // the base NaN/IsFinite guard will re-seed on the next valid call.
  if (N == 0)
  {
    // The next substep sample must not be differenced against a rotation or
    // positions from before this gap (it would be divided by one substep dt).
    ResetSubstepHistory();
    EmitZOHSamples(this, OutputRateHz, DeltaTime);
    return;
  }

  // Capture the mount (sensor relative to the vehicle body) once.
  if (!bMountCaptured)
  {
    const FTransform Rel = GetRootComponent()->GetRelativeTransform(); // parent = vehicle
    MountRelRot = Rel.GetRotation();
    MountOffsetLocal = Rel.GetLocation();
    bMountCaptured = true;
  }

  const ACarlaGameModeBase* GameMode = UCarlaStatics::GetGameMode(GetWorld());
  const float GRAVITY = (GameMode != nullptr) ? GameMode->IMUSensorGravity : -9.81f;
  // Convert UE's cm to meters (matches the base accelerometer convention).
  constexpr float TO_METERS = 1e-2f;

  // Compass uses the current frame value (low-rate orientation reference).
  const float Compass = ComputeCompass();

  // Precompute suffix sums of Dt for accurate per-substep timestamps.
  // SuffixDt[k] = sum of Frame[j].Dt for j = k+1 .. N-1.
  // This is the elapsed time from the END of substep k to the frame stamp,
  // which equals the per-substep time offset for sample k.
  TArray<double> SuffixDt;
  SuffixDt.SetNumUninitialized(N);
  SuffixDt[N - 1] = 0.0;
  for (int32 j = N - 2; j >= 0; --j)
  {
    SuffixDt[j] = SuffixDt[j + 1] + static_cast<double>(Frame[j + 1].Dt);
  }

  for (int32 k = 0; k < N; ++k)
  {
    const FIMUSubstepSample& S = Frame[k];

    // GetQ/GetP are the solved values at OnPostSolve_Internal time.
    const FQuat   BodyRot = S.RotQ;
    const FVector BodyPos = S.PosP;

    const FQuat   ImuRot = BodyRot * MountRelRot;
    const FVector ImuPos = BodyPos + BodyRot.RotateVector(MountOffsetLocal);

    carla::geom::Vector3D Gyro{};
    carla::geom::Vector3D Accel{};

    if (bSubstepPrevValid && S.Dt > 0.0f)
    {
      // Gyro: quaternion log map, also accurate for small angular rates.
      const auto Omega = carla::geom::AngularVelocityFromQuaternions(
          {PrevImuRot.W, PrevImuRot.X, PrevImuRot.Y, PrevImuRot.Z},
          {ImuRot.W, ImuRot.X, ImuRot.Y, ImuRot.Z},
          S.Dt);
      FVector wWorld(Omega[0], Omega[1], Omega[2]);
      FVector wLocal = ImuRot.UnrotateVector(wWorld);
      if (wLocal.ContainsNaN())
      {
        wLocal = FVector::ZeroVector;
      }
      Gyro = ComputeGyroscopeNoise(wLocal);

      // Accel: 2nd derivative (quadratic interpolation, same form as the base)
      // evaluated at the IMU point, plus gravity (cm -> m).
      if (!PrevImuPos[0].ContainsNaN() && !PrevImuPos[1].ContainsNaN() &&
          FMath::IsFinite(PrevSubDt) && PrevSubDt > 0.0f)
      {
        const float H1 = S.Dt;
        const float H2 = PrevSubDt;
        const float H12 = H1 + H2;
        const FVector A = PrevImuPos[1] / (H1 * H2);
        const FVector B = PrevImuPos[0] / (H2 * H12);
        const FVector C = ImuPos / (H1 * H12);
        FVector aWorld = TO_METERS * -2.0f * (A - B - C);
        aWorld.Z += GRAVITY;
        FVector aLocal = ImuRot.UnrotateVector(aWorld);
        Accel = ComputeAccelerometerNoise(aLocal);
      }
    }

    // Per-substep timestamp: use accumulated suffix dt for accuracy with
    // non-uniform substep sizes (robust to variable substep dt).
    PublishSubstepSample(SuffixDt[k], Accel, Gyro, Compass);

    // Advance integration state.
    PrevImuRot = ImuRot;
    bSubstepPrevValid = true;
    PrevImuPos[0] = PrevImuPos[1];
    PrevImuPos[1] = ImuPos;
    PrevSubDt = S.Dt;
  }

  // Substep path published each sample with its own timestamp; do NOT call
  // Super::PostPhysTick (avoids a duplicate frame-rate publish).
}
