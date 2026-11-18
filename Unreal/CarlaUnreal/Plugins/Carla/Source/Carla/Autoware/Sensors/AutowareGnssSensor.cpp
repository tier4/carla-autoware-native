// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Autoware/Sensors/AutowareGnssSensor.h"

#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Autoware/Game/AutowareWorldSettings.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/MapGen/LargeMapManager.h"

#include <util/ue-header-guard-begin.h>
#include "Components/PrimitiveComponent.h"
#include "HAL/PlatformProcess.h" // raw sink: GetDllHandle / GetDllExport
#include "Misc/CoreDelegates.h" // raw sink: shutdown_all at process exit
#include <util/ue-header-guard-end.h>

#include <util/disable-ue4-macros.h>
#include "carla/geom/Vector3D.h"
#include "carla/ros2/ROS2.h"
#if PLATFORM_LINUX
// These use M_PI, which only LibCarla's own targets define on MSVC; the raw
// sink is Linux only, so include them there.
#include "carla/ros2/publishers/ImuMath.h"
#include "carla/ros2/publishers/TransformQuaternion.h"
#endif
#include <util/enable-ue4-macros.h>

AAutowareGnssSensor::AAutowareGnssSensor(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
}

FActorDefinition AAutowareGnssSensor::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeAutowareGnssDefinition();
}

void AAutowareGnssSensor::BeginPlay()
{
  Super::BeginPlay();

  // Load MGRS data from the active level's world settings.
  LoadMgrsData();

  const UCarlaEpisode* episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  CurrentGeoProjection = episode->GetGeoProjection();
}

void AAutowareGnssSensor::EndPlay(EEndPlayReason::Type EndPlayReason)
{
  StopRawSink();
  Super::EndPlay(EndPlayReason);
}

void AAutowareGnssSensor::BeginDestroy()
{
  // Actors destroyed without EndPlay must not leave their sink attached.
  StopRawSink();
  Super::BeginDestroy();
}

// =========================================================================
// Raw sink: an external shared library receives the antenna's ground truth
// every tick through the C interface in carla_gnss_raw_sink.h. It is
// independent of the ROS 2 publish (enable_for_ros): each output can be
// switched on or off on its own.
// =========================================================================

#if PLATFORM_LINUX
namespace
{
  // Called from any thread (a sink-owned one too); UE_LOG is thread safe.
  void GnssRawSinkLog(void *User, int Level, const char *Message)
  {
    const FString &Name = *static_cast<const FString *>(User);
    const FString Text = UTF8_TO_TCHAR(Message);
    if (Level == CARLA_GNSS_RAW_SINK_LOG_ERROR)
    {
      UE_LOG(LogCarla, Error, TEXT("%s: %s"), *Name, *Text);
    }
    else if (Level == CARLA_GNSS_RAW_SINK_LOG_WARNING)
    {
      UE_LOG(LogCarla, Warning, TEXT("%s: %s"), *Name, *Text);
    }
    else
    {
      UE_LOG(LogCarla, Log, TEXT("%s: %s"), *Name, *Text);
    }
  }

  // A library that created a sink and exports the optional shutdown_all.
  struct FGnssRawSinkLibrary
  {
    FString Path;
    void *Dll = nullptr; // a handle of our own: keeps ShutdownAll loaded
    carla_gnss_raw_sink_shutdown_all_fn ShutdownAll = nullptr;
  };

  // Game thread only: sinks are created in PostPhysTick, and the engine
  // broadcasts OnPreExit on the game thread. Never freed.
  TArray<FGnssRawSinkLibrary> &GnssRawSinkLibraries()
  {
    static TArray<FGnssRawSinkLibrary> *Libraries = new TArray<FGnssRawSinkLibrary>();
    return *Libraries;
  }

  // Once, at process exit: after the worlds are torn down (EndPlay destroyed
  // the sinks) and before the modules unload. No log lock is held here, so a
  // library may still log while it stops.
  void ShutdownGnssRawSinkLibraries()
  {
    TArray<FGnssRawSinkLibrary> Libraries = MoveTemp(GnssRawSinkLibraries());
    GnssRawSinkLibraries().Reset();
    for (const FGnssRawSinkLibrary &Library : Libraries)
    {
      Library.ShutdownAll();
      FPlatformProcess::FreeDllHandle(Library.Dll);
    }
  }

  // Remembers a library that created a sink, once per path, if it exports
  // shutdown_all.
  void RegisterGnssRawSinkLibrary(const FString &Path)
  {
    TArray<FGnssRawSinkLibrary> &Libraries = GnssRawSinkLibraries();
    if (Libraries.ContainsByPredicate(
            [&Path](const FGnssRawSinkLibrary &Library)
            {
              return Library.Path.Equals(Path, ESearchCase::CaseSensitive);
            }))
    {
      return;
    }
    void *Dll = FPlatformProcess::GetDllHandle(*Path);
    if (Dll == nullptr)
    {
      return;
    }
    const auto ShutdownAll = reinterpret_cast<carla_gnss_raw_sink_shutdown_all_fn>(
        FPlatformProcess::GetDllExport(Dll, TEXT("carla_gnss_raw_sink_shutdown_all")));
    if (ShutdownAll == nullptr)
    {
      // Optional: the library has nothing to stop at exit.
      FPlatformProcess::FreeDllHandle(Dll);
      return;
    }
    static bool bShutdownHooked = false;
    if (!bShutdownHooked)
    {
      FCoreDelegates::OnPreExit.AddStatic(&ShutdownGnssRawSinkLibraries);
      bShutdownHooked = true;
    }
    Libraries.Add(FGnssRawSinkLibrary{Path, Dll, ShutdownAll});
  }
} // namespace
#endif // PLATFORM_LINUX

void AAutowareGnssSensor::StartRawSinkOnce()
{
  if (RawSinkLibrary.IsEmpty() || bRawSinkTried)
  {
    return;
  }
  bRawSinkTried = true;

#if PLATFORM_LINUX
  const UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  if ((Episode != nullptr) && !Episode->bIsPrimaryServer)
  {
    // The primary's sensor feeds the sink; a Multi-GPU mirror opens none.
    UE_LOG(LogCarla, Log, TEXT("%s: raw_sink_library is not opened on a secondary server"),
        *GetName());
    return;
  }
  if (!IsValid(MgrsDataAsset) || MgrsDataAsset->MgrsGridZone.IsEmpty())
  {
    UE_LOG(LogCarla, Error,
        TEXT("%s: raw_sink_library needs the map's MGRS grid zone (MgrsDataAsset in the "
             "world settings), which this map does not have; no raw output for this sensor."),
        *GetName());
    return;
  }

  RawSinkDll = FPlatformProcess::GetDllHandle(*RawSinkLibrary);
  if (RawSinkDll == nullptr)
  {
    UE_LOG(LogCarla, Error,
        TEXT("%s: could not load raw_sink_library '%s'; no raw output for this sensor."),
        *GetName(), *RawSinkLibrary);
    return;
  }
  const auto AbiVersion = reinterpret_cast<carla_gnss_raw_sink_abi_version_fn>(
      FPlatformProcess::GetDllExport(RawSinkDll, TEXT("carla_gnss_raw_sink_abi_version")));
  const auto Create = reinterpret_cast<carla_gnss_raw_sink_create_fn>(
      FPlatformProcess::GetDllExport(RawSinkDll, TEXT("carla_gnss_raw_sink_create")));
  RawSinkPush = reinterpret_cast<carla_gnss_raw_sink_push_fn>(
      FPlatformProcess::GetDllExport(RawSinkDll, TEXT("carla_gnss_raw_sink_push")));
  RawSinkDestroy = reinterpret_cast<carla_gnss_raw_sink_destroy_fn>(
      FPlatformProcess::GetDllExport(RawSinkDll, TEXT("carla_gnss_raw_sink_destroy")));
  if (AbiVersion == nullptr || Create == nullptr || RawSinkPush == nullptr ||
      RawSinkDestroy == nullptr || AbiVersion() != CARLA_GNSS_RAW_SINK_ABI_VERSION)
  {
    UE_LOG(LogCarla, Error,
        TEXT("%s: '%s' is not a GNSS raw sink of ABI version %d; "
             "no raw output for this sensor."),
        *GetName(), *RawSinkLibrary, CARLA_GNSS_RAW_SINK_ABI_VERSION);
    StopRawSink();
    return;
  }

  RawSinkLogName = new FString(GetName());
  RawSinkMgrsOffset = MgrsDataAsset->MgrsOffsetPosition;
  char ErrorText[512] = {0};
  RawSink = Create(TCHAR_TO_UTF8(*RawSinkConfig), TCHAR_TO_UTF8(*MgrsDataAsset->MgrsGridZone),
      &GnssRawSinkLog, RawSinkLogName, ErrorText, sizeof(ErrorText));
  if (RawSink == nullptr)
  {
    UE_LOG(LogCarla, Error,
        TEXT("%s: raw sink '%s' did not start: %s; no raw output for this sensor."),
        *GetName(), *RawSinkLibrary, UTF8_TO_TCHAR(ErrorText));
    StopRawSink();
    return;
  }
  RegisterGnssRawSinkLibrary(RawSinkLibrary);
  UE_LOG(LogCarla, Log, TEXT("%s: raw sink '%s' started (MGRS grid zone %s)"),
      *GetName(), *RawSinkLibrary, *MgrsDataAsset->MgrsGridZone);

  // A sink extrapolates the newest truth only briefly (tens of milliseconds),
  // so it needs a push every frame.
  const float TickInterval = GetActorTickInterval();
  if (TickInterval > 0.05f)
  {
    UE_LOG(LogCarla, Warning,
        TEXT("%s: sensor_tick is %.3f s; the raw sink needs the truth every frame "
             "(sensor_tick 0, or at most 0.05 s)."),
        *GetName(), TickInterval);
  }
#else
  UE_LOG(LogCarla, Error,
      TEXT("%s: raw_sink_library is only supported on Linux; no raw output for this sensor."),
      *GetName());
#endif
}

bool AAutowareGnssSensor::IsStreamEnabledForRos()
{
  // This sensor publishes to ROS 2 itself, not through the stream, so it has
  // to look at the stream's ROS flag the way the client's enable_for_ros /
  // disable_for_ros set it (single-GPU streaming server).
  auto *GameInstance = UCarlaStatics::GetGameInstance(GetWorld());
  if (GameInstance == nullptr)
  {
    return true;
  }
  const auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
  return GameInstance->GetServer().GetStreamingServer().IsEnabledForROS(StreamId);
}

void AAutowareGnssSensor::StopRawSink()
{
  if (RawSink != nullptr && RawSinkDestroy != nullptr)
  {
    // Detaches the sink from its receiver; its log callback is not called
    // after this.
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

void AAutowareGnssSensor::PushRawSinkTruth()
{
#if PLATFORM_LINUX
  // Antenna position (this sensor) in the map frame: Unreal's left-handed cm
  // turned into REP-103 m (y mirrored) plus the MGRS offset, the convention of
  // the ROS 2 pose topic. Unlike that topic, a large map's location is the
  // global one.
  const FVector AntennaLocation = GetActorLocation();
  FVector GlobalLocation = AntennaLocation;
  if (ALargeMapManager *LargeMap = UCarlaStatics::GetLargeMapManager(GetWorld()))
  {
    GlobalLocation = LargeMap->LocalToGlobalLocation(AntennaLocation);
  }

  // The vehicle carrying the antenna (else the sensor itself): its attitude,
  // as the C interface asks (the ROS 2 pose topic carries the sensor's own
  // rotation instead, so a sensor mounted with a rotation differs from it),
  // and the antenna's velocity as the body's translation plus angular
  // velocity x lever arm, which Unreal evaluates about the centre of mass
  // (V + W x (point - CoM)) for a simulating body. A body that does not
  // simulate physics gives its plain velocity and no angular velocity.
  const AActor *Body = GetAttachParentActor();
  if (Body == nullptr)
  {
    Body = this;
  }
  const FRotator Rotation = Body->GetActorRotation();
  FVector VelocityCmps = Body->GetVelocity();
  FVector AngularVelocityRadps = FVector::ZeroVector;
  if (auto *Root = Cast<UPrimitiveComponent>(Body->GetRootComponent());
      (Root != nullptr) && Root->IsSimulatingPhysics())
  {
    VelocityCmps = Root->GetPhysicsLinearVelocityAtPoint(AntennaLocation);
    AngularVelocityRadps = Root->GetPhysicsAngularVelocityInRadians();
  }

  // The CARLA -> REP-103 conversions of the ROS 2 publishers, applied here to
  // the vehicle between the world and the map frame: the TF/pose quaternion,
  // and the IMU's for the angular velocity (an axial vector, so mirroring y
  // negates x and z).
  const carla::ros2::TransformQuaternion Attitude = carla::ros2::TransformFromCarlaRotation(
      0.0f, 0.0f, 0.0f,
      static_cast<float>(Rotation.Pitch),
      static_cast<float>(Rotation.Yaw),
      static_cast<float>(Rotation.Roll));
  const auto AngularVelocity = carla::ros2::GyroToRos(
      static_cast<float>(AngularVelocityRadps.X),
      static_cast<float>(AngularVelocityRadps.Y),
      static_cast<float>(AngularVelocityRadps.Z));

  constexpr double CmToM = 0.01;
  carla_gnss_raw_truth Truth;
  Truth.sim_time_s = GetEpisode().GetElapsedGameTime();
  Truth.position_m[0] = GlobalLocation.X * CmToM + RawSinkMgrsOffset.X;
  Truth.position_m[1] = -GlobalLocation.Y * CmToM + RawSinkMgrsOffset.Y;
  Truth.position_m[2] = GlobalLocation.Z * CmToM + RawSinkMgrsOffset.Z;
  Truth.orientation_xyzw[0] = Attitude.rotation[1];
  Truth.orientation_xyzw[1] = Attitude.rotation[2];
  Truth.orientation_xyzw[2] = Attitude.rotation[3];
  Truth.orientation_xyzw[3] = Attitude.rotation[0];
  Truth.velocity_mps[0] = VelocityCmps.X * CmToM;
  Truth.velocity_mps[1] = -VelocityCmps.Y * CmToM;
  Truth.velocity_mps[2] = VelocityCmps.Z * CmToM;
  for (int Axis = 0; Axis < 3; ++Axis)
  {
    Truth.angular_velocity_radps[Axis] = AngularVelocity[Axis];
  }
  // Never blocks: the sink stamps the wall clock and does its I/O elsewhere.
  RawSinkPush(RawSink, &Truth);
#endif
}

void AAutowareGnssSensor::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AAutowareGnssSensor::PostPhysTick);

  StartRawSinkOnce();
  // Read once per frame: this frame's ROS 2 publish below follows it.
  bRos2StreamEnabled = IsStreamEnabledForRos();
  if (RawSink != nullptr)
  {
    // Independent of the ROS 2 publish below and of its ROS flag.
    PushRawSinkTruth();
  }

  FVector ActorLocation = GetActorLocation();
  ALargeMapManager * LargeMap = UCarlaStatics::GetLargeMapManager(GetWorld());
  if (LargeMap)
  {
    ActorLocation = LargeMap->LocalToGlobalLocation(ActorLocation);
  }
  carla::geom::Location Location = ActorLocation;
  carla::geom::GeoLocation CurrentLocation = CurrentGeoProjection.TransformToGeoLocation(Location);

  LatitudeValue = CurrentLocation.latitude;
  LongitudeValue = CurrentLocation.longitude;
  AltitudeValue = CurrentLocation.altitude;

  if (bApplyNoiseError)
  {
    // Compute the noise for the sensor
    const float LatError = RandomEngine->GetNormalDistribution(0.0f, LatitudeDeviation);
    const float LonError = RandomEngine->GetNormalDistribution(0.0f, LongitudeDeviation);
    const float AltError = RandomEngine->GetNormalDistribution(0.0f, AltitudeDeviation);

    // Apply the noise to the sensor
    LatitudeValue += LatitudeBias + LatError;
    LongitudeValue += LongitudeBias + LonError;
    AltitudeValue += AltitudeBias + AltError;
  }

  auto DataStream = GetDataStream(*this);

  // ROS2
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
    const FTransform SensorWorldTransform = GetActorTransform();
    double MgrsOffsetPosition[3] = {0.0, 0.0, 0.0};
    if (MgrsDataAsset)
    {
      MgrsOffsetPosition[0] = static_cast<double>(MgrsDataAsset->MgrsOffsetPosition.X);
      MgrsOffsetPosition[1] = static_cast<double>(MgrsDataAsset->MgrsOffsetPosition.Y);
      MgrsOffsetPosition[2] = static_cast<double>(MgrsDataAsset->MgrsOffsetPosition.Z);
    }

    if (ParentActor)
    {
      FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
      ROS2->ProcessDataFromAutowareGNSS(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, carla::geom::GeoLocation{LatitudeValue, LongitudeValue, AltitudeValue}, SensorWorldTransform, MgrsOffsetPosition, this);
    }
    else
    {
      ROS2->ProcessDataFromAutowareGNSS(DataStream.GetSensorType(), StreamId, DataStream.GetSensorTransform(), carla::geom::GeoLocation{LatitudeValue, LongitudeValue, AltitudeValue}, SensorWorldTransform, MgrsOffsetPosition, this);
    }
  }
  #endif

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("AAutowareGnssSensor Stream Send");
    DataStream.SerializeAndSend(*this, carla::geom::GeoLocation{LatitudeValue, LongitudeValue, AltitudeValue});
  }
}

void AAutowareGnssSensor::LoadMgrsData()
{
  if (MgrsDataAsset)
  {
    return; // Already loaded
  }

  if (const auto* WS = Cast<AAutowareWorldSettings>(GetWorld()->GetWorldSettings()))
  {
    if (WS->MgrsDataAssetSoftPtr.IsNull())
    {
      UE_LOG(LogCarla, Warning, TEXT("MGRS Data Asset SoftPtr not set in WorldSettings."));
      return;
    }

    UMgrsDataAsset* Data = WS->MgrsDataAssetSoftPtr.Get();
    if (!IsValid(Data))
    {
      Data = WS->MgrsDataAssetSoftPtr.LoadSynchronous();
    }

    MgrsDataAsset = Data;
  }
}

void AAutowareGnssSensor::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetAutowareGnss(ActorDescription, this);

  RawSinkLibrary = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
      TEXT("raw_sink_library"), ActorDescription.Variations, TEXT(""));
  RawSinkConfig = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
      TEXT("raw_sink_config"), ActorDescription.Variations, TEXT(""));
}

void AAutowareGnssSensor::SetLatitudeDeviation(float Value)
{
  LatitudeDeviation = Value;
}

void AAutowareGnssSensor::SetLongitudeDeviation(float Value)
{
  LongitudeDeviation = Value;
}

void AAutowareGnssSensor::SetAltitudeDeviation(float Value)
{
  AltitudeDeviation = Value;
}

void AAutowareGnssSensor::SetLatitudeBias(float Value)
{
  LatitudeBias = Value;
}

void AAutowareGnssSensor::SetLongitudeBias(float Value)
{
  LongitudeBias = Value;
}

void AAutowareGnssSensor::SetAltitudeBias(float Value)
{
  AltitudeBias = Value;
}

float AAutowareGnssSensor::GetLatitudeDeviation() const
{
  return LatitudeDeviation;
}
float AAutowareGnssSensor::GetLongitudeDeviation() const
{
  return LongitudeDeviation;
}
float AAutowareGnssSensor::GetAltitudeDeviation() const
{
  return AltitudeDeviation;
}

float AAutowareGnssSensor::GetLatitudeBias() const
{
  return LatitudeBias;
}
float AAutowareGnssSensor::GetLongitudeBias() const
{
  return LongitudeBias;
}
float AAutowareGnssSensor::GetAltitudeBias() const
{
  return AltitudeBias;
}

double AAutowareGnssSensor::GetLatitudeValue() const
{
  return LatitudeValue;
}

double AAutowareGnssSensor::GetLongitudeValue() const
{
  return LongitudeValue;
}

double AAutowareGnssSensor::GetAltitudeValue() const
{
  return AltitudeValue;
}

bool AAutowareGnssSensor::IsNoiseErrorEnabled() const
{
  return bApplyNoiseError;
}

void AAutowareGnssSensor::SetNoiseErrorActive(const bool bEnabled)
{
  bApplyNoiseError = bEnabled;
}

void AAutowareGnssSensor::GetMirrorState(carla::multigpu::SensorState &State) const
{
  if (RandomEngine != nullptr)
  {
    State.random_engine = RandomEngine->GetState();
  }
}

void AAutowareGnssSensor::SetMirrorState(const carla::multigpu::SensorState &State)
{
  if (State.random_engine && (RandomEngine != nullptr))
  {
    RandomEngine->SetState(*State.random_engine);
  }
}
