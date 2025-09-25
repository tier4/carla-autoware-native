// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/GnssSensor.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/MapGen/LargeMapManager.h"

#include <util/disable-ue4-macros.h>
#include "carla/geom/Vector3D.h"
#include "carla/ros2/ROS2.h"
#include <util/enable-ue4-macros.h>

#include "Autoware/Game/AutowareWorldSettings.h"

AGnssSensor::AGnssSensor(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
}

FActorDefinition AGnssSensor::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeGnssDefinition();
}

void AGnssSensor::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  UActorBlueprintFunctionLibrary::SetGnss(ActorDescription, this);
}

void AGnssSensor::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AGnssSensor::PostPhysTick);

  if (!MgrsDataAsset)
  {
    return;
  }
  
  FVector ActorLocation = GetActorLocation();
  
  // Apply MGRS offset (Unity equivalent)
  ActorLocation += MgrsDataAsset->MgrsOffsetPosition;
  const auto& OriginGeo = MgrsDataAsset->WorldOriginGeoCoordinate;
  
  carla::geom::Location Location{static_cast<float>(ActorLocation.X), static_cast<float>(ActorLocation.Y), static_cast<float>(ActorLocation.Z)};
  carla::geom::GeoLocation GeoOrigin(OriginGeo.Latitude, OriginGeo.Longitude, OriginGeo.Altitude);

  // Transform relative to world origin
  carla::geom::GeoLocation CurrentLocation;
  CurrentLocation.latitude  = OriginGeo.Latitude  + Location.y * 1e-5; // scale factor depends on conversion logic idk which is correct, keeping the previous in carla
  CurrentLocation.longitude = OriginGeo.Longitude + Location.x * 1e-5;
  CurrentLocation.altitude  = OriginGeo.Altitude  + Location.z;

  // bias + noise
  const float LatError = RandomEngine->GetNormalDistribution(0.0f, LatitudeDeviation);
  const float LonError = RandomEngine->GetNormalDistribution(0.0f, LongitudeDeviation);
  const float AltError = RandomEngine->GetNormalDistribution(0.0f, AltitudeDeviation);

  LatitudeValue = CurrentLocation.latitude + LatitudeBias + LatError;
  LongitudeValue = CurrentLocation.longitude + LongitudeBias + LonError;
  AltitudeValue = CurrentLocation.altitude + AltitudeBias + AltError;
  
  auto DataStream = GetDataStream(*this);
  carla::geom::GeoLocation OutData{LatitudeValue, LongitudeValue, AltitudeValue};

  #if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    const FTransform sensor_world_transform = GetActorTransform();

    if (AActor* ParentActor = GetAttachParentActor())
    {
      FTransform LocalTransformRelativeToParent =
          GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());

      ROS2->ProcessDataFromGNSS(DataStream.GetSensorType(), StreamId,
                                LocalTransformRelativeToParent, OutData,
                                sensor_world_transform, this);
    }
    else
    {
      ROS2->ProcessDataFromGNSS(DataStream.GetSensorType(), StreamId,
                                DataStream.GetSensorTransform(), OutData,
                                sensor_world_transform, this);
    }
  }
  #endif

  DataStream.SerializeAndSend(*this, OutData);
}

void AGnssSensor::SetLatitudeDeviation(float Value)
{
  LatitudeDeviation = Value;
}

void AGnssSensor::SetLongitudeDeviation(float Value)
{
  LongitudeDeviation = Value;
}

void AGnssSensor::SetAltitudeDeviation(float Value)
{
  AltitudeDeviation = Value;
}

void AGnssSensor::SetLatitudeBias(float Value)
{
  LatitudeBias = Value;
}

void AGnssSensor::SetLongitudeBias(float Value)
{
  LongitudeBias = Value;
}

void AGnssSensor::SetAltitudeBias(float Value)
{
  AltitudeBias = Value;
}

float AGnssSensor::GetLatitudeDeviation() const
{
  return LatitudeDeviation;
}
float AGnssSensor::GetLongitudeDeviation() const
{
  return LongitudeDeviation;
}
float AGnssSensor::GetAltitudeDeviation() const
{
  return AltitudeDeviation;
}

float AGnssSensor::GetLatitudeBias() const
{
  return LatitudeBias;
}
float AGnssSensor::GetLongitudeBias() const
{
  return LongitudeBias;
}
float AGnssSensor::GetAltitudeBias() const
{
  return AltitudeBias;
}

double AGnssSensor::GetLatitudeValue() const
{
  return LatitudeValue;
}

double AGnssSensor::GetLongitudeValue() const
{
  return LongitudeValue;
}

double AGnssSensor::GetAltitudeValue() const
{
  return AltitudeValue;
}

void AGnssSensor::BeginPlay()
{
  Super::BeginPlay();

  const UCarlaEpisode* episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  CurrentGeoReference = episode->GetGeoReference();

  LoadMgrsData();
}

void AGnssSensor::LoadMgrsData()
{
  if (MgrsDataAsset)
  {
    return;
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
