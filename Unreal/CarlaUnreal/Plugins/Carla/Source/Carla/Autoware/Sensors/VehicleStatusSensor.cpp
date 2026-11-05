// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Autoware/Sensors/VehicleStatusSensor.h"

#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleLightState.h"
#include "Engine/World.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Game/CarlaEngine.h"

namespace
{
// Defaults describe the Odaiba anchor used to convert the CARLA world pose of
// the ego into the Autoware (lanelet) map frame. Spawners such as
// carla_bridge.py override them per spawn via the sensor attributes below.
constexpr double DefaultReferenceMapX = 89626.180;
constexpr double DefaultReferenceMapY = 42257.898;
constexpr double DefaultReferenceMapZ = 6.4475;
constexpr double DefaultReferenceMapYawRad = 2.124242444006411;
constexpr double DefaultReferenceCarlaBaseX = -2382.261356801974;
constexpr double DefaultReferenceCarlaBaseY = 3077.154881891110;
constexpr double DefaultReferenceCarlaBaseZ = 10.9;
constexpr double DefaultReferenceCarlaBaseYawRad = -2.121702995108797;
constexpr double DefaultMapToCarlaScale = 1.0001113293488773;
constexpr double DefaultMapToCarlaXyYawRad = 0.00020967531865156985;
constexpr double DefaultMapToCarlaYawRad = 0.0011632706731004028;

FActorVariation MakeBoolVariation(const TCHAR* Name, const TCHAR* DefaultValue)
{
  FActorVariation Variation;
  Variation.Id = Name;
  Variation.Type = EActorAttributeType::Bool;
  Variation.RecommendedValues = {DefaultValue};
  Variation.bRestrictToRecommended = false;
  return Variation;
}

FActorVariation MakeFloatVariation(const TCHAR* Name, const double DefaultValue)
{
  FActorVariation Variation;
  Variation.Id = Name;
  Variation.Type = EActorAttributeType::Float;
  Variation.RecommendedValues = {FString::SanitizeFloat(DefaultValue)};
  Variation.bRestrictToRecommended = false;
  return Variation;
}
} // namespace

AVehicleStatusSensor::AVehicleStatusSensor(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PostPhysics;  // read ground-truth after physics update

  TargetRateHz = FMath::Clamp(TargetRateHz, MinRateHz, MaxRateHz);
  PrimaryActorTick.TickInterval = 1.0f / TargetRateHz;
}

FActorDefinition AVehicleStatusSensor::GetSensorDefinition()
{
  using namespace carla::rpc;

  FActorDefinition Definition;

  Definition.UId = 0;  // Carla usually ignores, spawner sets it
  Definition.Id = TEXT("sensor.other.vehicle_status");
  Definition.Class = StaticClass();
  Definition.Tags = TEXT("sensor,other,vehicle_status");

  // Optional attributes exposed in Python API
  {
    FActorAttribute SpeedUnits;
    SpeedUnits.Id = TEXT("speed_units");
    SpeedUnits.Type = EActorAttributeType::String;
    SpeedUnits.Value = TEXT("mps");
    Definition.Attributes.Emplace(MoveTemp(SpeedUnits));
  }

  Definition.Variations.Append({
    MakeBoolVariation(TEXT("publish_autoware_localization_ground_truth"), TEXT("false")),
    MakeFloatVariation(TEXT("reference_map_x"), DefaultReferenceMapX),
    MakeFloatVariation(TEXT("reference_map_y"), DefaultReferenceMapY),
    MakeFloatVariation(TEXT("reference_map_z"), DefaultReferenceMapZ),
    MakeFloatVariation(TEXT("reference_map_yaw_rad"), DefaultReferenceMapYawRad),
    MakeFloatVariation(TEXT("reference_carla_base_x"), DefaultReferenceCarlaBaseX),
    MakeFloatVariation(TEXT("reference_carla_base_y"), DefaultReferenceCarlaBaseY),
    MakeFloatVariation(TEXT("reference_carla_base_z"), DefaultReferenceCarlaBaseZ),
    MakeFloatVariation(TEXT("reference_carla_base_yaw_rad"), DefaultReferenceCarlaBaseYawRad),
    MakeFloatVariation(TEXT("map_to_carla_scale"), DefaultMapToCarlaScale),
    MakeFloatVariation(TEXT("map_to_carla_xy_yaw_rad"), DefaultMapToCarlaXyYawRad),
    MakeFloatVariation(TEXT("map_to_carla_yaw_rad"), DefaultMapToCarlaYawRad),
  });

  return Definition;
}

void AVehicleStatusSensor::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  const auto& Attributes = ActorDescription.Variations;
  bPublishAutowareLocalizationGroundTruth =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToBool(
          TEXT("publish_autoware_localization_ground_truth"),
          Attributes,
          false);
  ReferenceMapX = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("reference_map_x"), Attributes, DefaultReferenceMapX);
  ReferenceMapY = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("reference_map_y"), Attributes, DefaultReferenceMapY);
  ReferenceMapZ = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("reference_map_z"), Attributes, DefaultReferenceMapZ);
  ReferenceMapYawRad = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("reference_map_yaw_rad"), Attributes, DefaultReferenceMapYawRad);
  ReferenceCarlaBaseX = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("reference_carla_base_x"), Attributes, DefaultReferenceCarlaBaseX);
  ReferenceCarlaBaseY = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("reference_carla_base_y"), Attributes, DefaultReferenceCarlaBaseY);
  ReferenceCarlaBaseZ = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("reference_carla_base_z"), Attributes, DefaultReferenceCarlaBaseZ);
  ReferenceCarlaBaseYawRad = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("reference_carla_base_yaw_rad"), Attributes, DefaultReferenceCarlaBaseYawRad);
  MapToCarlaScale = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("map_to_carla_scale"), Attributes, DefaultMapToCarlaScale);
  MapToCarlaXyYawRad = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("map_to_carla_xy_yaw_rad"), Attributes, DefaultMapToCarlaXyYawRad);
  MapToCarlaYawRad = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      TEXT("map_to_carla_yaw_rad"), Attributes, DefaultMapToCarlaYawRad);
}

void AVehicleStatusSensor::BeginPlay()
{
  Super::BeginPlay();
  UE_LOG(LogTemp, Warning, TEXT("VehicleStatusSensor spawned."));
  GetWorldTimerManager().SetTimer(CheckParentTimerHandle, this, &AVehicleStatusSensor::CheckForParentVehicle, 1.0f, true);
}

void AVehicleStatusSensor::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
  Super::PostPhysTick(World, TickType, DeltaSeconds);
  CollectAndStream(DeltaSeconds);
}

void AVehicleStatusSensor::CheckForParentVehicle()
{
  if (OwningVehicle = FindAttachmentParentVehicle(); OwningVehicle.IsValid())
  {
    UE_LOG(LogTemp, Warning, TEXT("Attached to a vehicle: %s"), *OwningVehicle->GetName());
    GetWorldTimerManager().ClearTimer(CheckParentTimerHandle);
  }
}

TObjectPtr<ACarlaWheeledVehicle> AVehicleStatusSensor::FindAttachmentParentVehicle() const
{
  for (AActor* Parent = GetAttachParentActor(); Parent; Parent = Parent->GetAttachParentActor())
  {
    if (ACarlaWheeledVehicle* Vehicle = Cast<ACarlaWheeledVehicle>(Parent))
    {
      return Vehicle;
    }
  }
  return nullptr;
}

void AVehicleStatusSensor::CollectAndStream(float /*DeltaSeconds*/)
{
  if (!OwningVehicle.IsValid())
  {
    return;
  }

  TObjectPtr<ACarlaWheeledVehicle> Vehicle = OwningVehicle.Get();
  if (!IsValid(Vehicle))
  {
      return;
  }

  // Update cached velocity info
  SetVelocityInfoToLocal(Vehicle);

  // Get Max Steering
  const auto MaxSteerAngleInRadians = FMath::DegreesToRadians(Vehicle->GetMaximumSteerAngle());
  
  // Control flags
  const auto& Control = Vehicle->GetVehicleControl();
  const auto Flags = (Control.bReverse ? 0x01 : 0) | (Control.bManualGearShift ? 0x02 : 0);

  // Turn mask
  const auto& Lights = Vehicle->GetVehicleLightState();
  const bool bHazard = Lights.LeftBlinker && Lights.RightBlinker;
  const auto TurnMask = (Lights.LeftBlinker ? 0x01 : 0) | (Lights.RightBlinker ? 0x02 : 0) | (bHazard ? 0x04 : 0);

  FVehicleStatusData Msg
  {
    static_cast<double>(GetWorld()->GetTimeSeconds()),
    static_cast<float>(VelocityInfo.GetSpeed()),
    static_cast<float>(VelocityInfo.Velocity.X),
    static_cast<float>(VelocityInfo.Velocity.Y),
    static_cast<float>(VelocityInfo.Velocity.Z),
    static_cast<float>(VelocityInfo.AngularVelocity.X),
    static_cast<float>(VelocityInfo.AngularVelocity.Y),
    static_cast<float>(VelocityInfo.AngularVelocity.Z),
    static_cast<float>(Vehicle->GetActorRotation().Pitch),
    static_cast<float>(Vehicle->GetActorRotation().Yaw),
    static_cast<float>(Vehicle->GetActorRotation().Roll),
    static_cast<float>(Vehicle->GetVehicleControl().Steer * MaxSteerAngleInRadians),
    static_cast<int32_t>(Vehicle->GetVehicleCurrentGear()),
    static_cast<uint8_t>(TurnMask),
    static_cast<uint8_t>(Flags),
  };
 
  // Serialize message
  constexpr int32 MsgSize = sizeof(FVehicleStatusData);
  TArray<uint8> Buffer;
  Buffer.SetNumUninitialized(MsgSize);
  FMemory::Memcpy(Buffer.GetData(), &Msg, MsgSize);

  // UE client streaming
  if (AreClientsListening())
  {
      ASensor::SendDataToClient(
          *this,
          TArrayView<uint8>(Buffer),
          FCarlaEngine::GetFrameCounter());
  }

  // ROS2 forwarding
#if defined(WITH_ROS2)
  if (auto ROS2 = carla::ros2::ROS2::GetInstance(); ROS2->IsEnabled())
  {
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    carla::ros2::AutowareLocalizationConfig LocalizationConfig;
    LocalizationConfig.enabled = bPublishAutowareLocalizationGroundTruth;
    LocalizationConfig.reference_map_x = ReferenceMapX;
    LocalizationConfig.reference_map_y = ReferenceMapY;
    LocalizationConfig.reference_map_z = ReferenceMapZ;
    LocalizationConfig.reference_map_yaw = ReferenceMapYawRad;
    LocalizationConfig.reference_carla_base_x = ReferenceCarlaBaseX;
    LocalizationConfig.reference_carla_base_y = ReferenceCarlaBaseY;
    LocalizationConfig.reference_carla_base_z = ReferenceCarlaBaseZ;
    LocalizationConfig.reference_carla_base_yaw = ReferenceCarlaBaseYawRad;
    LocalizationConfig.map_to_carla_scale = MapToCarlaScale;
    LocalizationConfig.map_to_carla_xy_yaw = MapToCarlaXyYawRad;
    LocalizationConfig.map_to_carla_yaw = MapToCarlaYawRad;
    ROS2->ProcessDataFromStatusSensor(
      0,
      StreamId,
      GetActorTransform(),
      Msg,
      LocalizationConfig,
      Vehicle,
      this
  );
  }
#endif
}

void AVehicleStatusSensor::SetVelocityInfoToLocal(const AActor* VehicleActor)
{
  if (!VehicleActor)
  {
    return;
  }

  // World linear velocity (cm/s)
  const FVector WorldVel_cmps = VehicleActor->GetVelocity();

  // Convert to m/s, then rotate into local space
  const FQuat InvRot = VehicleActor->GetActorTransform().GetRotation().Inverse();
  VelocityInfo.Velocity = InvRot.RotateVector(CmpsToMps(WorldVel_cmps));

  // Physics angular velocity (rad/s, world space)
  FVector WorldAngVel = FVector::ZeroVector;
  if (UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(VehicleActor->GetRootComponent()))
  {
    if (RootPrim->IsSimulatingPhysics())
    {
      WorldAngVel = RootPrim->GetPhysicsAngularVelocityInRadians();
    }
  }

  // Rotate into local space
  VelocityInfo.AngularVelocity = InvRot.RotateVector(WorldAngVel);

  // Convert angular velocity vector into a Rotator for convenience
  VelocityInfo.RotationRate = VelocityInfo.AngularVelocity.Rotation();
}
