// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#include "Sensor/Autoware/VehicleStatusSensor.h"

#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleLightState.h"  
#include "Engine/World.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Game/CarlaEngine.h"

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

  return Definition;
}

void AVehicleStatusSensor::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
}

void AVehicleStatusSensor::BeginPlay()
{
  Super::BeginPlay();
  UE_LOG(LogTemp, Warning, TEXT("VehicleStatusSensor spawned."));
  GetWorldTimerManager().SetTimer(CheckParentTimerHandle, this, &AVehicleStatusSensor::CheckForParentVehicle, 0.1f, true);
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

  // Pack into message
  FVehicleStatusMessage Msg{};
  Msg.Timestamp       = GetWorld()->GetTimeSeconds();
  Msg.SpeedMps        = VelocityInfo.GetSpeed();
  Msg.LocalVelocity   = VelocityInfo.Velocity;
  Msg.LocalAngularVel = VelocityInfo.AngularVelocity;
  Msg.LocalRotationRate = FRotator(
      VelocityInfo.RotationRate.Pitch,
      VelocityInfo.RotationRate.Yaw,
      VelocityInfo.RotationRate.Roll
  );
  Msg.Steer = Vehicle->GetVehicleControl().Steer;
  Msg.Gear  = Vehicle->GetVehicleCurrentGear();

  // Control flags
  const auto& Control = Vehicle->GetVehicleControl();
  Msg.ControlFlags = (Control.bReverse ? 0x01 : 0) | (Control.bManualGearShift ? 0x02 : 0);

  // Turn mask
  const auto& Lights = Vehicle->GetVehicleLightState();
  const bool bHazard = Lights.LeftBlinker && Lights.RightBlinker;
  Msg.TurnMask = (Lights.LeftBlinker ? 0x01 : 0) | (Lights.RightBlinker ? 0x02 : 0) | (bHazard ? 0x04 : 0);

  // Serialize message
  constexpr int32 MsgSize = sizeof(FVehicleStatusMessage);
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
    std::vector<uint8_t> Raw(Buffer.GetData(), Buffer.GetData() + Buffer.Num());

    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    ROS2->ProcessDataFromStatusSensor(0, StreamId, GetActorTransform(), Raw, this);
    UE_LOG(LogTemp, Warning, TEXT("ros2 data sent"));
  }
#endif
  
}

void AVehicleStatusSensor::SetVelocityInfoToLocal(const AActor* VehicleActor)
{
  if (!VehicleActor) return;

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
