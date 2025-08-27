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
  Parent = GetAttachParentActor();
  ResolveVehicle();
}

void AVehicleStatusSensor::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
  Super::PostPhysTick(World, TickType, DeltaSeconds);
  CollectAndStream(DeltaSeconds);
}

bool AVehicleStatusSensor::ResolveVehicle()
{
  if (!Parent.IsValid())
  {
    return false;
  }
  
  if (Vehicle.IsValid())
  {
    return true;
  }

  AActor* ParentActor = Parent.Get();
  if (!ParentActor)
  {
    ParentActor = GetAttachParentActor();
  }
  
  if (!ParentActor)
  {
    return false;
  }

  // Try both common vehicle types
  if (auto* WV = Cast<ACarlaWheeledVehicle>(ParentActor))
  {
    Vehicle = WV;
    return true;
  }
  
  if (auto* CWV = Cast<ACarlaWheeledVehicle>(ParentActor))
  {
    Vehicle = CWV;
    return true;
  }

  return false;
}

void AVehicleStatusSensor::CollectAndStream(float /*DeltaSeconds*/)
{
  if (!ResolveVehicle())
  {
    return;
  }

  AActor* VehicleActor = Vehicle.Get();
  if (!VehicleActor)
  {
    return;
  }

  // Update cached velocity info
  SetVelocityInfo(VehicleActor);

  // Steering & control
  float steer = 0.0f;
  bool manual_gear = false;
  bool reverse = false;
  int32 current_gear = 0;

  if (auto* CWV = Cast<ACarlaWheeledVehicle>(VehicleActor))
  {
    const auto Control = CWV->GetVehicleControl();
    steer       = Control.Steer;
    reverse     = Control.bReverse;
    manual_gear = Control.bManualGearShift;
    current_gear = CWV->GetVehicleCurrentGear();
  }

  // Lights / indicators
  bool left_blinker = false, right_blinker = false, hazard = false;
  if (auto* CWV = Cast<ACarlaWheeledVehicle>(VehicleActor)) {
    const FVehicleLightState Lights = CWV->GetVehicleLightState();
    left_blinker  = Lights.LeftBlinker;
    right_blinker = Lights.RightBlinker;
    hazard = left_blinker && right_blinker;
  }

  // Pack into message - matching LibCarla serializer
  struct Packed
  {
    double timestamp;
    float speed_mps;
    float vel_x_mps, vel_y_mps, vel_z_mps; // local
    float angVel_x_mps, angVel_y_mps, angVel_z_mps; // local
    float rotr_pitch, rotr_yaw, rotr_roll; // local
    float steer;
    int32 gear;
    uint8 turn_mask;
    uint8 control_flags;
    uint8 _pad0;
    uint8 _pad1;
  } PACKED;

  Packed msg{};
  msg.timestamp = GetWorld()->GetTimeSeconds();
  msg.speed_mps = VelocityInfo.GetSpeed();
  msg.vel_x_mps = VelocityInfo.LocalVelocity.X;
  msg.vel_y_mps = VelocityInfo.LocalVelocity.Y;
  msg.angVel_x_mps = VelocityInfo.LocalAngularVelocity.X;
  msg.angVel_y_mps = VelocityInfo.LocalAngularVelocity.Y;
  msg.angVel_z_mps = VelocityInfo.LocalAngularVelocity.Z;
  msg.rotr_pitch = VelocityInfo.LocalRotationRate.Pitch;
  msg.rotr_yaw = VelocityInfo.LocalRotationRate.Yaw;
  msg.rotr_roll = VelocityInfo.LocalRotationRate.Roll;
  msg.steer = steer;
  msg.gear = current_gear;
  msg.turn_mask   = (left_blinker ? 0x01 : 0) | (right_blinker ? 0x02 : 0) | (hazard ? 0x04 : 0);
  msg.control_flags = (reverse ? 0x01 : 0) | (manual_gear ? 0x02 : 0);

  // Send via Carla UE5 API
  if (AreClientsListening())
  {
    TArray<uint8> Buffer;
    Buffer.SetNumUninitialized(sizeof(Packed));
    FMemory::Memcpy(Buffer.GetData(), &msg, sizeof(Packed));

    ASensor::SendDataToClient(
        *this,
        TArrayView<uint8>(Buffer),
        FCarlaEngine::GetFrameCounter());
  }
}

void AVehicleStatusSensor::SetVelocityInfo(const AActor* VehicleActor)
{
  if (!VehicleActor) return;

  // World linear velocity (cm/s)
  const FVector WorldVel_cmps = VehicleActor->GetVelocity();

  // Convert to m/s, then rotate into local space
  const FQuat InvRot = VehicleActor->GetActorTransform().GetRotation().Inverse();
  VelocityInfo.LocalVelocity = InvRot.RotateVector(CmpsToMps(WorldVel_cmps));

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
  VelocityInfo.LocalAngularVelocity = InvRot.RotateVector(WorldAngVel);

  // Convert angular velocity vector into a Rotator for convenience
  VelocityInfo.LocalRotationRate = VelocityInfo.LocalAngularVelocity.Rotation();
}
