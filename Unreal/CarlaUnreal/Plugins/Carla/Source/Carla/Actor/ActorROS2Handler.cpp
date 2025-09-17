// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "ActorROS2Handler.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/VehicleControl.h"

void ActorROS2Handler::operator()(carla::ros2::VehicleControl &Source)
{
  if (!_Actor) return;

  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(_Actor);
  if (!Vehicle) return;

  // setup control values
  FVehicleControl NewControl;
  NewControl.Throttle = Source.throttle;
  NewControl.Steer = Source.steer;
  NewControl.Brake = Source.brake;
  NewControl.bHandBrake = Source.hand_brake;
  NewControl.bReverse = Source.reverse;
  NewControl.bManualGearShift = Source.manual_gear_shift;
  NewControl.Gear = Source.gear;

  Vehicle->ApplyVehicleControl(NewControl, EVehicleInputPriority::User);
}

void ActorROS2Handler::operator()(carla::ros2::VehicleAckermannControl &Source)
{
  if (!_Actor) return;

  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(_Actor);
  if (!Vehicle) return;

  // setup control values
  FVehicleAckermannControl NewControl;
  NewControl.Steer = Source.steer;
  NewControl.SteerSpeed = Source.steer_speed;
  NewControl.Speed = Source.speed;
  NewControl.Acceleration = Source.acceleration;
  NewControl.Jerk = Source.jerk;

  Vehicle->ApplyVehicleAckermannControl(NewControl, EVehicleInputPriority::User);
}

void ActorROS2Handler::operator()(carla::ros2::MessageControl Message)
{
  if (!_Actor) return;

  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(_Actor);
  if (!Vehicle) return;

  Vehicle->PrintROS2Message(Message.message);

  // FString ROSMessage = Message.message;
  // UE_LOG(LogCarla, Warning, TEXT("ROS2 Message received: %s"), *ROSMessage);
}

bool ActorROS2Handler::FlattenSteeringCurve(AActor * Actor)
{
  if (!Actor) return false;

  ACarlaWheeledVehicle * const Vehicle = Cast<ACarlaWheeledVehicle>(Actor);
  if (!Vehicle) return false;

  UChaosWheeledVehicleMovementComponent * const MovementComponent = Vehicle->GetChaosWheeledVehicleMovementComponent();
  if (!MovementComponent) return false;

  FRichCurve * const Curve = MovementComponent->SteeringSetup.SteeringCurve.GetRichCurve();
  if (!Curve) return false;

  /// @note Flatten steering curve to be always 1.0 to properly map wheel angle to steering at all speeds
  Curve->Reset();
  Curve->AddKey(0.0f,   1.0f);
  Curve->AddKey(40.0f,  1.0f);
  Curve->AddKey(80.0f,  1.0f);
  Curve->AddKey(120.0f, 1.0f);
  Curve->AutoSetTangents();
  std::cerr << "Resetting SteeringCurve!" << std::endl;

  return true;
}
