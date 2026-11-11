// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "DefaultMovementComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

void UDefaultMovementComponent::CreateDefaultMovementComponent(ACarlaWheeledVehicle* Vehicle)
{
  UDefaultMovementComponent* DefaultMovementComponent = NewObject<UDefaultMovementComponent>(Vehicle);
  Vehicle->SetCarlaMovementComponent(DefaultMovementComponent);
  DefaultMovementComponent->RegisterComponent();
}

void UDefaultMovementComponent::BeginPlay()
{
  Super::BeginPlay();
}

void UDefaultMovementComponent::ProcessControl(FVehicleControl &Control)
{
  auto *MovementComponent = CarlaVehicle->GetChaosWheeledVehicleMovementComponent();
  // At high tick rates Chaos can sleep a slowly accelerating chassis again
  // before it clears the material's velocity threshold. Keep it awake under
  // throttle, and only write the sleep policy when the throttle state changes.
  const bool bThrottleApplied = Control.Throttle > 0.0f;
  if (bThrottleApplied != bWasThrottleApplied)
  {
    if (FBodyInstance* Body = CarlaVehicle->GetMesh()->GetBodyInstance())
    {
      if (const FPhysicsActorHandle PhysicsActor = Body->GetPhysicsActor())
      {
        FPhysicsCommand::ExecuteWrite(PhysicsActor, [this, bThrottleApplied](const FPhysicsActorHandle& Actor)
        {
          auto& Particle = Actor->GetGameThreadAPI();
          if (bThrottleApplied)
          {
            if (Particle.SleepType() != Chaos::ESleepType::NeverSleep)
            {
              PreviousSleepType = Particle.SleepType();
              Particle.SetSleepType(Chaos::ESleepType::NeverSleep);
            }
          }
          else if (PreviousSleepType.IsSet())
          {
            // Restore only our override; preserve policies set by other code.
            if (Particle.SleepType() == Chaos::ESleepType::NeverSleep)
            {
              Particle.SetSleepType(PreviousSleepType.GetValue());
            }
            PreviousSleepType.Reset();
          }
        });
        bWasThrottleApplied = bThrottleApplied;
      }
    }
  }
  if (bThrottleApplied)
  {
    MovementComponent->SetSleeping(false);
  }
  MovementComponent->SetThrottleInput(Control.Throttle);
  MovementComponent->SetSteeringInput(Control.Steer);
  MovementComponent->SetBrakeInput(Control.Brake);
  MovementComponent->SetHandbrakeInput(Control.bHandBrake);

  if (CarlaVehicle->GetVehicleControl().bReverse != Control.bReverse)
  {
    MovementComponent->SetUseAutomaticGears(!Control.bReverse);
    MovementComponent->SetTargetGear(Control.bReverse ? -1 : 1, true);
  }
  else
  {
    MovementComponent->SetUseAutomaticGears(!Control.bManualGearShift);
    if (Control.bManualGearShift)
    {
      MovementComponent->SetTargetGear(Control.Gear, true);
    }
  }
  Control.Gear = MovementComponent->GetCurrentGear();
}

// FVector GetVelocity() const override;

int32 UDefaultMovementComponent::GetVehicleCurrentGear() const
{
  return CarlaVehicle->GetVehicleMovementComponent()->GetCurrentGear();
}

float UDefaultMovementComponent::GetVehicleForwardSpeed() const
{
  return CarlaVehicle->GetVehicleMovementComponent()->GetForwardSpeed();
}
