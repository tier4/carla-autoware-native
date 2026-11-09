// Copyright (c) 2026 TIER IV, Inc.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "Components/ActorComponent.h"
#include "Components/PrimitiveComponent.h"
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include "VehicleAccelerationControl.generated.h"

/// Component that controls the actor with a constant acceleration (directly applied,
/// no pedal input). Similar to VehicleVelocityControl but for acceleration.
UCLASS(Blueprintable, BlueprintType, ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class CARLA_API UVehicleAccelerationControl : public UActorComponent
{
  GENERATED_BODY()

  // ===========================================================================
  /// @name Constructor and destructor
  // ===========================================================================
  /// @{
public:
  UVehicleAccelerationControl();

  /// @}
  // ===========================================================================
  /// @name Get functions
  // ===========================================================================
  /// @{
public:

  void BeginPlay() override;

  virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

  // Activate the component setting the target acceleration
  virtual void Activate(bool bReset=false) override;

  // Activate the component setting the target acceleration (in vehicle local space, e.g. X=forward)
  virtual void Activate(FVector Acceleration, bool bReset=false);

  // Deactivate the component
  virtual void Deactivate() override;

  // tier4: optional shaping of the acceleration command (first-order lag on the
  // target, then a jerk limit on the applied acceleration). Both are off by
  // default; while they are off the component behaves exactly as upstream.

  // Update the target acceleration without resetting the integrated state.
  // Acceleration is in vehicle local space. Units: cm/s^2.
  void SetTargetAcceleration(const FVector& Acceleration);

  // Set jerk limits (rate-of-change of acceleration) for each component.
  // Units: cm/s^3. Use <= 0 to disable the corresponding limit.
  void SetJerkLimit(float InJerkLimitPosCmps3, float InJerkLimitNegCmps3);

  // Set first-order lag time constant on the acceleration target. Units: seconds. Use <= 0 to disable.
  void SetFirstOrderLagTau(float InTauS) { TargetLagTauS = InTauS; }

  // True when a jerk limit or a first-order lag is set.
  bool IsShapingEnabled() const
  {
    return JerkLimitPosCmps3 > 0.0f || JerkLimitNegCmps3 > 0.0f || TargetLagTauS > 0.0f;
  }

private:
  // Target acceleration in vehicle local space (e.g. X forward, Y right, Z up). Units: cm/s^2
  UPROPERTY(Category = "Vehicle Acceleration Control", VisibleAnywhere)
  FVector TargetAcceleration;

  /// Integrated forward speed [cm/s] under our control; lateral component is left to physics for cornering
  float ControlledForwardSpeed = 0.0f;

  UPROPERTY(Transient, DuplicateTransient)
  TObjectPtr<UPrimitiveComponent> PrimitiveComponent;

  UPROPERTY(Transient, DuplicateTransient)
  TObjectPtr<AActor> OwnerVehicle;

  // tier4: target after the optional first-order lag. Units: cm/s^2.
  FVector FilteredTargetAcceleration = FVector::ZeroVector;

  // tier4: acceleration actually applied after the optional jerk limit. Units: cm/s^2.
  FVector AppliedAcceleration = FVector::ZeroVector;

  // tier4: maximum jerk (acceleration rate) limits. Units: cm/s^3. <= 0 disables (default).
  // The Neg limit is a positive magnitude used when the acceleration decreases.
  UPROPERTY(Category = "Vehicle Acceleration Control", EditAnywhere)
  float JerkLimitPosCmps3 = 0.0f;

  UPROPERTY(Category = "Vehicle Acceleration Control", EditAnywhere)
  float JerkLimitNegCmps3 = 0.0f;

  // tier4: first-order lag time constant for the target acceleration. Units: seconds. <= 0 disables (default).
  float TargetLagTauS = 0.0f;

};
