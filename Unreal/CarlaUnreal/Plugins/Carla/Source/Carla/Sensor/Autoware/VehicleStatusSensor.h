// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Sensor/Sensor.h"
#include "VehicleStatusSensor.generated.h"

/**
 * 
 */
UCLASS()
class CARLA_API AVehicleStatusSensor : public ASensor
{
	GENERATED_BODY()

public:
	explicit AVehicleStatusSensor(const FObjectInitializer& ObjectInitializer);

	// Carla blueprint definition (get sensor.other.vehicle_status in BP)
	static FActorDefinition GetSensorDefinition();

	// Called by Carla’s spawning pipeline with user attributes
	void Set(const FActorDescription &ActorDescription) override;

protected:
	virtual void BeginPlay() override;

public:
	// virtual void TickActor(float DeltaTime, ELevelTick TickType, FActorTickFunction& ThisTickFunction) override;
	virtual void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds) override;

private:
	// Cached parent vehicle
	TWeakObjectPtr<AActor> Parent;
	TWeakObjectPtr<ACarlaWheeledVehicle> Vehicle; // AWheeledVehicle

	// Helpers
	bool ResolveVehicle();
	void CollectAndStream(float DeltaSeconds);

	// Scale UE velocity from cm/s to m/s
	static inline float CmpsToMps(float v_cmps) { return v_cmps * 0.01f; }
};
