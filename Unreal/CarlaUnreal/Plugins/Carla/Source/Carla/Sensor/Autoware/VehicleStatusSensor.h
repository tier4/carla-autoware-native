// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Sensor/Sensor.h"
#include "VehicleStatusSensor.generated.h"

USTRUCT(BlueprintType)
struct FVelocityInfo
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity")
	FVector Velocity;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity")
	FRotator RotationRate;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Velocity")
	FVector AngularVelocity;

	FVelocityInfo()
		: Velocity(FVector::ZeroVector)
		, RotationRate(FRotator::ZeroRotator)
		, AngularVelocity(FVector::ZeroVector)
	{
	}

	float GetSpeed() const
	{
		return Velocity.Size();
	}
};

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
	virtual void Set(const FActorDescription &ActorDescription) override;
	
	// virtual void TickActor(float DeltaTime, ELevelTick TickType, FActorTickFunction& ThisTickFunction) override; //todo might need to change into this tick if PostPhysTick() is failing
	virtual void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds) override;

protected:
	virtual void BeginPlay() override;
	
	// Publish frequency
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Sensor")
	float TargetRateHz = 30.0f;

private:
	// Cached parent vehicle
	TWeakObjectPtr<AActor> Parent;
	TWeakObjectPtr<ACarlaWheeledVehicle> Vehicle; // AWheeledVehicle
	FVelocityInfo VelocityInfo;

	// Publish frequency
	float SecondsPerUpdate = 1.0f / TargetRateHz;
	double LastSentTimestamp = 0.0;

	// Helpers
	bool ResolveVehicle();
	void CollectAndStream(float DeltaSeconds);
	void SetVelocityInfoToLocal(const AActor* VehicleActor);

	// Scale UE velocity from cm/s to m/s
	template<typename T>
	static FORCEINLINE T CmpsToMps(const T& v_cmps)
	{
		return v_cmps * 0.01f;
	}
};
