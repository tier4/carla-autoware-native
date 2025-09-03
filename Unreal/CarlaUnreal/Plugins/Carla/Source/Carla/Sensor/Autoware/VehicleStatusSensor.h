// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Sensor/Sensor.h" 
#include "VehicleStatusSensor.generated.h"

//todo: This mimics the same structure used in carla. It can be be replaced with carla::sensor::data::VehicleStatusEvent.
// But it generates problems with serialization and data passing into ros2. I've tried plain struct to be passed, which didn't work either.
// All data are passed by value in ROS2::ProcessDataFromStatusSensor. I'm leaving this struct since in the future it should be used and replace passing by value.
// I'm not using pragma push and pop, since carla struct doesnt have it.
struct FVehicleStatusMessageRaw
{
	double timestamp;
	float speed_mps;
	float vel_x_mps, vel_y_mps, vel_z_mps; // local
	float angVel_x_mps, angVel_y_mps, angVel_z_mps; // local
	float rotr_pitch, rotr_yaw, rotr_roll; // local
	float steer;
	int32_t gear;
	uint8_t turn_mask;
	uint8_t control_flags;
};

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
	
	FString ToString() const
	{
		return FString::Printf(TEXT("Velocity: %s | Angular Velocity: %s | Rotation Rate: %s"),
			*Velocity.ToString(),
			*AngularVelocity.ToString(),
			*RotationRate.ToString());
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
	
	virtual void PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds) override;

protected:
	virtual void BeginPlay() override;
	
	// Publish frequency
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Sensor|Rate", meta=(ClampMin="1.0", UIMin="1.0"))
	float MinRateHz = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Sensor|Rate", meta=(ClampMin="1.0", UIMin="1.0"))
	float MaxRateHz = 1000.0f;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Sensor|Rate", meta=(ClampMin="1.0", ClampMax="1000.0", UIMin="1.0", UIMax="1000.0"))
	float TargetRateHz = 30.0f;

private:
	// Cached parent vehicle
	TWeakObjectPtr<ACarlaWheeledVehicle> OwningVehicle;
	FVelocityInfo VelocityInfo;

	// Find parent helpers
	FTimerHandle CheckParentTimerHandle;
	void CheckForParentVehicle();
	TObjectPtr<ACarlaWheeledVehicle> FindAttachmentParentVehicle() const;

	// Helpers
	void CollectAndStream(float DeltaSeconds);
	void SetVelocityInfoToLocal(const AActor* VehicleActor);

	// Scale UE velocity from cm/s to m/s
	template<typename T>
	static FORCEINLINE T CmpsToMps(const T& v_cmps)
	{
		return v_cmps * 0.01f;
	}
};
