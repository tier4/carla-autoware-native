// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Sensor/Sensor.h"
#include "VehicleStatusSensor.generated.h"

#pragma pack(push, 1)
struct FVehicleStatusMessage
{
	double Timestamp;
	float SpeedMps;
	FVector LocalVelocity;     // X, Y, Z
	FVector LocalAngularVel;   // X, Y, Z
	FRotator LocalRotationRate;
	float Steer;
	int32 Gear;
	uint8 TurnMask;
	uint8 ControlFlags;
	uint8 _Pad0 = 0;
	uint8 _Pad1 = 0;
};
#pragma pack(pop)

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
