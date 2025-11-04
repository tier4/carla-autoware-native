// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Sensor/GnssSensor.h"
#include "Autoware/Data/MgrsDataAsset.h"
#include "AutowareGnssSensor.generated.h"

UCLASS()
class CARLA_API AAutowareGnssSensor : public AGnssSensor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AAutowareGnssSensor(const FObjectInitializer& ObjectInitializer);
	
	// Temporary until AutowareGameModeBase is not valid to use. Later on swap to get this value from game mode.
	UFUNCTION(BlueprintCallable)
	void LoadMgrsData();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	// virtual void PrePhysTick(float DeltaSeconds) override;
	virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds) override;
	virtual carla::geom::GeoLocation ComputeGeoLocation() const override;

private:
	UPROPERTY()
	TObjectPtr<UMgrsDataAsset> MgrsDataAsset = nullptr;
};