// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Game/CarlaGameModeBase.h"
#include "AutowareGameModeBase.generated.h"

/**
 * 
 */
UCLASS()
class CARLA_API AAutowareGameModeBase : public ACarlaGameModeBase
{
	GENERATED_BODY()
	
public:
	AAutowareGameModeBase(const FObjectInitializer& ObjectInitializer);

protected:
	void InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) override;
	virtual void LoadGeoReference() override;
};
