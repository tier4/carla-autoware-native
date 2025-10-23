// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Data/MgrsDataAsset.h"
#include "GameFramework/WorldSettings.h"
#include "AutowareWorldSettings.generated.h"

/**
 * 
 */
UCLASS()
class CARLAAUTOWAREBRIDGE_API AAutowareWorldSettings : public AWorldSettings
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MGRS", meta = (DisplayName = "Mgrs Data Asset"))
	TSoftObjectPtr<UMgrsDataAsset> MgrsDataAssetSoftPtr;
};
