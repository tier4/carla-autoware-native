#pragma once

#include "CoreMinimal.h"
#include "FGeoLocation.h"
#include "Engine/DataAsset.h"
#include "MgrsDataAsset.generated.h"

/**
 * Data Asset containing MGRS-related configuration.
 */
UCLASS(BlueprintType)
class CARLAAUTOWAREBRIDGE_API UMgrsDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MGRS|ID Info")
	FGuid MgrsID = FGuid::NewGuid();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MGRS|ID Info")
	FName MgrsMapName;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MGRS|Data")
	FVector MgrsOffsetPosition;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MGRS|Data")
	FString MgrsGridZone;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MGRS|Data")
	FGeoLocation WorldOriginGeoCoordinate;
};
