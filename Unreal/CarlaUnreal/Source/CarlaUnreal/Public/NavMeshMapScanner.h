#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "NavMeshMapScanner.generated.h"

/**
 * 
 */

USTRUCT()
struct FNavMeshScanResult
{
	GENERATED_BODY()

	FString MapName;
	TArray<FString> FoundActors;
};

UCLASS()
class CARLAUNREAL_API UNavMeshMapScanner : public UObject
{
	GENERATED_BODY()
	
public:
	void StartScan(bool bIsRuntime);

private:
	void CollectMaps();
	void LoadNextMap();
	void OnMapLoaded(UWorld* LoadedWorld);
	void ScanCurrentWorld(UWorld* World);
	void ShowFinalReport();

	TArray<FString> MapsToScan;
	TArray<FNavMeshScanResult> ScanResults;
	int32 CurrentIndex = 0;
	bool bRuntimeMode = false;
};
