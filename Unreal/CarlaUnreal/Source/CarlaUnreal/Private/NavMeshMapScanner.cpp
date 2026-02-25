#include "NavMeshMapScanner.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"
#include "NavigationSystem.h"
#include "NavMesh/NavMeshBoundsVolume.h"
#include "NavMesh/RecastNavMesh.h"
#include "Editor.h"
#include "FileHelpers.h"

void UNavMeshMapScanner::StartScan(bool bIsRuntime)
{
    bRuntimeMode = bIsRuntime;

    CollectMaps();
    CurrentIndex = 0;

    FCoreUObjectDelegates::PostLoadMapWithWorld.AddUObject(
        this, &UNavMeshMapScanner::OnMapLoaded);

    LoadNextMap();
}

void UNavMeshMapScanner::CollectMaps()
{
    MapsToScan.Empty();
    ScanResults.Empty();

    FAssetRegistryModule& AssetRegistry =
        FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

    FARFilter Filter;

    // EXACT folder only (no subfolders)
    Filter.PackagePaths.Add("/Game/Carla/Maps");
    Filter.bRecursivePaths = false;   // 🔥 IMPORTANT

    Filter.ClassPaths.Add(UWorld::StaticClass()->GetClassPathName());

    TArray<FAssetData> AssetList;
    AssetRegistry.Get().GetAssets(Filter, AssetList);

    for (const FAssetData& Asset : AssetList)
    {
        MapsToScan.Add(Asset.PackageName.ToString());
    }

    UE_LOG(LogTemp, Warning, TEXT("Found %d maps (non-recursive)"), MapsToScan.Num());
}

void UNavMeshMapScanner::LoadNextMap()
{
    if (CurrentIndex >= MapsToScan.Num())
    {
        UE_LOG(LogTemp, Warning, TEXT("Scan Finished."));
        ShowFinalReport();
        return;
    }

    FString MapToLoad = MapsToScan[CurrentIndex];

    UE_LOG(LogTemp, Warning, TEXT("Loading Map: %s"), *MapToLoad);

#if WITH_EDITOR
    if (GEditor)
    {
        FEditorFileUtils::LoadMap(MapToLoad, false, true);
    }
    else
#endif
    {
        if (GEngine)
        {
            UGameplayStatics::OpenLevel(
                GEngine->GetCurrentPlayWorld(),
                FName(*MapToLoad));
        }
    }
}

void UNavMeshMapScanner::OnMapLoaded(UWorld* LoadedWorld)
{
    if (!LoadedWorld) return;

    ScanCurrentWorld(LoadedWorld);

    CurrentIndex++;

    LoadNextMap();
}

void UNavMeshMapScanner::ScanCurrentWorld(UWorld* World)
{
    FNavMeshScanResult Result;
    Result.MapName = World->GetName();

    for (TActorIterator<ANavMeshBoundsVolume> It(World); It; ++It)
    {
        Result.FoundActors.Add(It->GetName());
    }

    for (TActorIterator<ARecastNavMesh> It(World); It; ++It)
    {
        Result.FoundActors.Add(It->GetName());
    }

    if (Result.FoundActors.Num() > 0)
    {
        ScanResults.Add(Result);
    }
}

void UNavMeshMapScanner::ShowFinalReport()
{
    FString FinalMessage;

    if (ScanResults.Num() == 0)
    {
        FinalMessage = TEXT("No NavMesh found in any map.");
    }
    else
    {
        for (const FNavMeshScanResult& Result : ScanResults)
        {
            FinalMessage += FString::Printf(
                TEXT("Map: %s\n"),
                *Result.MapName);

            for (const FString& ActorName : Result.FoundActors)
            {
                FinalMessage += FString::Printf(
                    TEXT("   - %s\n"),
                    *ActorName);
            }

            FinalMessage += TEXT("\n");
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("===== NAVMESH SCAN RESULT =====\n%s"), *FinalMessage);

#if WITH_EDITOR
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            -1,
            15.f,
            FColor::Green,
            FinalMessage);
    }
#else
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            -1,
            15.f,
            FColor::Green,
            FinalMessage);
    }
#endif
}
