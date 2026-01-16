// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "AutowareGameModeBase.h"

#include "AutowareWorldSettings.h"

AAutowareGameModeBase::AAutowareGameModeBase(const FObjectInitializer& ObjectInitializer)
	:Super(ObjectInitializer)
{
}

void AAutowareGameModeBase::InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage)
{
	Super::InitGame(MapName, Options, ErrorMessage);
}

void AAutowareGameModeBase::LoadGeoReference()
{
	auto* WS = Cast<AAutowareWorldSettings>(GetWorld()->GetWorldSettings());

	if (!IsValid(WS))
	{
		Super::LoadGeoReference(); // Fallback to CarlaGameMode::LoadGeoReference default
		return;
	}
	
	UE_LOG(LogCarla, Warning, TEXT("Autoware Settings fetch succeded."));
	auto* Data = WS->MgrsDataAssetSoftPtr.LoadSynchronous();

	if (!IsValid(Data))
	{
		return;
	}
	
	carla::geom::GeoLocation GeoReference
	(
		Data->GeoReference.Latitude,
		Data->GeoReference.Longitude,
		Data->GeoReference.Altitude
	);
	Episode->MapGeoReference = GeoReference;

	UE_LOG(LogCarla, Warning, TEXT("MGRS Offset loaded successfuly."));
	StoreSpawnPoints();
}
