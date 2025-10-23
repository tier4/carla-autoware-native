// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.


#include "AutowareGnssSensor.h"

#include "Autoware/Game/AutowareWorldSettings.h"


// Sets default values
AAutowareGnssSensor::AAutowareGnssSensor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

// Called when the game starts or when spawned
void AAutowareGnssSensor::BeginPlay()
{
	Super::BeginPlay();
	LoadMgrsData();
}

void AAutowareGnssSensor::PostPhysTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
	Super::PostPhysTick(World, TickType, DeltaSeconds);
}

carla::geom::GeoLocation AAutowareGnssSensor::ComputeGeoLocation() const
{
	if (!MgrsDataAsset)
	{
		// Fallback to parent behavior if asset missing
		return Super::ComputeGeoLocation();
	}

	FVector ActorLocation = GetActorLocation();
	ActorLocation += MgrsDataAsset->MgrsOffsetPosition;
	const auto& OriginGeo = MgrsDataAsset->WorldOriginGeoCoordinate;

	carla::geom::Location Location{
		static_cast<float>(ActorLocation.X),
		static_cast<float>(ActorLocation.Y),
		static_cast<float>(ActorLocation.Z)
	};

	carla::geom::GeoLocation GeoOrigin(OriginGeo.Latitude, OriginGeo.Longitude, OriginGeo.Altitude);

	carla::geom::GeoLocation Result;
	Result.latitude  = OriginGeo.Latitude  + Location.y * 1e-5;
	Result.longitude = OriginGeo.Longitude + Location.x * 1e-5;
	Result.altitude  = OriginGeo.Altitude  + Location.z;

	return Result;
}

void AAutowareGnssSensor::LoadMgrsData()
{
	if (MgrsDataAsset)
	{
		return;	// Already loaded
	}

	if (const auto* WS = Cast<AAutowareWorldSettings>(GetWorld()->GetWorldSettings()))
	{
		if (WS->MgrsDataAssetSoftPtr.IsNull())
		{
			UE_LOG(LogCarla, Warning, TEXT("MGRS Data Asset SoftPtr not set in WorldSettings."));
			return;
		}

		UMgrsDataAsset* Data = WS->MgrsDataAssetSoftPtr.Get();
		if (!IsValid(Data))
		{
			Data = WS->MgrsDataAssetSoftPtr.LoadSynchronous();
		}

		MgrsDataAsset = Data;
	}
}
