// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "FGeoLocation.generated.h"

USTRUCT(BlueprintType)
struct FGeoLocation
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Geolocation")
	double Latitude = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Geolocation")
	double Longitude = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Geolocation")
	double Altitude = 0.0;

	FGeoLocation() {}

	FGeoLocation(double InLat, double InLon, double InAlt)
		: Latitude(InLat), Longitude(InLon), Altitude(InAlt) {}
};
