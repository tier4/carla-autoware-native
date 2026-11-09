// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Autoware/Game/AutowareGameModeBase.h"

#include "Carla.h"
#include "Carla/Autoware/Game/AutowareWorldSettings.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameInstance.h"
#include "Carla/Game/CarlaStatics.h"

#include <util/disable-ue4-macros.h>
#include <carla/geom/GeoProjection.h>
#include <util/enable-ue4-macros.h>

AAutowareGameModeBase::AAutowareGameModeBase(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
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
    Super::LoadGeoReference(); // Fallback to ACarlaGameModeBase::LoadGeoReference default
    return;
  }

  UE_LOG(LogCarla, Log, TEXT("Autoware Settings fetch succeeded."));
  auto* Data = WS->MgrsDataAssetSoftPtr.LoadSynchronous();

  if (!IsValid(Data))
  {
    return;
  }

  // tier4 stored a plain GeoLocation reference on the episode; our branch
  // migrated the georeference to carla::geom::GeoProjection. A Transverse
  // Mercator projection centered on the asset's reference point over the
  // default (spherical) ellipsoid reproduces the legacy GeoLocation::Transform
  // behavior.
  // tier4: the ellipsoid is a setting (AutowareEllipsoid): "sphere" (default,
  // as above) or "wgs84", which matches Autoware's map projection.
  carla::geom::Ellipsoid MgrsEllipsoid;
  if (const UCarlaGameInstance *GameInstance = UCarlaStatics::GetGameInstance(this))
  {
    const FString &Name = GameInstance->GetCarlaSettings().AutowareEllipsoid;
    if (Name.Equals(TEXT("wgs84"), ESearchCase::IgnoreCase))
    {
      MgrsEllipsoid = carla::geom::Ellipsoid{6378137.0, 298.257223563};
    }
    else if (!Name.IsEmpty() && !Name.Equals(TEXT("sphere"), ESearchCase::IgnoreCase))
    {
      UE_LOG(LogCarla, Error, TEXT("Unknown AutowareEllipsoid '%s' (expected sphere or wgs84); using sphere."), *Name);
    }
  }
  Episode->MapGeoProjection = carla::geom::GeoProjection::Make(
      carla::geom::TransverseMercatorParams(
          Data->GeoReference.Latitude,
          Data->GeoReference.Longitude,
          1.0, 0.0, 0.0,
          MgrsEllipsoid));

  UE_LOG(LogCarla, Log, TEXT("MGRS georeference loaded successfully."));
  StoreSpawnPoints();
}
