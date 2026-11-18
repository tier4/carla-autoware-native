// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/Sensor.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"
#include "Carla/Autoware/Data/MgrsDataAsset.h"

#include <util/disable-ue4-macros.h>
#include <carla/geom/GeoProjection.h>
#include <util/enable-ue4-macros.h>

#include "Carla/Sensor/carla_gnss_raw_sink.h"

#include "AutowareGnssSensor.generated.h"

/// Autoware GNSS sensor ("sensor.other.autoware_gnss"): streams GeoLocation
/// to the client like the regular GNSS sensor, but on the ROS2 side publishes
/// the sensor pose in the map frame (world transform + MGRS offset) as
/// <base>/pose and <base>/pose_with_covariance. Ported from
/// tier4/autoware-support.
UCLASS()
class CARLA_API AAutowareGnssSensor : public ASensor
{
  GENERATED_BODY()

public:

  AAutowareGnssSensor(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  void Set(const FActorDescription &ActorDescription);

  void GetMirrorState(carla::multigpu::SensorState &State) const override;

  void SetMirrorState(const carla::multigpu::SensorState &State) override;

  void SetLatitudeDeviation(float Value);
  void SetLongitudeDeviation(float Value);
  void SetAltitudeDeviation(float Value);

  void SetLatitudeBias(float Value);
  void SetLongitudeBias(float Value);
  void SetAltitudeBias(float Value);

  float GetLatitudeDeviation() const;
  float GetLongitudeDeviation() const;
  float GetAltitudeDeviation() const;

  float GetLatitudeBias() const;
  float GetLongitudeBias() const;
  float GetAltitudeBias() const;

  double GetLatitudeValue() const;
  double GetLongitudeValue() const;
  double GetAltitudeValue() const;

  UFUNCTION(BlueprintCallable)
  bool IsNoiseErrorEnabled() const;

  UFUNCTION(BlueprintCallable)
  void LoadMgrsData();

protected:

  virtual void BeginPlay() override;

  virtual void EndPlay(EEndPlayReason::Type EndPlayReason) override;

  virtual void BeginDestroy() override;

  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds) override;

  UFUNCTION(BlueprintCallable)
  void SetNoiseErrorActive(const bool bEnabled);

private:

  UPROPERTY()
  TObjectPtr<UMgrsDataAsset> MgrsDataAsset = nullptr;

  carla::geom::GeoProjection CurrentGeoProjection;

  bool bApplyNoiseError = false;

  float LatitudeDeviation;
  float LongitudeDeviation;
  float AltitudeDeviation;

  float LatitudeBias;
  float LongitudeBias;
  float AltitudeBias;

  double LatitudeValue;
  double LongitudeValue;
  double AltitudeValue;

  // ---- Raw sink (carla_gnss_raw_sink.h), set from blueprint attributes ----

  /// Shared library that receives the antenna's ground truth every tick (e.g.
  /// to emulate a real receiver's wire protocol). Empty = not used.
  /// Independent of the ROS 2 publish, which the client enables per sensor
  /// (enable_for_ros).
  FString RawSinkLibrary;

  /// Passed to the sink as is; its meaning belongs to the sink.
  FString RawSinkConfig;

  /// Loading is attempted once (first PostPhysTick), success or not.
  bool bRawSinkTried = false;

  void *RawSinkDll = nullptr;
  void *RawSink = nullptr;
  carla_gnss_raw_sink_push_fn RawSinkPush = nullptr;
  carla_gnss_raw_sink_destroy_fn RawSinkDestroy = nullptr;

  /// MGRS offset of the map frame, taken with the grid zone the sink was
  /// created with.
  FVector RawSinkMgrsOffset = FVector::ZeroVector;

  /// User data of the sink's log callback (the sensor name); freed after the
  /// sink is destroyed, since the callback may run on a sink thread until then.
  FString *RawSinkLogName = nullptr;

  void StartRawSinkOnce();
  void StopRawSink();
  void PushRawSinkTruth();

  // ---- ROS 2 publish switch ----

  /// The stream's ROS flag (enable_for_ros / disable_for_ros), read once per
  /// frame: this sensor publishes to ROS 2 itself, past the stream.
  bool bRos2StreamEnabled = true;

  bool IsStreamEnabledForRos();
};
