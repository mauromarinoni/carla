// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/RayCastLidar.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/Tagger.h"

#include <util/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include "carla/geom/Location.h"
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Kismet/KismetMathLibrary.h"
#include "Landscape.h"
#include <util/ue-header-guard-end.h>

#include <cmath>

namespace {

  /// Typical reflectivity per semantic tag on the 0-255 scale of automotive
  /// lidars (0-100: diffuse reflectance in percent, 101-255: retroreflectors),
  /// to be calibrated against real data through reflectivity_table.
  constexpr std::pair<crp::CityObjectLabel, float> DefaultReflectivity[] = {
    {crp::CityObjectLabel::Roads, 12.0f},
    {crp::CityObjectLabel::Sidewalks, 30.0f},
    {crp::CityObjectLabel::Buildings, 40.0f},
    {crp::CityObjectLabel::Walls, 40.0f},
    {crp::CityObjectLabel::Fences, 30.0f},
    {crp::CityObjectLabel::Poles, 40.0f},
    {crp::CityObjectLabel::TrafficLight, 40.0f},
    {crp::CityObjectLabel::TrafficSigns, 230.0f},
    {crp::CityObjectLabel::Vegetation, 50.0f},
    {crp::CityObjectLabel::Terrain, 35.0f},
    {crp::CityObjectLabel::Sky, 0.0f},
    {crp::CityObjectLabel::Pedestrians, 35.0f},
    {crp::CityObjectLabel::Rider, 35.0f},
    {crp::CityObjectLabel::Car, 30.0f},
    {crp::CityObjectLabel::Truck, 35.0f},
    {crp::CityObjectLabel::Bus, 35.0f},
    {crp::CityObjectLabel::Train, 35.0f},
    {crp::CityObjectLabel::Motorcycle, 30.0f},
    {crp::CityObjectLabel::Bicycle, 30.0f},
    {crp::CityObjectLabel::Static, 30.0f},
    {crp::CityObjectLabel::Dynamic, 30.0f},
    {crp::CityObjectLabel::Other, 30.0f},
    {crp::CityObjectLabel::Water, 5.0f},
    {crp::CityObjectLabel::RoadLines, 120.0f},
    {crp::CityObjectLabel::Ground, 25.0f},
    {crp::CityObjectLabel::Bridge, 35.0f},
    {crp::CityObjectLabel::RailTrack, 25.0f},
    {crp::CityObjectLabel::GuardRail, 60.0f},
    {crp::CityObjectLabel::Rock, 30.0f},
  };

  constexpr float DiffuseReflectivityLimit = 100.0f / 255.0f;

} // namespace

FActorDefinition ARayCastLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));
}


ARayCastLidar::ARayCastLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);
}

void ARayCastLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  SetSeed(Description.RandomSeed);
  LidarData = FLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);

  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();

  ReflectivityIntensity = Description.IntensityModel.Equals(TEXT("reflectivity"), ESearchCase::IgnoreCase);
  SetReflectivityTable(Description.ReflectivityTable);
}

void ARayCastLidar::SetReflectivityTable(const FString& Overrides)
{
  Reflectivity.Init(30.0f / 255.0f, 256);
  Reflectivity[static_cast<int32>(crp::CityObjectLabel::None)] = 30.0f / 255.0f;
  for (const auto& Entry : DefaultReflectivity)
  {
    Reflectivity[static_cast<int32>(Entry.first)] = Entry.second / 255.0f;
  }
  TArray<FString> Items;
  Overrides.ParseIntoArray(Items, TEXT(","));
  for (const FString& Item : Items)
  {
    FString Tag, Value;
    if (!Item.Split(TEXT(":"), &Tag, &Value))
    {
      UE_LOG(LogCarla, Warning, TEXT("reflectivity_table: ignoring '%s' (expected tag:value)"), *Item);
      continue;
    }
    Tag.TrimStartAndEndInline();
    const int32 Index = Tag.IsNumeric() ?
        FCString::Atoi(*Tag) : static_cast<int32>(ATagger::GetTagFromString(Tag));
    if (!Reflectivity.IsValidIndex(Index) || (!Tag.IsNumeric() && Index == 0 && !Tag.Equals(TEXT("None"))))
    {
      UE_LOG(LogCarla, Warning, TEXT("reflectivity_table: unknown tag '%s'"), *Tag);
      continue;
    }
    Reflectivity[Index] = FMath::Clamp(FCString::Atof(*Value), 0.0f, 255.0f) / 255.0f;
  }
}

void ARayCastLidar::SendData(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLidar::SendData);
  auto DataStream = GetDataStream(*this);
  auto SensorTransform = DataStream.GetSensorTransform();

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    DataStream.SerializeAndSend(*this, LidarData, DataStream.PopBufferFromPool());
  }
  // ROS2
  #if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    AActor* ParentActor = GetAttachParentActor();
    // Driver mode (rotating 360 deg sensors only): the publisher assembles the
    // steps into full rotations; it needs the sweep of this step. Every ray is
    // cast from the sensor pose at the step stamp (time_span 0).
    carla::ros2::LidarDriverOptions Driver;
    Driver.enabled = Description.RosDriverMode && Description.HorizontalFov >= 360.0f;
    Driver.cut_angle = Description.RosScanCutAngle;
    Driver.slice.delta_time = DeltaTime;
    Driver.slice.time_span = 0.0;
    Driver.slice.start_angle = LastSweepStartAngle;
    Driver.slice.sweep_angle = LastSweepAngle;
    Driver.slice.rays_per_channel = LastRaysPerChannel;
    // Passing the channel count and vertical FOV limits routes the publisher
    // through the extended Autoware XYZIRCAEDT layout (tier4 autoware-support
    // port); see ROS2::ProcessDataFromLidar.
    if (ParentActor)
    {
      FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
      ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent,
                                 Description.Channels, Description.UpperFovLimit, Description.LowerFovLimit, Driver, LidarData, this);
    }
    else
    {
      ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, SensorTransform,
                                 Description.Channels, Description.UpperFovLimit, Description.LowerFovLimit, Driver, LidarData, this);
    }
  }
  #endif
}

float ARayCastLidar::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

float ARayCastLidar::ComputeReflectivityIntensity(const FHitResult& HitInfo, const FVector& SensorLocation) const
{
  // Semantic tag as the semantic lidar reads it (landscapes have no tags)
  uint32_t Tag = static_cast<uint32_t>(crp::CityObjectLabel::None);
  const AActor* Actor = HitInfo.GetActor();
  if (Actor != nullptr && Actor->IsA<ALandscape>())
  {
    Tag = static_cast<uint32_t>(crp::CityObjectLabel::Terrain);
  }
  else if (HitInfo.Component.IsValid())
  {
    Tag = static_cast<uint32_t>(ATagger::GetTagOfTaggedComponent(*HitInfo.Component));
  }
  const float Value = Reflectivity.IsValidIndex(Tag) ? Reflectivity[Tag] : 0.0f;

  const FVector Incidence = (SensorLocation - HitInfo.ImpactPoint).GetSafeNormal();
  const float CosIncidence = FMath::Clamp(FVector::DotProduct(Incidence, HitInfo.ImpactNormal), 0.0f, 1.0f);
  // Diffuse surfaces scatter as cos(incidence) (Lambert); retroreflectors send
  // most of the light back whatever the angle
  if (Value <= DiffuseReflectivityLimit)
  {
    return Value * CosIncidence;
  }
  return Value * (0.5f + 0.5f * CosIncidence);
}

ARayCastLidar::FDetection ARayCastLidar::ComputeDetection(const FHitResult& HitInfo, const FTransform& InverseSensorTransform, const FVector& SensorLocation) const
{
  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = InverseSensorTransform.TransformPosition(HitPoint);

  if (ReflectivityIntensity)
  {
    Detection.intensity = ComputeReflectivityIntensity(HitInfo, SensorLocation);
    return Detection;
  }

  const float Distance = Detection.point.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  Detection.intensity = IntRec;

  return Detection;
}

  void ARayCastLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    Super::PreprocessRays(Channels, MaxPointsPerChannel);

    if (!DropOffGenActive)
    {
      return;
    }

    for (auto ch = 0u; ch < Channels; ch++) {
      for (auto p = 0u; p < MaxPointsPerChannel; p++) {
        RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
      }
    }
  }

  bool ARayCastLidar::PostprocessDetection(FDetection& Detection) const
  {
    if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon()) {
      const auto ForwardVector = Detection.point.MakeUnitVector();
      const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
      Detection.point += Noise;
    }

    if (ReflectivityIntensity && Description.IntensityNoiseStdDev > std::numeric_limits<float>::epsilon()) {
      Detection.intensity = FMath::Clamp(
          Detection.intensity + RandomEngine->GetNormalDistribution(0.0f, Description.IntensityNoiseStdDev), 0.0f, 1.0f);
    }

    const float Intensity = Detection.intensity;
    if(Intensity > Description.DropOffIntensityLimit)
      return true;
    else
      return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
  }

  void ARayCastLidar::ResetDetections(uint32_t Channels, uint32_t MaxPointsPerChannel)
  {
    ResetChannelDetections(Detections, Channels, MaxPointsPerChannel);
  }

  void ARayCastLidar::WriteDetectionAsync(uint32_t Channel, const FHitResult& HitInfo, const FTransform& InverseSensorTransform, const FVector& SensorLocation) {
    DEBUG_ASSERT(GetChannelCount() > Channel);
    Detections[Channel].emplace_back(ComputeDetection(HitInfo, InverseSensorTransform, SensorLocation));
  }

  void ARayCastLidar::ComputeAndSaveDetections(const FTransform& SensorTransform)
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

    CompactAndWriteDetections(Detections, PointsPerChannel, Description.Channels, LidarData,
        [this](FDetection& Detection) { return PostprocessDetection(Detection); });

#if WITH_EDITOR
    if (bSavingDataToDisk)
    {
      CopyDetectionsToPointCloud(Detections, PointsPerChannel, PointCloudLidarData);
    }
#endif
  }
