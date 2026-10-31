// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/RayCastSemanticLidar.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/Tagger.h"
#include "Carla/Sensor/CarlaLidarSubsystem.h"

#include <util/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Kismet/KismetMathLibrary.h"
#include "PhysicsEngine/PhysicsObjectExternalInterface.h"
#include "Async/ParallelFor.h"
#include <util/ue-header-guard-end.h>
#include "Landscape.h"

#include <cmath>
#include <cstdlib>

namespace crp = carla::rpc;

namespace {

  /// Parses a comma-separated list of numbers (channel table attribute).
  /// Returns false on a malformed value; an empty text gives an empty list.
  bool ParseChannelList(const FString &Text, TArray<float> &Values)
  {
    Values.Reset();
    if (Text.TrimStartAndEnd().IsEmpty())
    {
      return true;
    }
    TArray<FString> Items;
    Text.ParseIntoArray(Items, TEXT(","), /*InCullEmpty=*/false);
    for (FString &Item : Items)
    {
      Item.TrimStartAndEndInline();
      const auto Utf8 = StringCast<ANSICHAR>(*Item);
      const char *Begin = Utf8.Get();
      char *End = nullptr;
      const double Value = std::strtod(Begin, &End);
      if (Item.IsEmpty() || End == Begin || *End != '\0' || !std::isfinite(Value))
      {
        return false;
      }
      Values.Add(static_cast<float>(Value));
    }
    return true;
  }

} // namespace

FActorDefinition ARayCastSemanticLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast_semantic"));
}

ARayCastSemanticLidar::ARayCastSemanticLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
}

void ARayCastSemanticLidar::BeginPlay()
{
  Super::BeginPlay();
  if (UCarlaLidarSubsystem* LidarSubsystem = GetWorld()->GetSubsystem<UCarlaLidarSubsystem>())
  {
    LidarSubsystem->RegisterLidar(this);
  }
}

void ARayCastSemanticLidar::EndPlay(EEndPlayReason::Type EndPlayReason)
{
  if (UCarlaLidarSubsystem* LidarSubsystem = GetWorld()->GetSubsystem<UCarlaLidarSubsystem>())
  {
    LidarSubsystem->UnregisterLidar(this);
  }
  Super::EndPlay(EndPlayReason);
}

void ARayCastSemanticLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastSemanticLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  SemanticLidarData = FSemanticLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);
}

void ARayCastSemanticLidar::CreateLasers()
{
  const auto NumberOfLasers = Description.Channels;
  check(NumberOfLasers > 0u);
  const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
    (Description.UpperFovLimit - Description.LowerFovLimit) /
    static_cast<float>(NumberOfLasers - 1);
  LaserAngles.Empty(NumberOfLasers);
  for(auto i = 0u; i < NumberOfLasers; ++i)
  {
    const float VerticalAngle =
        Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
    LaserAngles.Emplace(VerticalAngle);
  }

  LaserAzimuthOffsets.Empty();
  LaserHorizontalSteps.Empty();
  bChannelTableElevations = false;
  EffectiveChannelCount = static_cast<float>(NumberOfLasers);
  BaseRayCounter = 0u;
  CreateChannelTable();
  CreateAzimuthWindow();
}

void ARayCastSemanticLidar::CreateAzimuthWindow()
{
  auto Wrap = [](float Angle) { const float A = std::fmod(Angle, 360.0f); return A < 0.0f ? A + 360.0f : A; };
  AzimuthWindowStart = Wrap(Description.AzimuthWindowStart);
  AzimuthWindowEnd = Wrap(Description.AzimuthWindowEnd);
  bAzimuthWindow = !FMath::IsNearlyEqual(AzimuthWindowStart, AzimuthWindowEnd);
  if (bAzimuthWindow && !FMath::IsNearlyEqual(Description.HorizontalFov, 360.0f))
  {
    UE_LOG(LogCarla, Error, TEXT("%s: azimuth_window_start / azimuth_window_end need horizontal_fov 360: whole rotation"),
        *GetName());
    bAzimuthWindow = false;
  }
  if (bAzimuthWindow)
  {
    UE_LOG(LogCarla, Log, TEXT("%s: azimuth window %.2f..%.2f deg"), *GetName(), AzimuthWindowStart, AzimuthWindowEnd);
  }
}

void ARayCastSemanticLidar::CreateChannelTable()
{
  if (Description.ChannelElevations.TrimStartAndEnd().IsEmpty() &&
      Description.ChannelAzimuthOffsets.TrimStartAndEnd().IsEmpty() &&
      Description.ChannelHorizontalSteps.TrimStartAndEnd().IsEmpty())
  {
    return;
  }

  // Each list is optional, but a present one has a value per channel; any
  // inconsistency drops the whole table (uniform channels).
  const int32 NumberOfLasers = static_cast<int32>(Description.Channels);
  TArray<float> Elevations, Offsets, Steps;
  auto Check = [&](const TCHAR *Name, const FString &Text, TArray<float> &Values) {
    if (!ParseChannelList(Text, Values))
    {
      UE_LOG(LogCarla, Error, TEXT("%s: %s is not a comma-separated list of numbers: uniform channels"), *GetName(), Name);
      return false;
    }
    if (Values.Num() != 0 && Values.Num() != NumberOfLasers)
    {
      UE_LOG(LogCarla, Error, TEXT("%s: %s has %d values for %d channels: uniform channels"),
          *GetName(), Name, Values.Num(), NumberOfLasers);
      return false;
    }
    return true;
  };
  if (!Check(TEXT("channel_elevations"), Description.ChannelElevations, Elevations) ||
      !Check(TEXT("channel_azimuth_offsets"), Description.ChannelAzimuthOffsets, Offsets) ||
      !Check(TEXT("channel_horizontal_steps"), Description.ChannelHorizontalSteps, Steps))
  {
    return;
  }
  for (const float Step : Steps)
  {
    if (Step < 1.0f || Step != FMath::RoundToFloat(Step))
    {
      UE_LOG(LogCarla, Error, TEXT("%s: channel_horizontal_steps must be integers >= 1 (got %g): uniform channels"),
          *GetName(), Step);
      return;
    }
  }

  if (Elevations.Num() != 0)
  {
    LaserAngles = Elevations;
    bChannelTableElevations = true;
  }
  LaserAzimuthOffsets = Offsets;
  if (Steps.Num() != 0)
  {
    EffectiveChannelCount = 0.0f;
    LaserHorizontalSteps.Reserve(Steps.Num());
    for (const float Step : Steps)
    {
      LaserHorizontalSteps.Add(static_cast<uint32>(Step));
      EffectiveChannelCount += 1.0f / Step;
    }
  }
  UE_LOG(LogCarla, Log, TEXT("%s: channel table for %d channels (elevations %s, azimuth offsets %s, horizontal steps %s, %.2f channels per ray)"),
      *GetName(), NumberOfLasers,
      Elevations.Num() ? TEXT("yes") : TEXT("no"),
      Offsets.Num() ? TEXT("yes") : TEXT("no"),
      Steps.Num() ? TEXT("yes") : TEXT("no"),
      EffectiveChannelCount);
}

void ARayCastSemanticLidar::SendData(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastSemanticLidar::SendData);
  auto DataStream = GetDataStream(*this);
  auto SensorTransform = DataStream.GetSensorTransform();
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    DataStream.SerializeAndSend(*this, SemanticLidarData, DataStream.PopBufferFromPool());
  }
  // ROS2
  #if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    AActor* ParentActor = GetAttachParentActor();
    if (ParentActor)
    {
      FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
      ROS2->ProcessDataFromSemanticLidar(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, SemanticLidarData, this);
    }
    else
    {
      ROS2->ProcessDataFromSemanticLidar(DataStream.GetSensorType(), StreamId, SensorTransform, SemanticLidarData, this);
    }
  }
  #endif
}

void ARayCastSemanticLidar::SimulateLidar(const float DeltaTime, bool bLockPhysics)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastSemanticLidar::SimulateLidar);
  const uint32 ChannelCount = Description.Channels;
  // Base rays per channel: with horizontal steps a channel casts only every
  // k-th of them and points_per_second counts the rays actually cast.
  const uint32 PointsToScanWithOneLaser =
    FMath::RoundHalfFromZero(
        Description.PointsPerSecond * DeltaTime /
        (EffectiveChannelCount > 0.0f ? EffectiveChannelCount : float(ChannelCount)));

  if (PointsToScanWithOneLaser <= 0)
  {
    UE_LOG(
        LogCarla,
        Warning,
        TEXT("%s: no points requested this frame, try increasing the number of points per second."),
        *GetName());
    return;
  }

  check(ChannelCount == LaserAngles.Num());

  const float CurrentHorizontalAngle = carla::geom::Math::ToDegrees(
      SemanticLidarData.GetHorizontalAngle());
  const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov
      * DeltaTime;
  const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;
  const float HalfHorizontalFov = Description.HorizontalFov / 2;

  ResetDetections(ChannelCount, PointsToScanWithOneLaser);
  PreprocessRays(ChannelCount, PointsToScanWithOneLaser);

  const FTransform ActorTransform = GetTransform();
  const FTransform InverseSensorTransform = ActorTransform.Inverse();
  const FVector LidarBodyLocation = ActorTransform.GetLocation();
  const FRotator LidarBodyRotation = ActorTransform.Rotator();
  const uint64 FirstBaseRay = BaseRayCounter;

  auto RunChannelRaycasts = [&]()
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(ChannelCount, [&](int32 idxChannel) {
      TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);

      FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
      TraceParams.bTraceComplex = true;
      TraceParams.bReturnPhysicalMaterial = false;

      const float VertAngle = LaserAngles[idxChannel];
      const std::vector<bool>& ChannelPreprocessCondition = RayPreprocessCondition[idxChannel];
      // Channel table: horizontal offset and firing step of this channel.
      const float AzimuthOffset = LaserAzimuthOffsets.Num() != 0 ? LaserAzimuthOffsets[idxChannel] : 0.0f;
      const uint32 HorizontalStep = LaserHorizontalSteps.Num() != 0 ? LaserHorizontalSteps[idxChannel] : 1u;

      for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) {
        // Rays dropped by a subclass's PreprocessRays.
        if (!ChannelPreprocessCondition[idxPtsOneLaser])
        {
          continue;
        }
        // The channel fires only on every HorizontalStep-th base ray.
        if (HorizontalStep > 1u && (FirstBaseRay + idxPtsOneLaser) % HorizontalStep != 0u)
        {
          continue;
        }

        FHitResult HitResult;
        const float SweepAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfLaserMeasure
            * idxPtsOneLaser, Description.HorizontalFov) - HalfHorizontalFov;
        // Azimuth window: the firing (sweep) position from +X along the rotation.
        if (bAzimuthWindow)
        {
          const float Position = SweepAngle < 0.0f ? SweepAngle + 360.0f : SweepAngle;
          const bool bInside = AzimuthWindowStart < AzimuthWindowEnd
              ? (Position >= AzimuthWindowStart && Position < AzimuthWindowEnd)
              : (Position >= AzimuthWindowStart || Position < AzimuthWindowEnd);
          if (!bInside)
          {
            continue;
          }
        }
        const float HorizAngle = SweepAngle + AzimuthOffset;

        if (ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams, LidarBodyLocation, LidarBodyRotation)) {
          WriteDetectionAsync(idxChannel, HitResult, InverseSensorTransform, LidarBodyLocation);
        }
      };
    });
  };

  if (bLockPhysics)
  {
    auto LockedPhysObject = FPhysicsObjectExternalInterface::LockRead(GetWorld()->GetPhysicsScene());
    RunChannelRaycasts();
    LockedPhysObject.Release();
  }
  else
  {
    RunChannelRaycasts();
  }

  ComputeAndSaveDetections(ActorTransform);

  LastSweepStartAngle = CurrentHorizontalAngle;
  LastSweepAngle = AngleDistanceOfTick;
  LastRaysPerChannel = PointsToScanWithOneLaser;
  BaseRayCounter += PointsToScanWithOneLaser;

  const float HorizontalAngle = carla::geom::Math::ToRadians(
      std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov));
  SemanticLidarData.SetHorizontalAngle(HorizontalAngle);
}

void ARayCastSemanticLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
  RayPreprocessCondition.resize(Channels);

  for (auto& conds : RayPreprocessCondition) {
    conds.clear();
    conds.resize(MaxPointsPerChannel);
    std::fill(conds.begin(), conds.end(), true);
  }
}

void ARayCastSemanticLidar::ResetDetections(uint32_t Channels, uint32_t MaxPointsPerChannel) {
  ResetChannelDetections(SemanticDetections, Channels, MaxPointsPerChannel);
}

void ARayCastSemanticLidar::WriteDetectionAsync(uint32_t Channel, const FHitResult& HitInfo, const FTransform& InverseSensorTransform, const FVector& SensorLocation) {
  DEBUG_ASSERT(GetChannelCount() > Channel);
  FSemanticDetection& Detection = SemanticDetections[Channel].emplace_back();
  ComputeRawDetection(HitInfo, InverseSensorTransform, SensorLocation, Detection);
}

void ARayCastSemanticLidar::ComputeAndSaveDetections(const FTransform& SensorTransform)
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  {
    PointsPerChannel[idxChannel] = SemanticDetections[idxChannel].size();
  }

  SemanticLidarData.ResetMemory(PointsPerChannel);
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  {
    SemanticLidarData.WritePointsSync(SemanticDetections[idxChannel]);
  }
  SemanticLidarData.WriteChannelCount(PointsPerChannel);
}

void ARayCastSemanticLidar::ComputeRawDetection(const FHitResult& HitInfo, const FTransform& InverseSensorTransform, const FVector& SensorLocation, FSemanticDetection& Detection) const
{
    static const uint32_t TerrainTag = static_cast<uint32_t>(ATagger::GetTagFromString("Terrain"));

    const FVector HitPoint = HitInfo.ImpactPoint;
    Detection.point = InverseSensorTransform.TransformPosition(HitPoint);

    const FVector VecInc = - (HitPoint - SensorLocation).GetSafeNormal();
    Detection.cos_inc_angle = FVector::DotProduct(VecInc, HitInfo.ImpactNormal);

    const FActorRegistry &Registry = GetEpisode().GetActorRegistry();

    const AActor* actor = HitInfo.GetActor();
    Detection.object_idx = 0;
    Detection.object_tag = static_cast<uint32_t>(crp::CityObjectLabel::None);

    // Given that landscapes do not have tags for now, assign it here if the
    // actor is a landscape, otherwise get the component tag.
    // GetTagOfTaggedComponent tolerates untagged components (World Partition
    // streams in HLOD/proxy geometry that never went through ATagger) by
    // falling back to the stencil value, where reading ComponentTags[0]
    // directly asserts on an empty array.
    if (actor != nullptr && actor->IsA<ALandscape>())
    {
      Detection.object_tag = TerrainTag;
    }
    else if (HitInfo.Component.IsValid())
    {
      Detection.object_tag = static_cast<uint32_t>(ATagger::GetTagOfTaggedComponent(*HitInfo.Component));
    }

    if (actor != nullptr)
    {
      const FCarlaActor* view = Registry.FindCarlaActor(actor);
      if (view != nullptr)
      {
        // On a Multi-GPU secondary, the id the primary's clients know.
        Detection.object_idx = view->GetPrimaryActorId();
      }
    }
    else
    {
      UE_LOG(LogCarla, Warning, TEXT("Actor not valid %p!!!!"), actor);
    }
}


bool ARayCastSemanticLidar::ShootLaser(const float VerticalAngle, const float HorizontalAngle, FHitResult& HitResult, FCollisionQueryParams& TraceParams,
                                       const FVector& LidarBodyLocation, const FRotator& LidarBodyRotation) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

  FRotator LaserRotation(VerticalAngle, HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
  FRotator ResultRotation = UKismetMathLibrary::ComposeRotators(
    LaserRotation,
    LidarBodyRotation
  );

  const auto Range = Description.Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRotation) + LidarBodyLocation;
  
  return GetWorld()->ParallelLineTraceSingleByChannel(
    HitResult,
    LidarBodyLocation,
    EndTrace,
    ECC_GameTraceChannel2,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam
  );
}

void ARayCastSemanticLidar::GetMirrorState(carla::multigpu::SensorState &State) const
{
  State.lidar_horizontal_angle = SemanticLidarData.GetHorizontalAngle();
  if (RandomEngine != nullptr)
  {
    State.random_engine = RandomEngine->GetState();
  }
}

void ARayCastSemanticLidar::SetMirrorState(const carla::multigpu::SensorState &State)
{
  if (State.lidar_horizontal_angle)
  {
    SemanticLidarData.SetHorizontalAngle(*State.lidar_horizontal_angle);
  }
  if (State.random_engine && (RandomEngine != nullptr))
  {
    RandomEngine->SetState(*State.random_engine);
  }
}
