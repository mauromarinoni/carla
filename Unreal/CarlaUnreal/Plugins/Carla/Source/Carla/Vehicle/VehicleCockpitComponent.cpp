// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Vehicle/VehicleCockpitComponent.h"
#include "Carla.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <util/ue-header-guard-begin.h>
#include "ChaosVehicleWheel.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TextureResource.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "WheeledVehiclePawn.h"
#include <util/ue-header-guard-end.h>

static TAutoConsoleVariable<int32> CVarDriverViewVehicleId(
    TEXT("carla.DriverView.VehicleId"),
    0,
    TEXT("CARLA actor id of the vehicle driven from a driver's view (a camera at the driver's eye), 0 for none. ")
    TEXT("Only that vehicle applies carla.DriverView.SteeringWheel and carla.DriverView.Mirrors."));

static TAutoConsoleVariable<int32> CVarDriverViewSteeringWheel(
    TEXT("carla.DriverView.SteeringWheel"),
    2,
    TEXT("Steering wheel of the driver's view vehicle: 0 static, 1 hidden (the driver holds a physical wheel), ")
    TEXT("2 turning with the road wheels."));

static TAutoConsoleVariable<int32> CVarDriverViewMirrors(
    TEXT("carla.DriverView.Mirrors"),
    1,
    TEXT("Rear-view mirrors of the driver's view vehicle: 1 rendered by their scene captures (one more view each), ")
    TEXT("0 the glass material's own reflection."));

static TAutoConsoleVariable<int32> CVarDriverViewMirrorInterval(
    TEXT("carla.DriverView.MirrorInterval"),
    1,
    TEXT("Each mirror capture is rendered every this many frames, the mirrors staggered (2: half the cost, the ")
    TEXT("images at half the rate)."));

static TAutoConsoleVariable<int32> CVarDriverViewMirrorShadows(
    TEXT("carla.DriverView.MirrorShadows"),
    1,
    TEXT("Dynamic shadows in the mirror captures (0: cheaper, flat lighting)."));

static TAutoConsoleVariable<int32> CVarDriverViewMirrorDebug(
    TEXT("carla.DriverView.MirrorDebug"),
    0,
    TEXT("1: log the projection of each mirror capture and, every 40 frames, the mean / max of its render target."));

static const FName SteeringWheelTag(TEXT("Carla.SteeringWheel"));
static const FName SteeringColumnTag(TEXT("Carla.SteeringColumn"));
static const FName MirrorTag(TEXT("Carla.Mirror"));
static const FName CaptureParameter(TEXT("Capture"));
static const FName CaptureWeightParameter(TEXT("CaptureWeight"));

// A part split out of the body (steering wheel, column, mirror glass) follows
// the body as drawn: attached to the skeletal mesh's root bone, not to the
// component. After a hard collision Chaos can leave the root bone's pose, the
// body on screen, apart from the component's transform (the actor's): parts
// attached to the component were then drawn several centimetres off the body.
static void AttachToRootBone(USceneComponent &Part)
{
  USkeletalMeshComponent *Body = Cast<USkeletalMeshComponent>(Part.GetAttachParent());
  if (Body == nullptr || Part.GetAttachSocketName() != NAME_None || Body->GetNumBones() == 0)
  {
    return;
  }
  Part.AttachToComponent(Body, FAttachmentTransformRules::KeepWorldTransform, Body->GetBoneName(0));
}

UVehicleCockpitComponent::UVehicleCockpitComponent()
{
  PrimaryComponentTick.bCanEverTick = true;
  // After the vehicle simulation: the steering wheel shows this frame's road
  // wheel angle.
  PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void UVehicleCockpitComponent::BeginPlay()
{
  Super::BeginPlay();

  const AActor *Owner = GetOwner();
  if (Owner == nullptr)
  {
    return;
  }
  TInlineComponentArray<UStaticMeshComponent *> Meshes(Owner);
  for (UStaticMeshComponent *Mesh : Meshes)
  {
    if (Mesh->ComponentHasTag(SteeringWheelTag))
    {
      Mesh->SetMobility(EComponentMobility::Movable);
      AttachToRootBone(*Mesh);
      SteeringWheels.Add({Mesh, Mesh->GetRelativeRotation().Quaternion()});
    }
    else if (Mesh->ComponentHasTag(SteeringColumnTag))
    {
      AttachToRootBone(*Mesh);
      SteeringColumns.Add(Mesh);
    }
  }
  TInlineComponentArray<USceneCaptureComponent2D *> Captures(Owner);
  for (USceneCaptureComponent2D *Capture : Captures)
  {
    if (!Capture->ComponentHasTag(MirrorTag))
    {
      continue;
    }
    // Rendered by TickComponent while enabled, as the CARLA cameras (a capture
    // every frame is not rendered with -RenderOffScreen).
    Capture->bCaptureEveryFrame = false;
    // A rear view, not a sensor: without the costly global illumination,
    // reflections, ambient occlusion and volumetric effects, at lower LODs.
    Capture->ShowFlags.SetLumenGlobalIllumination(false);
    Capture->ShowFlags.SetLumenReflections(false);
    Capture->ShowFlags.SetScreenSpaceReflections(false);
    Capture->ShowFlags.SetAmbientOcclusion(false);
    Capture->ShowFlags.SetDistanceFieldAO(false);
    Capture->ShowFlags.SetVolumetricFog(false);
    Capture->ShowFlags.SetContactShadows(false);
    Capture->ShowFlags.SetMotionBlur(false);
    Capture->LODDistanceFactor = 2.0f;
    Capture->MaxViewDistanceOverride = 30000.0f;
    Capture->bCaptureOnMovement = false;
    Capture->TextureTarget = nullptr;
    UMeshComponent *Glass = Cast<UMeshComponent>(Capture->GetAttachParent());
    if (Glass == nullptr)
    {
      UE_LOG(LogCarla, Warning, TEXT("%s: mirror capture %s is not attached to a mesh"),
          *Owner->GetName(), *Capture->GetName());
      continue;
    }
    AttachToRootBone(*Glass);
    Mirrors.Add({Capture, Glass});
  }
  // A mirror shows neither its own glass nor the others (no mirror in a mirror).
  for (const FMirror &Mirror : Mirrors)
  {
    for (const FMirror &Other : Mirrors)
    {
      Mirror.Capture->HideComponent(Other.Glass.Get());
    }
  }
  MirrorTargets.SetNum(Mirrors.Num());
  MirrorMaterials.SetNum(Mirrors.Num());
  if (SteeringWheels.IsEmpty() && SteeringColumns.IsEmpty() && Mirrors.IsEmpty())
  {
    SetComponentTickEnabled(false);
  }
}

void UVehicleCockpitComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  SetMirrorCaptures(false);
  Super::EndPlay(EndPlayReason);
}

bool UVehicleCockpitComponent::IsDriverViewVehicle()
{
  const int32 Id = CVarDriverViewVehicleId.GetValueOnGameThread();
  if (Id <= 0)
  {
    return false;
  }
  if (ActorId == 0u)
  {
    const UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(this);
    const FCarlaActor *CarlaActor = Episode != nullptr ? Episode->FindCarlaActor(GetOwner()) : nullptr;
    if (CarlaActor == nullptr)
    {
      return false;
    }
    ActorId = CarlaActor->GetActorId();
  }
  return static_cast<uint32>(Id) == ActorId;
}

float UVehicleCockpitComponent::GetSteeringWheelAngle() const
{
  // The front wheels' steering as the vehicle animation shows it (the angle
  // the Python API reports): Chaos's own, or the overridden poses of a
  // replay.
  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(GetOwner());
  if (Vehicle == nullptr)
  {
    return 0.0f;
  }
  const float RoadWheelAngle = 0.5f * (Vehicle->GetWheelSteerAngle(EVehicleWheelLocation::FL_Wheel) +
      Vehicle->GetWheelSteerAngle(EVehicleWheelLocation::FR_Wheel));
  const FRichCurve *Curve = SteeringWheelAngleCurve.GetRichCurveConst();
  if (Curve != nullptr && Curve->GetNumKeys() > 0)
  {
    return Curve->Eval(RoadWheelAngle);
  }
  return RoadWheelAngle * SteeringRatio;
}

void UVehicleCockpitComponent::SetWheelMode(EWheelMode Mode)
{
  const float Angle = Mode == EWheelMode::Turning ? GetSteeringWheelAngle() : 0.0f;
  if (Mode == WheelMode && FMath::IsNearlyEqual(Angle, ShownWheelAngle, 0.01f))
  {
    return;
  }
  for (const FSteeringWheel &Wheel : SteeringWheels)
  {
    if (UStaticMeshComponent *Component = Wheel.Component.Get())
    {
      Component->SetVisibility(Mode != EWheelMode::Hidden);
      // Positive roll about the column (local X, away from the driver) turns
      // the wheel clockwise as the driver sees it.
      Component->SetRelativeRotation(Wheel.RestRotation * FRotator(0.0f, 0.0f, Angle).Quaternion());
    }
  }
  for (const TWeakObjectPtr<UStaticMeshComponent> &Column : SteeringColumns)
  {
    if (UStaticMeshComponent *Component = Column.Get())
    {
      Component->SetVisibility(Mode != EWheelMode::Hidden);
    }
  }
  WheelMode = Mode;
  ShownWheelAngle = Angle;
}

// The projection of the glass's bounding box seen from the capture, near plane
// on the glass: off-axis (the glass is not in front of the mirrored eye) and
// parallel to the glass, so the glass's local y / z map linearly to u / v.
static bool FitCaptureToGlass(USceneCaptureComponent2D &Capture, const UStaticMeshComponent &Glass, float &Aspect)
{
  const UStaticMesh *Mesh = Glass.GetStaticMesh();
  if (Mesh == nullptr)
  {
    return false;
  }
  const FBox Box = Mesh->GetBoundingBox();
  const FTransform GlassToCapture =
      Glass.GetComponentTransform().GetRelativeTransform(Capture.GetComponentTransform());
  float MinA = TNumericLimits<float>::Max(), MaxA = TNumericLimits<float>::Lowest();
  float MinB = MinA, MaxB = MaxA, Near = MinA;
  for (int32 i = 0; i < 8; ++i)
  {
    const FVector Corner(
        (i & 1) ? Box.Max.X : Box.Min.X, (i & 2) ? Box.Max.Y : Box.Min.Y, (i & 4) ? Box.Max.Z : Box.Min.Z);
    const FVector P = GlassToCapture.TransformPosition(Corner);
    if (P.X <= UE_KINDA_SMALL_NUMBER)
    {
      return false;
    }
    MinA = FMath::Min(MinA, P.Y / P.X);
    MaxA = FMath::Max(MaxA, P.Y / P.X);
    MinB = FMath::Min(MinB, P.Z / P.X);
    MaxB = FMath::Max(MaxB, P.Z / P.X);
    Near = FMath::Min(Near, P.X);
  }
  if (MaxA - MinA <= UE_KINDA_SMALL_NUMBER || MaxB - MinB <= UE_KINDA_SMALL_NUMBER)
  {
    return false;
  }
  // View space: x right, y up, z forward; reversed Z, infinite far plane.
  FMatrix Projection(ForceInitToZero);
  Projection.M[0][0] = 2.0f / (MaxA - MinA);
  Projection.M[1][1] = 2.0f / (MaxB - MinB);
  Projection.M[2][0] = -(MaxA + MinA) / (MaxA - MinA);
  Projection.M[2][1] = -(MaxB + MinB) / (MaxB - MinB);
  Projection.M[2][3] = 1.0f;
  Projection.M[3][2] = Near * 0.999f;
  Capture.bUseCustomProjectionMatrix = true;
  Capture.CustomProjectionMatrix = Projection;
  // The custom projection does not cull: a symmetric view that holds it does.
  const float HalfTan = FMath::Max(FMath::Max(FMath::Abs(MinA), FMath::Abs(MaxA)),
      FMath::Max(FMath::Abs(MinB), FMath::Abs(MaxB)));
  Capture.FOVAngle = FMath::Min(2.0f * FMath::RadiansToDegrees(FMath::Atan(HalfTan)) + 2.0f, 170.0f);
  Aspect = (MaxA - MinA) / (MaxB - MinB);
  if (CVarDriverViewMirrorDebug.GetValueOnGameThread() != 0)
  {
    UE_LOG(LogCarla, Log, TEXT("mirror %s: a [%f, %f] b [%f, %f] near %f fov %f capture at %s"), *Capture.GetName(),
        MinA, MaxA, MinB, MaxB, Near, Capture.FOVAngle, *Capture.GetComponentLocation().ToString());
  }
  return true;
}

void UVehicleCockpitComponent::SetMirrorCaptures(bool bEnabled)
{
  if (bEnabled == bMirrorCaptures)
  {
    return;
  }
  for (int32 i = 0; i < Mirrors.Num(); ++i)
  {
    USceneCaptureComponent2D *Capture = Mirrors[i].Capture.Get();
    UStaticMeshComponent *Glass = Cast<UStaticMeshComponent>(Mirrors[i].Glass.Get());
    if (Capture == nullptr || Glass == nullptr)
    {
      continue;
    }
    if (!bEnabled)
    {
      Capture->bAlwaysPersistRenderingState = false;
      Capture->TextureTarget = nullptr;
      if (MirrorMaterials[i] != nullptr)
      {
        MirrorMaterials[i]->SetScalarParameterValue(CaptureWeightParameter, 0.0f);
      }
      continue;
    }
    float Aspect = 1.0f;
    if (!FitCaptureToGlass(*Capture, *Glass, Aspect))
    {
      UE_LOG(LogCarla, Warning, TEXT("%s: mirror capture %s does not see its glass"),
          *GetOwner()->GetName(), *Capture->GetName());
      continue;
    }
    if (MirrorTargets[i] == nullptr)
    {
      const int32 *Width = MirrorTargetWidths.Find(Capture->GetFName());
      const int32 W = FMath::Clamp(Width != nullptr ? *Width : DefaultMirrorTargetWidth, 16, 4096);
      const int32 H = FMath::Clamp(FMath::RoundToInt(W / Aspect), 16, 4096);
      UTextureRenderTarget2D *Target = NewObject<UTextureRenderTarget2D>(this);
      // Linear HDR scene colour: the driver's camera exposes and tone maps it
      // with the rest of the view.
      Target->RenderTargetFormat = RTF_RGBA16f;
      Target->ClearColor = FLinearColor::Black;
      Target->InitAutoFormat(W, H);
      Target->UpdateResourceImmediate(true);
      MirrorTargets[i] = Target;
    }
    if (MirrorMaterials[i] == nullptr)
    {
      MirrorMaterials[i] = Glass->CreateDynamicMaterialInstance(0);
    }
    Capture->CaptureSource = SCS_SceneColorHDR;
    Capture->TextureTarget = MirrorTargets[i];
    // Its view state (temporal history, exposure) kept between the captures.
    Capture->bAlwaysPersistRenderingState = true;
    if (MirrorMaterials[i] != nullptr)
    {
      MirrorMaterials[i]->SetTextureParameterValue(CaptureParameter, MirrorTargets[i]);
      MirrorMaterials[i]->SetScalarParameterValue(CaptureWeightParameter, 1.0f);
    }
  }
  bMirrorCaptures = bEnabled;
}

void UVehicleCockpitComponent::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction *ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

  const bool bDriverView = IsDriverViewVehicle();
  EWheelMode Mode = EWheelMode::Static;
  if (bDriverView)
  {
    const int32 Value = CVarDriverViewSteeringWheel.GetValueOnGameThread();
    Mode = Value == 1 ? EWheelMode::Hidden : Value == 2 ? EWheelMode::Turning : EWheelMode::Static;
  }
  SetWheelMode(Mode);
  SetMirrorCaptures(bDriverView && CVarDriverViewMirrors.GetValueOnGameThread() != 0);
  if (bMirrorCaptures)
  {
    const uint32 Interval = static_cast<uint32>(FMath::Max(1, CVarDriverViewMirrorInterval.GetValueOnGameThread()));
    const bool bShadows = CVarDriverViewMirrorShadows.GetValueOnGameThread() != 0;
    ++CaptureFrames;
    for (int32 i = 0; i < Mirrors.Num(); ++i)
    {
      USceneCaptureComponent2D *Capture = Mirrors[i].Capture.Get();
      const UMeshComponent *Glass = Mirrors[i].Glass.Get();
      // Only a glass some view showed lately (the driver's camera turned to
      // it): out of the view a mirror costs nothing.
      if (Capture != nullptr && Capture->TextureTarget != nullptr && Glass != nullptr &&
          Glass->WasRecentlyRendered(0.5f) && (CaptureFrames + i) % Interval == 0u)
      {
        Capture->ShowFlags.SetDynamicShadows(bShadows);
        Capture->CaptureScene();
      }
    }
  }

  if (CVarDriverViewMirrorDebug.GetValueOnGameThread() != 0 && (++DebugFrames % 40) == 0)
  {
    UE_LOG(LogCarla, Log, TEXT("cockpit %s: driver view %d, wheel mode %d, angle %f (computed %f), wheels %d, mirrors %d"),
        *GetOwner()->GetName(), bDriverView, static_cast<int32>(WheelMode), ShownWheelAngle, GetSteeringWheelAngle(),
        SteeringWheels.Num(), Mirrors.Num());
  }
  if (bMirrorCaptures && CVarDriverViewMirrorDebug.GetValueOnGameThread() != 0 && (DebugFrames % 40) == 0)
  {
    for (int32 i = 0; i < MirrorTargets.Num(); ++i)
    {
      FTextureRenderTargetResource *Resource =
          MirrorTargets[i] != nullptr ? MirrorTargets[i]->GameThread_GetRenderTargetResource() : nullptr;
      TArray<FLinearColor> Pixels;
      if (Resource == nullptr || !Resource->ReadLinearColorPixels(Pixels) || Pixels.IsEmpty())
      {
        UE_LOG(LogCarla, Log, TEXT("mirror %d: no pixels"), i);
        continue;
      }
      FLinearColor Sum(0, 0, 0, 0);
      float Max = 0.0f;
      int32 Bad = 0;
      for (const FLinearColor &P : Pixels)
      {
        if (!FMath::IsFinite(P.R) || !FMath::IsFinite(P.G) || !FMath::IsFinite(P.B))
        {
          ++Bad;
          continue;
        }
        Sum += P;
        Max = FMath::Max(Max, P.GetMax());
      }
      Sum /= FMath::Max(1, Pixels.Num() - Bad);
      UE_LOG(LogCarla, Log, TEXT("mirror %d %dx%d: mean (%g %g %g) max %g non-finite %d, material weight %s"), i,
          MirrorTargets[i]->SizeX, MirrorTargets[i]->SizeY, Sum.R, Sum.G, Sum.B, Max, Bad,
          MirrorMaterials[i] != nullptr ? TEXT("set") : TEXT("none"));
    }
  }
}
