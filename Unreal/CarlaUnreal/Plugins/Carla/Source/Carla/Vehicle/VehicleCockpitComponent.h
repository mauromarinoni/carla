// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "Components/ActorComponent.h"
#include "Curves/CurveFloat.h"
#include <util/ue-header-guard-end.h>

#include "VehicleCockpitComponent.generated.h"

class UMaterialInstanceDynamic;
class UMeshComponent;
class USceneCaptureComponent2D;
class UStaticMeshComponent;
class UTextureRenderTarget2D;

/// Cockpit of a vehicle seen from a driver's view (a camera at the driver's
/// eye): a steering wheel that turns with the road wheels or is hidden (for a
/// driver holding a physical wheel), and rear-view mirrors that reflect the
/// scene through a scene capture each.
///
/// Sibling components found by tag:
///  - "Carla.SteeringWheel": a static mesh whose local X axis is the steering
///    column; it rolls about it by the steering wheel angle.
///  - "Carla.SteeringColumn": static meshes hidden with the steering wheel
///    (column shroud, stalks).
///  - "Carla.Mirror": a scene capture attached to the mirror glass, a flat
///    static mesh in the plane x = 0 of its local frame, x towards the viewer.
///    The capture sits at the viewer's eye mirrored through the glass, looking
///    along the glass's x; its projection is the glass's bounding box seen from
///    there, clipped at the glass plane, so the glass maps the render target
///    by its local y (u, along +y) and z (v, top to bottom). The glass
///    material takes the capture through the texture parameter "Capture" and
///    the scalar parameter "CaptureWeight" (0: the material's own reflection,
///    1: the capture).
///
/// The driver's view is chosen at run time through console variables (the
/// console_command RPC): carla.DriverView.VehicleId names the vehicle, and
/// only that one turns or hides its wheel (carla.DriverView.SteeringWheel)
/// and renders its mirrors (carla.DriverView.Mirrors). Every other vehicle
/// keeps a static wheel and no mirror capture, so it costs nothing.
UCLASS(ClassGroup = (Carla), meta = (BlueprintSpawnableComponent))
class CARLA_API UVehicleCockpitComponent : public UActorComponent
{
  GENERATED_BODY()

public:

  UVehicleCockpitComponent();

  /// Steering wheel angle (deg, positive clockwise as seen by the driver)
  /// from the road wheel angle (deg, the mean of the front wheels as the
  /// vehicle animation shows them, positive to the right). Without keys, the angle times SteeringRatio.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA Cockpit")
  FRuntimeFloatCurve SteeringWheelAngleCurve;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA Cockpit")
  float SteeringRatio = 15.0f;

  /// Render target width of each mirror capture, by component name; the
  /// height follows the glass's aspect ratio.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA Cockpit")
  TMap<FName, int32> MirrorTargetWidths;

  /// Width of a mirror capture missing from MirrorTargetWidths.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CARLA Cockpit")
  int32 DefaultMirrorTargetWidth = 512;

  /// Steering wheel angle shown in turning mode (deg).
  UFUNCTION(BlueprintCallable, Category = "CARLA Cockpit")
  float GetSteeringWheelAngle() const;

protected:

  virtual void BeginPlay() override;

  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

  virtual void TickComponent(
      float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction *ThisTickFunction) override;

private:

  struct FSteeringWheel
  {
    TWeakObjectPtr<UStaticMeshComponent> Component;
    FQuat RestRotation = FQuat::Identity;
  };

  struct FMirror
  {
    TWeakObjectPtr<USceneCaptureComponent2D> Capture;
    TWeakObjectPtr<UMeshComponent> Glass;
  };

  enum class EWheelMode : int32 { Static = 0, Hidden = 1, Turning = 2 };

  bool IsDriverViewVehicle();

  void SetWheelMode(EWheelMode Mode);

  void SetMirrorCaptures(bool bEnabled);

  TArray<FSteeringWheel> SteeringWheels;

  TArray<TWeakObjectPtr<UStaticMeshComponent>> SteeringColumns;

  TArray<FMirror> Mirrors;

  UPROPERTY(Transient)
  TArray<TObjectPtr<UTextureRenderTarget2D>> MirrorTargets;

  UPROPERTY(Transient)
  TArray<TObjectPtr<UMaterialInstanceDynamic>> MirrorMaterials;

  /// CARLA actor id of the owner, 0 until known.
  uint32 ActorId = 0u;

  EWheelMode WheelMode = EWheelMode::Static;

  float ShownWheelAngle = 0.0f;

  bool bMirrorCaptures = false;

  uint32 DebugFrames = 0u;

  uint32 CaptureFrames = 0u;
};
