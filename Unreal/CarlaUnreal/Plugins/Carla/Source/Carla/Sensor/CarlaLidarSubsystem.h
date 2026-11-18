// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Async/TaskGraphInterfaces.h"
#include <util/ue-header-guard-end.h>

#include "CarlaLidarSubsystem.generated.h"

class ARayCastSemanticLidar;

/// Subsystem responsible for simulating all ray-cast LiDARs of a world.
///
/// Registered LiDARs are simulated asynchronously via TaskGraph high-priority
/// worker tasks while the Game Thread processes remaining sensors (cameras, etc.).
/// FSensorManager::PostPhysTick calls StartLidarSimulations() at the beginning of
/// the post-physics tick, and FinishLidarSimulations() after other sensors finish.
UCLASS()
class CARLA_API UCarlaLidarSubsystem : public UWorldSubsystem
{
  GENERATED_BODY()

public:

  virtual void Deinitialize() override;

  void RegisterLidar(ARayCastSemanticLidar *Lidar);

  void UnregisterLidar(ARayCastSemanticLidar *Lidar);

  bool HasActiveLidars() const
  {
    return ActiveLidars.Num() > 0;
  }

  int32 GetActiveLidarCount() const
  {
    return ActiveLidars.Num();
  }

  void StartLidarSimulations(UWorld *World, float DeltaSeconds);

  void FinishLidarSimulations(UWorld *World, float DeltaSeconds);

  /// Waits for the data of the last frame's LiDARs to be sent (carla.Lidar.AsyncSend): FCarlaEngine::OnPreTick calls
  /// it before the frame's time and the server's commands change anything the sending reads.
  void WaitForPendingSend();

private:

  TArray<TWeakObjectPtr<ARayCastSemanticLidar>> ActiveLidars;

  /// LiDARs due this frame, snapshotted on the game thread for the simulation task.
  TArray<TObjectPtr<ARayCastSemanticLidar>> LidarsToSimulate;

  FGraphEventRef LidarSimulationTask;

  /// Sending of the LiDARs' data on a worker (ARayCastSemanticLidar::PrepareAsyncSend), after the simulation task;
  /// SendJobs, one per LiDAR of LidarsToSimulate (empty: sent on the game thread), stay alive until it is done.
  TArray<TUniqueFunction<void()>> SendJobs;
  FGraphEventRef LidarSendTask;
};
