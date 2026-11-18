// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/CarlaLidarSubsystem.h"
#include "Carla.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Sensor/RayCastSemanticLidar.h"
#include "Carla/Sensor/SensorManager.h"

#include <util/ue-header-guard-begin.h>
#include "Async/ParallelFor.h"
#include "PhysicsEngine/PhysicsObjectExternalInterface.h"
#include <util/ue-header-guard-end.h>

static TAutoConsoleVariable<int32> CVarLidarBatch(
    TEXT("carla.Lidar.Batch"), 1,
    TEXT("Simulate multiple LiDARs in parallel (1) or one after another (0)."));

static TAutoConsoleVariable<int32> CVarLidarAsyncSend(
    TEXT("carla.Lidar.AsyncSend"), 1,
    TEXT("Send the LiDARs' data (clients, ROS 2) on a worker after the ray casts, finished before the next frame (1), ")
    TEXT("or on the game thread at the end of the post-physics tick (0)."));

void UCarlaLidarSubsystem::WaitForPendingSend()
{
  if (LidarSendTask.IsValid())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(UCarlaLidarSubsystem::WaitForPendingSend);
    FTaskGraphInterface::Get().WaitUntilTaskCompletes(LidarSendTask);
    LidarSendTask = nullptr;
  }
  SendJobs.Reset();
}

void UCarlaLidarSubsystem::Deinitialize()
{
  if (LidarSimulationTask.IsValid())
  {
    FTaskGraphInterface::Get().WaitUntilTaskCompletes(LidarSimulationTask);
    LidarSimulationTask = nullptr;
  }
  WaitForPendingSend();
  LidarsToSimulate.Empty();
  ActiveLidars.Empty();
  Super::Deinitialize();
}

void UCarlaLidarSubsystem::RegisterLidar(ARayCastSemanticLidar *Lidar)
{
  if (Lidar != nullptr)
  {
    ActiveLidars.AddUnique(Lidar);

    // Remove from FSensorManager so this subsystem is the sole manager of LiDAR ticking
    UCarlaEpisode* CurrentEpisode = UCarlaStatics::GetCurrentEpisode(GetWorld());
    if (CurrentEpisode != nullptr)
    {
      CurrentEpisode->GetSensorManager().DeRegisterSensor(Lidar);
    }
  }
}

void UCarlaLidarSubsystem::UnregisterLidar(ARayCastSemanticLidar *Lidar)
{
  // Its data of the last frame may still be being sent.
  WaitForPendingSend();
  ActiveLidars.Remove(Lidar);
  ActiveLidars.RemoveAll([](const TWeakObjectPtr<ARayCastSemanticLidar>& Ptr) { return !Ptr.IsValid(); });
}

void UCarlaLidarSubsystem::StartLidarSimulations(UWorld *World, [[maybe_unused]] float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(UCarlaLidarSubsystem::StartLidarSimulations);

  if (World == nullptr || World->GetPhysicsScene() == nullptr || !HasActiveLidars())
  {
    return;
  }

  WaitForPendingSend();

  // The CVars from the command line too (-carla-lidar-async-send=0|1): a Shipping build runs no console commands.
  static bool bCommandLineRead = false;
  if (!bCommandLineRead)
  {
    bCommandLineRead = true;
    int32 Value = 0;
    if (FParse::Value(FCommandLine::Get(), TEXT("-carla-lidar-async-send="), Value))
    {
      CVarLidarAsyncSend->Set(Value, ECVF_SetByCommandline);
      UE_LOG(LogCarla, Log, TEXT("carla.Lidar.AsyncSend = %d (command line)"), Value);
    }
  }

  // Snapshot on the game thread; the task must not read ActiveLidars while the game thread can modify it.
  LidarsToSimulate.Reset();
  for (const TWeakObjectPtr<ARayCastSemanticLidar> &WeakLidar : ActiveLidars)
  {
    ARayCastSemanticLidar *Lidar = WeakLidar.Get();
    if (Lidar != nullptr && Lidar->IsReadyToTick())
    {
      LidarsToSimulate.Add(Lidar);
    }
  }

  if (LidarsToSimulate.Num() == 0)
  {
    return;
  }

  const bool bBatch = (CVarLidarBatch.GetValueOnGameThread() != 0) && (LidarsToSimulate.Num() > 1);

  // Dispatch raycast simulation as a high-priority Task Graph job on worker threads.
  // Each LiDAR uses its own tick-interval delta, not the physics substep delta.
  LidarSimulationTask = FFunctionGraphTask::CreateAndDispatchWhenReady(
    [this, World, bBatch]()
    {
      TRACE_CPUPROFILER_EVENT_SCOPE(TaskGraphLidarSimulations);
      auto LockedPhysObject = FPhysicsObjectExternalInterface::LockRead(World->GetPhysicsScene());

      if (bBatch)
      {
        ParallelFor(LidarsToSimulate.Num(), [this](int32 Index)
        {
          ARayCastSemanticLidar *Lidar = LidarsToSimulate[Index];
          Lidar->SimulateLidar(Lidar->GetReadyToTickDeltaSeconds(), /*bLockPhysics=*/false);
        });
      }
      else
      {
        for (ARayCastSemanticLidar *Lidar : LidarsToSimulate)
        {
          Lidar->SimulateLidar(Lidar->GetReadyToTickDeltaSeconds(), /*bLockPhysics=*/false);
        }
      }

      LockedPhysObject.Release();
    },
    TStatId(),
    nullptr,
    ENamedThreads::AnyBackgroundHiPriTask
  );

  // The data is sent as soon as the ray casts end, alongside the cameras' readback and the rest of the frame.
  if (CVarLidarAsyncSend.GetValueOnGameThread() != 0)
  {
    bool bAnyJob = false;
    for (ARayCastSemanticLidar *Lidar : LidarsToSimulate)
    {
      SendJobs.Add(Lidar->PrepareAsyncSend(Lidar->GetReadyToTickDeltaSeconds()));
      bAnyJob |= static_cast<bool>(SendJobs.Last());
    }
    if (bAnyJob)
    {
      FGraphEventArray Prerequisites;
      Prerequisites.Add(LidarSimulationTask);
      LidarSendTask = FFunctionGraphTask::CreateAndDispatchWhenReady(
        [this]()
        {
          TRACE_CPUPROFILER_EVENT_SCOPE(TaskGraphLidarSend);
          // One after another, as on the game thread: concurrent with the cameras' ROS 2 sending only, which
          // already runs on the render thread.
          for (TUniqueFunction<void()> &Job : SendJobs)
          {
            if (Job)
            {
              Job();
            }
          }
        },
        TStatId(),
        &Prerequisites,
        ENamedThreads::AnyBackgroundHiPriTask
      );
    }
  }
}

void UCarlaLidarSubsystem::FinishLidarSimulations(UWorld *World, [[maybe_unused]] float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(UCarlaLidarSubsystem::FinishLidarSimulations);

  if (LidarSimulationTask.IsValid())
  {
    FTaskGraphInterface::Get().WaitUntilTaskCompletes(LidarSimulationTask);
    LidarSimulationTask = nullptr;
  }

  // Streaming and ROS2 publishing stay on the game thread.
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(LidarSendData);
    for (int32 Index = 0; Index < LidarsToSimulate.Num(); ++Index)
    {
      ARayCastSemanticLidar *Lidar = LidarsToSimulate[Index];
      if (IsValid(Lidar))
      {
        // Sent by the send task (StartLidarSimulations) or here.
        if (!SendJobs.IsValidIndex(Index) || !SendJobs[Index])
        {
          Lidar->SendData(Lidar->GetReadyToTickDeltaSeconds());
        }
        Lidar->ClearReadyToTick();
      }
    }
  }
  LidarsToSimulate.Reset();
}
