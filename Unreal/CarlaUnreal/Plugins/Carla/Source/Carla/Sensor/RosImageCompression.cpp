// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/RosImageCompression.h"
#include "Carla.h"
#include "Carla/Sensor/Sensor.h"

#include <util/ue-header-guard-begin.h>
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include <util/ue-header-guard-end.h>

void LoadRosImageCompression()
{
  FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
}

FRosCompressedImage CompressImageForRos(ASensor &Sensor, const FColor *Pixels, int32 Width, int32 Height)
{
  FRosCompressedImage Result;
  const auto Transport = Sensor.GetAttribute(TEXT("ros_image_transport"));
  if (!Transport.has_value() || Transport->Value == TEXT("raw") || Pixels == nullptr || Width <= 0 || Height <= 0)
  {
    return Result;
  }
  Result.bPublishRaw = Transport->Value == TEXT("both");

  int32 Quality = 90;
  if (const auto QualityAttribute = Sensor.GetAttribute(TEXT("ros_jpeg_quality")); QualityAttribute.has_value())
  {
    Quality = FMath::Clamp(FCString::Atoi(*QualityAttribute->Value), 1, 100);
  }

  IImageWrapperModule *Module = FModuleManager::GetModulePtr<IImageWrapperModule>(TEXT("ImageWrapper"));
  TSharedPtr<IImageWrapper> Wrapper = Module ? Module->CreateImageWrapper(EImageFormat::JPEG) : nullptr;
  const int64 Size = static_cast<int64>(Width) * Height * sizeof(FColor);
  if (!Wrapper.IsValid() || !Wrapper->SetRaw(Pixels, Size, Width, Height, ERGBFormat::BGRA, 8))
  {
    UE_LOG(LogCarla, Warning, TEXT("%s: JPEG compression unavailable, publishing the raw image"), *Sensor.GetName());
    Result.bPublishRaw = true;
    return Result;
  }
  const TArray64<uint8> Jpeg = Wrapper->GetCompressed(Quality);
  Result.Data.assign(Jpeg.GetData(), Jpeg.GetData() + Jpeg.Num());
  // as a camera driver through compressed_image_transport writes it (the JPEG
  // has no alpha: decoded, it is bgr8 as on the real cameras)
  Result.Format = "bgr8; jpeg compressed bgr8";
  return Result;
}
