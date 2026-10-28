// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include <cstdint>
#include <string>
#include <vector>

class ASensor;

/// The ROS 2 image transport of an RGB camera (ros_image_transport attribute):
/// the JPEG to publish as sensor_msgs/CompressedImage, if any, and whether the
/// raw sensor_msgs/Image is published too.
struct FRosCompressedImage
{
  std::vector<uint8_t> Data;
  std::string Format;
  bool bPublishRaw = true;
};

/// Compresses a BGRA image as the sensor's ros_image_transport and
/// ros_jpeg_quality attributes ask (nothing for "raw"). Thread safe: it runs
/// in the readback callbacks of the cameras.
CARLA_API FRosCompressedImage CompressImageForRos(ASensor &Sensor, const FColor *Pixels, int32 Width, int32 Height);

/// Loads the image compression module; call it on the game thread.
CARLA_API void LoadRosImageCompression();
