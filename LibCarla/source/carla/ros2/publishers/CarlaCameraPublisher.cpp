// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/publishers/CarlaCameraPublisher.h"

#include "carla/Logging.h"
#include "carla/ros2/publishers/CameraIntrinsics.h"
#include "carla/ros2/publishers/PublisherImpl.h"
#include "carla/ros2/types/msg/CameraInfo.h"
#include "carla/ros2/types/msg/CompressedImage.h"
#include "carla/ros2/types/msg/Image.h"

#include <cmath>
#include <utility>

namespace carla {
namespace ros2 {

struct CarlaCameraImageMsgTraits {
  using msg_type = msg::Image;
};

struct CarlaCameraInfoMsgTraits {
  using msg_type = msg::CameraInfo;
};

struct CarlaCameraCompressedImageMsgTraits {
  using msg_type = msg::CompressedImage;
};

CarlaCameraPublisher::CarlaCameraPublisher(
    std::string base_topic_name, std::string frame_id, bool exact_topic)
  : BasePublisher(std::move(base_topic_name), std::move(frame_id)),
    _impl_image(std::make_shared<PublisherImpl<CarlaCameraImageMsgTraits>>()),
    _impl_camera_info(std::make_shared<PublisherImpl<CarlaCameraInfoMsgTraits>>()) {
  std::string info_topic = GetBaseTopicName() + "/camera_info";
  _image_topic = GetBaseTopicName() + "/image";
  if (exact_topic) {
    _image_topic = GetBaseTopicName();
    const auto slash = _image_topic.find_last_of('/');
    info_topic = (slash == std::string::npos ? std::string{} : _image_topic.substr(0, slash + 1)) + "camera_info";
  }
  if (!_impl_image->Init(_image_topic)) {
    log_error("CarlaCameraPublisher: failed to initialise image writer for", _image_topic);
  }
  if (!_impl_camera_info->Init(info_topic)) {
    log_error("CarlaCameraPublisher: failed to initialise camera_info writer for", info_topic);
  }
}

CarlaCameraPublisher::~CarlaCameraPublisher() = default;

bool CarlaCameraPublisher::Publish() {
  // Publish every written writer so a failure on one does not silence the
  // others. Image goes first to match the previous per-sensor publishers.
  bool ok = true;
  if (_image_written) {
    ok &= _impl_image->Publish();
  }
  if (_compressed_written && _impl_compressed) {
    ok &= _impl_compressed->Publish();
  }
  ok &= _impl_camera_info->Publish();
  _image_written = false;
  _compressed_written = false;
  return ok;
}

bool CarlaCameraPublisher::WriteCompressedImage(
    int32_t seconds,
    uint32_t nanoseconds,
    std::string format,
    std::vector<uint8_t> data) {
  if (!_impl_compressed) {
    _impl_compressed = std::make_shared<PublisherImpl<CarlaCameraCompressedImageMsgTraits>>();
    if (!_impl_compressed->Init(_image_topic + "/compressed")) {
      log_error("CarlaCameraPublisher: failed to initialise compressed image writer for", _image_topic);
      _impl_compressed.reset();
      return false;
    }
  }
  auto *message = _impl_compressed->GetMessage();
  message->header.stamp.sec = seconds;
  message->header.stamp.nanosec = nanoseconds;
  message->header.frame_id = GetFrameId();
  message->format = std::move(format);
  message->data = std::move(data);
  _compressed_written = true;
  return true;
}

std::vector<uint8_t> CarlaCameraPublisher::ComputeImage(
    uint32_t height, uint32_t width, const uint8_t *data) const {
  const size_t size =
      static_cast<size_t>(height) * static_cast<size_t>(width) * GetChannels();
  return std::vector<uint8_t>(data, data + size);
}

bool CarlaCameraPublisher::WriteCameraInfo(
    int32_t seconds,
    uint32_t nanoseconds,
    uint32_t x_offset,
    uint32_t y_offset,
    uint32_t height,
    uint32_t width,
    float fov,
    bool do_rectify) {
  auto *message = _impl_camera_info->GetMessage();
  message->header.stamp.sec = seconds;
  message->header.stamp.nanosec = nanoseconds;
  message->header.frame_id = GetFrameId();

  const auto k = ComputeIntrinsics(width, height, fov);

  message->height = height;
  message->width = width;
  message->distortion_model = "plumb_bob";
  message->d = {0.0, 0.0, 0.0, 0.0, 0.0};
  message->k = {k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0};
  message->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  message->p = {k.fx, 0.0, k.cx, 0.0, 0.0, k.fy, k.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  message->binning_x = 0;
  message->binning_y = 0;

  message->roi.x_offset = x_offset;
  message->roi.y_offset = y_offset;
  message->roi.height = height;
  message->roi.width = width;
  message->roi.do_rectify = do_rectify;

  return true;
}

bool CarlaCameraPublisher::WriteImage(
    int32_t seconds,
    uint32_t nanoseconds,
    uint32_t height,
    uint32_t width,
    const uint8_t *data) {
  return WriteImage(seconds, nanoseconds, height, width, ComputeImage(height, width, data));
}

bool CarlaCameraPublisher::WriteImage(
    int32_t seconds,
    uint32_t nanoseconds,
    uint32_t height,
    uint32_t width,
    std::vector<uint8_t> data) {
  auto *message = _impl_image->GetMessage();
  message->header.stamp.sec = seconds;
  message->header.stamp.nanosec = nanoseconds;
  message->header.frame_id = GetFrameId();

  message->width = width;
  message->height = height;
  message->encoding = GetEncoding();
  message->is_bigendian = 0;
  message->step = width * GetChannels() * static_cast<uint32_t>(sizeof(uint8_t));
  message->data = std::move(data);
  _image_written = true;

  return true;
}

}  // namespace ros2
}  // namespace carla
