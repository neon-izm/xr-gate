#include "capture_service_cpp/sources/camera_source.hpp"

#include "capture_service_cpp/image_transform.hpp"
#include "capture_service_cpp/timing/xreal_device_clock.hpp"
#include "capture_service_cpp/vendor/nreal_light_usb.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>

namespace xr_capture_cpp {
namespace {

class NrealLightCameraSource final : public ICameraSource {
 public:
  NrealLightCameraSource(RuntimeConfig cfg, std::shared_ptr<XrealDeviceClock> clock)
      : cfg_(std::move(cfg)), clock_(std::move(clock)) {}

  std::string name() const override { return "nreal_light"; }

  std::vector<StreamSpec> stream_specs(uint32_t slot_count) const override {
    const auto& c = cfg_.camera;
    const auto payload = static_cast<uint32_t>(c.output_width * c.output_height);
    return {
        StreamSpec{c.left_stream_id, kKindImage, "IMAGE", static_cast<uint32_t>(c.output_width),
                   static_cast<uint32_t>(c.output_height), kFormatGray8, "GRAY8", payload, slot_count,
                   c.left_frame_id, "Nreal Light OV580 left SLAM camera"},
        StreamSpec{c.right_stream_id, kKindImage, "IMAGE", static_cast<uint32_t>(c.output_width),
                   static_cast<uint32_t>(c.output_height), kFormatGray8, "GRAY8", payload, slot_count,
                   c.right_frame_id, "Nreal Light OV580 right SLAM camera"}};
  }

  void open() override {
    if (cfg_.camera.timestamp_mode == "device" && !clock_) {
      throw std::runtime_error("Nreal Light device camera timestamps require the shared IMU clock mapper");
    }
    usb_ = acquire_nreal_light_usb(cfg_.nreal_light);
  }

  SourceReadStatus read(StereoFrame& output) override {
    output = {};
    if (usb_->device_lost()) return SourceReadStatus::EndOfStream;
    NrealLightPacket packet;
    if (!usb_->take_frame(packet, 100)) return SourceReadStatus::Timeout;
    if (!decode_nreal_light_frame(packet.bytes.data(), packet.bytes.size(), pixels_)) {
      ++decode_fail_;
      return SourceReadStatus::TransportActivity;
    }

    uint64_t timestamp_ns = packet.receive_ns;
    if (cfg_.camera.timestamp_mode == "device") {
      const uint64_t device_ns = pixels_.device_ns + static_cast<uint64_t>(usb_->camera_offset_ns());
      const auto mapped = clock_->map_camera_ns(device_ns, packet.receive_ns);
      if (!mapped || *mapped <= last_timestamp_ns_) {
        ++unmapped_;
        return SourceReadStatus::TransportActivity;
      }
      timestamp_ns = *mapped;
    }

    const cv::Mat left(kNrealLightEyeHeight, kNrealLightEyeWidth, CV_8UC1, pixels_.left.data());
    const cv::Mat right(kNrealLightEyeHeight, kNrealLightEyeWidth, CV_8UC1, pixels_.right.data());
    output.left = transform_gray_image(left, cfg_.camera.left_transform.rotate, cfg_.camera.left_transform.flip);
    output.right = transform_gray_image(right, cfg_.camera.right_transform.rotate, cfg_.camera.right_transform.flip);
    if (output.left.data == left.data) output.left = left.clone();
    if (output.right.data == right.data) output.right = right.clone();
    if (output.left.cols != cfg_.camera.output_width || output.left.rows != cfg_.camera.output_height) {
      throw std::runtime_error("Nreal Light output is " + std::to_string(output.left.cols) + "x" +
                               std::to_string(output.left.rows) + ", configured " +
                               std::to_string(cfg_.camera.output_width) + "x" +
                               std::to_string(cfg_.camera.output_height));
    }
    output.timestamp_ns = timestamp_ns;
    last_timestamp_ns_ = timestamp_ns;
    ++published_;

    const uint64_t now = steady_ns();
    if (now - last_log_ns_ > 2000000000ULL) {
      std::cerr << "[capture_service_cpp] camera source=nreal_light pairs=" << published_
                << " decode_fail=" << decode_fail_ << " unmapped=" << unmapped_
                << " latency_ms=" << (static_cast<double>(packet.receive_ns) - static_cast<double>(timestamp_ns)) * 1e-6
                << std::endl;
      last_log_ns_ = now;
    }
    return SourceReadStatus::Data;
  }

  void close() override { usb_.reset(); }

 private:
  RuntimeConfig cfg_;
  std::shared_ptr<XrealDeviceClock> clock_;
  std::shared_ptr<NrealLightUsb> usb_;
  NrealLightStereoPixels pixels_;
  uint64_t last_timestamp_ns_ = 0;
  uint64_t decode_fail_ = 0;
  uint64_t unmapped_ = 0;
  uint64_t published_ = 0;
  uint64_t last_log_ns_ = steady_ns();
};

}  // namespace

std::unique_ptr<ICameraSource> make_nreal_light_camera_source(
    const RuntimeConfig& cfg, std::shared_ptr<XrealDeviceClock> clock) {
  return std::make_unique<NrealLightCameraSource>(cfg, std::move(clock));
}

}  // namespace xr_capture_cpp
