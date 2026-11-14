#include "capture_service_cpp/sources/imu_source.hpp"

#include "capture_service_cpp/timing/xreal_device_clock.hpp"
#include "capture_service_cpp/vendor/nreal_light_usb.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>

namespace xr_capture_cpp {
namespace {

class NrealLightImuSource final : public IImuSource {
 public:
  NrealLightImuSource(RuntimeConfig cfg, std::shared_ptr<XrealDeviceClock> clock)
      : cfg_(std::move(cfg)), clock_(std::move(clock)) {}

  std::string name() const override { return "nreal_light"; }

  std::vector<StreamSpec> stream_specs(uint32_t slot_count, uint32_t raw_slot_count) const override {
    std::vector<StreamSpec> specs;
    if (cfg_.imu.raw_enabled) {
      specs.push_back(StreamSpec{cfg_.imu.raw_stream_id, kKindBytes, "BYTES", 0, 0, kFormatBytes, "BYTES",
                                 static_cast<uint32_t>(cfg_.imu.raw_payload_size), raw_slot_count,
                                 cfg_.imu.raw_frame_id, "Raw OV580 HID IMU report"});
    }
    specs.push_back(StreamSpec{cfg_.imu.stream_id, kKindImu, "IMU", 0, 0, kFormatImuF32Le, "IMU_F32_LE", 24,
                               slot_count, cfg_.imu.frame_id,
                               "Nreal Light gyro rad/s and accelerometer m/s^2 in the OV580 IMU frame"});
    return specs;
  }

  void open() override {
    if (!clock_) throw std::runtime_error("Nreal Light IMU requires the shared device clock mapper");
    usb_ = acquire_nreal_light_usb(cfg_.nreal_light);
  }

  SourceReadStatus read(ImuReadResult& result) override {
    result = {};
    if (usb_->device_lost()) return SourceReadStatus::EndOfStream;
    NrealLightPacket packet;
    if (!usb_->take_imu(packet, 100)) return SourceReadStatus::Timeout;
    result.receive_timestamp_ns = packet.receive_ns;
    const size_t raw_size = std::min(packet.bytes.size(), cfg_.imu.raw_payload_size);
    result.raw_packet.assign(packet.bytes.begin(), packet.bytes.begin() + static_cast<long>(raw_size));

    auto sample = parse_nreal_light_imu(packet.bytes.data(), packet.bytes.size());
    if (!sample) return SourceReadStatus::TransportActivity;
    if (sample->device_ns <= last_device_ns_) return SourceReadStatus::TransportActivity;
    last_device_ns_ = sample->device_ns;
    if (cfg_.nreal_light.apply_factory_imu) apply_nreal_light_imu_factory(usb_->factory().imu, *sample);

    // The IMU trains the device clock model even when samples are published on the host clock.
    const auto mapped = clock_->map_imu_ns(sample->device_ns, packet.receive_ns);
    uint64_t timestamp_ns = packet.receive_ns;
    if (cfg_.camera.timestamp_mode == "device") {
      if (!mapped) return SourceReadStatus::TransportActivity;
      timestamp_ns = *mapped;
    }
    for (int i = 0; i < 3; ++i) {
      result.sample.gyro_rad_s[static_cast<size_t>(i)] = static_cast<float>(sample->gyro_rad_s[i]);
      result.sample.accel_m_s2[static_cast<size_t>(i)] = static_cast<float>(sample->accel_m_s2[i]);
    }
    result.sample.timestamp_ns = timestamp_ns;
    result.sample.source_sequence = static_cast<uint32_t>(++count_);
    result.sample.source_timestamp_valid = cfg_.camera.timestamp_mode == "device";
    result.has_sample = true;
    if (count_ % 1000 == 0) {
      const auto d = clock_->diagnostics();
      std::cerr << "[capture_service_cpp] imu source=nreal_light samples=" << count_
                << " clock_anchors=" << d.anchor_count << " clock_drift_ppm=" << d.drift_ppm << std::endl;
    }
    return SourceReadStatus::Data;
  }

  void close() override { usb_.reset(); }

 private:
  RuntimeConfig cfg_;
  std::shared_ptr<XrealDeviceClock> clock_;
  std::shared_ptr<NrealLightUsb> usb_;
  uint64_t last_device_ns_ = 0;
  uint64_t count_ = 0;
};

}  // namespace

std::unique_ptr<IImuSource> make_nreal_light_imu_source(
    const RuntimeConfig& cfg, std::shared_ptr<XrealDeviceClock> clock) {
  return std::make_unique<NrealLightImuSource>(cfg, std::move(clock));
}

}  // namespace xr_capture_cpp
