#include "capture_service_cpp/sources/imu_source.hpp"

#include <stdexcept>

namespace xr_capture_cpp {

std::unique_ptr<IImuSource> make_xreal_hid_imu_source(
    const RuntimeConfig& cfg, std::shared_ptr<XrealDeviceClock> clock);
std::unique_ptr<IImuSource> make_serial_imu_source(const RuntimeConfig& cfg);
std::unique_ptr<IImuSource> make_synthetic_imu_source(const RuntimeConfig& cfg);
#ifdef XR_CAPTURE_HAS_NREAL_LIGHT
std::unique_ptr<IImuSource> make_nreal_light_imu_source(
    const RuntimeConfig& cfg, std::shared_ptr<XrealDeviceClock> clock);
#endif

std::unique_ptr<IImuSource> create_imu_source(
    const RuntimeConfig& cfg, std::shared_ptr<XrealDeviceClock> xreal_clock) {
  if (cfg.imu.driver == "xreal_hid") return make_xreal_hid_imu_source(cfg, std::move(xreal_clock));
  if (cfg.imu.driver == "serial") return make_serial_imu_source(cfg);
  if (cfg.imu.driver == "synthetic") return make_synthetic_imu_source(cfg);
#ifdef XR_CAPTURE_HAS_NREAL_LIGHT
  if (cfg.imu.driver == "nreal_light") return make_nreal_light_imu_source(cfg, std::move(xreal_clock));
#else
  if (cfg.imu.driver == "nreal_light") {
    throw std::runtime_error("imu.driver=nreal_light needs a build with libusb-1.0");
  }
#endif
  throw std::runtime_error("unsupported IMU source: " + cfg.imu.driver);
}

}  // namespace xr_capture_cpp
