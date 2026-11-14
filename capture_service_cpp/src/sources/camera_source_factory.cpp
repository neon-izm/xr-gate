#include "capture_service_cpp/sources/camera_source.hpp"

#include <stdexcept>

namespace xr_capture_cpp {

std::unique_ptr<ICameraSource> make_xreal_camera_source(
    const RuntimeConfig& cfg, std::shared_ptr<XrealDeviceClock> clock);
std::unique_ptr<ICameraSource> make_opencv_stereo_camera_source(const RuntimeConfig& cfg);
#ifdef XR_CAPTURE_HAS_NREAL_LIGHT
std::unique_ptr<ICameraSource> make_nreal_light_camera_source(
    const RuntimeConfig& cfg, std::shared_ptr<XrealDeviceClock> clock);
#endif

std::unique_ptr<ICameraSource> create_camera_source(
    const RuntimeConfig& cfg, std::shared_ptr<XrealDeviceClock> xreal_clock) {
  if (cfg.camera.driver == "xreal_ultra") return make_xreal_camera_source(cfg, std::move(xreal_clock));
  if (cfg.camera.driver == "opencv") return make_opencv_stereo_camera_source(cfg);
#ifdef XR_CAPTURE_HAS_NREAL_LIGHT
  if (cfg.camera.driver == "nreal_light") return make_nreal_light_camera_source(cfg, std::move(xreal_clock));
#else
  if (cfg.camera.driver == "nreal_light") {
    throw std::runtime_error("camera.driver=nreal_light needs a build with libusb-1.0");
  }
#endif
  throw std::runtime_error("unsupported camera source: " + cfg.camera.driver);
}

}  // namespace xr_capture_cpp
