#include "capture_service_cpp/platform/camera_controls.hpp"

#include <iostream>
#include <stdexcept>

namespace xr_capture_cpp {

// AVFoundation does not expose UVC extension controls through OpenCV, and the
// Nreal Light driver configures its stream through libusb instead.
void apply_camera_controls(const CameraDeviceConfig& cfg, const std::string& label) {
  if (cfg.controls.values.empty()) return;
  const std::string message = label + ": camera controls are not supported on macOS";
  if (cfg.controls.strict) throw std::runtime_error(message);
  std::cerr << "[capture_service_cpp][WARN] " << message << std::endl;
}

}  // namespace xr_capture_cpp
