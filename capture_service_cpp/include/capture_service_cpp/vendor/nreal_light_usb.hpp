#pragma once

#include "capture_service_cpp/common.hpp"
#include "capture_service_cpp/vendor/nreal_light_codec.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace xr_capture_cpp {

struct NrealLightPacket {
  std::vector<uint8_t> bytes;
  uint64_t receive_ns = 0;
};

// One libusb session on the OV580 (05a9:0680). It claims the UVC streaming
// interface and the HID interface, reads the factory config once, then keeps
// bulk frame transfers and the IMU interrupt transfer in flight. macOS needs
// root to detach the kernel drivers from both interfaces.
class NrealLightUsb {
 public:
  explicit NrealLightUsb(const NrealLightConfig& cfg);
  ~NrealLightUsb();
  NrealLightUsb(const NrealLightUsb&) = delete;
  NrealLightUsb& operator=(const NrealLightUsb&) = delete;

  const NrealLightFactory& factory() const;
  int64_t camera_offset_ns() const;

  bool take_frame(NrealLightPacket& out, int timeout_ms);
  bool take_imu(NrealLightPacket& out, int timeout_ms);
  bool device_lost() const;

 private:
  struct State;
  std::unique_ptr<State> state_;
};

// The camera and IMU sources run in separate threads but must share one
// session. The first caller opens the device; it closes with the last owner.
std::shared_ptr<NrealLightUsb> acquire_nreal_light_usb(const NrealLightConfig& cfg);

// When started through sudo, switch back to SUDO_UID/SUDO_GID once the USB
// interfaces are claimed. The SHM segments and the registry are created with
// mode 0600, so Basalt running as the user can only read them if they are not
// owned by root. Returns false when there is nothing to drop.
bool drop_sudo_privileges(const NrealLightConfig& cfg);

}  // namespace xr_capture_cpp
