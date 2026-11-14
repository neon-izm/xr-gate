#include "capture_service_cpp/vendor/nreal_light_usb.hpp"

#include <hidapi/hidapi.h>
#include <libusb.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace xr_capture_cpp {
namespace {

constexpr int kUvcInterface = 1;
constexpr uint8_t kBulkEndpoint = 0x81;
constexpr size_t kFrameQueueLimit = 4;
constexpr size_t kImuQueueLimit = 8000;

std::string usb_error(const std::string& what, int status) {
  std::string text = what + ": " + libusb_error_name(status);
  if (geteuid() != 0 && (status == LIBUSB_ERROR_ACCESS || status == LIBUSB_ERROR_BUSY)) {
    text += " (macOS needs root to detach the OV580 UVC and HID drivers)";
  }
  return text;
}

// Without a host talking to the glasses MCU, the OV580 stream stalls and the
// OV580 can drop off the bus. NrealLightSLAM and ar-drivers send "@3 1"
// (SDK active) and then an "@K" heartbeat every 250 ms.
class NrealLightMcu {
 public:
  NrealLightMcu() {
    hid_init();
    hid_device_info* infos = hid_enumerate(kNrealLightMcuVendorId, kNrealLightMcuProductId);
    for (hid_device_info* info = infos; info != nullptr && device_ == nullptr; info = info->next) {
      hid_device* device = hid_open_path(info->path);
      if (device == nullptr) continue;
      device_ = device;
      if (!request('@', '3', {'1'})) {
        hid_close(device_);
        device_ = nullptr;
      }
    }
    hid_free_enumeration(infos);
    if (device_ == nullptr) throw std::runtime_error("Nreal Light MCU 0486:573c did not answer the SDK command");
    running_ = true;
    heartbeat_ = std::thread([this] { heartbeat(); });
  }

  ~NrealLightMcu() {
    running_ = false;
    if (heartbeat_.joinable()) heartbeat_.join();
    if (device_) hid_close(device_);
  }

  NrealLightMcu(const NrealLightMcu&) = delete;
  NrealLightMcu& operator=(const NrealLightMcu&) = delete;

 private:
  bool send(uint8_t category, uint8_t command, std::vector<uint8_t> data) {
    const auto encoded = encode_nreal_light_mcu({category, command, std::move(data)});
    return encoded && hid_write(device_, encoded->data(), encoded->size()) >= 0;
  }

  bool request(uint8_t category, uint8_t command, std::vector<uint8_t> data) {
    if (!send(category, command, std::move(data))) return false;
    for (int i = 0; i < 16; ++i) {
      unsigned char buffer[64] = {};
      const int n = hid_read_timeout(device_, buffer, sizeof(buffer), 100);
      if (n <= 0) return false;
      const auto reply = decode_nreal_light_mcu(buffer, static_cast<size_t>(n));
      if (reply && reply->category == category + 1 && reply->command == command) return true;
    }
    return false;
  }

  void heartbeat() {
    auto next = std::chrono::steady_clock::now();
    while (running_) {
      if (std::chrono::steady_clock::now() >= next) {
        send('@', 'K', {'x'});
        next += std::chrono::milliseconds(250);
      }
      // Drain key, proximity and heartbeat replies so the MCU queue does not fill.
      unsigned char buffer[64];
      hid_read_timeout(device_, buffer, sizeof(buffer), 50);
    }
  }

  hid_device* device_ = nullptr;
  std::atomic<bool> running_{false};
  std::thread heartbeat_;
};

}  // namespace

struct NrealLightUsb::State {
  struct BulkSlot {
    State* state = nullptr;
    libusb_transfer* transfer = nullptr;
    std::vector<unsigned char> buffer;
    std::atomic<bool> inflight{false};
  };

  NrealLightConfig cfg;
  NrealLightFactory factory;
  std::unique_ptr<NrealLightMcu> mcu;
  int64_t camera_offset_ns = kNrealLightDefaultCameraOffsetUs * 1000;

  libusb_context* context = nullptr;
  libusb_device_handle* handle = nullptr;
  int hid_interface = -1;
  uint8_t interrupt_in = 0;
  int interrupt_packet = 128;
  bool claimed_video = false;
  bool claimed_hid = false;

  std::vector<std::unique_ptr<BulkSlot>> bulks;
  libusb_transfer* imu = nullptr;
  std::vector<unsigned char> imu_buffer;
  std::atomic<bool> imu_inflight{false};

  std::atomic<bool> running{false};
  std::atomic<bool> lost{false};
  // Set after overflow, stall or error. Committing again on this firmware can
  // drop the OV580 off the bus, so the session stops reading frames instead.
  std::atomic<bool> stream_broken{false};
  std::atomic<int> good_frames{0};
  std::thread event_thread;

  std::mutex mutex;
  std::condition_variable frame_cv;
  std::condition_variable imu_cv;
  std::deque<NrealLightPacket> frames;
  std::deque<NrealLightPacket> imus;
  uint64_t dropped_frames = 0;
  uint64_t bad_bulk = 0;

  bool any_inflight() const {
    if (imu_inflight.load()) return true;
    for (const auto& slot : bulks) {
      if (slot->inflight.load()) return true;
    }
    return false;
  }

  void open_device() {
    if (cfg.mcu_heartbeat) mcu = std::make_unique<NrealLightMcu>();
    if (libusb_init(&context) != 0) throw std::runtime_error("libusb_init failed");
    // The OV580 enumerates again a moment after the MCU powers it on.
    for (int i = 0; i < 100 && !handle; ++i) {
      handle = libusb_open_device_with_vid_pid(context, kNrealLightOv580VendorId, kNrealLightOv580ProductId);
      if (!handle) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!handle) throw std::runtime_error("Nreal Light OV580 05a9:0680 not found");
    libusb_set_auto_detach_kernel_driver(handle, 1);

    claim(kUvcInterface, "claim OV580 UVC interface");
    claimed_video = true;
    find_hid_endpoint();
    claim(hid_interface, "claim OV580 HID interface");
    claimed_hid = true;
  }

  void claim(int interface_number, const std::string& what) {
    int status = libusb_claim_interface(handle, interface_number);
    if (status != 0) {
      libusb_detach_kernel_driver(handle, interface_number);
      status = libusb_claim_interface(handle, interface_number);
    }
    if (status != 0) throw std::runtime_error(usb_error(what, status));
  }

  void find_hid_endpoint() {
    libusb_config_descriptor* config = nullptr;
    if (libusb_get_active_config_descriptor(libusb_get_device(handle), &config) != 0 || !config) {
      throw std::runtime_error("cannot read OV580 configuration descriptor");
    }
    for (uint8_t i = 0; i < config->bNumInterfaces && hid_interface < 0; ++i) {
      const libusb_interface& iface = config->interface[i];
      if (iface.num_altsetting < 1) continue;
      const libusb_interface_descriptor& desc = iface.altsetting[0];
      if (desc.bInterfaceClass != LIBUSB_CLASS_HID) continue;
      for (uint8_t e = 0; e < desc.bNumEndpoints; ++e) {
        const libusb_endpoint_descriptor& ep = desc.endpoint[e];
        const bool interrupt = (ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_INTERRUPT;
        const bool in = (ep.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN;
        if (!interrupt || !in) continue;
        hid_interface = desc.bInterfaceNumber;
        interrupt_in = ep.bEndpointAddress;
        // macOS IOHID cuts report 2 to 64 bytes; the raw endpoint delivers 128.
        interrupt_packet = 128;
        break;
      }
    }
    libusb_free_config_descriptor(config);
    if (hid_interface < 0) throw std::runtime_error("OV580 HID interrupt IN endpoint not found");
  }

  void hid_command(uint8_t cmd, uint8_t subcmd) {
    unsigned char payload[7] = {kNrealLightReportCommand, cmd, subcmd, 0, 0, 0, 0};
    const int written = libusb_control_transfer(handle, 0x21, 0x09, 0x0202,
                                                static_cast<uint16_t>(hid_interface), payload,
                                                sizeof(payload), 1000);
    if (written < 0) throw std::runtime_error(usb_error("OV580 SET_REPORT", written));
  }

  std::vector<uint8_t> hid_command_reply(uint8_t cmd, uint8_t subcmd) {
    hid_command(cmd, subcmd);
    for (int i = 0; i < 400; ++i) {
      unsigned char buffer[128] = {};
      int got = 0;
      const int status = libusb_interrupt_transfer(handle, interrupt_in, buffer, interrupt_packet, &got, 250);
      if (status != 0) throw std::runtime_error(usb_error("OV580 interrupt read", status));
      if (got > 0 && buffer[0] == kNrealLightReportCommand) return {buffer, buffer + got};
    }
    throw std::runtime_error("OV580 did not answer a HID command");
  }

  void read_factory() {
    hid_command_reply(kNrealLightCmdImuStream, 0x00);
    hid_command_reply(kNrealLightCmdConfigStart, 0x00);
    std::vector<uint8_t> blob;
    for (int i = 0; i < 1024; ++i) {
      const auto report = hid_command_reply(kNrealLightCmdConfigRead, 0x00);
      if (report.size() < 3 || report[1] != 1) break;
      const size_t len = std::min<size_t>(report[2], report.size() - 3);
      if (len == 0) break;
      blob.insert(blob.end(), report.begin() + 3, report.begin() + 3 + static_cast<long>(len));
    }
    if (blob.empty()) throw std::runtime_error("OV580 factory config is empty");

    std::string error;
    if (!parse_nreal_light_factory(blob, factory, error)) throw std::runtime_error(error);
    if (!cfg.factory_config_path.empty()) {
      std::ofstream out(cfg.factory_config_path, std::ios::binary | std::ios::trunc);
      out.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
      if (!out) throw std::runtime_error("cannot write " + cfg.factory_config_path);
    }
    int64_t offset_us = cfg.camera_timestamp_offset_us;
    if (offset_us == 0) {
      offset_us = factory.camera_imu_timestamp_offset_us.value_or(kNrealLightDefaultCameraOffsetUs);
    }
    camera_offset_ns = offset_us * 1000;
    std::cerr << "[capture_service_cpp] nreal_light factory config bytes=" << blob.size()
              << " path=" << cfg.factory_config_path << " camera_offset_us=" << offset_us << std::endl;
  }

  void commit_uvc() {
    auto commit = kNrealLightUvcCommit;
    const int written = libusb_control_transfer(handle, 0x21, 0x01, 0x0200, kUvcInterface, commit.data(),
                                                static_cast<uint16_t>(commit.size()), 1000);
    if (written < 0) throw std::runtime_error(usb_error("UVC commit", written));
  }

  void start_streams() {
    libusb_clear_halt(handle, kBulkEndpoint);
    commit_uvc();
    hid_command(kNrealLightCmdImuStream, 0x01);
    running = true;

    imu_buffer.resize(static_cast<size_t>(interrupt_packet));
    imu = libusb_alloc_transfer(0);
    libusb_fill_interrupt_transfer(imu, handle, interrupt_in, imu_buffer.data(),
                                   static_cast<int>(imu_buffer.size()), &State::on_imu, this, 0);
    if (libusb_submit_transfer(imu) != 0) throw std::runtime_error("cannot submit OV580 IMU transfer");
    imu_inflight = true;

    for (int i = 0; i < cfg.bulk_transfers; ++i) {
      auto slot = std::make_unique<BulkSlot>();
      slot->state = this;
      slot->buffer.resize(kNrealLightBulkSize);
      slot->transfer = libusb_alloc_transfer(0);
      libusb_fill_bulk_transfer(slot->transfer, handle, kBulkEndpoint, slot->buffer.data(),
                                static_cast<int>(slot->buffer.size()), &State::on_bulk, slot.get(), 1000);
      if (libusb_submit_transfer(slot->transfer) != 0) {
        libusb_free_transfer(slot->transfer);
        break;
      }
      slot->inflight = true;
      bulks.push_back(std::move(slot));
    }
    if (bulks.empty()) throw std::runtime_error("cannot submit OV580 bulk transfers");
    event_thread = std::thread([this] { events(); });
  }

  static void LIBUSB_CALL on_bulk(libusb_transfer* transfer) {
    auto* slot = static_cast<BulkSlot*>(transfer->user_data);
    State* state = slot->state;
    slot->inflight = false;
    const uint64_t now = steady_ns();
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED &&
        transfer->actual_length == static_cast<int>(kNrealLightBulkSize) && transfer->buffer[0] != 0) {
      NrealLightPacket packet{{transfer->buffer, transfer->buffer + transfer->actual_length}, now};
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->frames.size() >= kFrameQueueLimit) {
          state->frames.pop_front();
          ++state->dropped_frames;
        }
        state->frames.push_back(std::move(packet));
      }
      ++state->good_frames;
      state->frame_cv.notify_one();
    } else if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE) {
      state->lost = true;
    } else if (transfer->status != LIBUSB_TRANSFER_CANCELLED) {
      if (++state->bad_bulk <= 8) {
        std::cerr << "[capture_service_cpp][WARN] nreal_light bulk status=" << libusb_error_name(transfer->status)
                  << " length=" << transfer->actual_length << std::endl;
      }
    }
    if (transfer->status == LIBUSB_TRANSFER_OVERFLOW || transfer->status == LIBUSB_TRANSFER_STALL ||
        transfer->status == LIBUSB_TRANSFER_ERROR) {
      if (!state->stream_broken.exchange(true) && state->good_frames.load() > 0) {
        std::cerr << "[capture_service_cpp][ERROR] nreal_light stereo stream lost frame alignment; restart capture"
                  << std::endl;
      }
      state->frame_cv.notify_all();
      return;
    }
    if (state->running && !state->stream_broken && !state->lost && libusb_submit_transfer(transfer) == 0) {
      slot->inflight = true;
    }
  }

  static void LIBUSB_CALL on_imu(libusb_transfer* transfer) {
    auto* state = static_cast<State*>(transfer->user_data);
    state->imu_inflight = false;
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED && transfer->actual_length > 0) {
      NrealLightPacket packet{{transfer->buffer, transfer->buffer + transfer->actual_length}, steady_ns()};
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->imus.size() >= kImuQueueLimit) state->imus.pop_front();
        state->imus.push_back(std::move(packet));
      }
      state->imu_cv.notify_one();
    } else if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE) {
      state->lost = true;
    }
    if (state->running && !state->lost && libusb_submit_transfer(transfer) == 0) state->imu_inflight = true;
  }

  void events() {
    while (running || any_inflight()) {
      if (!running) {
        for (auto& slot : bulks) {
          if (slot->inflight) libusb_cancel_transfer(slot->transfer);
        }
        if (imu_inflight) libusb_cancel_transfer(imu);
      }
      timeval tv{0, 100000};
      libusb_handle_events_timeout_completed(context, &tv, nullptr);
    }
  }

  bool wait_stream_started(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex);
    frame_cv.wait_for(lock, timeout, [&] { return good_frames.load() >= 3 || stream_broken.load(); });
    return good_frames.load() >= 3 && !stream_broken.load();
  }

  void reset_session() {
    std::lock_guard<std::mutex> lock(mutex);
    frames.clear();
    imus.clear();
    hid_interface = -1;
    lost = false;
    stream_broken = false;
    good_frames = 0;
    bad_bulk = 0;
  }

  void close() {
    running = false;
    frame_cv.notify_all();
    imu_cv.notify_all();
    if (event_thread.joinable()) event_thread.join();
    for (auto& slot : bulks) {
      if (slot->transfer) libusb_free_transfer(slot->transfer);
    }
    bulks.clear();
    if (imu) libusb_free_transfer(imu);
    imu = nullptr;
    if (handle && claimed_hid) {
      try {
        hid_command(kNrealLightCmdImuStream, 0x00);
      } catch (...) {
      }
      libusb_release_interface(handle, hid_interface);
    }
    if (handle && claimed_video) {
      // CLEAR_FEATURE(ENDPOINT_HALT) stops a bulk UVC stream.
      libusb_clear_halt(handle, kBulkEndpoint);
      libusb_release_interface(handle, kUvcInterface);
    }
    claimed_hid = claimed_video = false;
    if (handle) libusb_close(handle);
    handle = nullptr;
    if (context) libusb_exit(context);
    context = nullptr;
    mcu.reset();
  }
};

NrealLightUsb::NrealLightUsb(const NrealLightConfig& cfg) : state_(std::make_unique<State>()) {
  state_->cfg = cfg;
  // On macOS every open and close re-enumerates the OV580. Roughly every second
  // session the stream does not start or starts out of frame alignment.
  // Closing the session and opening it again starts it cleanly.
  constexpr int kAttempts = 4;
  for (int attempt = 1;; ++attempt) {
    try {
      state_->open_device();
      if (attempt == 1) state_->read_factory();
      state_->start_streams();
      if (state_->wait_stream_started(std::chrono::milliseconds(2500))) return;
      if (attempt == kAttempts) throw std::runtime_error("Nreal Light stereo stream did not start");
      std::cerr << "[capture_service_cpp][WARN] nreal_light stereo stream did not start; reopening (attempt "
                << attempt + 1 << "/" << kAttempts << ")" << std::endl;
    } catch (...) {
      state_->close();
      throw;
    }
    state_->close();
    state_->reset_session();
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
}

NrealLightUsb::~NrealLightUsb() { state_->close(); }

const NrealLightFactory& NrealLightUsb::factory() const { return state_->factory; }

int64_t NrealLightUsb::camera_offset_ns() const { return state_->camera_offset_ns; }

bool NrealLightUsb::device_lost() const { return state_->lost.load(); }

bool NrealLightUsb::take_frame(NrealLightPacket& out, int timeout_ms) {
  std::unique_lock<std::mutex> lock(state_->mutex);
  state_->frame_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                            [&] { return !state_->frames.empty() || !state_->running; });
  if (state_->frames.empty()) return false;
  out = std::move(state_->frames.front());
  state_->frames.pop_front();
  return true;
}

bool NrealLightUsb::take_imu(NrealLightPacket& out, int timeout_ms) {
  std::unique_lock<std::mutex> lock(state_->mutex);
  state_->imu_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                          [&] { return !state_->imus.empty() || !state_->running; });
  if (state_->imus.empty()) return false;
  out = std::move(state_->imus.front());
  state_->imus.pop_front();
  return true;
}

std::shared_ptr<NrealLightUsb> acquire_nreal_light_usb(const NrealLightConfig& cfg) {
  static std::mutex mutex;
  static std::weak_ptr<NrealLightUsb> shared;
  std::lock_guard<std::mutex> lock(mutex);
  if (auto existing = shared.lock()) return existing;
  auto created = std::make_shared<NrealLightUsb>(cfg);
  shared = created;
  return created;
}

bool drop_sudo_privileges(const NrealLightConfig& cfg) {
  if (geteuid() != 0) return false;
  const char* uid_text = std::getenv("SUDO_UID");
  const char* gid_text = std::getenv("SUDO_GID");
  if (uid_text == nullptr || gid_text == nullptr) return false;
  const auto uid = static_cast<uid_t>(std::strtoul(uid_text, nullptr, 10));
  const auto gid = static_cast<gid_t>(std::strtoul(gid_text, nullptr, 10));
  if (uid == 0) return false;
  if (!cfg.factory_config_path.empty() && ::chown(cfg.factory_config_path.c_str(), uid, gid) != 0) {
    std::cerr << "[capture_service_cpp][WARN] cannot chown " << cfg.factory_config_path << std::endl;
  }
  if (::setgid(gid) != 0 || ::setuid(uid) != 0) {
    throw std::runtime_error("cannot drop root privileges to SUDO_UID=" + std::string(uid_text));
  }
  std::cerr << "[capture_service_cpp] nreal_light running as uid=" << uid << " after claiming USB" << std::endl;
  return true;
}

}  // namespace xr_capture_cpp
