#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace xr_capture_cpp {

constexpr uint16_t kNrealLightOv580VendorId = 0x05a9;
constexpr uint16_t kNrealLightOv580ProductId = 0x0680;
constexpr int kNrealLightEyeWidth = 640;
constexpr int kNrealLightEyeHeight = 480;
constexpr size_t kNrealLightUvcChunk = 0x8000;
constexpr size_t kNrealLightBulkSize = 615908;
constexpr int64_t kNrealLightDefaultCameraOffsetUs = 39000;

constexpr uint8_t kNrealLightReportImu = 0x01;
constexpr uint8_t kNrealLightReportCommand = 0x02;
constexpr uint8_t kNrealLightCmdImuStream = 0x19;
constexpr uint8_t kNrealLightCmdConfigStart = 0x14;
constexpr uint8_t kNrealLightCmdConfigRead = 0x15;

// UVC probe/commit for the 640x480 stereo SLAM stream at 30 Hz.
extern const std::array<uint8_t, 34> kNrealLightUvcCommit;

struct NrealLightStereoPixels {
  std::vector<uint8_t> left;
  std::vector<uint8_t> right;
  // OV580 clock in nanoseconds, before the factory camera offset.
  uint64_t device_ns = 0;
};

// Removes the UVC header at every 0x8000 bytes and splits even rows into the
// left eye and odd rows into the right eye. The u64 after the pixels is the
// frame time on the OV580 clock.
bool decode_nreal_light_frame(const uint8_t* bulk, size_t size, NrealLightStereoPixels& out);

struct NrealLightImuSample {
  uint64_t device_ns = 0;
  std::array<double, 3> gyro_rad_s{};
  std::array<double, 3> accel_m_s2{};
};

// Gyro and accelerometer in the OV580 IMU frame, the frame used by the factory
// imu_q_cam. No axis flip is applied here; ar-drivers-rs flips y and z, which
// would turn the factory camera extrinsics by 180 degrees about x.
std::optional<NrealLightImuSample> parse_nreal_light_imu(const uint8_t* report, size_t size);

struct NrealLightImuFactory {
  std::array<double, 3> gyro_bias{};
  std::array<double, 3> accel_bias{};
  // Upper triangular: scale on the diagonal, skew xy, xz, yz above it.
  std::array<double, 9> accel_map{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  std::array<double, 9> gyro_map{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  // Row-major. Calibrated acceleration times this is added to the gyro.
  std::array<double, 9> g_sensitivity{};
  // Hamilton rotation that maps gyro axes onto accelerometer axes.
  std::array<double, 9> accel_from_gyro{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
};

struct NrealLightFactory {
  std::string json;
  NrealLightImuFactory imu;
  std::optional<int64_t> camera_imu_timestamp_offset_us;
};

// Reads the JSON object and the text trailer from the 0x15 config records.
bool parse_nreal_light_factory(const std::vector<uint8_t>& blob, NrealLightFactory& out, std::string& error);

void apply_nreal_light_imu_factory(const NrealLightImuFactory& factory, NrealLightImuSample& sample);

constexpr uint16_t kNrealLightMcuVendorId = 0x0486;
constexpr uint16_t kNrealLightMcuProductId = 0x573c;

struct NrealLightMcuPacket {
  uint8_t category = 0;
  uint8_t command = 0;
  std::vector<uint8_t> data;
};

// MCU HID packet: STX ':' category ':' command ':' data ':' '0' ':' crc32 hex ':' ETX,
// zero padded to 64 bytes.
std::optional<std::array<uint8_t, 64>> encode_nreal_light_mcu(const NrealLightMcuPacket& packet);
std::optional<NrealLightMcuPacket> decode_nreal_light_mcu(const uint8_t* bytes, size_t size);

}  // namespace xr_capture_cpp
