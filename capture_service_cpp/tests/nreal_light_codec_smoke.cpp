#include "capture_service_cpp/vendor/nreal_light_codec.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

using namespace xr_capture_cpp;

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(1);
  }
}

bool near(double a, double b, double tolerance = 1e-9) { return std::fabs(a - b) <= tolerance; }

template <typename T>
void put(std::vector<uint8_t>& buffer, size_t offset, T value) {
  std::memcpy(buffer.data() + offset, &value, sizeof(T));
}

void test_factory() {
  const std::string text =
      std::string("\x10\x00header", 8) +
      "\n\n{\"IMU\":{\"device_1\":{\"gyro_bias\":[0.1,0.2,0.3],\"accel_bias\":[1,2,3],"
      "\"scale_accel\":[1,1,1],\"scale_gyro\":[2,1,1],\"skew_accel\":[0,0,0],\"skew_gyro\":[0,0,0],"
      "\"gyro_g_sensitivity\":[0,0,0,0,0,0,0,0,0],\"accel_q_gyro\":[0,0,0,1]}}}"
      "\n\n\ncamera_imu_timestamp_offset_us = 39000\nsensor_orientation = -2 -1 -3\n\n";
  const std::vector<uint8_t> blob(text.begin(), text.end());
  NrealLightFactory factory;
  std::string error;
  require(parse_nreal_light_factory(blob, factory, error), error.c_str());
  require(factory.camera_imu_timestamp_offset_us == 39000, "trailer camera offset");
  require(near(factory.imu.gyro_bias[2], 0.3), "gyro bias");
  require(near(factory.imu.gyro_map[0], 2.0), "gyro scale");

  NrealLightImuSample sample;
  sample.gyro_rad_s = {1.1, 0.2, 0.3};
  sample.accel_m_s2 = {1.0, 2.0, 12.81};
  apply_nreal_light_imu_factory(factory.imu, sample);
  require(near(sample.gyro_rad_s[0], 2.0), "gyro bias then scale");
  require(near(sample.gyro_rad_s[1], 0.0), "gyro y bias");
  require(near(sample.accel_m_s2[2], 9.81), "accel bias");
}

void test_imu_report() {
  std::vector<uint8_t> report(128, 0);
  report[0] = kNrealLightReportImu;
  put<uint64_t>(report, 44, 5000000ULL);
  put<uint32_t>(report, 52, 1);
  put<uint32_t>(report, 56, 1);
  put<int32_t>(report, 60, 90);
  put<int32_t>(report, 64, -90);
  put<int32_t>(report, 68, 0);
  put<uint64_t>(report, 72, 5000000ULL);
  put<uint32_t>(report, 80, 1);
  put<uint32_t>(report, 84, 2);
  put<int32_t>(report, 88, 0);
  put<int32_t>(report, 92, -2);
  put<int32_t>(report, 96, 0);
  const auto sample = parse_nreal_light_imu(report.data(), report.size());
  require(sample.has_value(), "IMU report parses");
  require(sample->device_ns == 5000000ULL, "IMU device time is nanoseconds");
  require(near(sample->gyro_rad_s[0], M_PI / 2, 1e-12), "gyro x in rad/s");
  require(near(sample->gyro_rad_s[1], -M_PI / 2, 1e-12), "gyro y keeps the OV580 sign");
  require(near(sample->accel_m_s2[1], -9.81, 1e-12), "accel y keeps the OV580 sign");
}

void test_frame() {
  const size_t pixels = static_cast<size_t>(kNrealLightEyeWidth) * kNrealLightEyeHeight;
  std::vector<uint8_t> payload(pixels * 2 + 8, 0);
  for (int y = 0; y < kNrealLightEyeHeight; ++y) {
    std::memset(payload.data() + static_cast<size_t>(y) * 2 * kNrealLightEyeWidth, 10, kNrealLightEyeWidth);
    std::memset(payload.data() + (static_cast<size_t>(y) * 2 + 1) * kNrealLightEyeWidth, 200, kNrealLightEyeWidth);
  }
  put<uint64_t>(payload, pixels * 2, 123456789ULL);

  std::vector<uint8_t> bulk;
  size_t read = 0;
  constexpr uint8_t kHeader = 12;
  while (read < payload.size()) {
    bulk.insert(bulk.end(), kHeader, 0);
    bulk[bulk.size() - kHeader] = kHeader;
    const size_t len = std::min(kNrealLightUvcChunk - kHeader, payload.size() - read);
    bulk.insert(bulk.end(), payload.begin() + static_cast<long>(read), payload.begin() + static_cast<long>(read + len));
    read += len;
  }
  NrealLightStereoPixels frame;
  require(decode_nreal_light_frame(bulk.data(), bulk.size(), frame), "frame decodes");
  require(frame.device_ns == 123456789ULL, "frame device time");
  require(frame.left[0] == 10 && frame.left[pixels - 1] == 10, "even rows are left");
  require(frame.right[0] == 200 && frame.right[pixels - 1] == 200, "odd rows are right");
}

void test_mcu_packet() {
  const auto encoded = encode_nreal_light_mcu({'@', 'K', {'x'}});
  require(encoded.has_value(), "heartbeat encodes");
  const std::string text(reinterpret_cast<const char*>(encoded->data()), 20);
  require(text == std::string("\x02:@:K:x:0:69dd970a:\x03"), "heartbeat matches zlib crc32");
  const auto decoded = decode_nreal_light_mcu(encoded->data(), encoded->size());
  require(decoded && decoded->category == '@' && decoded->command == 'K' &&
              decoded->data == std::vector<uint8_t>{'x'},
          "heartbeat decodes");
}

}  // namespace

int main() {
  test_factory();
  test_imu_report();
  test_frame();
  test_mcu_packet();
  std::cout << "nreal_light_codec_smoke ok" << std::endl;
  return 0;
}
