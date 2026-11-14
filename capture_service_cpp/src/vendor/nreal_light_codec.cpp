#include "capture_service_cpp/vendor/nreal_light_codec.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <cstring>
#include <exception>

namespace xr_capture_cpp {

const std::array<uint8_t, 34> kNrealLightUvcCommit = {
    0x01, 0x00, 0x01, 0x01, 0x15, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x65, 0x09, 0x00, 0x00, 0x80,
    0x00, 0x00, 0x80, 0xd1, 0xf0, 0x08, 0x08, 0xf0, 0xa9, 0x18,
};

namespace {

constexpr double kDegToRad = 0.017453292519943295;
// ar-drivers-rs scales the Nreal Light accelerometer with this g.
constexpr double kNrealLightGravity = 9.81;

template <typename T>
T read_le(const uint8_t* p) {
  T value{};
  std::memcpy(&value, p, sizeof(T));
  return value;
}

bool strip_uvc_headers(const uint8_t* bulk, size_t size, std::vector<uint8_t>& payload) {
  payload.clear();
  payload.reserve(size);
  size_t read = 0;
  while (read < size) {
    const uint8_t header = bulk[read];
    if (header == 0 || read + header > size) return false;
    read += header;
    size_t len = kNrealLightUvcChunk - (read % kNrealLightUvcChunk);
    if (read + len > size) len = size - read;
    payload.insert(payload.end(), bulk + read, bulk + read + len);
    read += len;
  }
  return true;
}

void mul3(const std::array<double, 9>& m, const std::array<double, 3>& v, std::array<double, 3>& out) {
  for (int row = 0; row < 3; ++row) {
    out[row] = m[row * 3] * v[0] + m[row * 3 + 1] * v[1] + m[row * 3 + 2] * v[2];
  }
}

bool read_numbers(const cv::FileNode& node, size_t count, std::vector<double>& out) {
  out.clear();
  if (!node.isSeq() || node.size() < count) return false;
  for (const auto& item : node) out.push_back(static_cast<double>(item));
  return out.size() >= count;
}

std::array<double, 9> upper_map(const std::vector<double>& scale, const std::vector<double>& skew) {
  return {scale[0], skew[0], skew[1], 0.0, scale[1], skew[2], 0.0, 0.0, scale[2]};
}

std::array<double, 9> hamilton_matrix(double x, double y, double z, double w) {
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  x /= n;
  y /= n;
  z /= n;
  w /= n;
  return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
          2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
          2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

std::optional<int64_t> trailer_integer(const std::string& trailer, const std::string& key) {
  const size_t at = trailer.find(key + " = ");
  if (at == std::string::npos) return std::nullopt;
  try {
    return std::stoll(trailer.substr(at + key.size() + 3));
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

}  // namespace

bool decode_nreal_light_frame(const uint8_t* bulk, size_t size, NrealLightStereoPixels& out) {
  if (bulk == nullptr) return false;
  std::vector<uint8_t> payload;
  if (!strip_uvc_headers(bulk, size, payload)) return false;
  const size_t pixels = static_cast<size_t>(kNrealLightEyeWidth) * kNrealLightEyeHeight;
  if (payload.size() < pixels * 2 + 8) return false;
  out.left.resize(pixels);
  out.right.resize(pixels);
  const size_t row = kNrealLightEyeWidth;
  for (int y = 0; y < kNrealLightEyeHeight; ++y) {
    const size_t src = static_cast<size_t>(y) * 2 * row;
    std::memcpy(out.left.data() + y * row, payload.data() + src, row);
    std::memcpy(out.right.data() + y * row, payload.data() + src + row, row);
  }
  out.device_ns = read_le<uint64_t>(payload.data() + pixels * 2);
  return out.device_ns != 0;
}

std::optional<NrealLightImuSample> parse_nreal_light_imu(const uint8_t* report, size_t size) {
  constexpr size_t kNeed = 44 + 8 + 4 + 4 + 12 + 8 + 4 + 4 + 12;
  if (report == nullptr || size < kNeed || report[0] != kNrealLightReportImu) return std::nullopt;
  const uint8_t* p = report + 44;
  NrealLightImuSample sample;
  sample.device_ns = read_le<uint64_t>(p);
  p += 8;
  const double gyro_mul = read_le<uint32_t>(p);
  const double gyro_div = read_le<uint32_t>(p + 4);
  p += 8;
  if (gyro_div == 0.0) return std::nullopt;
  for (int i = 0; i < 3; ++i) {
    sample.gyro_rad_s[i] = read_le<int32_t>(p + 4 * i) * (gyro_mul / gyro_div) * kDegToRad;
  }
  p += 12 + 8;
  const double accel_mul = read_le<uint32_t>(p);
  const double accel_div = read_le<uint32_t>(p + 4);
  p += 8;
  if (accel_div == 0.0) return std::nullopt;
  for (int i = 0; i < 3; ++i) {
    sample.accel_m_s2[i] = read_le<int32_t>(p + 4 * i) * (accel_mul / accel_div) * kNrealLightGravity;
  }
  if (sample.device_ns == 0) return std::nullopt;
  return sample;
}

bool parse_nreal_light_factory(const std::vector<uint8_t>& blob, NrealLightFactory& out, std::string& error) {
  const std::string text(blob.begin(), blob.end());
  const size_t begin = text.find("\n\n{");
  if (begin == std::string::npos) {
    error = "factory config has no JSON object";
    return false;
  }
  const size_t end = text.find("\n\n", begin + 2);
  if (end == std::string::npos) {
    error = "factory config JSON is not terminated";
    return false;
  }
  out.json = text.substr(begin + 2, end - begin - 2);
  out.camera_imu_timestamp_offset_us = trailer_integer(text.substr(end), "camera_imu_timestamp_offset_us");

  try {
    cv::FileStorage fs(out.json, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    const cv::FileNode imu = fs["IMU"]["device_1"];
    std::vector<double> gyro_bias, accel_bias, scale_accel, scale_gyro, skew_accel, skew_gyro, g_sens, q;
    if (!read_numbers(imu["gyro_bias"], 3, gyro_bias) || !read_numbers(imu["accel_bias"], 3, accel_bias) ||
        !read_numbers(imu["scale_accel"], 3, scale_accel) || !read_numbers(imu["scale_gyro"], 3, scale_gyro) ||
        !read_numbers(imu["skew_accel"], 3, skew_accel) || !read_numbers(imu["skew_gyro"], 3, skew_gyro) ||
        !read_numbers(imu["gyro_g_sensitivity"], 9, g_sens) || !read_numbers(imu["accel_q_gyro"], 4, q)) {
      error = "factory config IMU.device_1 is incomplete";
      return false;
    }
    for (int i = 0; i < 3; ++i) {
      out.imu.gyro_bias[i] = gyro_bias[i];
      out.imu.accel_bias[i] = accel_bias[i];
    }
    out.imu.accel_map = upper_map(scale_accel, skew_accel);
    out.imu.gyro_map = upper_map(scale_gyro, skew_gyro);
    for (int i = 0; i < 9; ++i) out.imu.g_sensitivity[i] = g_sens[i];
    out.imu.accel_from_gyro = hamilton_matrix(q[0], q[1], q[2], q[3]);
  } catch (const cv::Exception& e) {
    error = std::string("factory config JSON parse failed: ") + e.what();
    return false;
  }
  return true;
}

void apply_nreal_light_imu_factory(const NrealLightImuFactory& factory, NrealLightImuSample& sample) {
  std::array<double, 3> accel{};
  std::array<double, 3> gyro{};
  for (int i = 0; i < 3; ++i) {
    accel[i] = sample.accel_m_s2[i] - factory.accel_bias[i];
    gyro[i] = sample.gyro_rad_s[i] - factory.gyro_bias[i];
  }
  std::array<double, 3> accel_cal{};
  std::array<double, 3> gyro_cal{};
  std::array<double, 3> g_term{};
  mul3(factory.accel_map, accel, accel_cal);
  mul3(factory.gyro_map, gyro, gyro_cal);
  mul3(factory.g_sensitivity, accel_cal, g_term);
  for (int i = 0; i < 3; ++i) gyro_cal[i] += g_term[i];
  mul3(factory.accel_from_gyro, gyro_cal, sample.gyro_rad_s);
  sample.accel_m_s2 = accel_cal;
}

namespace {

constexpr uint8_t kMcuStx = 0x02;
constexpr uint8_t kMcuEtx = 0x03;
constexpr uint8_t kMcuColon = ':';

uint32_t crc32_ieee(const uint8_t* data, size_t size) {
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return crc ^ 0xffffffffu;
}

int hex_digit(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  // The MCU pads short checksums with spaces.
  if (c == ' ') return 0;
  return -1;
}

}  // namespace

std::optional<std::array<uint8_t, 64>> encode_nreal_light_mcu(const NrealLightMcuPacket& packet) {
  std::array<uint8_t, 64> out{};
  if (packet.data.size() + 20 > out.size()) return std::nullopt;
  size_t n = 0;
  out[n++] = kMcuStx;
  out[n++] = kMcuColon;
  out[n++] = packet.category;
  out[n++] = kMcuColon;
  out[n++] = packet.command;
  out[n++] = kMcuColon;
  for (uint8_t byte : packet.data) out[n++] = byte;
  out[n++] = kMcuColon;
  out[n++] = '0';
  out[n++] = kMcuColon;
  static const char* kHex = "0123456789abcdef";
  const uint32_t crc = crc32_ieee(out.data(), n);
  for (int shift = 28; shift >= 0; shift -= 4) out[n++] = static_cast<uint8_t>(kHex[(crc >> shift) & 0xf]);
  out[n++] = kMcuColon;
  out[n++] = kMcuEtx;
  return out;
}

std::optional<NrealLightMcuPacket> decode_nreal_light_mcu(const uint8_t* bytes, size_t size) {
  if (bytes == nullptr || size < 4 || bytes[0] != kMcuStx) return std::nullopt;
  size_t end = 0;
  while (end < size && bytes[end] != kMcuEtx) ++end;
  if (end == size || end < 12 || bytes[end - 1] != kMcuColon) return std::nullopt;
  const size_t crc_at = end - 9;
  for (size_t i = 0; i < 8; ++i) {
    if (hex_digit(bytes[crc_at + i]) < 0) return std::nullopt;
  }
  std::vector<std::vector<uint8_t>> parts(1);
  for (size_t i = 1; i < crc_at; ++i) {
    if (bytes[i] == kMcuColon) {
      parts.emplace_back();
    } else {
      parts.back().push_back(bytes[i]);
    }
  }
  if (parts.size() < 4 || parts[1].empty() || parts[2].empty()) return std::nullopt;
  NrealLightMcuPacket packet;
  packet.category = parts[1][0];
  packet.command = parts[2][0];
  packet.data = parts[3];
  return packet;
}

}  // namespace xr_capture_cpp
