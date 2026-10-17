#ifndef IO__SERIAL_IMU_PROTOCOL_HPP
#define IO__SERIAL_IMU_PROTOCOL_HPP

#include <linux/can.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace io
{
// new_uart1.png 标准：27 字节，01 00 ... 00 0D；pitch/roll/yaw 在偏移 2/6/10。
// 兼容新版 Gimbal 的 26 字节有效帧 + 可配置 stride（通常为27，末尾00为对齐补充）。
// uart44：SP + 保留区 + pitch@19/roll@23/yaw@27 + ... + 0D，欧拉角转四元数。
// 仅负责拆帧和欧拉角 -> 四元数转换，不打开设备。
class SerialIMUProtocol
{
public:
  explicit SerialIMUProtocol(const YAML::Node & yaml, int quaternion_canid)
  : canid_(quaternion_canid)
  {
    if (canid_ > CAN_SFF_MASK) throw std::invalid_argument("Invalid quaternion_canid");
    if (yaml && yaml["protocol"]) {
      auto protocol = yaml["protocol"].as<std::string>();
      if (protocol != "uart27" && protocol != "uart26" && protocol != "uart44")
        throw std::invalid_argument("serial_imu.protocol must be uart26, uart27 or uart44");
      frame_size_ = frame_stride_ = protocol == "uart26" ? 26 : 27;
      if (protocol == "uart44") {
        frame_size_ = frame_stride_ = 44;
        header_ = {'S', 'P'};
        tail_ = {0x0d};
        offsets_ = {23, 19, 27};
      }
    }
    if (yaml) {
      if (yaml["frame_size"]) {
        frame_size_ = yaml["frame_size"].as<size_t>();
        frame_stride_ = frame_size_;
      }
      if (yaml["frame_stride"]) frame_stride_ = yaml["frame_stride"].as<size_t>();
      if (yaml["header"]) header_ = bytes(yaml["header"]);
      if (yaml["tail"]) tail_ = bytes(yaml["tail"]);
      if (yaml["rpy_offsets"]) offsets_ = yaml["rpy_offsets"].as<std::array<size_t, 3>>();
      if (yaml["rpy_signs"]) signs_ = yaml["rpy_signs"].as<std::array<double, 3>>();
      if (yaml["rpy_bias_deg"]) bias_ = yaml["rpy_bias_deg"].as<std::array<double, 3>>();
      if (yaml["max_abs_angle_deg"]) max_angle_ = yaml["max_abs_angle_deg"].as<double>();
      if (yaml["inverse"]) inverse_ = yaml["inverse"].as<bool>();
      auto unit = yaml["angle_unit"] ? yaml["angle_unit"].as<std::string>() : "degrees";
      if (unit != "degrees" && unit != "radians")
        throw std::invalid_argument("angle_unit must be degrees or radians");
      radians_ = unit == "radians";
      auto endian = yaml["float_endian"] ? yaml["float_endian"].as<std::string>() : "little";
      if (endian != "little" && endian != "big")
        throw std::invalid_argument("float_endian must be little or big");
      big_endian_ = endian == "big";
    }
    if ((frame_size_ != 26 && frame_size_ != 27 && frame_size_ != 44) || frame_stride_ < frame_size_ ||
        frame_stride_ > frame_size_ + 64 || header_.empty() || tail_.empty() ||
        header_.size() + tail_.size() + 12 > frame_size_ || !std::isfinite(max_angle_) || max_angle_ <= 0)
      throw std::invalid_argument("Invalid serial IMU frame layout/angle limit");
    for (size_t i = 0; i < 3; ++i) {
      if (offsets_[i] < header_.size() || offsets_[i] > frame_size_ - tail_.size() - 4 ||
          !std::isfinite(signs_[i]) || std::abs(signs_[i]) != 1 || !std::isfinite(bias_[i]))
        throw std::invalid_argument("Invalid serial IMU rpy_offsets/signs/bias");
      for (size_t j = 0; j < i; ++j)
        if (std::max(offsets_[i], offsets_[j]) - std::min(offsets_[i], offsets_[j]) < 4)
          throw std::invalid_argument("Overlapping serial IMU float fields");
    }
  }

  void reset()
  {
    buffer_.clear();
    synced_ = false;
  }

  std::vector<can_frame> feed(const uint8_t * data, size_t size)
  {
    std::vector<can_frame> frames;
    // 逐字节推进，缓存最多 frame_stride + frame_size；支持分包/粘包和垃圾数据。
    for (size_t i = 0; i < size; ++i) {
      buffer_.push_back(data[i]);
      while (buffer_.size() >= frame_size_) {
        can_frame frame{};
        if (!decode(buffer_.data(), frame)) {
          buffer_.erase(buffer_.begin());
          synced_ = false;
          continue;
        }
        if (!synced_) {
          if (buffer_.size() < frame_stride_ + frame_size_) break;
          can_frame next{};
          if (!decode(buffer_.data() + frame_stride_, next)) {
            buffer_.erase(buffer_.begin());
            continue;
          }
          synced_ = true;  // 首次同步/丢字节后，要求连续两帧有效。
        }
        if (buffer_.size() < frame_stride_) break;
        frames.push_back(frame);
        buffer_.erase(buffer_.begin(), buffer_.begin() + frame_stride_);
      }
    }
    return frames;
  }

  bool orientation(const uint8_t * data, size_t size, Eigen::Quaterniond & q) const
  {
    if (size != frame_size_ || !std::equal(header_.begin(), header_.end(), data) ||
        !std::equal(tail_.begin(), tail_.end(), data + frame_size_ - tail_.size())) return false;
    std::array<double, 3> angles;
    for (size_t i = 0; i < 3; ++i) {
      uint32_t bits = 0;
      for (size_t j = 0; j < 4; ++j)
        bits |= uint32_t(data[offsets_[i] + j]) << (8 * (big_endian_ ? 3 - j : j));
      float value;
      static_assert(sizeof(value) == sizeof(bits), "Requires 32-bit float");
      std::memcpy(&value, &bits, sizeof(value));
      double degrees = radians_ ? double(value) * 180 / PI : double(value);
      if (!std::isfinite(degrees) || std::abs(degrees) > max_angle_) return false;
      angles[i] = std::remainder(signs_[i] * degrees + bias_[i], 360.0) * PI / 180;
    }
    q = Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitX());
    q.normalize();
    if (inverse_) q = q.conjugate();
    return true;
  }

private:
  static constexpr double PI = 3.14159265358979323846;
  canid_t canid_;
  size_t frame_size_ = 27, frame_stride_ = 27;
  std::vector<uint8_t> header_{0x01, 0x00}, tail_{0x00, 0x0d}, buffer_;
  // 固件已执行 Vision_Pitch=-INS.Roll、Vision_Roll=-INS.Pitch；这里不重复取负。
  std::array<size_t, 3> offsets_{6, 2, 10};  // roll, pitch, yaw
  std::array<double, 3> signs_{1, 1, 1}, bias_{0, 0, 0};
  double max_angle_ = 36000;
  bool radians_ = false, big_endian_ = false, inverse_ = false, synced_ = false;

  static std::vector<uint8_t> bytes(const YAML::Node & node)
  {
    std::vector<uint8_t> result;
    for (const auto & value : node.as<std::vector<int>>()) {
      if (value < 0 || value > 255) throw std::invalid_argument("Frame marker outside 0..255");
      result.push_back(static_cast<uint8_t>(value));
    }
    return result;
  }

  bool decode(const uint8_t * data, can_frame & frame) const
  {
    Eigen::Quaterniond q;
    if (!orientation(data, frame_size_, q)) return false;
    const std::array<double, 4> values{q.x(), q.y(), q.z(), q.w()};
    frame.can_id = canid_;
    frame.can_dlc = 8;
    for (size_t i = 0; i < 4; ++i) {
      auto value = static_cast<uint16_t>(static_cast<int16_t>(std::lround(values[i] * 1e4)));
      frame.data[2 * i] = value >> 8;
      frame.data[2 * i + 1] = value & 0xff;
    }
    return true;
  }
};
}  // namespace io

#endif  // IO__SERIAL_IMU_PROTOCOL_HPP
