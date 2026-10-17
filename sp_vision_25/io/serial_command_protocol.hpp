#ifndef IO__SERIAL_COMMAND_PROTOCOL_HPP
#define IO__SERIAL_COMMAND_PROTOCOL_HPP

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "io/command.hpp"

namespace io
{
// new_uart2.png 标准：FF + yaw/pitch/distance，小端 float 在偏移 1/5/9，共13字节。
// sp44：沿用电控44字节欧拉角包；pitch/roll/yaw@19/23/27，速度@31/35。
// 原包没有加速度字段，忽略命令中的acc，保留区和CRC固定为0。
class SerialCommandProtocol
{
public:
  explicit SerialCommandProtocol(const YAML::Node & yaml)
  {
    if (yaml) {
      auto protocol = yaml["protocol"] ? yaml["protocol"].as<std::string>() : "uart13";
      if (protocol != "uart13" && protocol != "sp44")
        throw std::invalid_argument("UART TX protocol must be uart13 or sp44");
      sp44_ = protocol == "sp44";
      if (yaml["header"]) {
        if (sp44_) {
          if (yaml["header"].as<std::vector<int>>() != std::vector<int>{'S', 'P'})
            throw std::invalid_argument("sp44 TX header must be [0x53, 0x50]");
        } else {
          int value = yaml["header"].as<int>();
          if (value < 0 || value > 255) throw std::invalid_argument("Invalid UART TX header");
          header_ = static_cast<uint8_t>(value);
        }
      }
      if (yaml["tail"]) {
        auto tail = yaml["tail"].as<std::vector<int>>();
        if (sp44_ && tail != std::vector<int>{0x0d})
          throw std::invalid_argument("sp44 TX tail must be [0x0d]");
        if (!sp44_ && !tail.empty())
          throw std::invalid_argument("new_uart2 UART TX has no tail; remove tx.tail");
      }
      auto unit = yaml["angle_unit"] ? yaml["angle_unit"].as<std::string>() : "degrees";
      if (unit != "degrees" && unit != "radians")
        throw std::invalid_argument("UART TX angle_unit must be degrees or radians");
      degrees_ = unit == "degrees";
    }
  }

  std::vector<uint8_t> encode(const Command & command) const
  {
    const double scale = degrees_ ? 180.0 / 3.14159265358979323846 : 1.0;
    if (sp44_) {
      std::vector<uint8_t> frame(44, 0);
      frame[0] = 'S'; frame[1] = 'P';
      frame[2] = command.control ? (command.shoot ? 2 : 1) : 0;
      put_float(frame, 19, command.pitch * scale);
      // command没有roll，23..26保持0；3..18保持原包保留区。
      put_float(frame, 27, command.yaw * scale);
      put_float(frame, 31, command.pitch_vel * scale);
      put_float(frame, 35, command.yaw_vel * scale);
      frame[43] = 0x0d;  // 39..40保留、41..42 CRC：全部固定为0。
      return frame;
    }
    std::vector<uint8_t> frame(13, 0);
    frame[0] = header_;
    // 新标准无 control/shoot 字段，不能把这些标志插入帧中，否则所有float错位。
    put_float(frame, 1, command.yaw * scale);
    put_float(frame, 5, command.pitch * scale);
    if (command.horizon_distance < 0) throw std::invalid_argument("Negative UART target distance");
    put_float(frame, 9, command.horizon_distance);  // 米，保持原 Command 单位。
    return frame;
  }

  size_t frame_size() const { return sp44_ ? 44 : 13; }

private:
  bool sp44_ = false;
  uint8_t header_ = 0xff;
  bool degrees_ = true;

  static void put_float(std::vector<uint8_t> & frame, size_t offset, double value)
  {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
      throw std::invalid_argument("UART command cannot be represented as finite float32");
    float number = static_cast<float>(value);
    uint32_t bits;
    static_assert(sizeof(number) == sizeof(bits), "Requires 32-bit float");
    std::memcpy(&bits, &number, sizeof(bits));
    for (size_t i = 0; i < 4; ++i) frame[offset + i] = (bits >> (8 * i)) & 0xff;
  }
};
}  // namespace io

#endif  // IO__SERIAL_COMMAND_PROTOCOL_HPP
