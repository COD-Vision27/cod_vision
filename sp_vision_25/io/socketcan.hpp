#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/epoll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "io/serial_imu_protocol.hpp"
#include "io/serial_command_protocol.hpp"
#include "tools/logger.hpp"

namespace io
{
class SocketCAN
{
public:
  SocketCAN(const YAML::Node & yaml, std::function<void(const can_frame &)> rx_handler)
  : use_can_(yaml["use_can"] ? yaml["use_can"].as<bool>() : true),
    rx_handler_(std::move(rx_handler))
  {
    if (use_can_) {
      // 串口模式完全跳过 CAN 配置检查、socket、SIOCGIFINDEX 和 bind。
      if (!yaml["can_interface"]) throw std::invalid_argument("Missing can_interface");
      interface_ = yaml["can_interface"].as<std::string>();
      if (interface_.empty() || interface_.size() >= IFNAMSIZ)
        throw std::invalid_argument("Invalid can_interface");
      send_canid_ = yaml["send_canid"].as<int>();
      if (send_canid_ < 0 || send_canid_ > CAN_SFF_MASK)
        throw std::invalid_argument("Invalid send_canid");
    } else {
      auto serial = yaml["serial_imu"];
      interface_ = serial && serial["port"] ? serial["port"].as<std::string>() : "/dev/ttyACM0";
      auto baud = serial && serial["baudrate"] ? serial["baudrate"].as<int>() : 921600;
      baud_ = baudrate(baud);
      if (serial && serial["timeout_ms"]) timeout_ms_ = serial["timeout_ms"].as<int>();
      if (serial && serial["reconnect_ms"]) reconnect_ms_ = serial["reconnect_ms"].as<int>();
      if (interface_.empty() || timeout_ms_ <= 0 || reconnect_ms_ <= 0)
        throw std::invalid_argument("Invalid serial IMU port/timeouts");
      protocol_ = std::make_unique<SerialIMUProtocol>(serial, yaml["quaternion_canid"].as<int>());
      auto tx = serial ? serial["tx"] : YAML::Node();
      tx_protocol_ = std::make_unique<SerialCommandProtocol>(tx);
      if (tx && tx["write_timeout_ms"]) write_timeout_ms_ = tx["write_timeout_ms"].as<int>();
      if (write_timeout_ms_ <= 0 || write_timeout_ms_ > 1000)
        throw std::invalid_argument("UART write_timeout_ms must be in 1..1000");
      tools::logger()->info(
        "[SocketCAN] UART mode: {} @ {}. CAN checks skipped; Euler RX and float command TX.",
        interface_, baud);
    }
    // 单一工作线程负责打开、接收和重连，避免旧版 daemon/read 之间的竞态。
    tools::logger();
    read_thread_ = std::thread([this] {
      while (!quit_) {
        try {
          if (socket_fd_ < 0) open();
          if (reconnect_requested_) throw std::runtime_error("reconnect after UART write failure");
          read();
        } catch (const std::exception & e) {
          tools::logger()->warn("[SocketCAN] {} {}: {}", use_can_ ? "CAN" : "Serial IMU",
                                interface_, e.what());
          close();
          std::unique_lock<std::mutex> lock(wait_mutex_);
          wake_.wait_for(lock, std::chrono::milliseconds(reconnect_ms_), [this] { return quit_.load(); });
        }
      }
      close();
    });
  }

  ~SocketCAN()
  {
    quit_ = true;
    wake_.notify_all();
    if (read_thread_.joinable()) read_thread_.join();
  }

  void write(can_frame * frame) const
  {
    if (!use_can_) throw std::invalid_argument("UART TX requires Command, not a CAN frame");
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (socket_fd_ < 0) throw std::runtime_error("CAN is disconnected");
    ssize_t count;
    do {
      count = ::write(socket_fd_, frame, sizeof(can_frame));
    } while (count < 0 && errno == EINTR);
    if (count != sizeof(can_frame)) throw std::runtime_error("Unable to write complete CAN frame");
  }

  void write(const Command & command) const
  {
    if (use_can_) {
      can_frame frame{};
      frame.can_id = send_canid_;
      frame.can_dlc = 8;
      frame.data[0] = command.control ? 1 : 0;
      frame.data[1] = command.shoot ? 1 : 0;
      const double values[]{command.yaw, command.pitch, command.horizon_distance};
      for (size_t i = 0; i < 3; ++i) {
        double scaled = values[i] * 1e4;
        if (!std::isfinite(scaled) || scaled < -32768 || scaled > 32767)
          throw std::invalid_argument("CAN command outside int16 range");
        auto value = static_cast<uint16_t>(static_cast<int16_t>(scaled));
        frame.data[2 + 2 * i] = value >> 8;
        frame.data[3 + 2 * i] = value & 0xff;
      }
      write(&frame);
      return;
    }
    const auto data = tx_protocol_->encode(command);
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (socket_fd_ < 0 || reconnect_requested_) throw std::runtime_error("UART is disconnected");
    try {
      write_uart(data);
    } catch (...) {
      // 半帧/超时后禁止继续追加下一帧，由接收线程关闭并重新打开；不重发旧命令。
      reconnect_requested_ = true;
      throw;
    }
  }

private:
  const bool use_can_;
  std::string interface_;
  int socket_fd_ = -1, epoll_fd_ = -1;
  int timeout_ms_ = 500, reconnect_ms_ = 1000;
  int send_canid_ = 0xff, write_timeout_ms_ = 20;
  speed_t baud_ = B921600;
  termios old_termios_{};
  bool termios_saved_ = false, serial_exclusive_ = false;
  std::atomic<bool> quit_{false};
  mutable std::atomic<bool> reconnect_requested_{false};
  std::thread read_thread_;
  mutable std::mutex io_mutex_;
  std::mutex wait_mutex_;
  std::condition_variable wake_;
  std::function<void(const can_frame &)> rx_handler_;
  std::unique_ptr<SerialIMUProtocol> protocol_;
  std::unique_ptr<SerialCommandProtocol> tx_protocol_;
  std::chrono::steady_clock::time_point last_valid_;

  static speed_t baudrate(int baud)
  {
    switch (baud) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      case 1000000: return B1000000;
      default: throw std::invalid_argument("Unsupported serial IMU baudrate");
    }
  }

  static void check(bool failed, const char * operation)
  {
    if (failed) throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
  }

  void write_uart(const std::vector<uint8_t> & data) const
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(write_timeout_ms_);
    size_t offset = 0;
    while (offset < data.size()) {
      if (quit_ || std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("UART command write timeout");
      auto count = ::write(socket_fd_, data.data() + offset, data.size() - offset);
      if (count > 0) {
        offset += static_cast<size_t>(count);
        continue;
      }
      if (count < 0 && errno == EINTR) continue;
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        check(true, "writing UART command");
      if (count == 0) throw std::runtime_error("UART write returned zero bytes");
      auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
      if (remaining <= 0) throw std::runtime_error("UART command write timeout");
      pollfd device{socket_fd_, POLLOUT, 0};
      int ready = poll(&device, 1, static_cast<int>(remaining));
      if (ready < 0 && errno == EINTR) continue;
      check(ready < 0, "waiting to write UART command");
      if (device.revents & (POLLERR | POLLHUP | POLLNVAL))
        throw std::runtime_error("UART disconnected during command write");
    }
  }

  void open()
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (use_can_) {
      socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
      check(socket_fd_ < 0, "opening CAN socket");
      ifreq ifr{};
      std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
      check(ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0, "getting CAN interface index");
      sockaddr_can addr{};
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      check(bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0, "binding CAN");
    } else {
      socket_fd_ = ::open(interface_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      check(socket_fd_ < 0, "opening serial IMU");
      check(flock(socket_fd_, LOCK_EX | LOCK_NB) < 0, "locking serial IMU");
      check(ioctl(socket_fd_, TIOCEXCL) < 0, "exclusively opening serial IMU");
      serial_exclusive_ = true;
      check(tcgetattr(socket_fd_, &old_termios_) < 0, "reading serial settings");
      termios_saved_ = true;
      auto settings = old_termios_;
      cfmakeraw(&settings);
      settings.c_cflag |= CLOCAL | CREAD;
      settings.c_cflag &= ~(CRTSCTS | HUPCL | CSTOPB | PARENB);
      settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;
      settings.c_cc[VMIN] = 1;
      settings.c_cc[VTIME] = 0;
      check(cfsetispeed(&settings, baud_) < 0 || cfsetospeed(&settings, baud_) < 0, "setting baudrate");
      check(tcsetattr(socket_fd_, TCSANOW, &settings) < 0, "setting serial raw mode");
      check(tcflush(socket_fd_, TCIFLUSH) < 0, "discarding old serial data");
      protocol_->reset();
      last_valid_ = std::chrono::steady_clock::now();
    }
    epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
    check(epoll_fd_ < 0, "creating epoll");
    epoll_event event{};
    event.events = EPOLLIN;
    event.data.fd = socket_fd_;
    check(epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, socket_fd_, &event) < 0, "adding device to epoll");
    reconnect_requested_ = false;
    tools::logger()->info("[SocketCAN] {} opened: {}", use_can_ ? "CAN" : "Serial IMU", interface_);
  }

  void read()
  {
    epoll_event event{};
    int count = epoll_wait(epoll_fd_, &event, 1, 20);
    if (count < 0 && errno == EINTR) return;
    check(count < 0, "waiting for device data");
    if (count && (event.events & (EPOLLERR | EPOLLHUP)))
      throw std::runtime_error("device disconnected");
    if (count && (event.events & EPOLLIN)) {
      if (use_can_) {
        can_frame frame{};
        auto received = recv(socket_fd_, &frame, sizeof(frame), MSG_DONTWAIT | MSG_TRUNC);
        if (received < 0 && (errno == EAGAIN || errno == EINTR)) return;
        check(received < 0, "reading CAN");
        if (received == sizeof(frame) && frame.can_dlc <= CAN_MAX_DLEN &&
            !(frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG))) rx_handler_(frame);
      } else {
        uint8_t data[4096];
        auto received = ::read(socket_fd_, data, sizeof(data));
        if (received < 0 && (errno == EAGAIN || errno == EINTR)) return;
        check(received < 0, "reading serial IMU");
        if (received == 0) throw std::runtime_error("serial IMU EOF");
        // 与 CAN 分支使用同一个回调：上层仍然只收到原四元数 CAN 帧。
        for (const auto & frame : protocol_->feed(data, static_cast<size_t>(received))) {
          last_valid_ = std::chrono::steady_clock::now();
          rx_handler_(frame);
        }
      }
    }
    if (!use_can_ && std::chrono::steady_clock::now() - last_valid_ >
                       std::chrono::milliseconds(timeout_ms_))
      throw std::runtime_error("no valid IMU frame within timeout");
  }

  void close()
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (epoll_fd_ >= 0) ::close(epoll_fd_);
    if (socket_fd_ >= 0) {
      if (termios_saved_) tcsetattr(socket_fd_, TCSANOW, &old_termios_);
      if (serial_exclusive_) ioctl(socket_fd_, TIOCNXCL);
      ::close(socket_fd_);
    }
    socket_fd_ = epoll_fd_ = -1;
    termios_saved_ = serial_exclusive_ = false;
    if (protocol_) protocol_->reset();
  }
};
}  // namespace io

#endif  // IO__SOCKETCAN_HPP
