#include "io/gimbal/gimbal.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{f              | false | 是否开火}"
  "{read-only r    | | 只读取姿态，不发送指令}"
  "{latency        | | 无应答收发耗时测试；发送原顺序SP44，mode=0、角度/速度为0，忽略加速度}"
  "{duration       | 10 | 延迟测试时长，秒}"
  "{interval-ms    | 10 | 延迟测试发送周期，毫秒}"
  "{csv            | | 延迟统计CSV输出路径，可选}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int latency_test(const cv::CommandLineParser & cli, const std::string & config_path,
                 const tools::Exiter & exiter)
{
  const double duration = cli.get<double>("duration");
  const double interval_ms = cli.get<double>("interval-ms");
  const bool read_only = cli.has("read-only");
  if (!std::isfinite(duration) || duration <= 0 || !std::isfinite(interval_ms) || interval_ms < 1)
    throw std::invalid_argument("duration must be positive and interval-ms must be >= 1");
  if (cli.get<bool>("f")) throw std::invalid_argument("Latency test does not support firing (-f)");
  if (!cli.check()) {
    cli.printErrors();
    throw std::invalid_argument("Invalid latency test arguments");
  }
  std::ofstream csv;
  auto csv_path = cli.get<std::string>("csv");
  if (!csv_path.empty()) {
    csv.open(csv_path);
    if (!csv) throw std::runtime_error("Cannot open latency CSV: " + csv_path);
    csv << "elapsed_s,rx_bytes,rx_frames,rx_hz,rx_interval_mean_ms,rx_interval_jitter_ms,"
           "rx_interval_max_ms,rx_age_ms,rx_parse_mean_ms,rx_parse_max_ms,rx_bad_tails,"
           "rx_discarded_bytes,tx_attempts,tx_frames,tx_errors,tx_write_mean_ms,"
           "tx_write_jitter_ms,tx_write_max_ms\n";
  }
  // 不等待上行帧，不调用阻塞姿态插值，TX不依赖RX或下位机应答。
  io::Gimbal gimbal(config_path, false);
  tools::logger()->info(
    "[Gimbal latency] RX=44 bytes, TX={} bytes; {}",
    gimbal.tx_frame_size(), read_only ? "read-only" :
      "SP Euler original order, zero angles/velocities, acceleration ignored, CRC=0, tail=0D, mode=0; no reply required");
  tools::logger()->info(
    "[Gimbal latency] TX write = OS write time; RX parse = complete frame to state update; "
    "RX period/age use PC timestamps. These are not MCU one-way latency or RTT.");
  const auto started = std::chrono::steady_clock::now();
  auto next_send = started, next_report = started + 1s;
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(interval_ms));
  auto report = [&](bool final) {
    auto stats = gimbal.communication_stats();
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - started).count();
    double rx_hz = stats.rx_frames / elapsed;
    double rx_jitter = stats.rx_frames > 2
      ? std::sqrt(std::max(0.0, stats.rx_interval_m2 / (stats.rx_frames - 2))) : 0;
    double tx_jitter = stats.tx_frames > 1
      ? std::sqrt(std::max(0.0, stats.tx_write_m2 / (stats.tx_frames - 1))) : 0;
    double age = stats.rx_frames
      ? std::chrono::duration<double, std::milli>(now - stats.last_rx).count() : -1;
    tools::logger()->info(
      "[Gimbal latency{}] {:.2f}s RX {} bytes/{} frames ({:.1f} Hz), "
      "period mean/jitter/max {:.3f}/{:.3f}/{:.3f} ms, age {:.3f} ms (-1=no RX), "
      "parse mean/max {:.4f}/{:.4f} ms, bad tails {}, discarded {}; "
      "TX {}/{} successful, errors {}, write mean/jitter/max {:.4f}/{:.4f}/{:.4f} ms",
      final ? " final" : "", elapsed, stats.rx_bytes, stats.rx_frames, rx_hz,
      stats.rx_interval_mean_ms, rx_jitter, stats.rx_interval_max_ms, age,
      stats.rx_parse_mean_ms, stats.rx_parse_max_ms, stats.rx_bad_tails, stats.rx_discarded_bytes,
      stats.tx_frames, stats.tx_attempts, stats.tx_errors,
      stats.tx_write_mean_ms, tx_jitter, stats.tx_write_max_ms);
    if (csv.is_open()) {
      csv << elapsed << ',' << stats.rx_bytes << ',' << stats.rx_frames << ',' << rx_hz << ','
          << stats.rx_interval_mean_ms << ',' << rx_jitter << ',' << stats.rx_interval_max_ms << ','
          << age << ',' << stats.rx_parse_mean_ms << ',' << stats.rx_parse_max_ms << ','
          << stats.rx_bad_tails << ',' << stats.rx_discarded_bytes << ',' << stats.tx_attempts << ','
          << stats.tx_frames << ',' << stats.tx_errors << ',' << stats.tx_write_mean_ms << ','
          << tx_jitter << ',' << stats.tx_write_max_ms << '\n';
      csv.flush();
      if (!csv) throw std::runtime_error("Failed to write latency CSV");
    }
    return stats;
  };
  while (!exiter.exit()) {
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - started).count() >= duration) break;
    if (!read_only && now >= next_send) {
      gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
      next_send += period;
      if (next_send <= std::chrono::steady_clock::now())
        next_send = std::chrono::steady_clock::now() + period;  // 不突发补发。
    }
    if (now >= next_report) {
      report(false);
      next_report = now + 1s;
    }
    std::this_thread::sleep_for(1ms);
  }
  auto stats = report(true);
  if (!stats.rx_frames) tools::logger()->error("[Gimbal latency] No valid 44-byte SP RX frames");
  return stats.rx_frames && (read_only || (stats.tx_frames && !stats.tx_errors)) ? 0 : 1;
}

int main(int argc, char * argv[]) try
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto test_fire = cli.get<bool>("f");
  auto read_only = cli.has("read-only");
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  if (cli.has("latency")) return latency_test(cli, config_path, exiter);
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);

  auto t0 = std::chrono::steady_clock::now();
  auto last_mode = gimbal.mode();
  uint16_t last_bullet_count = 0;

  auto fire = false;
  auto fire_count = 0;
  auto fire_stamp = std::chrono::steady_clock::now();
  auto first_fired = false;
  auto last_pose_log = t0 - 1s;

  while (!exiter.exit()) {
    auto mode = gimbal.mode();

    if (mode != last_mode) {
      tools::logger()->info("Gimbal mode changed: {}", gimbal.str(mode));
      last_mode = mode;
    }

    auto t = std::chrono::steady_clock::now();
    auto q = gimbal.q(t);
    auto state = gimbal.state();
    auto ypr = tools::eulers(q, 2, 1, 0);
    if (read_only && t - last_pose_log >= 1s) {
      tools::logger()->info(
        "[Gimbal test] {}: yaw {:.2f}, pitch {:.2f}, roll {:.2f} degree; bullet speed {:.2f} m/s",
        gimbal.str(mode), ypr[0] * 180 / M_PI, ypr[1] * 180 / M_PI,
        ypr[2] * 180 / M_PI, state.bullet_speed);
      last_pose_log = t;
    }

    auto fired = state.bullet_count > last_bullet_count;
    last_bullet_count = state.bullet_count;

    if (!first_fired && fired) {
      first_fired = true;
      tools::logger()->info("Gimbal first fired after: {:.3f}s", tools::delta_time(t, fire_stamp));
    }

    if (fire && fire_count > 20) {
      // 0.2 s
      fire = false;
      fire_count = 0;
    } else if (!fire && fire_count > 100) {
      // 1s
      fire = true;
      fire_count = 0;
      fire_stamp = t;
      first_fired = false;
    }
    fire_count++;

    if (!read_only) gimbal.send(true, test_fire && fire, 1, 0, 0, 0, 0, 0);

    nlohmann::json data;
    data["q_yaw"] = ypr[0];
    data["q_pitch"] = ypr[1];
    data["yaw"] = state.yaw;
    data["vyaw"] = state.yaw_vel;
    data["pitch"] = state.pitch;
    data["vpitch"] = state.pitch_vel;
    data["bullet_speed"] = state.bullet_speed;
    data["bullet_count"] = state.bullet_count;
    data["fired"] = fired ? 1 : 0;
    data["fire"] = test_fire && fire ? 1 : 0;
    data["t"] = tools::delta_time(t, t0);
    plotter.plot(data);

    std::this_thread::sleep_for(9ms);
  }

  if (!read_only) gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
catch (const std::exception & e)
{
  tools::logger()->error("[gimbal_test] {}", e.what());
  return 1;
}
