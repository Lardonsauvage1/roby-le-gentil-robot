#include "roby_hardware/encoder_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace roby_hardware
{

namespace
{
constexpr int kRespTimeoutUs = 30000;     // 30 ms
constexpr uint16_t kNoMeasureCode = 0xFFFE;
constexpr uint8_t kHeaderByte = 0xFF;
}  // namespace

EncoderDriver::~EncoderDriver()
{
  shutdown();
}

bool EncoderDriver::init(const Config & cfg)
{
  config_ = cfg;
  if (config_.mock) {
    initialized_ = true;
    return true;
  }

  // --- Ouverture serie -------------------------------------------------------
  serial_fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    std::cerr << "[EncoderDriver] open(" << config_.port << ") failed: "
              << std::strerror(errno) << std::endl;
    return false;
  }

  struct termios tio;
  if (::tcgetattr(serial_fd_, &tio) != 0) {
    std::cerr << "[EncoderDriver] tcgetattr failed: " << std::strerror(errno) << std::endl;
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
  ::cfmakeraw(&tio);
  speed_t baud_const = B115200;
  if (config_.baud == 57600) baud_const = B57600;
  ::cfsetispeed(&tio, baud_const);
  ::cfsetospeed(&tio, baud_const);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;  // on gere le timeout en software via poll/clock
  if (::tcsetattr(serial_fd_, TCSANOW, &tio) != 0) {
    std::cerr << "[EncoderDriver] tcsetattr failed: " << std::strerror(errno) << std::endl;
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
  ::tcflush(serial_fd_, TCIOFLUSH);

#ifdef HAS_GPIOD
  // --- Ligne GPIO DE/RE ------------------------------------------------------
  chip_ = ::gpiod_chip_open(config_.gpio_chip.c_str());
  if (!chip_) {
    std::cerr << "[EncoderDriver] gpiod_chip_open(" << config_.gpio_chip
              << ") failed: " << std::strerror(errno) << std::endl;
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
  de_re_line_ = ::gpiod_chip_get_line(chip_, config_.de_re_pin);
  if (!de_re_line_) {
    std::cerr << "[EncoderDriver] gpiod_chip_get_line(" << config_.de_re_pin
              << ") failed" << std::endl;
    ::gpiod_chip_close(chip_);
    chip_ = nullptr;
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
  if (::gpiod_line_request_output(de_re_line_, "roby_hardware_encoder", 0) < 0) {
    std::cerr << "[EncoderDriver] gpiod_line_request_output failed: "
              << std::strerror(errno) << std::endl;
    ::gpiod_chip_close(chip_);
    chip_ = nullptr;
    de_re_line_ = nullptr;
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
#else
  std::cerr << "[EncoderDriver] HAS_GPIOD not defined — DE/RE control disabled (mock only)\n";
  ::close(serial_fd_);
  serial_fd_ = -1;
  return false;
#endif

  initialized_ = true;
  return true;
}

void EncoderDriver::shutdown()
{
  // Stoppe le thread async avant de liberer les fd
  stop_polling_thread();

#ifdef HAS_GPIOD
  if (de_re_line_) {
    ::gpiod_line_release(de_re_line_);
    de_re_line_ = nullptr;
  }
  if (chip_) {
    ::gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
#endif
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  initialized_ = false;
}

void EncoderDriver::add_joint(const JointSpec & spec)
{
  joint_specs_.push_back(spec);
  states_[spec.slave_id] = MotorState{};
}

void EncoderDriver::set_coupling(int joint_from, int joint_to, double ratio)
{
  couplings_.push_back({joint_from, joint_to, ratio});
}

void EncoderDriver::de_re_tx()
{
#ifdef HAS_GPIOD
  if (de_re_line_) {
    ::gpiod_line_set_value(de_re_line_, 1);
  }
#endif
}

void EncoderDriver::de_re_rx()
{
#ifdef HAS_GPIOD
  if (de_re_line_) {
    ::gpiod_line_set_value(de_re_line_, 0);
  }
#endif
}

std::optional<double> EncoderDriver::query(int slave_id)
{
  if (config_.mock || serial_fd_ < 0) {
    return std::nullopt;
  }

  ::tcflush(serial_fd_, TCIFLUSH);

  // Emission : DE/RE HIGH, ecrire 1 byte (ID), flush, sleep 1ms pour laisser
  // le dernier bit partir physiquement, puis DE/RE LOW.
  de_re_tx();
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  uint8_t id_byte = static_cast<uint8_t>(slave_id);
  if (::write(serial_fd_, &id_byte, 1) != 1) {
    de_re_rx();
    return std::nullopt;
  }
  ::tcdrain(serial_fd_);
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  de_re_rx();

  // Lecture : attendre 0xFF, puis ID, puis 2 bytes data, dans un budget de 30 ms.
  auto deadline = std::chrono::steady_clock::now()
    + std::chrono::microseconds(config_.query_timeout_us);

  // Etat machine 0=cherche header, 1=cherche ID confirm, 2=lit data[0], 3=lit data[1]
  int state = 0;
  uint8_t recv_id = 0;
  uint8_t data[2] = {0, 0};

  while (std::chrono::steady_clock::now() < deadline) {
    uint8_t b;
    ssize_t n = ::read(serial_fd_, &b, 1);
    if (n != 1) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      continue;
    }
    switch (state) {
      case 0:
        if (b == kHeaderByte) state = 1;
        break;
      case 1:
        if (b == static_cast<uint8_t>(slave_id)) {
          recv_id = b;
          state = 2;
        } else if (b == kHeaderByte) {
          state = 1;  // re-sync sur 0xFF
        } else {
          state = 0;
        }
        break;
      case 2:
        data[0] = b;
        state = 3;
        break;
      case 3:
        data[1] = b;
        uint16_t val = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        if (val == kNoMeasureCode) {
          return std::nullopt;  // pulseIn timeout cote Arduino
        }
        return (static_cast<double>(val) / 65535.0) * 360.0;
    }
  }
  (void)recv_id;
  return std::nullopt;
}

double EncoderDriver::wrap_to_180(double deg)
{
  double r = std::fmod(deg + 180.0, 360.0);
  if (r < 0) r += 360.0;
  return r - 180.0;
}

void EncoderDriver::update_tracker(int slave_id, std::optional<double> raw_deg)
{
  auto it = states_.find(slave_id);
  if (it == states_.end()) return;
  MotorState & st = it->second;

  if (!raw_deg.has_value()) {
    return;  // garde l'etat precedent
  }
  double raw = raw_deg.value();

  // 1ere lecture : on calibre l'unwrapped par rapport au raw_init du JointSpec
  if (std::isnan(st.last_raw_deg)) {
    double raw_init = 0;
    for (const auto & js : joint_specs_) {
      if (js.slave_id == slave_id) {
        raw_init = js.raw_init_deg;
        break;
      }
    }
    st.unwrapped_deg = wrap_to_180(raw - raw_init);
  } else {
    double diff = wrap_to_180(raw - st.last_raw_deg);
    // Rejet d'outliers : un saut > max_step_deg ne peut pas etre physique
    // (cinematique max << 1000 deg/s a 50 Hz cycle = 20 deg/cycle).
    if (std::abs(diff) > config_.max_step_deg) {
      st.outliers_count += 1;
      return;  // garde last_raw et unwrapped intacts
    }
    st.unwrapped_deg += diff;
  }
  st.last_raw_deg = raw;
  st.median_buffer.push_back(st.unwrapped_deg);
  while (st.median_buffer.size() > config_.median_filter_size) {
    st.median_buffer.pop_front();
  }
}

double EncoderDriver::median(const std::deque<double> & buf)
{
  std::vector<double> sorted(buf.begin(), buf.end());
  std::sort(sorted.begin(), sorted.end());
  size_t n = sorted.size();
  if (n == 0) return std::numeric_limits<double>::quiet_NaN();
  if (n % 2 == 1) return sorted[n / 2];
  return 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
}

std::optional<double> EncoderDriver::motor_to_joint_rad(const JointSpec & spec) const
{
  auto it = states_.find(spec.slave_id);
  if (it == states_.end() || it->second.median_buffer.empty()) {
    return std::nullopt;
  }
  double filtered_unwrap_deg = median(it->second.median_buffer);
  double j = filtered_unwrap_deg * M_PI / 180.0
    * static_cast<double>(spec.gear_num) / static_cast<double>(spec.gear_den);
  if (spec.inverted) j = -j;
  return j;
}

void EncoderDriver::poll_all()
{
  // Si le thread async tourne, no-op (le thread fait deja le poll).
  if (poll_thread_.joinable()) {
    return;
  }
  std::lock_guard<std::mutex> lock(states_mutex_);
  for (const auto & spec : joint_specs_) {
    auto raw = query(spec.slave_id);
    update_tracker(spec.slave_id, raw);
  }
}

void EncoderDriver::poll_loop()
{
  while (!stop_polling_.load(std::memory_order_acquire)) {
    for (const auto & spec : joint_specs_) {
      if (stop_polling_.load(std::memory_order_acquire)) break;
      // query() prend ~6 ms et bloque, mais HORS lock (impacts uniquement le
      // thread async, pas le ros2_control read()).
      auto raw = query(spec.slave_id);
      std::lock_guard<std::mutex> lock(states_mutex_);
      update_tracker(spec.slave_id, raw);
    }
  }
}

void EncoderDriver::start_polling_thread()
{
  if (poll_thread_.joinable()) return;  // deja demarre
  if (config_.mock || serial_fd_ < 0) return;  // pas de poll en mock
  stop_polling_.store(false, std::memory_order_release);
  poll_thread_ = std::thread(&EncoderDriver::poll_loop, this);
}

void EncoderDriver::stop_polling_thread()
{
  if (!poll_thread_.joinable()) return;
  stop_polling_.store(true, std::memory_order_release);
  poll_thread_.join();
}

std::optional<double> EncoderDriver::get_joint_position_rad(int joint_idx) const
{
  // Lock pour acceder aux states_ pendant que poll_loop peut les modifier
  std::lock_guard<std::mutex> lock(states_mutex_);

  const JointSpec * spec = nullptr;
  for (const auto & js : joint_specs_) {
    if (js.joint_idx == joint_idx) {
      spec = &js;
      break;
    }
  }
  if (!spec) return std::nullopt;

  auto pos = motor_to_joint_rad(*spec);
  if (!pos.has_value()) return std::nullopt;

  double result = pos.value();
  for (const auto & c : couplings_) {
    if (c.to_joint == joint_idx) {
      for (const auto & js : joint_specs_) {
        if (js.joint_idx == c.from_joint) {
          auto from_pos = motor_to_joint_rad(js);
          if (from_pos.has_value()) {
            result += from_pos.value() * c.ratio;
          }
          break;
        }
      }
    }
  }
  return result;
}

int EncoderDriver::outliers_count(int joint_idx) const
{
  for (const auto & js : joint_specs_) {
    if (js.joint_idx == joint_idx) {
      auto it = states_.find(js.slave_id);
      if (it != states_.end()) return it->second.outliers_count;
    }
  }
  return 0;
}

bool EncoderDriver::load_calibration_yaml(const std::string & yaml_path)
{
  std::ifstream f(yaml_path);
  if (!f) {
    std::cerr << "[EncoderDriver] cannot open calibration YAML: " << yaml_path << std::endl;
    return false;
  }

  // Parsing minimal : on cherche les lignes "  motor_N: <value>" sous
  // "encoder_raw_init_deg:". Pas de gestion complete YAML — juste ce format.
  std::map<int, double> raw_inits;
  std::string line;
  bool in_section = false;
  while (std::getline(f, line)) {
    // strip leading whitespace pour la detection de section
    auto first_non_ws = line.find_first_not_of(" \t");
    if (first_non_ws == std::string::npos) continue;

    std::string trimmed = line.substr(first_non_ws);
    if (trimmed[0] == '#') continue;  // commentaire

    if (trimmed.find("encoder_raw_init_deg:") == 0) {
      in_section = true;
      continue;
    }
    if (in_section) {
      // Format attendu : "motor_N: VALUE"
      // Si la ligne ne commence pas par un espace (donc first_non_ws == 0), on sort de la section.
      if (first_non_ws == 0) {
        in_section = false;
        continue;
      }
      auto colon = trimmed.find(':');
      if (colon == std::string::npos) continue;
      std::string key = trimmed.substr(0, colon);
      std::string val = trimmed.substr(colon + 1);
      // strip
      while (!val.empty() && (val.front() == ' ' || val.front() == '\t')) val.erase(val.begin());
      // motor_N
      if (key.rfind("motor_", 0) == 0) {
        try {
          int n = std::stoi(key.substr(6));
          double v = std::stod(val);
          raw_inits[n] = v;
        } catch (...) {
          // skip
        }
      }
    }
  }

  if (raw_inits.empty()) {
    std::cerr << "[EncoderDriver] no motor_N entries found in " << yaml_path << std::endl;
    return false;
  }

  // Update joint_specs_ : map motor_N <-> slave_id N
  for (auto & spec : joint_specs_) {
    auto it = raw_inits.find(spec.slave_id);
    if (it != raw_inits.end()) {
      spec.raw_init_deg = it->second;
    }
  }
  return true;
}

}  // namespace roby_hardware
