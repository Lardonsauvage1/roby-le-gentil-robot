#include "roby_hardware/servo_driver.hpp"

#include <cmath>
#include <algorithm>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#endif

namespace roby_hardware
{

static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / M_PI;

ServoDriver::~ServoDriver()
{
  shutdown();
}

bool ServoDriver::init(const ServoConfig & config)
{
  config_ = config;
  current_angle_deg_ = config_.angle_init_deg;

  if (config_.mock) {
    return true;
  }

#ifdef __linux__
  i2c_fd_ = open(config_.i2c_bus.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    return false;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, config_.pca9685_address) < 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  if (!init_pca9685()) {
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // Set initial position
  set_angle_deg(current_angle_deg_);
#endif

  return true;
}

void ServoDriver::shutdown()
{
#ifdef __linux__
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
#endif
  pca9685_initialized_ = false;
}

bool ServoDriver::init_pca9685()
{
#ifdef __linux__
  if (i2c_fd_ < 0) return false;

  // Reset: sleep mode
  write_register(PCA9685_MODE1, 0x10);  // sleep
  // Set prescaler for 50Hz
  write_register(PCA9685_PRESCALE, PCA9685_PRESCALE_50HZ);
  // Wake up, auto-increment
  write_register(PCA9685_MODE1, 0x20);  // auto-increment enabled
  // Wait for oscillator (500µs per datasheet)
  usleep(500);
  // Clear restart bit
  write_register(PCA9685_MODE1, 0xA0);  // restart + auto-increment

  pca9685_initialized_ = true;
  return true;
#else
  return false;
#endif
}

void ServoDriver::set_angle_deg(double angle_deg)
{
  // Apply inversion
  double effective_angle = angle_deg;
  if (config_.inverted) {
    effective_angle = (config_.angle_max_deg + config_.angle_min_deg) - angle_deg;
  }

  // Clamp to valid range
  effective_angle = std::clamp(effective_angle, config_.angle_min_deg, config_.angle_max_deg);
  current_angle_deg_ = angle_deg;

  if (config_.mock) {
    return;
  }

  // Convert to PWM
  double pulse_us = angle_to_pulse_us(effective_angle);
  // PCA9685: 12-bit resolution over 20ms period
  // ON at tick 0, OFF at (pulse_us / 20000) * 4096
  uint16_t off_tick = static_cast<uint16_t>((pulse_us / 20000.0) * 4096.0);

  write_channel(config_.channel, 0, off_tick);
}

double ServoDriver::get_angle_deg() const
{
  return current_angle_deg_;
}

double ServoDriver::deg_to_rad(double deg)
{
  return deg * DEG_TO_RAD;
}

double ServoDriver::rad_to_deg(double rad)
{
  return rad * RAD_TO_DEG;
}

double ServoDriver::angle_to_pulse_us(double angle_deg)
{
  return 500.0 + (angle_deg / 180.0) * 2000.0;
}

uint16_t ServoDriver::angle_to_duty(double angle_deg)
{
  double pulse_us = angle_to_pulse_us(angle_deg);
  return static_cast<uint16_t>((pulse_us / 20000.0) * 65535.0);
}

void ServoDriver::write_channel(int channel, uint16_t on, uint16_t off)
{
#ifdef __linux__
  if (i2c_fd_ < 0 || !pca9685_initialized_) return;

  uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
  uint8_t data[5] = {
    reg,
    static_cast<uint8_t>(on & 0xFF),
    static_cast<uint8_t>((on >> 8) & 0x0F),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>((off >> 8) & 0x0F)
  };
  write(i2c_fd_, data, 5);
#else
  (void)channel;
  (void)on;
  (void)off;
#endif
}

void ServoDriver::write_register(uint8_t reg, uint8_t value)
{
#ifdef __linux__
  if (i2c_fd_ < 0) return;

  uint8_t data[2] = {reg, value};
  write(i2c_fd_, data, 2);
#else
  (void)reg;
  (void)value;
#endif
}

}  // namespace roby_hardware
