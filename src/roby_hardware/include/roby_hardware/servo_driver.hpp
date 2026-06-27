#ifndef ROBY_HARDWARE__SERVO_DRIVER_HPP_
#define ROBY_HARDWARE__SERVO_DRIVER_HPP_

#include <cstdint>
#include <string>

namespace roby_hardware
{

struct ServoConfig
{
  int channel = 0;            // PCA9685 channel (0-15)
  double angle_min_deg = 0.0;
  double angle_max_deg = 180.0;
  double angle_init_deg = 90.0;
  bool inverted = false;
  bool mock = false;
  std::string i2c_bus = "/dev/i2c-1";
  int pca9685_address = 0x40;
};

class ServoDriver
{
public:
  ServoDriver() = default;
  ~ServoDriver();

  /// Initialize the driver. Returns false on failure.
  bool init(const ServoConfig & config);

  /// Release I2C resources.
  void shutdown();

  /// Set servo angle in degrees. Clamped to [angle_min, angle_max].
  void set_angle_deg(double angle_deg);

  /// Get last commanded angle in degrees.
  double get_angle_deg() const;

  /// Convert angle in degrees to radians.
  static double deg_to_rad(double deg);

  /// Convert radians to degrees.
  static double rad_to_deg(double rad);

  /// Convert angle to PWM duty cycle value (0-65535).
  static uint16_t angle_to_duty(double angle_deg);

  /// Convert angle to pulse width in microseconds.
  static double angle_to_pulse_us(double angle_deg);

  /// Get config for testing.
  const ServoConfig & get_config() const { return config_; }

private:
  bool init_pca9685();
  void write_channel(int channel, uint16_t on, uint16_t off);
  void write_register(uint8_t reg, uint8_t value);

  ServoConfig config_;
  double current_angle_deg_ = 0.0;
  uint16_t last_off_tick_ = 0xFFFF;   // sentinelle : force la 1re ecriture (init)
  int i2c_fd_ = -1;
  bool pca9685_initialized_ = false;

  // PCA9685 registers
  static constexpr uint8_t PCA9685_MODE1 = 0x00;
  static constexpr uint8_t PCA9685_PRESCALE = 0xFE;
  static constexpr uint8_t PCA9685_LED0_ON_L = 0x06;
  static constexpr uint8_t PCA9685_PRESCALE_50HZ = 121;  // 25MHz / (4096 * 50Hz) - 1
};

}  // namespace roby_hardware

#endif  // ROBY_HARDWARE__SERVO_DRIVER_HPP_
