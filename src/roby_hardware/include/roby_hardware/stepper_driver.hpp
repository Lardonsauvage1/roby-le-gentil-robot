#ifndef ROBY_HARDWARE__STEPPER_DRIVER_HPP_
#define ROBY_HARDWARE__STEPPER_DRIVER_HPP_

#include <cstdint>
#include <string>
#include <chrono>

#ifdef HAS_GPIOD
#include <gpiod.h>
#endif

namespace roby_hardware
{

struct StepperConfig
{
  int step_pin = 0;
  int dir_pin = 0;
  int gear_ratio_num = 1;    // numerator (motor side)
  int gear_ratio_den = 1;    // denominator (axis side)
  int steps_per_rev = 12800; // motor-side microstepping
  bool inverted = false;
  bool mock = false;
  std::string gpio_chip = "/dev/gpiochip0";
};

class StepperDriver
{
public:
  StepperDriver() = default;
  ~StepperDriver();

  /// Initialize the driver with configuration. Returns false on failure.
  bool init(const StepperConfig & config);

  /// Release GPIO resources.
  void shutdown();

  /// Move toward target position (in radians). Call once per control cycle.
  /// Returns the number of steps actually executed this cycle.
  int move_toward(double target_rad, int max_steps_per_cycle);

  /// Prepare a move: calculate steps needed, set direction. Returns steps to do.
  /// Call before step_once() loop for interleaved multi-motor stepping.
  int prepare_move(double target_rad, int max_steps_per_cycle);

  /// Execute a single step pulse and update counter. Returns true if step was sent.
  bool step_once();

  /// Get remaining steps from last prepare_move().
  int remaining_steps() const { return prepared_remaining_; }

  /// Pulse groupe : reste-t-il un pas prepare a emettre ?
  bool has_pending_step() const { return prepared_remaining_ > 0; }

  /// Pulse groupe : lever la ligne STEP (sans attente). Appeler pour tous les
  /// moteurs en attente, puis UN busy-wait partage, puis lower_step_and_commit().
  void raise_step();

  /// Pulse groupe : baisser la ligne STEP + decrementer + maj compteur de pas.
  void lower_step_and_commit();

  /// Get current position in radians (from step counter).
  double get_position_rad() const;

  /// Get current position in steps.
  int64_t get_position_steps() const;

  /// Set current position without moving (for initialization).
  void set_position_rad(double rad);

  /// Convert radians to steps (axis-side radians to motor steps).
  int64_t rad_to_steps(double rad) const;

  /// Convert steps to radians.
  double steps_to_rad(int64_t steps) const;

  /// Get the steps per axis revolution (after gear reduction).
  double steps_per_axis_rev() const;

private:
  void set_direction(bool forward);
  void pulse_step();

  StepperConfig config_;
  int64_t current_steps_ = 0;
  bool current_direction_ = true;  // true = forward
  bool direction_initialized_ = false;
  std::chrono::steady_clock::time_point last_direction_change_;

  static constexpr int DIRECTION_CHANGE_DELAY_MS = 50;
  static constexpr int PULSE_WIDTH_US = 3;  // slightly above 2µs minimum
  static constexpr int DIR_SETUP_US = 10;   // CL86Y: DIR stable >=5us avant la 1ere impulsion PUL

  int prepared_remaining_ = 0;
  bool prepared_forward_ = true;

#ifdef HAS_GPIOD
  struct gpiod_chip * chip_ = nullptr;
  struct gpiod_line * step_line_ = nullptr;
  struct gpiod_line * dir_line_ = nullptr;
#endif
};

}  // namespace roby_hardware

#endif  // ROBY_HARDWARE__STEPPER_DRIVER_HPP_
