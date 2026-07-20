#include "roby_hardware/stepper_driver.hpp"

#include <cmath>
#include <thread>
#include <chrono>

namespace roby_hardware
{

static constexpr double TWO_PI = 2.0 * M_PI;

StepperDriver::~StepperDriver()
{
  shutdown();
}

bool StepperDriver::init(const StepperConfig & config)
{
  config_ = config;
  current_steps_ = 0;
  direction_initialized_ = false;

  if (config_.mock) {
    return true;
  }

#ifdef HAS_GPIOD
  chip_ = gpiod_chip_open(config_.gpio_chip.c_str());
  if (!chip_) {
    return false;
  }

  // Request STEP line as output (initial LOW)
  step_line_ = gpiod_chip_get_line(chip_, config_.step_pin);
  if (!step_line_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    return false;
  }
  if (gpiod_line_request_output(step_line_, "roby_hardware", 0) < 0) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    step_line_ = nullptr;
    return false;
  }

  // Request DIR line as output (initial LOW)
  dir_line_ = gpiod_chip_get_line(chip_, config_.dir_pin);
  if (!dir_line_) {
    gpiod_line_release(step_line_);
    step_line_ = nullptr;
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    return false;
  }
  if (gpiod_line_request_output(dir_line_, "roby_hardware", 0) < 0) {
    gpiod_line_release(step_line_);
    step_line_ = nullptr;
    dir_line_ = nullptr;
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    return false;
  }
#endif

  return true;
}

void StepperDriver::shutdown()
{
#ifdef HAS_GPIOD
  if (step_line_) {
    gpiod_line_release(step_line_);
    step_line_ = nullptr;
  }
  if (dir_line_) {
    gpiod_line_release(dir_line_);
    dir_line_ = nullptr;
  }
  if (chip_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
#endif
}

double StepperDriver::steps_per_axis_rev() const
{
  // Motor steps per rev * (gear_den / gear_num) = steps per axis revolution
  return static_cast<double>(config_.steps_per_rev) *
         static_cast<double>(config_.gear_ratio_den) /
         static_cast<double>(config_.gear_ratio_num);
}

int64_t StepperDriver::rad_to_steps(double rad) const
{
  // radians -> fraction of axis revolution -> motor steps
  double axis_revs = rad / TWO_PI;
  double motor_steps = axis_revs * steps_per_axis_rev();
  return static_cast<int64_t>(std::round(motor_steps));
}

double StepperDriver::steps_to_rad(int64_t steps) const
{
  double axis_revs = static_cast<double>(steps) / steps_per_axis_rev();
  return axis_revs * TWO_PI;
}

double StepperDriver::get_position_rad() const
{
  return steps_to_rad(current_steps_);
}

int64_t StepperDriver::get_position_steps() const
{
  return current_steps_;
}

void StepperDriver::set_position_rad(double rad)
{
  current_steps_ = rad_to_steps(rad);
}

int StepperDriver::move_toward(double target_rad, int max_steps_per_cycle)
{
  int64_t target_steps = rad_to_steps(target_rad);
  int64_t delta = target_steps - current_steps_;

  if (delta == 0) {
    return 0;
  }

  bool forward = (delta > 0);

  int64_t abs_delta = std::abs(delta);
  int64_t steps_to_do = std::min(abs_delta, static_cast<int64_t>(max_steps_per_cycle));

  // Handle direction change delay
  if (direction_initialized_ && forward != current_direction_) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_direction_change_).count();
    if (elapsed_ms < DIRECTION_CHANGE_DELAY_MS) {
      return 0;  // Wait for direction stabilization
    }
  }

  set_direction(forward);

  // Generate step pulses, spread evenly over the control cycle (~10ms at 100Hz)
  // to avoid burst-then-pause vibration
  int inter_step_us = 0;
  if (steps_to_do > 1) {
    // 10000µs (10ms cycle) / steps_to_do, minus pulse time
    inter_step_us = static_cast<int>(10000 / steps_to_do) - (2 * PULSE_WIDTH_US);
    if (inter_step_us < 0) inter_step_us = 0;
    // Cap at a reasonable value to avoid exceeding the cycle
    if (inter_step_us > 2000) inter_step_us = 2000;
  }

  for (int64_t i = 0; i < steps_to_do; ++i) {
    pulse_step();
    if (inter_step_us > 0 && i < steps_to_do - 1) {
      std::this_thread::sleep_for(std::chrono::microseconds(inter_step_us));
    }
  }

  // Update counter (always in logical direction, inversion is handled in GPIO only)
  if (forward) {
    current_steps_ += steps_to_do;
  } else {
    current_steps_ -= steps_to_do;
  }

  return static_cast<int>(steps_to_do);
}

int StepperDriver::prepare_move(double target_rad, int max_steps_per_cycle)
{
  int64_t target_steps = rad_to_steps(target_rad);
  int64_t delta = target_steps - current_steps_;

  if (delta == 0) {
    prepared_remaining_ = 0;
    return 0;
  }

  prepared_forward_ = (delta > 0);

  int64_t abs_delta = std::abs(delta);
  int64_t steps_to_do = std::min(abs_delta, static_cast<int64_t>(max_steps_per_cycle));

  // Handle direction change delay
  if (direction_initialized_ && prepared_forward_ != current_direction_) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_direction_change_).count();
    if (elapsed_ms < DIRECTION_CHANGE_DELAY_MS) {
      prepared_remaining_ = 0;
      return 0;
    }
  }

  set_direction(prepared_forward_);
  prepared_remaining_ = static_cast<int>(steps_to_do);
  return prepared_remaining_;
}

bool StepperDriver::step_once()
{
  if (prepared_remaining_ <= 0) {
    return false;
  }

  pulse_step();
  prepared_remaining_--;

  // Update counter
  if (prepared_forward_) {
    current_steps_++;
  } else {
    current_steps_--;
  }

  return true;
}

void StepperDriver::raise_step()
{
  if (config_.mock || prepared_remaining_ <= 0) {
    return;
  }
#ifdef HAS_GPIOD
  if (!dry_run_ && step_line_) {
    gpiod_line_set_value(step_line_, 1);
  }
#endif
}

void StepperDriver::lower_step_and_commit()
{
  if (prepared_remaining_ <= 0) {
    return;
  }
#ifdef HAS_GPIOD
  if (!config_.mock && !dry_run_ && step_line_) {
    gpiod_line_set_value(step_line_, 0);
  }
#endif
  prepared_remaining_--;
  if (prepared_forward_) {
    current_steps_++;
  } else {
    current_steps_--;
  }
}

void StepperDriver::set_direction(bool forward)
{
  if (direction_initialized_ && forward == current_direction_) {
    return;
  }

  current_direction_ = forward;
  direction_initialized_ = true;
  last_direction_change_ = std::chrono::steady_clock::now();

#ifdef HAS_GPIOD
  if (!dry_run_ && dir_line_) {
    int dir_val = forward ? 1 : 0;
    if (config_.inverted) dir_val = !dir_val;
    gpiod_line_set_value(dir_line_, dir_val);
    // CL86Y : DIR doit etre stable >=5us avant la 1ere impulsion PUL. Busy-wait
    // DIR_SETUP_US, uniquement lors d un changement de sens (cout negligeable).
    { auto e = std::chrono::steady_clock::now() + std::chrono::microseconds(DIR_SETUP_US);
      while (std::chrono::steady_clock::now() < e) { } }
  }
#endif
}

void StepperDriver::pulse_step()
{
  if (config_.mock) {
    return;
  }

#ifdef HAS_GPIOD
  if (!dry_run_ && step_line_) {
    // Busy-wait pour la largeur d impulsion : sans priorite RT, sleep_for(3us)
    // deborde a ~130us => chaque pas coute ~260us => write() explose (overrun
    // RT). L attente active est precise a la us.
    gpiod_line_set_value(step_line_, 1);
    { auto e = std::chrono::steady_clock::now() + std::chrono::microseconds(PULSE_WIDTH_US);
      while (std::chrono::steady_clock::now() < e) { } }
    gpiod_line_set_value(step_line_, 0);
    { auto e = std::chrono::steady_clock::now() + std::chrono::microseconds(PULSE_WIDTH_US);
      while (std::chrono::steady_clock::now() < e) { } }
  }
#endif
}

}  // namespace roby_hardware
