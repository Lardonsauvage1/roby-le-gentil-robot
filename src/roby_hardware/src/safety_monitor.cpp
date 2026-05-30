#include "roby_hardware/safety_monitor.hpp"

#include <cmath>
#include <algorithm>

namespace roby_hardware
{

void SafetyMonitor::init(const std::vector<JointSafetyConfig> & configs)
{
  configs_ = configs;
}

double SafetyMonitor::decel_factor(size_t joint_idx, double position_rad) const
{
  const auto & cfg = configs_[joint_idx];
  double range = cfg.position_max_rad - cfg.position_min_rad;
  double zone = range * cfg.decel_zone_fraction;

  if (zone <= 0.0) {
    return 1.0;
  }

  double dist_from_min = position_rad - cfg.position_min_rad;
  double dist_from_max = cfg.position_max_rad - position_rad;
  double dist_to_edge = std::min(dist_from_min, dist_from_max);

  if (dist_to_edge >= zone) {
    return 1.0;
  }

  if (dist_to_edge <= 0.0) {
    return cfg.decel_min_factor;
  }

  // Linear interpolation from decel_min_factor to 1.0
  double t = dist_to_edge / zone;
  return cfg.decel_min_factor + t * (1.0 - cfg.decel_min_factor);
}

double SafetyMonitor::clamp_command(
  size_t joint_idx, double current_rad, double command_rad) const
{
  if (joint_idx >= configs_.size()) {
    return command_rad;
  }

  const auto & cfg = configs_[joint_idx];

  // 1. Clamp to hard limits
  double clamped = std::clamp(command_rad, cfg.position_min_rad, cfg.position_max_rad);

  // 2. Apply velocity limit
  double delta = clamped - current_rad;
  double max_delta = cfg.max_velocity_rad_per_tick;

  // 3. Apply deceleration zone factor
  double factor = decel_factor(joint_idx, current_rad);
  max_delta *= factor;

  if (std::abs(delta) > max_delta) {
    delta = std::copysign(max_delta, delta);
  }

  return current_rad + delta;
}

int SafetyMonitor::check_deviation(
  size_t joint_idx, double actual_rad, double commanded_rad) const
{
  if (joint_idx >= configs_.size()) {
    return 0;
  }

  const auto & cfg = configs_[joint_idx];
  double dev = std::abs(actual_rad - commanded_rad);

  if (dev >= cfg.critical_deviation_rad) {
    return 2;
  }
  if (dev >= cfg.warning_deviation_rad) {
    return 1;
  }
  return 0;
}

bool SafetyMonitor::check_all_deviations(
  const std::vector<double> & actual,
  const std::vector<double> & commanded) const
{
  size_t n = std::min({actual.size(), commanded.size(), configs_.size()});
  for (size_t i = 0; i < n; ++i) {
    if (check_deviation(i, actual[i], commanded[i]) >= 2) {
      return true;  // critical
    }
  }
  return false;
}

double SafetyMonitor::comm_watchdog_factor(int cycles_since_last_command)
{
  if (cycles_since_last_command <= COMM_WATCHDOG_START) {
    return 1.0;
  }
  if (cycles_since_last_command >= COMM_WATCHDOG_STOP) {
    return 0.0;
  }
  // Linear ramp-down
  double t = static_cast<double>(cycles_since_last_command - COMM_WATCHDOG_START) /
             static_cast<double>(COMM_WATCHDOG_STOP - COMM_WATCHDOG_START);
  return 1.0 - t;
}

}  // namespace roby_hardware
