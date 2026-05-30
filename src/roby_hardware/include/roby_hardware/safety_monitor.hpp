#ifndef ROBY_HARDWARE__SAFETY_MONITOR_HPP_
#define ROBY_HARDWARE__SAFETY_MONITOR_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace roby_hardware
{

struct JointSafetyConfig
{
  std::string name;
  double position_min_rad = 0.0;   // lower soft limit
  double position_max_rad = 0.0;   // upper soft limit
  double max_velocity_rad_per_tick = 0.0;  // max position change per control cycle
  double warning_deviation_rad = 0.0;      // watchdog warning threshold
  double critical_deviation_rad = 0.0;     // watchdog critical threshold (trigger stop)
  double decel_zone_fraction = 0.10;       // 10% of range at each end
  double decel_min_factor = 0.25;          // minimum velocity scaling in decel zone
};

class SafetyMonitor
{
public:
  SafetyMonitor() = default;

  /// Configure with joint safety parameters.
  void init(const std::vector<JointSafetyConfig> & configs);

  /// Clamp a commanded position for one joint. Returns the safe position.
  /// current_rad: current actual position
  /// command_rad: desired target position
  double clamp_command(size_t joint_idx, double current_rad, double command_rad) const;

  /// Check deviation between commanded and actual position.
  /// Returns 0 = OK, 1 = warning, 2 = critical.
  int check_deviation(size_t joint_idx, double actual_rad, double commanded_rad) const;

  /// Check all joints. Returns true if any joint is in critical state.
  bool check_all_deviations(
    const std::vector<double> & actual,
    const std::vector<double> & commanded) const;

  /// Communication watchdog: call with cycles_since_last_command.
  /// Returns velocity scaling factor (1.0 = normal, 0.0 = stopped).
  static double comm_watchdog_factor(int cycles_since_last_command);

  /// Get number of configured joints.
  size_t num_joints() const { return configs_.size(); }

  /// Get config for a joint (for testing).
  const JointSafetyConfig & get_config(size_t idx) const { return configs_[idx]; }

private:
  /// Compute deceleration factor based on position in joint range.
  double decel_factor(size_t joint_idx, double position_rad) const;

  std::vector<JointSafetyConfig> configs_;

  static constexpr int COMM_WATCHDOG_START = 50;   // start slowing after 50 cycles (0.5s)
  static constexpr int COMM_WATCHDOG_STOP = 100;   // full stop after 100 cycles (1.0s)
};

}  // namespace roby_hardware

#endif  // ROBY_HARDWARE__SAFETY_MONITOR_HPP_
