#include "roby_hardware/roby_system.hpp"

#include <cmath>
#include <limits>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace roby_hardware
{

static constexpr double DEG_TO_RAD = M_PI / 180.0;

// Helper to parse hardware parameters
std::string RobySystem::get_param(
  const std::string & name, const std::string & default_val) const
{
  auto it = info_.hardware_parameters.find(name);
  if (it != info_.hardware_parameters.end()) {
    return it->second;
  }
  return default_val;
}

int RobySystem::get_param_int(const std::string & name, int default_val) const
{
  auto s = get_param(name, "");
  if (s.empty()) return default_val;
  try {
    // Handle hex (0x prefix)
    if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
      return static_cast<int>(std::stoul(s, nullptr, 16));
    }
    return std::stoi(s);
  } catch (...) {
    return default_val;
  }
}

double RobySystem::get_param_double(const std::string & name, double default_val) const
{
  auto s = get_param(name, "");
  if (s.empty()) return default_val;
  try {
    return std::stod(s);
  } catch (...) {
    return default_val;
  }
}

bool RobySystem::get_param_bool(const std::string & name, bool default_val) const
{
  auto s = get_param(name, "");
  if (s.empty()) return default_val;
  return (s == "true" || s == "True" || s == "1");
}

hardware_interface::CallbackReturn RobySystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse global parameters
  std::string gpio_chip = get_param("gpio_chip", "/dev/gpiochip4");
  int steps_per_rev = get_param_int("stepper_steps_per_rev", 12800);
  std::string i2c_bus = get_param("i2c_bus", "/dev/i2c-1");
  int pca_addr = get_param_int("pca9685_address", 0x40);

  // Coupling
  coupling_enabled_ = get_param_bool("coupling_enabled", false);
  if (coupling_enabled_) {
    int m2_num = get_param_int("coupling_ratio_m2_num", 6000);
    int m2_den = get_param_int("coupling_ratio_m2_den", 45056);
    coupling_ratio_m2_ = static_cast<double>(m2_num) / static_cast<double>(m2_den);
    // M3 ratio = (15*20)/(44*32)
    coupling_ratio_m3_ = (15.0 * 20.0) / (44.0 * 32.0);
  }

  // Initialize joints
  joints_.resize(info_.joints.size());
  stepper_index_.resize(info_.joints.size(), -1);
  servo_index_.resize(info_.joints.size(), -1);

  std::vector<JointSafetyConfig> safety_configs;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    joints_[i].name = joint.name;

    // Read initial position from state interface params
    for (const auto & si : joint.state_interfaces) {
      if (si.name == "position") {
        auto it = si.parameters.find("initial_value");
        if (it != si.parameters.end()) {
          try {
            joints_[i].position = std::stod(it->second);
            joints_[i].command = joints_[i].position;
            joints_[i].prev_position = joints_[i].position;
          } catch (...) {}
        }
      }
    }

    // Parse joint type
    std::string type_str = get_param(joint.name + "_type", "mock");

    if (type_str == "stepper") {
      joints_[i].type = JointType::STEPPER;

      StepperConfig cfg;
      cfg.gpio_chip = gpio_chip;
      cfg.steps_per_rev = steps_per_rev;
      cfg.step_pin = get_param_int(joint.name + "_step_pin", 0);
      cfg.dir_pin = get_param_int(joint.name + "_dir_pin", 0);
      cfg.gear_ratio_num = get_param_int(joint.name + "_gear_ratio_num", 1);
      cfg.gear_ratio_den = get_param_int(joint.name + "_gear_ratio_den", 1);
      cfg.inverted = get_param_bool(joint.name + "_inverted", false);

      // Check if GPIO is actually available
#ifndef HAS_GPIOD
      cfg.mock = true;
#else
      cfg.mock = false;
#endif

      auto stepper = std::make_unique<StepperDriver>();
      if (!stepper->init(cfg)) {
        RCLCPP_ERROR(rclcpp::get_logger("RobySystem"),
          "Failed to init stepper for %s", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      stepper->set_position_rad(joints_[i].position);

      stepper_index_[i] = static_cast<int>(steppers_.size());
      steppers_.push_back(std::move(stepper));

    } else if (type_str == "servo") {
      joints_[i].type = JointType::SERVO;

      ServoConfig cfg;
      cfg.i2c_bus = i2c_bus;
      cfg.pca9685_address = pca_addr;
      cfg.channel = get_param_int(joint.name + "_servo_channel", 0);
      cfg.angle_min_deg = get_param_double(joint.name + "_angle_min_deg", 0.0);
      cfg.angle_max_deg = get_param_double(joint.name + "_angle_max_deg", 180.0);
      cfg.angle_init_deg = get_param_double(joint.name + "_angle_init_deg", 90.0);
      cfg.inverted = get_param_bool(joint.name + "_inverted", false);

#ifdef __linux__
      // Only try real I2C if the device exists
      cfg.mock = (access(cfg.i2c_bus.c_str(), F_OK) != 0);
#else
      cfg.mock = true;
#endif

      auto servo = std::make_unique<ServoDriver>();
      if (!servo->init(cfg)) {
        RCLCPP_ERROR(rclcpp::get_logger("RobySystem"),
          "Failed to init servo for %s", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      servo_index_[i] = static_cast<int>(servos_.size());
      servos_.push_back(std::move(servo));

    } else {
      joints_[i].type = JointType::MOCK;
    }

    // Build safety config from URDF joint limits
    JointSafetyConfig sc;
    sc.name = joint.name;

    // Get limits from URDF joint definition
    if (!joint.command_interfaces.empty()) {
      // Use limits from URDF if available, otherwise use wide defaults
      sc.position_min_rad = -M_PI;
      sc.position_max_rad = M_PI;

      // Parse from the joint parameters in HardwareInfo
      // The actual limits come from the URDF <limit> tag, available via joint info
    }

    // Set velocity and deviation thresholds based on joint type
    if (joints_[i].type == JointType::STEPPER) {
      sc.max_velocity_rad_per_tick = 3.0 * DEG_TO_RAD;   // 3°/tick
      sc.warning_deviation_rad = 5.0 * DEG_TO_RAD;        // 5°
      sc.critical_deviation_rad = 15.0 * DEG_TO_RAD;      // 15°
    } else {
      sc.max_velocity_rad_per_tick = 8.0 * DEG_TO_RAD;   // 8°/tick
      sc.warning_deviation_rad = 8.0 * DEG_TO_RAD;        // 8°
      sc.critical_deviation_rad = 20.0 * DEG_TO_RAD;      // 20°
    }

    safety_configs.push_back(sc);
  }

  safety_.init(safety_configs);

  RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
    "Initialized with %zu joints (%zu steppers, %zu servos)",
    joints_.size(), steppers_.size(), servos_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobySystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobySystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set commands to current positions (no jump on activation)
  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].command = joints_[i].position;
    joints_[i].prev_position = joints_[i].position;
  }
  cycles_since_command_ = 0;

  RCLCPP_INFO(rclcpp::get_logger("RobySystem"), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobySystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Shutdown all drivers
  for (auto & s : steppers_) {
    s->shutdown();
  }
  for (auto & s : servos_) {
    s->shutdown();
  }

  RCLCPP_INFO(rclcpp::get_logger("RobySystem"), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobySystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    interfaces.emplace_back(
      joints_[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].position);
    interfaces.emplace_back(
      joints_[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RobySystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    interfaces.emplace_back(
      joints_[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command);
  }
  return interfaces;
}

hardware_interface::return_type RobySystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) dt = 0.01;  // fallback 100Hz

  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].prev_position = joints_[i].position;

    if (joints_[i].type == JointType::STEPPER && stepper_index_[i] >= 0) {
      joints_[i].position = steppers_[stepper_index_[i]]->get_position_rad();
    } else if (joints_[i].type == JointType::SERVO && servo_index_[i] >= 0) {
      double angle_deg = servos_[servo_index_[i]]->get_angle_deg();
      joints_[i].position = ServoDriver::deg_to_rad(angle_deg);
    }
    // MOCK joints: position = command (set in write)

    joints_[i].velocity = (joints_[i].position - joints_[i].prev_position) / dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobySystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Communication watchdog: reset when any command differs from position
  // (the controller continuously writes to joints_[i].command)
  bool any_command_active = false;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (std::abs(joints_[i].command - joints_[i].position) > 1e-6) {
      any_command_active = true;
      break;
    }
  }
  if (any_command_active) {
    cycles_since_command_ = 0;
  } else {
    cycles_since_command_++;
  }
  double comm_factor = SafetyMonitor::comm_watchdog_factor(cycles_since_command_);

  for (size_t i = 0; i < joints_.size(); ++i) {
    double cmd = joints_[i].command;

    // Apply coupling compensation for axis 3 BEFORE safety clamping
    // so the clamp works on the actual motor target, not the raw joint command
    if (coupling_enabled_ && joints_[i].name == "joint_3") {
      for (size_t j = 0; j < joints_.size(); ++j) {
        if (joints_[j].name == "joint_2") {
          cmd = compensate_coupling(cmd, joints_[j].position);
          break;
        }
      }
    }

    // Apply safety clamping (on the effective command, after coupling)
    cmd = safety_.clamp_command(i, joints_[i].position, cmd);

    // Apply communication watchdog scaling
    if (comm_factor < 1.0) {
      double delta = cmd - joints_[i].position;
      cmd = joints_[i].position + delta * comm_factor;
    }

    if (joints_[i].type == JointType::STEPPER && stepper_index_[i] >= 0) {
      // For stepper: prepare move (sets direction, calculates steps)
      auto & sc = safety_.get_config(i);
      double max_rad = sc.max_velocity_rad_per_tick;
      int max_steps = static_cast<int>(
        steppers_[stepper_index_[i]]->rad_to_steps(max_rad));
      if (max_steps < 1) max_steps = 1;

      steppers_[stepper_index_[i]]->prepare_move(cmd, max_steps);

    } else if (joints_[i].type == JointType::SERVO && servo_index_[i] >= 0) {
      double angle_deg = ServoDriver::rad_to_deg(cmd);
      servos_[servo_index_[i]]->set_angle_deg(angle_deg);

    } else {
      // MOCK: directly set position
      joints_[i].position = cmd;
    }
  }

  // Interleave step pulses across all active steppers for smooth motion
  {
    int total_steps = 0;
    for (auto & s : steppers_) {
      total_steps += s->remaining_steps();
    }
    if (total_steps > 0) {
      // Calculate inter-step delay to spread pulses over the 10ms cycle
      // Spread steps over the control cycle
      int cycle_us = 10000;  // 100Hz
      int inter_step_us = static_cast<int>(cycle_us / total_steps) - (2 * 3);
      if (inter_step_us < 0) inter_step_us = 0;
      if (inter_step_us > 2000) inter_step_us = 2000;

      bool any_active = true;
      while (any_active) {
        any_active = false;
        for (auto & s : steppers_) {
          if (s->step_once()) {
            any_active = true;
            if (inter_step_us > 0) {
              std::this_thread::sleep_for(std::chrono::microseconds(inter_step_us));
            }
          }
        }
      }
    }
  }

  // Check deviations (watchdog)
  // For coupled joints, compare against the effective (compensated) command
  std::vector<double> actual, commanded;
  for (size_t i = 0; i < joints_.size(); ++i) {
    actual.push_back(joints_[i].position);
    // For joint_3 with coupling, use the compensated target instead of raw command
    if (coupling_enabled_ && joints_[i].name == "joint_3") {
      double compensated = joints_[i].command;
      for (size_t j = 0; j < joints_.size(); ++j) {
        if (joints_[j].name == "joint_2") {
          compensated = compensate_coupling(joints_[i].command, joints_[j].position);
          break;
        }
      }
      commanded.push_back(compensated);
    } else {
      commanded.push_back(joints_[i].command);
    }
  }
  if (safety_.check_all_deviations(actual, commanded)) {
    // Log which joint is deviating for debugging
    for (size_t i = 0; i < actual.size() && i < commanded.size(); ++i) {
      double dev = std::abs(actual[i] - commanded[i]);
      if (dev > 0.01) {
        RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
          "Deviation on %s: actual=%.4f cmd=%.4f dev=%.4f rad (%.1f deg)",
          joints_[i].name.c_str(), actual[i], commanded[i], dev, dev * 180.0 / M_PI);
      }
    }
    RCLCPP_ERROR(rclcpp::get_logger("RobySystem"),
      "CRITICAL: Joint deviation exceeded threshold! Requesting deactivation.");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

double RobySystem::compensate_coupling(
  double joint3_cmd_rad, double joint2_pos_rad) const
{
  if (!coupling_enabled_ || coupling_ratio_m3_ == 0.0) {
    return joint3_cmd_rad;
  }
  // position_moteur3 = position_axe3_cible - (position_axe2 * RATIO_M2 / RATIO_M3)
  return joint3_cmd_rad - (joint2_pos_rad * coupling_ratio_m2_ / coupling_ratio_m3_);
}

}  // namespace roby_hardware

PLUGINLIB_EXPORT_CLASS(roby_hardware::RobySystem, hardware_interface::SystemInterface)
