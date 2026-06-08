#ifndef ROBY_HARDWARE__ROBY_SYSTEM_HPP_
#define ROBY_HARDWARE__ROBY_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "roby_hardware/stepper_driver.hpp"
#include "roby_hardware/servo_driver.hpp"
#include "roby_hardware/safety_monitor.hpp"
#include "roby_hardware/encoder_driver.hpp"
#include "roby_hardware/pid.hpp"

namespace roby_hardware
{

enum class JointType
{
  STEPPER,
  SERVO,
  MOCK
};

struct JointInfo
{
  std::string name;
  JointType type = JointType::MOCK;
  double position = 0.0;
  double velocity = 0.0;
  double command = 0.0;
  double prev_position = 0.0;
  double servo_offset_deg = 0.0;  // centre servo (0 rad joint = cet angle)
  // Closed-loop encodeur (feedback) en complement du feedforward (command).
  // Gains a 0 par defaut => correction nulle => open-loop. Voir pid.hpp / BUG-005.
  PidState pid;
};

class RobySystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobySystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Parse a hardware parameter, returning default if not found.
  std::string get_param(const std::string & name, const std::string & default_val = "") const;
  int get_param_int(const std::string & name, int default_val = 0) const;
  double get_param_double(const std::string & name, double default_val = 0.0) const;
  bool get_param_bool(const std::string & name, bool default_val = false) const;

  /// Apply coupling compensation for axes 2/3.
  double compensate_coupling(double joint3_cmd_rad, double joint2_pos_rad) const;

  /// Callback de reglage PID live (topic /roby/pid_gains).
  /// Message Float64MultiArray : [joint_number, kp, ki, kd, deadband].
  void on_pid_gains(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::vector<JointInfo> joints_;
  std::vector<std::unique_ptr<StepperDriver>> steppers_;
  std::vector<std::unique_ptr<ServoDriver>> servos_;
  SafetyMonitor safety_;

  // Coupling parameters
  bool coupling_enabled_ = false;
  double coupling_ratio_m2_ = 0.0;  // RATIO_AXE_3_M2
  double coupling_ratio_m3_ = 0.0;  // RATIO_AXE_3_M3

  // Map joint index to stepper/servo index
  std::vector<int> stepper_index_;  // -1 if not a stepper
  std::vector<int> servo_index_;    // -1 if not a servo

  int cycles_since_command_ = 0;

  // Watchdog deviation : nb de cycles consecutifs ou la deviation depasse le
  // seuil critique. Desactivation seulement apres kCriticalDeviationDebounce
  // cycles (debounce) => un glitch encodeur d un seul echantillon (burst EMI)
  // est ignore (le compteur retombe a 0), un vrai runaway persiste et coupe.
  int critical_deviation_streak_ = 0;
  static constexpr int kCriticalDeviationDebounce = 8;

  // Encoder feedback (option B : state_interface "position" = encoder reading)
  // Active via param `encoder_enabled` (default false). Quand actif, la position
  // publiee sur /joint_states refletera la vraie position physique mesuree, et
  // non plus le compteur de steps open-loop.
  bool encoder_enabled_ = false;
  std::unique_ptr<EncoderDriver> encoder_;

  // Reglage PID live (tuning) : noeud + thread d'execution dedie, ecoute
  // /roby/pid_gains pour changer kp/ki/kd/deadband a chaud sans relancer.
  rclcpp::Node::SharedPtr tuning_node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_sub_;
  std::thread tuning_thread_;
  std::atomic<bool> tuning_running_{false};

  // --- Partie B : recalage one-shot au settle (joint_2/3 open-loop) ---------
  // A l'arret (consigne stable + axes immobiles), grosse mediane des lectures
  // encodeur (robuste au bruit) -> recale le compteur de pas dessus -> le
  // feedforward comble l'ecart, puis stop. Max kSettleMaxCorrections / mouvement.
  void settle_recalibrate();
  std::vector<double> prev_commands_;
  std::vector<double> prev_step_pos_;
  std::vector<std::vector<double>> settle_samples_;
  int settle_counter_ = 0;
  int settle_phase_ = 0;
  int settle_correction_count_ = 0;
  static constexpr int kSettleWaitCycles = 25;
  static constexpr int kSettleCollectN = 60;
  static constexpr double kSettleStepEps = 5e-5;
  static constexpr double kSettleDeadbandRad = 0.0087;   // ~0.5 deg
  static constexpr double kSettleMaxCorrRad = 0.35;      // ~20 deg : au-dela = aberrant
  static constexpr int kSettleMaxCorrections = 2;
};

}  // namespace roby_hardware

#endif  // ROBY_HARDWARE__ROBY_SYSTEM_HPP_
