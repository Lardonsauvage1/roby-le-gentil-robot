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
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "roby_hardware/stepper_driver.hpp"
#include "roby_hardware/servo_driver.hpp"
#include "roby_hardware/safety_monitor.hpp"

namespace roby_hardware
{

enum class JointType
{
  STEPPER,
  SERVO,
  MOCK,
  // Moteur BLDC pilote par une carte externe (axe 5 : B-G431B-ESC1 / SimpleFOC).
  // Le plugin ne parle pas a la carte : il echange consigne/mesure par topics
  // avec le noeud roby_wrist_bldc, seul maitre du port serie (cf ADR-004).
  BLDC
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

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  ~RobySystem() override;

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

  /// Callbacks verrou tete (/head_lock) et pince (/gripper). Ils NE font QUE
  /// poser une cible atomique ; l'ecriture I2C est faite dans write() (thread
  /// RT), seul maitre du bus PCA9685 => pas de collision (cf. servo_driver.cpp).
  void on_head_lock(const std_msgs::msg::Bool::SharedPtr msg);
  void on_gripper(const std_msgs::msg::Bool::SharedPtr msg);
  // Reglage LIVE du serrage : angle brut en degres (topic /roby/gripper_deg).
  void on_gripper_deg(const std_msgs::msg::Float64::SharedPtr msg);

  /// Axe BLDC : etat publie par le noeud roby_wrist_bldc [position, courant, flags].
  void on_bldc_state(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  /// Thread 100 Hz qui publie la derniere consigne BLDC (hors thread RT).
  void bldc_publish_loop();
  // Arrete les fils de fond (publication BLDC, executeur des abonnements) ; idempotent.
  void stop_background_threads();
  /// Mesure BLDC exploitable (liaison OK + recale + fraiche) ? Remplit `pos`.
  bool bldc_feedback(double & pos) const;

  std::vector<JointInfo> joints_;
  std::vector<std::unique_ptr<StepperDriver>> steppers_;
  std::vector<std::unique_ptr<ServoDriver>> servos_;
  bool dry_run_ = false;
  SafetyMonitor safety_;

  // Butees de position lues dans l'URDF (info_.limits), par joint et dans l'ordre
  // de info_.joints. Appliquees en espace ARTICULAIRE dans write(), AVANT la
  // compensation de couplage : apres couplage la valeur est cote MOTEUR et son
  // domaine est decale, une limite d'axe y serait fausse (cf 544c22e / 69fd8a6).
  // Paire {min, max} ; min >= max => axe sans butee declaree, aucun clamp.
  std::vector<std::pair<double, double>> joint_pos_limits_;
  std::vector<bool> limit_warned_;   // avertissement une seule fois par axe (RT)

  // Coupling parameters
  bool coupling_enabled_ = false;
  double coupling_ratio_m2_ = 0.0;  // RATIO_AXE_3_M2
  double coupling_ratio_m3_ = 0.0;  // RATIO_AXE_3_M3

  // Map joint index to stepper/servo index
  std::vector<int> stepper_index_;  // -1 if not a stepper
  std::vector<int> servo_index_;    // -1 if not a servo

  int cycles_since_command_ = 0;

  // Noeud + thread d'execution dedie aux topics hors thread RT : verrou tete,
  // pince, reglage live du serrage et pont BLDC (mesure + consigne).
  rclcpp::Node::SharedPtr topics_node_;
  std::thread topics_thread_;
  std::atomic<bool> topics_running_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_lock_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_deg_sub_;

  // Verrou tete (CH2) + pince (CH3) du changeur d'outil : hors chaine
  // cinematique, pilotes par topic. Instances ServoDriver dediees, ecrites
  // UNIQUEMENT dans write() (thread RT) => un seul maitre du bus I2C.
  std::unique_ptr<ServoDriver> lock_servo_;
  std::unique_ptr<ServoDriver> gripper_servo_;
  static constexpr double kNoServoTarget = -1000.0;  // sentinelle "aucune cible"
  std::atomic<double> lock_target_deg_{kNoServoTarget};
  std::atomic<double> gripper_target_deg_{kNoServoTarget};
  // Rampe de commande servo (anti-saccade de slew : le servo brouttait sur un
  // step brutal 70<->115, surtout en cyclage rapide). On approche la cible de
  // kServoRampDeg par cycle RT (100Hz) => ~0.3s pour 45deg, mouvement lisse.
  static constexpr double kServoRampDeg = 1.5;  // deg/cycle @100Hz
  double lock_cmd_deg_ = 50.0;      // angle commande courant du verrou (rampe)
  double gripper_cmd_deg_ = 120.0;  // angle commande courant de la pince (rampe)
  double lock_locked_deg_ = 50.0;    // 2026-07-07 : 50=verrouille (inverse ancienne calib)
  double lock_unlocked_deg_ = 75.0;  // 75=deverrouille
  double gripper_open_deg_ = 120.0;
  double gripper_closed_deg_ = 55.0;

  // --- Axe BLDC (joint_N_type = bldc) : pont par topics vers roby_wrist_bldc ---
  // write() (thread RT) ne fait que poser la consigne dans un atomique ; un
  // thread dedie la publie a 100 Hz. La mesure arrive par callback (thread des
  // topics) dans des atomiques lus par read(). Aucun E/S serie ni DDS dans le
  // thread RT. Un seul joint BLDC supporte.
  int bldc_joint_ = -1;  // index dans joints_, -1 = aucun
  std::string bldc_command_topic_ = "/roby/wrist_bldc/command";
  std::string bldc_state_topic_ = "/roby/wrist_bldc/state";
  double bldc_state_timeout_s_ = 0.2;
  double bldc_wait_on_activate_s_ = 8.0;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bldc_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr bldc_state_sub_;
  std::thread bldc_pub_thread_;
  std::atomic<bool> bldc_pub_running_{false};
  std::atomic<double> bldc_cmd_{0.0};
  std::atomic<bool> bldc_cmd_valid_{false};
  std::atomic<double> bldc_meas_pos_{0.0};
  std::atomic<int> bldc_meas_flags_{0};
  std::atomic<int64_t> bldc_meas_stamp_ns_{0};  // steady_clock
  // Derniere consigne envoyee : reference "courante" du clamp de vitesse (comme
  // le compteur de pas des steppers), pour ne pas boucler sur la mesure bruitee.
  double bldc_last_cmd_ = 0.0;
  bool bldc_feedback_ok_ = false;  // pour ne journaliser que les transitions
  // Bits de flags publies par le noeud (cf roby_wrist_bldc/node.py).
  static constexpr int kBldcLinkOk = 1;
  static constexpr int kBldcHomed = 2;
};

}  // namespace roby_hardware

#endif  // ROBY_HARDWARE__ROBY_SYSTEM_HPP_
