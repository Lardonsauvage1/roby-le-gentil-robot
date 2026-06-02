#include "roby_hardware/roby_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
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

      // Gains PID closed-loop encodeur (feedback). Defaut 0 => open-loop pur
      // (comportement identique a avant). Voir pid.hpp / BUG-005.
      joints_[i].pid.kp = get_param_double(joint.name + "_pid_kp", 0.0);
      joints_[i].pid.ki = get_param_double(joint.name + "_pid_ki", 0.0);
      joints_[i].pid.kd = get_param_double(joint.name + "_pid_kd", 0.0);
      joints_[i].pid.i_clamp = get_param_double(joint.name + "_pid_i_clamp", 0.0);
      joints_[i].pid.deadband_settled =
        get_param_double(joint.name + "_pid_deadband", 0.0);
      joints_[i].pid.deadband = joints_[i].pid.deadband_settled;  // actif initial
      joints_[i].pid.deadband_ramp =
        get_param_double(joint.name + "_pid_deadband_ramp", 0.0);
      // Deadband deux-phases (precision) : serree pendant le mouvement, large au
      // repos. settle_cycles=0 => desactive (deadband fixe = settled, comme avant).
      joints_[i].pid.deadband_moving =
        get_param_double(joint.name + "_pid_deadband_moving", 0.0);
      joints_[i].pid.settle_cycles =
        get_param_int(joint.name + "_pid_settle_cycles", 0);

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
      joints_[i].servo_offset_deg =
        get_param_double(joint.name + "_servo_offset_deg", cfg.angle_init_deg);

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

  // --- Encoder driver (option B : feedback boucle ouverte -> ferme) -----------
  encoder_enabled_ = get_param_bool("encoder_enabled", false);
  if (encoder_enabled_) {
    EncoderDriver::Config ecfg;
    ecfg.port = get_param("encoder_port", "/dev/ttyAMA0");
    ecfg.baud = get_param_int("encoder_baud", 115200);
    ecfg.de_re_pin = get_param_int("encoder_de_re_pin", 26);
    ecfg.gpio_chip = get_param("encoder_gpio_chip", "/dev/gpiochip4");

    encoder_ = std::make_unique<EncoderDriver>();
    if (!encoder_->init(ecfg)) {
      RCLCPP_ERROR(rclcpp::get_logger("RobySystem"),
        "Failed to init EncoderDriver — falling back to step counter only");
      encoder_.reset();
      encoder_enabled_ = false;
    } else {
      // Enregistre un joint par stepper (1 esclave RS-485 par moteur)
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        if (joints_[i].type != JointType::STEPPER) continue;
        const auto & jname = info_.joints[i].name;
        int slave_id = get_param_int(jname + "_encoder_slave", 0);
        if (slave_id <= 0) continue;  // joint sans encoder mappe
        EncoderDriver::JointSpec js;
        js.joint_idx = static_cast<int>(i);
        js.slave_id = slave_id;
        js.gear_num = get_param_int(jname + "_gear_ratio_num", 1);
        js.gear_den = get_param_int(jname + "_gear_ratio_den", 1);
        js.inverted = get_param_bool(jname + "_encoder_inverted", false);
        js.raw_init_deg = 0.0;
        encoder_->add_joint(js);
      }

      // Couplage axe 2 -> 3 (utilise les ratios deja parses)
      if (coupling_enabled_) {
        // joint_3 += joint_2 * (m2/m3) — trouve les indices par nom
        int j2_idx = -1, j3_idx = -1;
        for (size_t i = 0; i < joints_.size(); ++i) {
          if (joints_[i].name == "joint_2") j2_idx = static_cast<int>(i);
          if (joints_[i].name == "joint_3") j3_idx = static_cast<int>(i);
        }
        if (j2_idx >= 0 && j3_idx >= 0 && coupling_ratio_m3_ > 0) {
          encoder_->set_coupling(j2_idx, j3_idx, coupling_ratio_m2_ / coupling_ratio_m3_);
        }
      }

      // Charge encoder_calibration.yaml depuis le package share
      std::string calib_path;
      try {
        calib_path = ament_index_cpp::get_package_share_directory("roby_hardware")
          + "/config/encoder_calibration.yaml";
      } catch (...) {
        calib_path = "";
      }
      if (calib_path.empty() || !encoder_->load_calibration_yaml(calib_path)) {
        RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
          "encoder_calibration.yaml non charge (%s) — raw_init defaults a 0",
          calib_path.c_str());
      } else {
        RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
          "Encoder calibration chargee depuis %s", calib_path.c_str());
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
    "Initialized with %zu joints (%zu steppers, %zu servos, encoder %s)",
    joints_.size(), steppers_.size(), servos_.size(),
    encoder_enabled_ ? "ENABLED" : "disabled");

  // Recap des joints en closed-loop (gains != 0). Si aucun => open-loop pur.
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].pid.enabled()) {
      RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
        "%s closed-loop PID: kp=%.4f ki=%.4f kd=%.4f i_clamp=%.4f db_settled=%.4f "
        "ramp=%.4f db_moving=%.4f settle=%d",
        joints_[i].name.c_str(), joints_[i].pid.kp, joints_[i].pid.ki,
        joints_[i].pid.kd, joints_[i].pid.i_clamp, joints_[i].pid.deadband_settled,
        joints_[i].pid.deadband_ramp, joints_[i].pid.deadband_moving,
        joints_[i].pid.settle_cycles);
    }
  }
  if (!encoder_enabled_) {
    bool any_pid = false;
    for (auto & j : joints_) any_pid = any_pid || j.pid.enabled();
    if (any_pid) {
      RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
        "Gains PID configures mais encoder DESACTIVE => correction inactive "
        "(feedback impossible sans mesure). Open-loop effectif.");
    }
  }

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
  // Encoder warmup : demarre le thread de polling async, puis attend que les
  // buffers medians soient remplis avant d'exposer la valeur sur state_interface.
  if (encoder_enabled_ && encoder_) {
    encoder_->start_polling_thread();
    // ~250 ms pour remplir le buffer median (N=5) a 56 Hz poll rate
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    // Set joints_[i].position depuis encoder (= vraie position physique)
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (stepper_index_[i] < 0) continue;
      auto pos = encoder_->get_joint_position_rad(static_cast<int>(i));
      if (pos.has_value()) {
        joints_[i].position = pos.value();
      }
    }
    // Aligner le step counter de chaque stepper avec sa position MOTOR-SIDE
    // (apres compensation du couplage pour joint_3). Sinon le 1er write voit
    // un delta enorme entre stepper.current_steps_ (=0 ou stale) et la cmd
    // compensee, et envoie des steps parasites pendant plusieurs cycles.
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (stepper_index_[i] < 0) continue;
      double motor_side_rad = joints_[i].position;
      if (coupling_enabled_ && joints_[i].name == "joint_3") {
        for (size_t j = 0; j < joints_.size(); ++j) {
          if (joints_[j].name == "joint_2") {
            motor_side_rad = compensate_coupling(joints_[i].position, joints_[j].position);
            break;
          }
        }
      }
      if (steppers_[stepper_index_[i]]) {
        steppers_[stepper_index_[i]]->set_position_rad(motor_side_rad);
      }
    }
  }

  // Set commands to current positions (no jump on activation)
  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].command = joints_[i].position;
    joints_[i].prev_position = joints_[i].position;
    // Reset l'etat PID : pas de windup/derivee herites d'une activation
    // precedente (l'integrale doit repartir de zero a la pose courante).
    joints_[i].pid.reset();
  }
  cycles_since_command_ = 0;
  // --- Reglage PID live (tuning) : topic /roby/pid_gains -------------------
  // Noeud + thread d'execution dedie pour pouvoir changer kp/ki/kd/deadband a
  // chaud sans rebuild/relaunch. Message Float64MultiArray :
  //   data = [joint_number, kp, ki, kd, deadband]   (joint_number : 1..5)
  // Ex : ros2 topic pub --once /roby/pid_gains std_msgs/msg/Float64MultiArray \
  //        "{data: [2, 0.2, 0.0, 0.0, 0.02]}"
  if (!tuning_node_) {
    tuning_node_ = std::make_shared<rclcpp::Node>("roby_pid_tuning");
    pid_sub_ = tuning_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/roby/pid_gains", 10,
      std::bind(&RobySystem::on_pid_gains, this, std::placeholders::_1));
    tuning_running_ = true;
    tuning_thread_ = std::thread([this]() {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(tuning_node_);
      while (tuning_running_ && rclcpp::ok()) {
        exec.spin_some(std::chrono::milliseconds(50));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
    RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
      "Reglage PID live actif : topic /roby/pid_gains [joint_n, kp, ki, kd, deadband]");
  }

  RCLCPP_INFO(rclcpp::get_logger("RobySystem"), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void RobySystem::on_pid_gains(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 5) {
    RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
      "/roby/pid_gains : attendu [joint_n, kp, ki, kd, deadband] (5 valeurs, "
      "ou 6 avec deadband_ramp)");
    return;
  }
  int jn = static_cast<int>(msg->data[0]);
  std::string target = "joint_" + std::to_string(jn);
  for (auto & j : joints_) {
    if (j.name == target) {
      j.pid.kp = msg->data[1];
      j.pid.ki = msg->data[2];
      j.pid.kd = msg->data[3];
      j.pid.deadband_settled = msg->data[4];  // deadband au repos (large)
      // 6e = deadband_ramp ; 7e = deadband_moving (serree, deux-phases) ;
      // 8e = settle_cycles (0 => two-phase off).
      if (msg->data.size() >= 6) { j.pid.deadband_ramp = msg->data[5]; }
      if (msg->data.size() >= 7) { j.pid.deadband_moving = msg->data[6]; }
      if (msg->data.size() >= 8) {
        j.pid.settle_cycles = static_cast<int>(msg->data[7]);
      }
      j.pid.reset();  // repart propre (deadband=settled, pas de windup herite)
      RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
        "PID %s LIVE : kp=%.4f ki=%.4f kd=%.4f db_settled=%.4f ramp=%.4f "
        "db_moving=%.4f settle=%d",
        target.c_str(), j.pid.kp, j.pid.ki, j.pid.kd, j.pid.deadband_settled,
        j.pid.deadband_ramp, j.pid.deadband_moving, j.pid.settle_cycles);
      return;
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
    "/roby/pid_gains : joint '%s' introuvable", target.c_str());
}

hardware_interface::CallbackReturn RobySystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop le thread de reglage PID live
  if (tuning_running_) {
    tuning_running_ = false;
    if (tuning_thread_.joinable()) {
      tuning_thread_.join();
    }
    pid_sub_.reset();
    tuning_node_.reset();
  }

  // Shutdown all drivers
  for (auto & s : steppers_) {
    s->shutdown();
  }
  for (auto & s : servos_) {
    s->shutdown();
  }
  if (encoder_) {
    encoder_->shutdown();
    encoder_.reset();
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

  // Encoder : poll est fait par le thread async en background, read() prend
  // juste la derniere valeur filtree via get_joint_position_rad() (non bloquant).

  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].prev_position = joints_[i].position;

    if (joints_[i].type == JointType::STEPPER && stepper_index_[i] >= 0) {
      // En option B : remplace la position step-counter par la lecture
      // encoder (vraie position physique). Fallback step-counter si encoder
      // pas dispo (mode degrade).
      bool used_encoder = false;
      if (encoder_enabled_ && encoder_) {
        auto pos = encoder_->get_joint_position_rad(static_cast<int>(i));
        if (pos.has_value()) {
          joints_[i].position = pos.value();
          used_encoder = true;
        }
      }
      if (!used_encoder) {
        joints_[i].position = steppers_[stepper_index_[i]]->get_position_rad();
      }
    } else if (joints_[i].type == JointType::SERVO && servo_index_[i] >= 0) {
      double angle_deg = servos_[servo_index_[i]]->get_angle_deg();
      joints_[i].position =
        ServoDriver::deg_to_rad(angle_deg - joints_[i].servo_offset_deg);
    }
    // MOCK joints: position = command (set in write)

    joints_[i].velocity = (joints_[i].position - joints_[i].prev_position) / dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobySystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) dt = 0.01;  // fallback 100Hz (coherent avec read())

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

  // Cible EFFECTIVE de joint_2 (commande + correction PID) pour la compensation
  // de couplage de joint_3 ci-dessous. Init a la commande (fallback) ; mise a
  // jour avec la correction PID quand on traite joint_2 (qui precede joint_3).
  double joint_2_effective_cmd = 0.0;
  for (auto & j2 : joints_) {
    if (j2.name == "joint_2") { joint_2_effective_cmd = j2.command; break; }
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    // cmd = feedforward (consigne planifiee, appliquee telle quelle, aucun
    // retard capteur) + correction PID closed-loop encodeur.
    double cmd = joints_[i].command;

    // --- Closed-loop encodeur (feedback) -------------------------------------
    // N'agit que sur les steppers avec gains configures ET encoder actif.
    // error = consigne - position MESUREE (joints_[i].position = encodeur en
    // option B, cf. read()). Le PID ne corrige que l'erreur residuelle lente
    // (pas perdus, derive, gravite) ; le feedforward fait le mouvement rapide.
    // Decouplage feedforward/feedback => l'axe reste rapide malgre la latence
    // encodeur. Voir pid.hpp / BUG-005.
    if (encoder_enabled_ && joints_[i].type == JointType::STEPPER &&
        joints_[i].pid.enabled())
    {
      // Deux-phases : recalcule la deadband active selon que la consigne bouge
      // (deadband serree => precision) ou est stabilisee (large => anti-jitter).
      joints_[i].pid.update_deadband(joints_[i].command);
      double error = joints_[i].command - joints_[i].position;
      cmd += pid_step(joints_[i].pid, error, dt);
    }

    // Memorise la cible effective de joint_2 (commande + correction PID) pour
    // la compensation de couplage de joint_3 (cf. plus bas).
    if (joints_[i].name == "joint_2") {
      joint_2_effective_cmd = cmd;
    }

    // Apply coupling compensation for axis 3 BEFORE safety clamping
    // so the clamp works on the actual motor target, not the raw joint command.
    // IMPORTANT : on utilise joints_[j].command (commande fixe en hold), PAS
    // joints_[j].position. Avec encoder feedback, .position varie en temps reel
    // (manipulations manuelles, micro-mouvements), ce qui ferait osciller la
    // compensation et envoyer des steps parasites a chaque cycle (overrun).
    // La commande de joint_2 est ce que le controller veut atteindre, donc
    // c'est la bonne reference pour pre-compenser motor_3.
    // Couplage : compense motor_3 avec joint_2_effective_cmd (commande + PID),
    // PAS la position encodeur brute. Inclure la correction PID de joint_2 fait
    // suivre motor_3 aux vrais mouvements de motor_2 (closed-loop joint_2) =>
    // joint_3 reste en place => plus de vibration parasite couplee. La deadband
    // de joint_2 garantit correction=0 au repos (pas d injection de bruit).
    if (coupling_enabled_ && joints_[i].name == "joint_3") {
      cmd = compensate_coupling(cmd, joint_2_effective_cmd);
    }

    // Apply safety clamping (on the effective command, after coupling).
    // IMPORTANT : on utilise stepper.get_position_rad() (step counter) au lieu
    // de joints_[i].position (=encoder) comme reference "current". Avec encoder
    // feedback, joints_[i].position varie en temps reel selon la position
    // physique. Si on l'utilise dans clamp_command (qui retourne current+delta
    // limited), cmd se met a tracker la position physique => le stepper envoie
    // des steps pour suivre la derive, c'est un closed-loop implicite non voulu.
    double current_for_clamp = joints_[i].position;
    if (joints_[i].type == JointType::STEPPER && stepper_index_[i] >= 0) {
      current_for_clamp = steppers_[stepper_index_[i]]->get_position_rad();
    }
    cmd = safety_.clamp_command(i, current_for_clamp, cmd);

    // Apply communication watchdog scaling (meme principe : utiliser step counter)
    if (comm_factor < 1.0) {
      double delta = cmd - current_for_clamp;
      cmd = current_for_clamp + delta * comm_factor;
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
      double angle_deg = joints_[i].servo_offset_deg + ServoDriver::rad_to_deg(cmd);
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

  // Check deviations (watchdog) : comparer actual et commanded DANS LE MEME
  // espace.
  //  - encoder ON (option B) : actual = position encodeur = ESPACE-JOINT pour
  //    tous les axes (le driver reconstruit deja l'angle joint_3 couple). Donc
  //    commanded = consigne brute (espace-joint). NE PAS re-compenser joint_3,
  //    sinon on compare joint-space vs motor-space => fausse deviation egale au
  //    terme de couplage (pos_j2 * ratio ~ 18 deg) qui declenche a tort le
  //    safety au demarrage (BUG-005, ancien "18.9 deg sur joint_3").
  //  - encoder OFF : actual = step counter = ESPACE-MOTEUR. La, joint_3 doit
  //    etre compare a la consigne compensee (espace-moteur). Ancien comportement.
  std::vector<double> actual, commanded;
  for (size_t i = 0; i < joints_.size(); ++i) {
    actual.push_back(joints_[i].position);
    if (!encoder_enabled_ && coupling_enabled_ && joints_[i].name == "joint_3") {
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
    critical_deviation_streak_++;
    // Log which joint is deviating for debugging (chaque cycle de la serie)
    for (size_t i = 0; i < actual.size() && i < commanded.size(); ++i) {
      double dev = std::abs(actual[i] - commanded[i]);
      if (dev > 0.01) {
        RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
          "Deviation on %s: actual=%.4f cmd=%.4f dev=%.4f rad (%.1f deg) [streak %d/%d]",
          joints_[i].name.c_str(), actual[i], commanded[i], dev, dev * 180.0 / M_PI,
          critical_deviation_streak_, kCriticalDeviationDebounce);
      }
    }
    // Debounce : ne couper qu apres N cycles consecutifs au-dessus du seuil.
    // Un glitch encodeur d un seul echantillon (burst EMI) retombe au cycle
    // suivant => ignore. Un vrai runaway persiste et grandit => atteint N => coupe.
    if (critical_deviation_streak_ >= kCriticalDeviationDebounce) {
      RCLCPP_ERROR(rclcpp::get_logger("RobySystem"),
        "CRITICAL: Joint deviation exceeded threshold %d cycles consecutifs -> deactivation.",
        critical_deviation_streak_);
      return hardware_interface::return_type::ERROR;
    }
  } else {
    critical_deviation_streak_ = 0;
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
