#include "roby_hardware/roby_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>
#include <algorithm>
#include <cstdlib>

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

  // Dry-run : si ROBY_DRY_RUN defini, AUCUNE impulsion GPIO (test sans moteurs).
  dry_run_ = (std::getenv("ROBY_DRY_RUN") != nullptr);
  if (dry_run_) RCLCPP_WARN(rclcpp::get_logger("RobySystem"), "*** DRY-RUN actif : aucune impulsion GPIO ***");

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

  // Butees URDF : info_.limits est peuple par ros2_control depuis <limit lower/upper>.
  // Elles sont appliquees en espace ARTICULAIRE dans write() (avant couplage).
  joint_pos_limits_.assign(info_.joints.size(), {1.0, -1.0});   // {min>max} = pas de butee
  limit_warned_.assign(info_.joints.size(), false);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    auto it = info_.limits.find(info_.joints[i].name);
    if (it != info_.limits.end() && it->second.has_position_limits) {
      joint_pos_limits_[i] = {it->second.min_position, it->second.max_position};
      RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
        "%s : butees URDF [%.4f, %.4f] rad (clamp en espace articulaire)",
        info_.joints[i].name.c_str(), it->second.min_position, it->second.max_position);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
        "%s : aucune butee de position dans l'URDF -> AUCUN clamp pour cet axe.",
        info_.joints[i].name.c_str());
    }
  }

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

      stepper->set_dry_run(dry_run_);
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

    } else if (type_str == "bldc") {
      // Axe BLDC via le noeud roby_wrist_bldc (topics). La carte fait l'asservissement,
      // le noeud la rampe et la securite ; ici on ne fait que relayer.
      if (bldc_joint_ >= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("RobySystem"),
          "%s : un seul joint bldc supporte (deja %s)", joint.name.c_str(),
          joints_[bldc_joint_].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      joints_[i].type = JointType::BLDC;
      bldc_joint_ = static_cast<int>(i);
      bldc_command_topic_ = get_param(joint.name + "_bldc_command_topic", bldc_command_topic_);
      bldc_state_topic_ = get_param(joint.name + "_bldc_state_topic", bldc_state_topic_);
      bldc_state_timeout_s_ = get_param_double(joint.name + "_bldc_state_timeout_s", 0.2);
      bldc_wait_on_activate_s_ = get_param_double(joint.name + "_bldc_wait_on_activate_s", 16.0);
      bldc_last_cmd_ = joints_[i].position;
      RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
        "%s : BLDC externe, consigne -> %s, mesure <- %s", joint.name.c_str(),
        bldc_command_topic_.c_str(), bldc_state_topic_.c_str());

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

    // Set velocity and deviation thresholds based on joint type.
    // max_accel_rad_per_tick2 limite la variation de vitesse par cycle => un
    // rattrapage apres overrun RT rampe au lieu de sauter (anti "petit saut").
    // ~1/10 de la vitesse max => atteint la vitesse max en ~10 cycles (100ms).
    if (joints_[i].type == JointType::STEPPER) {
      sc.max_velocity_rad_per_tick = 3.0 * DEG_TO_RAD;   // 3°/tick
      sc.max_accel_rad_per_tick2 = 0.3 * DEG_TO_RAD;      // 0.3°/tick²
      sc.warning_deviation_rad = 5.0 * DEG_TO_RAD;        // 5°
      sc.critical_deviation_rad = 15.0 * DEG_TO_RAD;      // 15°
    } else {
      sc.max_velocity_rad_per_tick = 8.0 * DEG_TO_RAD;   // 8°/tick
      sc.max_accel_rad_per_tick2 = 0.8 * DEG_TO_RAD;      // 0.8°/tick²
      sc.warning_deviation_rad = 8.0 * DEG_TO_RAD;        // 8°
      sc.critical_deviation_rad = 20.0 * DEG_TO_RAD;      // 20°
    }

    safety_configs.push_back(sc);
  }

  safety_.init(safety_configs);

  RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
    "Initialized with %zu joints (%zu steppers, %zu servos, %d bldc)",
    joints_.size(), steppers_.size(), servos_.size(), bldc_joint_ >= 0 ? 1 : 0);

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
  // Aligner le step counter de chaque stepper avec sa position MOTOR-SIDE
  // (apres compensation du couplage pour joint_3). Sans cet alignement, une
  // pose de depart non nulle laisserait le compteur joint_3 non-couple alors
  // que write() commande couple => mismatch = terme de couplage => runaway au
  // demarrage (slew violent). Fix 2026-06-27 (diagnostic dry_run).
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

  // Set commands to current positions (no jump on activation)
  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].command = joints_[i].position;
    joints_[i].prev_position = joints_[i].position;
  }
  cycles_since_command_ = 0;

  // --- Topics hors thread RT : noeud + thread d'execution dedie -------------
  if (!topics_node_) {
    topics_node_ = std::make_shared<rclcpp::Node>("roby_hardware_topics");
    // Verrou tete (/head_lock) + pince (/gripper). Les callbacks posent une cible ; write() (thread RT) ecrit le bus.
    head_lock_sub_ = topics_node_->create_subscription<std_msgs::msg::Bool>(
      "/head_lock", 10,
      std::bind(&RobySystem::on_head_lock, this, std::placeholders::_1));
    gripper_sub_ = topics_node_->create_subscription<std_msgs::msg::Bool>(
      "/gripper", 10,
      std::bind(&RobySystem::on_gripper, this, std::placeholders::_1));
    // Angle BRUT de la pince, pour regler le serrage a chaud sans rebuild :
    //   ros2 topic pub --once /roby/gripper_deg std_msgs/msg/Float64 "{data: 76.0}"
    gripper_deg_sub_ = topics_node_->create_subscription<std_msgs::msg::Float64>(
      "/roby/gripper_deg", 10,
      std::bind(&RobySystem::on_gripper_deg, this, std::placeholders::_1));
    // Instancie les servos verrou/pince (hors chaine cinematique). Re-init a
    // chaque activation. angle_init : verrou=verrouille, pince=ouverte (etats surs).
    // Calibre 2026-07-07 : 50 deg = VERROUILLE, 75 deg = DEVERROUILLE (inverse
    // de l'ancienne note 2026-06-02 ; palonnier remonte). Surchargeable par param.
    lock_locked_deg_    = get_param_double("head_lock_locked_deg", 50.0);
    lock_unlocked_deg_  = get_param_double("head_lock_unlocked_deg", 75.0);
    gripper_open_deg_   = get_param_double("gripper_open_deg", 120.0);
    gripper_closed_deg_ = get_param_double("gripper_closed_deg", 55.0);
    {
      std::string i2c_bus = get_param("i2c_bus", "/dev/i2c-1");
      int pca_addr = get_param_int("pca9685_address", 0x40);
#ifdef __linux__
      bool i2c_mock = (access(i2c_bus.c_str(), F_OK) != 0);
#else
      bool i2c_mock = true;
#endif
      ServoConfig lc;
      lc.i2c_bus = i2c_bus; lc.pca9685_address = pca_addr;
      lc.channel = get_param_int("head_lock_channel", 2);
      lc.angle_min_deg = 0.0; lc.angle_max_deg = 180.0;
      lc.angle_init_deg = lock_locked_deg_; lc.inverted = false; lc.mock = i2c_mock;
      lock_servo_ = std::make_unique<ServoDriver>();
      lock_servo_->init(lc);
      ServoConfig gc;
      gc.i2c_bus = i2c_bus; gc.pca9685_address = pca_addr;
      gc.channel = get_param_int("gripper_channel", 3);
      gc.angle_min_deg = 0.0; gc.angle_max_deg = 180.0;
      gc.angle_init_deg = gripper_open_deg_; gc.inverted = false; gc.mock = i2c_mock;
      gripper_servo_ = std::make_unique<ServoDriver>();
      gripper_servo_->init(gc);
    }
    // La rampe part de l'etat init reel des servos (verrou verrouille, pince
    // ouverte) : le 1er mouvement commande sera lisse depuis la vraie position.
    lock_cmd_deg_ = lock_locked_deg_;
    gripper_cmd_deg_ = gripper_open_deg_;
    lock_target_deg_.store(kNoServoTarget);
    gripper_target_deg_.store(kNoServoTarget);
    // Axe BLDC : mesure recue sur le meme noeud/thread.
    if (bldc_joint_ >= 0) {
      bldc_meas_flags_.store(0);
      bldc_state_sub_ = topics_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        bldc_state_topic_, 10,
        std::bind(&RobySystem::on_bldc_state, this, std::placeholders::_1));
      bldc_cmd_pub_ = topics_node_->create_publisher<std_msgs::msg::Float64>(
        bldc_command_topic_, 10);
    }
    topics_running_ = true;
    topics_thread_ = std::thread([this]() {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(topics_node_);
      while (topics_running_ && rclcpp::ok()) {
        exec.spin_some(std::chrono::milliseconds(50));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }

  // --- Axe BLDC : partir de la position MESUREE (aucun saut a l'activation) ---
  // Le noeud roby_wrist_bldc recale la carte au nid a son lancement ; on attend
  // sa premiere mesure valide (liaison + recale). A defaut, on part de
  // initial_value (= pose du nid) et write() restera en open-loop jusqu'a ce que
  // la mesure arrive (voir read()).
  if (bldc_joint_ >= 0) {
    auto & bj = joints_[bldc_joint_];
    double meas = 0.0;
    auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(bldc_wait_on_activate_s_);
    while (!bldc_feedback(meas) && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    bldc_feedback_ok_ = bldc_feedback(meas);
    if (bldc_feedback_ok_) {
      bj.position = meas;
      RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
        "%s : mesure BLDC recue, depart a %.4f rad", bj.name.c_str(), meas);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
        "%s : AUCUNE mesure BLDC valide en %.1f s (noeud wrist_bldc lance ? carte "
        "branchee ? recalee ?) -> depart a initial_value %.4f, open-loop en attendant",
        bj.name.c_str(), bldc_wait_on_activate_s_, bj.position);
    }
    bj.command = bj.position;
    bj.prev_position = bj.position;
    bldc_last_cmd_ = bj.command;
    bldc_cmd_.store(bj.command);
    bldc_cmd_valid_.store(!dry_run_);  // dry-run : aucune consigne vers le moteur
    bldc_pub_running_ = true;
    bldc_pub_thread_ = std::thread(&RobySystem::bldc_publish_loop, this);
  }

  RCLCPP_INFO(rclcpp::get_logger("RobySystem"), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void RobySystem::on_bldc_state(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // [position, courant, flags] (roby_wrist_bldc/node.py). Position avant flags :
  // read() ne lit la position que si les flags la declarent valide.
  if (msg->data.size() < 3) {
    return;
  }
  bldc_meas_pos_.store(msg->data[0]);
  bldc_meas_flags_.store(static_cast<int>(msg->data[2]));
  bldc_meas_stamp_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool RobySystem::bldc_feedback(double & pos) const
{
  const int flags = bldc_meas_flags_.load();
  if ((flags & kBldcLinkOk) == 0 || (flags & kBldcHomed) == 0) {
    return false;  // pas de liaison, ou position dans un repere non recale
  }
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const double age_s = static_cast<double>(now_ns - bldc_meas_stamp_ns_.load()) * 1e-9;
  if (age_s > bldc_state_timeout_s_) {
    return false;  // noeud mort ou bloque
  }
  const double p = bldc_meas_pos_.load();
  if (!std::isfinite(p)) {
    return false;
  }
  pos = p;
  return true;
}

void RobySystem::bldc_publish_loop()
{
  // 100 Hz, hors thread RT : write() ne fait que poser bldc_cmd_.
  const auto period = std::chrono::milliseconds(10);
  auto next = std::chrono::steady_clock::now();
  std_msgs::msg::Float64 msg;
  while (bldc_pub_running_) {
    next += period;
    std::this_thread::sleep_until(next);
    const auto now = std::chrono::steady_clock::now();
    if (now - next > 5 * period) {
      next = now;  // gros retard : pas de rafale de rattrapage
    }
    if (bldc_cmd_valid_.load() && bldc_cmd_pub_) {
      msg.data = bldc_cmd_.load();
      bldc_cmd_pub_->publish(msg);
    }
  }
}

void RobySystem::on_head_lock(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Ne fait QUE poser la cible ; l'ecriture I2C est faite par write() (thread RT),
  // seul maitre du bus PCA9685 => jamais de collision.
  lock_target_deg_.store(msg->data ? lock_locked_deg_ : lock_unlocked_deg_);
  RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
    "/head_lock -> %s (%.1f deg)", msg->data ? "VERROUILLE" : "DEVERROUILLE",
    msg->data ? lock_locked_deg_ : lock_unlocked_deg_);
}

void RobySystem::on_gripper(const std_msgs::msg::Bool::SharedPtr msg)
{
  // data=true => pince FERMEE (cf. roby_fine_jog / roby_tool_pickup). Idem : la
  // cible est posee ici, l'ecriture bus se fait dans write().
  gripper_target_deg_.store(msg->data ? gripper_closed_deg_ : gripper_open_deg_);
  RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
    "/gripper -> %s (%.1f deg)", msg->data ? "FERME" : "OUVRE",
    msg->data ? gripper_closed_deg_ : gripper_open_deg_);
}

void RobySystem::on_gripper_deg(const std_msgs::msg::Float64::SharedPtr msg)
{
  // Commande un angle BRUT (reglage du serrage). Borne dur : hors de cette plage
  // le servo force en butee mecanique (provisoire, sous-dimensionne).
  const double kMin = 40.0, kMax = 130.0;
  double a = msg->data;
  if (a < kMin || a > kMax) {
    RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
      "/roby/gripper_deg %.1f REFUSE (hors [%.0f,%.0f])", a, kMin, kMax);
    return;
  }
  gripper_target_deg_.store(a);
  RCLCPP_INFO(rclcpp::get_logger("RobySystem"), "/roby/gripper_deg -> %.1f deg", a);
}

void RobySystem::stop_background_threads()
{
  // Idempotent, et joint un fil meme s'il s'est termine seul : un std::thread encore
  // joignable a sa destruction appelle std::terminate.

  // Axe BLDC : plus de consigne publiee => le noeud wrist_bldc detecte le
  // silence et s'arrete sur rampe ; la carte tient la position (moteur asservi).
  bldc_pub_running_ = false;
  if (bldc_pub_thread_.joinable()) {
    bldc_pub_thread_.join();
  }
  bldc_cmd_valid_.store(false);

  // Stop le thread d'execution des abonnements
  topics_running_ = false;
  if (topics_thread_.joinable()) {
    topics_thread_.join();
  }
  head_lock_sub_.reset();
  gripper_sub_.reset();
  gripper_deg_sub_.reset();
  bldc_state_sub_.reset();
  bldc_cmd_pub_.reset();
  topics_node_.reset();
}

// Chemin d'ERREUR (watchdog d'ecart : write() rend ERROR) : le composant part en
// FINALIZED sans passer par on_deactivate. Avant le 2026-09-13, le fil BLDC continuait
// alors de publier la derniere consigne a 100 Hz -- le noeud wrist_bldc ne voyait jamais
// le silence qui l'arrete sur rampe -- et les fils encore joignables provoquaient
// std::terminate a la sortie du processus. On arrete les FILS ; les pilotes des moteurs
// ne sont PAS coupes ici (couper les steppers pourrait laisser tomber le bras).
hardware_interface::CallbackReturn RobySystem::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  stop_background_threads();
  return hardware_interface::SystemInterface::on_error(previous_state);
}

hardware_interface::CallbackReturn RobySystem::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  stop_background_threads();
  return hardware_interface::SystemInterface::on_shutdown(previous_state);
}

RobySystem::~RobySystem()
{
  stop_background_threads();
}

hardware_interface::CallbackReturn RobySystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_background_threads();

  // Shutdown all drivers
  for (auto & s : steppers_) {
    s->shutdown();
  }
  for (auto & s : servos_) {
    s->shutdown();
  }
  if (lock_servo_) lock_servo_->shutdown();
  if (gripper_servo_) gripper_servo_->shutdown();

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
      // Position = compteur de pas (open-loop : aucun retour capteur vers le Pi ;
      // les CL86Y des axes 2/3 bouclent en interne sur leur propre codeur).
      double raw = steppers_[stepper_index_[i]]->get_position_rad();
      // Le compteur de pas de joint_3 est en repere MOTEUR (write() a applique
      // la compensation couplage). On refait l'inverse pour que /joint_states
      // donne l'angle AXE reel.
      if (coupling_enabled_ && joints_[i].name == "joint_3") {
        for (size_t j = 0; j < joints_.size(); ++j) {
          if (joints_[j].name == "joint_2") {
            raw += joints_[j].position * (coupling_ratio_m2_ / coupling_ratio_m3_);
            break;
          }
        }
      }
      joints_[i].position = raw;
    } else if (joints_[i].type == JointType::SERVO && servo_index_[i] >= 0) {
      double angle_deg = servos_[servo_index_[i]]->get_angle_deg();
      joints_[i].position =
        ServoDriver::deg_to_rad(angle_deg - joints_[i].servo_offset_deg);
    } else if (joints_[i].type == JointType::BLDC) {
      // Mesure reelle de la carte (codeur AS5600, repere recale au nid). Sans
      // mesure valide : position = derniere consigne (posee par write()), comme
      // un joint mock, et on le signale une fois par transition.
      double meas = 0.0;
      const bool ok = bldc_feedback(meas);
      if (ok) {
        joints_[i].position = meas;
      }
      if (ok != bldc_feedback_ok_) {
        bldc_feedback_ok_ = ok;
        if (ok) {
          RCLCPP_INFO(rclcpp::get_logger("RobySystem"),
            "%s : mesure BLDC retablie (%.4f rad)", joints_[i].name.c_str(), meas);
        } else {
          RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
            "%s : mesure BLDC PERDUE (noeud, liaison ou recalage) -> position = consigne",
            joints_[i].name.c_str());
        }
      }
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
    // BLDC : comparer a la derniere consigne envoyee, pas a la mesure (toujours
    // un peu differente => le watchdog se croirait sans cesse sollicite).
    const double ref = (joints_[i].type == JointType::BLDC) ? bldc_last_cmd_ : joints_[i].position;
    if (std::abs(joints_[i].command - ref) > 1e-6) {
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

  // Consigne de joint_2 : reference de la compensation de couplage de joint_3.
  double joint_2_effective_cmd = 0.0;
  for (auto & j2 : joints_) {
    if (j2.name == "joint_2") { joint_2_effective_cmd = j2.command; break; }
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    double cmd = joints_[i].command;

    // Couplage : motor_3 est pre-compense avec la COMMANDE de joint_2 (fixe en
    // hold), cote moteur, apres le clamp des butees ci-dessous.

    // --- Butees URDF : clamp en espace ARTICULAIRE, AVANT le couplage (2026-07-20)
    //
    // C'est ICI que la limite a un sens : cmd est encore une position d'AXE.
    // Apres compensate_coupling ce sera une position de MOTEUR, dont le domaine
    // est decale par le couplage 2/3 (rapport 0.625) -- y appliquer une limite
    // d'axe epinglait joint_3 a sa butee et le rendait totalement inerte
    // (regression 544c22e, constatee moteurs desolidarises, annulee en 69fd8a6).
    //
    // Le clamp de safety_.clamp_command() reste applique plus bas SUR LA VALEUR
    // MOTEUR : il garde son role de limitation de VITESSE, et ses bornes de
    // position (+/-PI) sont assez larges pour ne pas gener le domaine couple.
    if (i < joint_pos_limits_.size() && joint_pos_limits_[i].first < joint_pos_limits_[i].second) {
      const double lo = joint_pos_limits_[i].first, hi = joint_pos_limits_[i].second;
      if (cmd < lo || cmd > hi) {
        if (!limit_warned_[i]) {          // une seule fois par axe : pas de spam RT
          RCLCPP_WARN(rclcpp::get_logger("RobySystem"),
            "%s : consigne %.4f hors butee URDF [%.4f, %.4f] -> clampee",
            joints_[i].name.c_str(), cmd, lo, hi);
          limit_warned_[i] = true;
        }
        cmd = std::clamp(cmd, lo, hi);
      }
    }

    if (coupling_enabled_ && joints_[i].name == "joint_3") {
      cmd = compensate_coupling(cmd, joint_2_effective_cmd);
    }

    // Apply safety clamping (on the effective command, after coupling).
    // Reference "current" = compteur de pas, en repere MOTEUR comme cmd (apres
    // couplage) ; joints_[i].position est en repere AXE (cf. read()).
    double current_for_clamp = joints_[i].position;
    if (joints_[i].type == JointType::STEPPER && stepper_index_[i] >= 0) {
      current_for_clamp = steppers_[stepper_index_[i]]->get_position_rad();
    } else if (joints_[i].type == JointType::BLDC) {
      current_for_clamp = bldc_last_cmd_;  // meme raison : pas de boucle sur la mesure
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
      if (!dry_run_) servos_[servo_index_[i]]->set_angle_deg(angle_deg);

    } else if (joints_[i].type == JointType::BLDC) {
      // Pose la consigne ; bldc_publish_loop() la publie (hors thread RT). Le
      // noeud wrist_bldc applique rampe, butees et securite avant la carte.
      bldc_last_cmd_ = cmd;
      if (!dry_run_) bldc_cmd_.store(cmd);
      if (!bldc_feedback_ok_) {
        joints_[i].position = cmd;  // pas de mesure : open-loop comme un mock
      }

    } else {
      // MOCK: directly set position
      joints_[i].position = cmd;
    }
  }

  // Verrou tete (CH2) + pince (CH3) : applique la cible posee par les callbacks
  // /head_lock /gripper. Ecrit ICI (thread RT) => SEUL maitre du bus I2C, jamais
  // depuis le callback => plus de collision (cf. incident servo_driver.cpp).
  // set_angle_deg est write-on-change : aucun trafic bus tant que la cible ne
  // change pas (donc aucun surcout RT au repos).
  // Rampe douce vers la cible (kServoRampDeg/cycle) : evite l'a-coup/bourdonnement
  // du servo sur un step brutal, surtout en cyclage rapide (ouvre/ferme rapproches).
  // Ecritures I2C seulement pendant la rampe (write-on-change), puis 0 au repos.
  if (!dry_run_) {
    double lt = lock_target_deg_.load();
    if (lt != kNoServoTarget && lock_servo_) {
      lock_cmd_deg_ += std::clamp(lt - lock_cmd_deg_, -kServoRampDeg, kServoRampDeg);
      lock_servo_->set_angle_deg(lock_cmd_deg_);
    }
    double gt = gripper_target_deg_.load();
    if (gt != kNoServoTarget && gripper_servo_) {
      gripper_cmd_deg_ += std::clamp(gt - gripper_cmd_deg_, -kServoRampDeg, kServoRampDeg);
      gripper_servo_->set_angle_deg(gripper_cmd_deg_);
    }
  }

  // Pulse groupe (anti-overrun multi-axes, 2026-06-26) : a chaque passe on leve
  // ENSEMBLE les lignes STEP des moteurs ayant un pas en attente, UN busy-wait
  // partage (largeur d impulsion), puis on baisse + commit ensemble. Supprime le
  // 3x de spin CPU vs pulser chaque moteur separement (cf project_gpio_overrun).
  {
    int max_steps_pass = 0;
    for (auto & s : steppers_) {
      int r = s->remaining_steps();
      if (r > max_steps_pass) max_steps_pass = r;
    }
    if (max_steps_pass > 0) {
      int cycle_us = 6000;   // etale sur ~6ms, marge sous le cycle 10ms
      int inter_step_us = static_cast<int>(cycle_us / max_steps_pass) - (2 * 3);
      if (inter_step_us < 0) inter_step_us = 0;
      if (inter_step_us > 2000) inter_step_us = 2000;

      for (int pass = 0; pass < max_steps_pass; ++pass) {
        bool any = false;
        for (auto & s : steppers_) {
          if (s->has_pending_step()) { s->raise_step(); any = true; }
        }
        if (!any) break;
        // largeur d impulsion HAUT, partagee entre tous les moteurs (busy-wait)
        { auto e = std::chrono::steady_clock::now() + std::chrono::microseconds(3);
          while (std::chrono::steady_clock::now() < e) { } }
        for (auto & s : steppers_) {
          if (s->has_pending_step()) s->lower_step_and_commit();
        }
        // largeur d impulsion BAS partagee + espacement inter-pas (busy-wait)
        { auto e = std::chrono::steady_clock::now() + std::chrono::microseconds(3 + inter_step_us);
          while (std::chrono::steady_clock::now() < e) { } }
      }
    }
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
