#ifndef ROBY_HARDWARE__ENCODER_DRIVER_HPP_
#define ROBY_HARDWARE__ENCODER_DRIVER_HPP_

#include <atomic>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#ifdef HAS_GPIOD
#include <gpiod.h>
#endif

#include "roby_hardware/outlier_filter.hpp"

namespace roby_hardware
{

/// Driver de lecture des encodeurs AS5048A via le bus RS-485 (UART0 sur Pi5).
///
/// Architecture :
///   Pi5 maitre (UART0 + GPIO26 pour DE/RE du MAX485) <-> bus RS-485
///   <-> N Arduino esclaves (un par moteur) lisant chacun un AS5048A en PWM.
///
/// Protocole :
///   Maitre ecrit 1 byte (ID esclave).
///   Esclave repond 4 bytes : 0xFF + ID + raw_angle 16-bit big-endian.
///   raw_angle = 0xFFFE  =>  pas de mesure (pulseIn timeout cote Arduino).
///
/// Conversion raw -> joint angle :
///   1. raw[deg] = (raw_value / 65535) * 360
///   2. delta_raw = wrap_to_180(raw_now - raw_init)  (au boot)
///   3. unwrap incremental : on accumule wrap_to_180(raw_now - raw_prev) entre
///      reads consecutifs, en rejetant les sauts > MAX_STEP_DEG (outliers
///      d'acquisition pulseIn cote Arduino, ~2-8% selon capteur). PORTE DE
///      SORTIE : apres max_consec_rejects rejets consecutifs, on accepte quand
///      meme (re-synchro) — sinon le filtre se bloque sur une valeur perimee
///      lors d'un vrai mouvement rapide (lock-up observe 2026-05-31).
///   4. filtre median glissant (N=5) sur unwrapped pour absorber le bruit
///      residuel.
///   5. joint_angle[rad] = filtered_unwrapped[rad] * gear_num/gear_den
///      * (inverted ? -1 : +1).
///   6. couplage 2->3 : joint_3 += joint_2 * coupling_j2_to_j3.
class EncoderDriver
{
public:
  struct Config
  {
    std::string port = "/dev/ttyAMA0";
    int baud = 115200;
    int de_re_pin = 26;
    std::string gpio_chip = "/dev/gpiochip4";
    int query_timeout_us = 30000;        ///< 30 ms par query
    size_t median_filter_size = 5;       ///< taille du buffer median glissant
    double max_step_deg = 20.0;          ///< seuil rejet outlier (deg / cycle)
    int max_consec_rejects = 5;          ///< porte de sortie : accepte apres N rejets
    bool mock = false;                   ///< si true, RS-485 mocke (pour tests)
  };

  /// Description d'un joint piloté par un encodeur sur le bus.
  struct JointSpec
  {
    int joint_idx;       ///< Indice du joint cote RobySystem (0-based)
    int slave_id;        ///< ID de l'esclave RS-485 (1, 2, 3, ...)
    int gear_num;        ///< Numerateur du ratio motor->joint
    int gear_den;        ///< Denominateur (gear_den > gear_num pour reduction)
    bool inverted;       ///< Signe encodeur vs convention URDF
    double raw_init_deg; ///< Angle brut AS5048A a la pose URDF zero (calibration)
  };

  EncoderDriver() = default;
  ~EncoderDriver();

  /// Ouvre le port serie + acquiert la ligne GPIO DE/RE. Retourne false si echec.
  bool init(const Config & cfg);

  /// Liberation des ressources (serie + GPIO).
  void shutdown();

  /// Enregistre un joint a lire. A appeler apres init(), avant le 1er poll().
  void add_joint(const JointSpec & spec);

  /// Configure le couplage mecanique : joint_to += joint_from * ratio.
  /// Pour Roby : joint_3 += joint_2 * (m2/m3) ~= 0.6248
  void set_coupling(int joint_from, int joint_to, double ratio);

  /// (No-op si polling async actif.) Force un poll synchrone de tous les
  /// esclaves. Utilise pour le warmup ou en mode mock/test.
  void poll_all();

  /// Demarre le thread de polling async. Tourne en background, query les
  /// esclaves a sa propre vitesse (~56 Hz). Doit etre appele apres add_joint
  /// pour tous les joints, et avant le 1er get_joint_position_rad.
  void start_polling_thread();

  /// Stoppe et join le thread. Idempotent.
  void stop_polling_thread();

  /// Retourne l'angle joint en radians, ou nullopt si pas encore de lecture
  /// valide pour ce joint. Inclut filtre median + couplage.
  std::optional<double> get_joint_position_rad(int joint_idx) const;

  /// Nombre d'outliers rejetes (cumul depuis init) pour un joint donne.
  int outliers_count(int joint_idx) const;

  /// Charge encoder_raw_init_deg depuis un YAML format minimal (parsing manuel).
  /// Format attendu :
  ///   encoder_raw_init_deg:
  ///     motor_1: 49.3788
  ///     motor_2: 347.6896
  ///     motor_3: 124.6748
  /// Met a jour raw_init_deg sur les JointSpec deja enregistres dont le
  /// slave_id correspond a motor_N. Retourne false si fichier introuvable ou
  /// format invalide.
  bool load_calibration_yaml(const std::string & yaml_path);

private:
  /// Etat de tracking pour un seul moteur (au sens slave_id).
  struct MotorState
  {
    double last_raw_deg = std::numeric_limits<double>::quiet_NaN();
    double unwrapped_deg = std::numeric_limits<double>::quiet_NaN();
    std::deque<double> median_buffer;
    int outliers_count = 0;
    int consecutive_rejects = 0;   ///< rejets consecutifs (pour la porte de sortie)
  };

  /// Query un esclave (envoie 1 byte ID, attend 4 bytes reponse).
  /// Retourne le raw angle en degres [0, 360), ou nullopt si timeout/no_measure.
  std::optional<double> query(int slave_id);

  /// Met a jour MotorState avec un nouveau raw read (ou pas de mesure si nullopt).
  void update_tracker(int slave_id, std::optional<double> raw_deg);

  /// Convertit l'unwrapped filtre en angle joint en rad (sans couplage).
  std::optional<double> motor_to_joint_rad(const JointSpec & spec) const;

  /// Mediane du buffer (sans modifier le buffer).
  static double median(const std::deque<double> & buf);

  /// Ramene un angle dans ]-180, 180].
  static double wrap_to_180(double deg);

  /// RS-485 : drive DE/RE high pour emission.
  void de_re_tx();
  /// RS-485 : drive DE/RE low pour reception.
  void de_re_rx();

  Config config_;
  bool initialized_ = false;
  int serial_fd_ = -1;

#ifdef HAS_GPIOD
  struct gpiod_chip * chip_ = nullptr;
  struct gpiod_line * de_re_line_ = nullptr;
#endif

  std::vector<JointSpec> joint_specs_;
  std::map<int, MotorState> states_;  ///< clef = slave_id

  struct Coupling
  {
    int from_joint;
    int to_joint;
    double ratio;
  };
  std::vector<Coupling> couplings_;

  // Polling async : un thread query les esclaves en boucle, update states_
  // sous lock. RobySystem::read() lit juste les positions filtrees via
  // get_joint_position_rad (non bloquant, ~µs).
  std::thread poll_thread_;
  std::atomic<bool> stop_polling_{false};
  mutable std::mutex states_mutex_;

  void poll_loop();
};

}  // namespace roby_hardware

#endif  // ROBY_HARDWARE__ENCODER_DRIVER_HPP_
