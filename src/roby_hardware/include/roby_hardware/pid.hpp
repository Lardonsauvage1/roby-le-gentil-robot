#ifndef ROBY_HARDWARE__PID_HPP_
#define ROBY_HARDWARE__PID_HPP_

#include <algorithm>

namespace roby_hardware
{

// Gains + etat d'un PID position pour un joint.
// Usage prevu : closed-loop encodeur en COMPLEMENT du feedforward.
// La consigne planifiee (command) est appliquee telle quelle (feedforward,
// aucun retard capteur) ; ce PID ne calcule QUE la correction de l'erreur
// residuelle lente (pas perdus, derive, gravite). C'est ce decouplage
// feedforward/feedback qui permet de garder l'axe rapide malgre la latence
// de l'encodeur : voir BUG-005.
struct PidState
{
  // Gains (0 par defaut => correction nulle => comportement open-loop identique)
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  // Anti-windup : borne le terme integral en valeur absolue (rad). 0 => pas de
  // terme integral (desactive). Empeche l'accumulation quand l'axe sature.
  double i_clamp = 0.0;
  // Deadband (rad) : si |error| <= deadband, correction = 0 (le bruit capteur
  // residuel n'est pas corrige). 0 => desactivee. Permet au moteur de s'ARRETER
  // vraiment au repos (pas de micro-stepping permanent), donc au driver de
  // passer en courant reduit (anti-chauffe) et supprime le jitter/pompage.
  double deadband = 0.0;

  // Etat interne (reset a l'activation)
  double integral = 0.0;
  double prev_error = 0.0;

  bool enabled() const { return kp != 0.0 || ki != 0.0 || kd != 0.0; }

  void reset()
  {
    integral = 0.0;
    prev_error = 0.0;
  }
};

// Un pas de PID. `error` = consigne - position_mesuree (rad). `dt` en secondes.
// Met a jour l'etat (integral, prev_error) et retourne la CORRECTION a ajouter
// a la consigne feedforward (rad). dt <= 0 => correction integrale/derivee
// ignoree pour ce pas (garde le terme proportionnel).
inline double pid_step(PidState & s, double error, double dt)
{
  // Deadband anti-jitter : sous le seuil (bruit capteur residuel), aucune
  // correction => le moteur ne micro-steppe pas => il s'arrete reellement et le
  // driver peut passer en courant reduit. L'integrale est gelee (pas
  // d'accumulation sur du bruit). prev_error suivi pour une derivee coherente
  // au prochain depassement.
  if (s.deadband > 0.0 && std::abs(error) <= s.deadband) {
    s.prev_error = error;
    return 0.0;
  }

  double p_term = s.kp * error;

  double i_term = 0.0;
  double d_term = 0.0;
  if (dt > 0.0) {
    s.integral += error * dt;
    // Anti-windup : clamp l'integral pour que |ki * integral| <= i_clamp.
    if (s.i_clamp > 0.0 && s.ki != 0.0) {
      double integral_max = s.i_clamp / s.ki;
      if (integral_max < 0.0) integral_max = -integral_max;
      s.integral = std::clamp(s.integral, -integral_max, integral_max);
    } else if (s.ki == 0.0) {
      s.integral = 0.0;
    }
    i_term = s.ki * s.integral;
    d_term = s.kd * (error - s.prev_error) / dt;
  }
  s.prev_error = error;

  return p_term + i_term + d_term;
}

}  // namespace roby_hardware

#endif  // ROBY_HARDWARE__PID_HPP_
