#ifndef ROBY_HARDWARE__PID_HPP_
#define ROBY_HARDWARE__PID_HPP_

#include <algorithm>
#include <cmath>

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
  // Largeur (rad) de la rampe douce juste AU-DESSUS de la deadband. Sans elle,
  // la correction saute de 0 (dans la deadband) a sa pleine valeur des qu'on
  // sort : ce MUR cree un cycle limite (pompage) quand le residuel stationne au
  // bord de la deadband (cf. vibration joint_2 a -25deg). Avec la rampe, la
  // correction monte lineairement de 0 (a |error|=deadband) a 1 (a
  // |error|=deadband+deadband_ramp) => transition continue => pas de pompage.
  // 0 => mur d'origine (retro-compatible : aucun changement de comportement).
  double deadband_ramp = 0.0;

  // --- Deadband DEUX-PHASES (precision + anti-jitter) ----------------------
  // `deadband` ci-dessus est la deadband ACTIVE utilisee par pid_step ; elle est
  // recalculee chaque cycle par update_deadband(command) selon la phase :
  //  - PHASE MOUVEMENT (commande qui bouge, ou < settle_cycles cycles de commande
  //    stable) => deadband serree `deadband_moving` => le PID corrige finement =>
  //    atterrissage precis (la rampe lisse pour eviter de vibrer).
  //  - PHASE STABILISEE (commande constante depuis settle_cycles cycles) =>
  //    deadband large `deadband_settled` => le PID se fige => pas de jitter au repos.
  // settle_cycles <= 0 => two-phase DESACTIVE : deadband = deadband_settled
  // toujours (retro-compatible avec la deadband fixe d'avant).
  double deadband_settled = 0.0;   // deadband au repos (large). Config.
  double deadband_moving = 0.0;    // deadband pendant le mouvement (serree). Config.
  int settle_cycles = 0;           // cycles de commande stable avant bascule settled.

  // Etat interne (reset a l'activation)
  double integral = 0.0;
  double prev_error = 0.0;
  int stable_count = 0;            // cycles consecutifs de commande stable
  double last_command = 0.0;       // derniere commande vue (detection de stabilite)

  bool enabled() const { return kp != 0.0 || ki != 0.0 || kd != 0.0; }

  // Recalcule la deadband ACTIVE (`deadband`) selon la phase deux-phases.
  // A appeler chaque cycle AVANT pid_step, avec la consigne du joint (JTC).
  void update_deadband(double command)
  {
    if (settle_cycles <= 0) { deadband = deadband_settled; return; }
    if (std::abs(command - last_command) < 1e-6) {
      if (stable_count < settle_cycles) stable_count++;
    } else {
      stable_count = 0;
    }
    last_command = command;
    deadband = (stable_count >= settle_cycles) ? deadband_settled : deadband_moving;
  }

  void reset()
  {
    integral = 0.0;
    prev_error = 0.0;
    stable_count = 0;
    last_command = 0.0;
    deadband = deadband_settled;
  }
};

// Un pas de PID. `error` = consigne - position_mesuree (rad). `dt` en secondes.
// Met a jour l'etat (integral, prev_error) et retourne la CORRECTION a ajouter
// a la consigne feedforward (rad). dt <= 0 => correction integrale/derivee
// ignoree pour ce pas (garde le terme proportionnel).
inline double pid_step(PidState & s, double error, double dt)
{
  const double abs_err = std::abs(error);

  // Deadband anti-jitter : sous le seuil (bruit capteur residuel), aucune
  // correction => le moteur ne micro-steppe pas => il s'arrete reellement et le
  // driver peut passer en courant reduit. L'integrale est gelee (pas
  // d'accumulation sur du bruit). prev_error suivi pour une derivee coherente
  // au prochain depassement.
  if (s.deadband > 0.0 && abs_err <= s.deadband) {
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

  double correction = p_term + i_term + d_term;

  // Rampe douce en sortie de deadband (cf. champ deadband_ramp). Met a l'echelle
  // la correction par un facteur qui monte de 0 (au bord de la deadband) a 1
  // (a deadband + deadband_ramp), supprimant la discontinuite a l'origine du
  // pompage. N'affecte PAS l'etat interne (integral, prev_error) : seul le
  // signal de sortie est adouci. deadband_ramp <= 0 => pleine correction (mur).
  if (s.deadband > 0.0 && s.deadband_ramp > 0.0) {
    double scale = (abs_err - s.deadband) / s.deadband_ramp;
    if (scale < 1.0) {
      correction *= (scale > 0.0 ? scale : 0.0);
    }
  }

  return correction;
}

}  // namespace roby_hardware

#endif  // ROBY_HARDWARE__PID_HPP_
