#ifndef ROBY_HARDWARE__OUTLIER_FILTER_HPP_
#define ROBY_HARDWARE__OUTLIER_FILTER_HPP_

#include <cmath>

namespace roby_hardware
{

// Decide si une lecture encodeur est ACCEPTEE (true) ou REJETEE (false), selon
// une limitation de vitesse avec PORTE DE SORTIE.
//
//  - `diff_deg`  : ecart (deg) entre la lecture brute et la derniere ACCEPTEE
//                  (deja wrap a +-180).
//  - `max_step_deg` : un saut plus grand = non physique => candidat outlier.
//  - `consecutive_rejects` : compteur de rejets consecutifs (etat, modifie ici).
//  - `max_consec_rejects`  : porte de sortie. Apres ce nombre de rejets
//                  consecutifs, on ACCEPTE quand meme la lecture (on suppose que
//                  le capteur a reellement change de valeur — mouvement rapide,
//                  re-synchro — et pas un simple glitch). Sans cette porte, le
//                  filtre se BLOQUE sur une valeur perimee (lock-up observe en
//                  mouvement rapide le 2026-05-31). Voir BUG-005 / encodeurs.
//
// Retour true => integrer la lecture ; false => l'ignorer (garder l'etat).
inline bool accept_with_escape(
  double diff_deg, double max_step_deg,
  int & consecutive_rejects, int max_consec_rejects)
{
  if (std::abs(diff_deg) > max_step_deg &&
      consecutive_rejects < max_consec_rejects)
  {
    consecutive_rejects += 1;
    return false;  // outlier rejete (garde la derniere valeur acceptee)
  }
  // Accepte : soit lecture normale (saut <= seuil), soit porte de sortie
  // atteinte (trop de rejets consecutifs => re-synchro sur la nouvelle valeur).
  consecutive_rejects = 0;
  return true;
}

}  // namespace roby_hardware

#endif  // ROBY_HARDWARE__OUTLIER_FILTER_HPP_
