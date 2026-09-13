"""Generateur de consigne en ligne, limite en vitesse et en acceleration.

Appele a chaque tick (100 Hz), il fait avancer une consigne vers une cible qui
peut changer a tout moment :

- cible lointaine (saut) : profil trapezoidal, freinage anticipe => arrivee
  sur la cible sans depassement ;
- cible qui bouge deja de facon faisable (flux 100 Hz du
  JointTrajectoryController) : la vitesse de la cible est anticipee
  (feedforward) => suivi exact, sans le retard v^2/2a d'un simple limiteur.

- cible qui S'ARRETE NET alors que la consigne la suivait a la vitesse v :
  l'acceleration etant bornee, la consigne depasse d'environ v^2 / (2 * a_max)
  puis revient exactement sur la cible (0,03 rad pour v = 0,3 rad/s et
  a_max = 1,5 rad/s^2). Inevitable sans connaitre la trajectoire a l'avance ;
  mesure le 2026-09-13, a verifier sur le poignet monte (US-031).

Module pur (pas d'E/S, pas de thread) : teste unitairement.
"""

from __future__ import annotations

import math


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class SetpointLimiter:
    """Rampe en ligne vers une cible, |v| <= max_velocity, |a| <= max_acceleration."""

    def __init__(self, max_velocity: float, max_acceleration: float) -> None:
        if max_velocity <= 0.0 or max_acceleration <= 0.0:
            raise ValueError("max_velocity et max_acceleration doivent etre > 0")
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self._position = 0.0
        self._velocity = 0.0
        self._target = 0.0
        self._prev_target = 0.0

    @property
    def position(self) -> float:
        return self._position

    @property
    def velocity(self) -> float:
        return self._velocity

    @property
    def target(self) -> float:
        return self._target

    def reset(self, position: float) -> None:
        """Repart a l'arret sur `position` (qui devient aussi la cible)."""
        self._position = position
        self._velocity = 0.0
        self._target = position
        self._prev_target = position

    def set_target(self, target: float) -> None:
        self._target = target

    def at_target(self, tolerance: float = 1e-6) -> bool:
        return (
            abs(self._target - self._position) <= tolerance
            and abs(self._velocity) <= tolerance
        )

    def step(self, dt: float) -> float:
        """Avance d'un pas de `dt` secondes et renvoie la nouvelle consigne."""
        if dt <= 0.0:
            return self._position

        vmax = self.max_velocity
        amax = self.max_acceleration

        # Vitesse de la cible (feedforward). Bornee : un saut de consigne donne
        # une "vitesse" enorme sur un seul tick, qu'on ne veut pas suivre.
        target_velocity = _clamp((self._target - self._prev_target) / dt, -vmax, vmax)
        self._prev_target = self._target

        error = self._target - self._position
        # Vitesse relative maximale qui permet encore de s'arreter SUR la cible
        # (v^2 = 2*a*d), en retirant le chemin parcouru pendant ce tick.
        braking_distance = max(
            abs(error) - abs(self._velocity - target_velocity) * dt, 0.0
        )
        approach_speed = min(vmax, math.sqrt(2.0 * amax * braking_distance))
        desired = target_velocity + math.copysign(approach_speed, error)
        # Jamais plus vite que la vitesse qui pose exactement sur la cible dans
        # ce tick : c'est ce qui rend le suivi d'un flux faisable exact.
        if error > 0.0:
            desired = min(desired, error / dt)
        elif error < 0.0:
            desired = max(desired, error / dt)
        desired = _clamp(desired, -vmax, vmax)

        max_dv = amax * dt
        self._velocity += _clamp(desired - self._velocity, -max_dv, max_dv)
        self._position += self._velocity * dt
        if abs(self._target - self._position) < 1e-12:
            self._position = self._target  # absorbe l'arrondi flottant
        return self._position
