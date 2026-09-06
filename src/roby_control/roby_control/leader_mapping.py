"""Correspondance bras guide -> Roby (US-019) — sans ROS, donc testable seule.

Conversion, par articulation :

    ecart  = deroulement_[-pi,pi](q_leader - zero)   <-- indispensable, voir plus bas
    q_roby = zero_urdf + signe * echelle * ecart     puis CLAMP aux butees URDF

`zero_urdf` est l'angle de ROBY correspondant a la position `zero` du leader. Il n'est
PAS toujours nul : les butees de Roby sont asymetriques sur plusieurs axes (l'epaule va
de -1,6 a +2,1, centre a +0,25). Viser le zero URDF avec un leader centre sur sa course
ferait deborder d'un cote et n'atteindrait jamais l'autre — la "course pleine" ne serait
pas realisee.

Les trois parametres se determinent au montage et sont **persistes dans un YAML
versionne**, jamais codes en dur :
  - signe   : le leader et Roby peuvent tourner en sens opposes autour du meme axe
  - zero    : offset entre la position mecanique du servo et le zero URDF de Roby
  - echelle : les debattements des deux bras ne sont pas identiques

Le clamp n'est pas un detail de confort. Mesure du 2026-09-05 : sur l'axe 5, le leader
offre **207,5 deg** alors que la butee URDF de Roby n'en autorise que **183,3**. Sans
clamp, un geste parfaitement normal sur le bras guide commanderait une position hors
butee. Sur les autres axes c'est l'inverse (le leader est plus court) : une partie de la
course de Roby reste inatteignable a la main, ce qui est benin mais doit etre documente.

Le DEROULEMENT n'est pas un raffinement : 4 des 6 axes franchissent le zero du codeur
(mesure US-017). Une soustraction brute donnerait un ecart de presque un tour la ou le
mouvement reel est de quelques degres — le bras partirait en butee.

Les valeurs produites sont des angles **URDF**. Le couplage mecanique des axes 2/3 est
l'affaire du driver ros2_control, jamais de cette calibration.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import yaml

DEFAULT_CALIB = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config", "leader_calibration.yaml"
)


@dataclass
class JointMapping:
    """Correspondance d'une articulation. Angles en radians."""

    id: int
    nom_leader: str
    nom_urdf: str
    signe: float = 1.0
    zero: float = 0.0        # position LEADER de reference (rad)
    zero_urdf: float = 0.0   # angle ROBY correspondant a `zero`
    echelle: float = 1.0
    # Ecarts au zero que le leader peut reellement produire (rad). None => symetrique
    # +/- demi-course. La pince fait exception : sa course va de 0 (fermee) a +course.
    ecart_min: float | None = None
    ecart_max: float | None = None
    urdf_min: float = -math.pi
    urdf_max: float = math.pi
    continu: bool = False
    calibre: bool = False  # False tant que signe/zero/echelle n'ont pas ete releves
    course_leader: float | None = None  # amplitude mesuree (rad), None si continu
    notes: str = ""

    def ecart_deroule(self, q_leader: float) -> float:
        """Ecart au zero, DEROULE dans [-pi, pi].

        Indispensable : 4 des 6 axes du leader franchissent le zero du codeur (mesure
        US-017). Une soustraction brute `q_leader - zero` donnerait un ecart de presque
        un tour la ou le mouvement reel est de quelques degres. Le deroulement est
        valide tant que la course reste sous 360 deg — verifie sur tous les axes
        (maximum 236 deg).
        """
        return (q_leader - self.zero + math.pi) % (2.0 * math.pi) - math.pi

    def convertir(self, q_leader: float) -> tuple[float, bool]:
        """Angle leader (rad) -> angle URDF Roby (rad).

        Retourne (angle, clampe) : `clampe` vaut True si la demande sortait des butees
        et a ete ramenee. L'appelant DOIT remonter cette information : un suivi clampe
        parait bon a l'oeil alors qu'il ne suit plus.
        """
        q = self.zero_urdf + self.signe * self.echelle * self.ecart_deroule(q_leader)
        if q < self.urdf_min:
            return self.urdf_min, True
        if q > self.urdf_max:
            return self.urdf_max, True
        return q, False

    def convertir_inverse(self, q_roby: float) -> tuple[float, bool]:
        """Angle URDF Roby -> angle leader. Sert au REALIGNEMENT (US-022).

        Le leader doit rejoindre la pose ou se trouve deja le vrai bras, AVANT que le
        suivi ne commence — sinon le premier instant de suivi ferait sauter Roby vers la
        pose de la main.

        Retourne (q_leader, atteignable). `atteignable` vaut False si la position
        demandee sort de la course mecanique du bras guide : c'est possible des que
        l'echelle est < course_roby/course_leader, donc **sur l'axe 1**, laisse en 1:1
        avec ~124 deg de Roby hors d'atteinte. Dans ce cas le realignement doit ECHOUER
        proprement (et le dire), jamais forcer contre la butee du leader.
        """
        ecart = (q_roby - self.zero_urdf) / (self.signe * self.echelle)
        atteignable = True
        if not self.continu and self.course_leader is not None:
            lo, hi = self.bornes_ecart()
            if not lo - 1e-9 <= ecart <= hi + 1e-9:
                atteignable = False
        return (self.zero + ecart) % (2.0 * math.pi), atteignable

    def bornes_ecart(self) -> tuple[float, float]:
        """Ecarts au zero que le leader peut produire mecaniquement.

        Par defaut symetriques (+/- demi-course) : c'est le cas quand le zero est pris au
        milieu du debattement. `ecart_min`/`ecart_max` permettent de decrire un zero
        excentre — la pince, dont le zero est la position FERMEE, donc a une extremite.
        """
        if self.course_leader is None:
            return (-math.pi, math.pi)
        demi = self.course_leader / 2.0
        lo = -demi if self.ecart_min is None else self.ecart_min
        hi = demi if self.ecart_max is None else self.ecart_max
        return lo, hi

    @property
    def course_urdf(self) -> float:
        return self.urdf_max - self.urdf_min

    def couverture(self) -> dict:
        """Compare la course du leader a celle de Roby, une fois l'echelle appliquee."""
        if self.continu or self.course_leader is None:
            return {"continu": True}
        lo, hi = self.bornes_ecart()
        atteignable = abs(self.echelle) * (hi - lo)
        return {
            "continu": False,
            "course_leader_rad": self.course_leader,
            "course_urdf_rad": self.course_urdf,
            "atteignable_rad": atteignable,
            # >0 : des positions de Roby resteront hors d'atteinte a la main
            "inatteignable_rad": max(0.0, self.course_urdf - atteignable),
            # >0 : le leader peut demander hors butee -> le clamp interviendra
            "debordement_rad": max(0.0, atteignable - self.course_urdf),
        }


@dataclass
class LeaderCalibration:
    joints: list = field(default_factory=list)

    @property
    def par_nom(self):
        return {j.nom_urdf: j for j in self.joints}

    def convertir_tous(self, angles_leader: dict) -> tuple[dict, list]:
        """{nom_leader: rad} -> ({nom_urdf: rad}, [noms clampes])."""
        out, clampes = {}, []
        for j in self.joints:
            if j.nom_leader not in angles_leader:
                continue
            q, clampe = j.convertir(angles_leader[j.nom_leader])
            out[j.nom_urdf] = q
            if clampe:
                clampes.append(j.nom_urdf)
        return out, clampes

    def non_calibres(self) -> list:
        return [j.nom_urdf for j in self.joints if not j.calibre]


def charger(chemin=None) -> LeaderCalibration:
    chemin = chemin or DEFAULT_CALIB
    if not os.path.exists(chemin):
        raise FileNotFoundError("calibration leader introuvable : %s" % chemin)
    with open(chemin, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    joints = []
    for e in data.get("joints", []):
        joints.append(
            JointMapping(
                id=int(e["id"]),
                nom_leader=e["nom_leader"],
                nom_urdf=e["nom_urdf"],
                signe=float(e.get("signe", 1.0)),
                zero=float(e.get("zero", 0.0)),
                zero_urdf=float(e.get("zero_urdf", 0.0)),
                ecart_min=(None if e.get("ecart_min") is None else float(e["ecart_min"])),
                ecart_max=(None if e.get("ecart_max") is None else float(e["ecart_max"])),
                echelle=float(e.get("echelle", 1.0)),
                urdf_min=float(e["urdf_min"]),
                urdf_max=float(e["urdf_max"]),
                continu=bool(e.get("continu", False)),
                calibre=bool(e.get("calibre", False)),
                course_leader=(None if e.get("course_leader") is None
                               else float(e["course_leader"])),
                notes=e.get("notes", ""),
            )
        )
    return LeaderCalibration(joints=joints)
