"""Teleoperation en POSITION du bras simule par le bras guide.

Le guide devient une maquette : on le met dans une pose, Roby prend la meme. C'est
l'inverse du mode joystick (`leader_teleop_sim`), ou l'ecart au zero commandait une
VITESSE.

    q_roby = conversion_US019(q_leader)      puis CLAMP aux butees URDF

Aucun message vers le vrai robot : ce noeud n'anime que le modele pour RViz.

Deux consequences a connaitre, elles ne sont pas des defauts de ce noeud :

  - le guide doit etre LIBRE a la main, donc couple coupe -- et il s'affaisse alors,
    Roby suivant l'affaissement. C'est ce que le mode joystick contournait. La reponse
    propre est l'assistance gravite (US-025).
  - en 1:1, environ 124 deg de la course de la base de Roby restent inatteignables :
    la course du guide est plus courte (US-019). L'echelle par axe, lue dans la
    calibration, corrige cela partout sauf sur la base, laissee volontairement en 1:1.

    bash ~/roby_leader_teleop_pos.sh
"""

from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from roby_control.leader_mapping import charger


class TeleopPos(Node):
    def __init__(self):
        super().__init__("leader_teleop_pos")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("calib_file", "")
        self.declare_parameter("timeout_s", 0.5)
        # Lissage exponentiel : 0 = aucun (suit le guide au pas pres), 1 = fige.
        # Le tremblement de la main se voit tres bien sur un affichage 3D ; un lissage
        # leger le retire sans retard perceptible.
        self.declare_parameter("lissage", 0.2)

        self._chemin = self.get_parameter("calib_file").value or None
        self.cal = charger(self._chemin)
        self._mtime = self._mtime_calib()
        manquants = self.cal.non_calibres()
        if manquants:
            self.get_logger().warn("articulations NON calibrees : %s" % manquants)

        self.timeout = float(self.get_parameter("timeout_s").value)
        self.alpha = min(0.95, max(0.0, float(self.get_parameter("lissage").value)))
        self.q = {}
        self.dernier = None
        self._clamps = set()
        self._n_clamp = 0

        # Rechargement A CHAUD de la calibration : regler un signe ou une echelle ne
        # doit pas imposer de relancer le noeud, ni de recouper le couple du guide.
        # C'est ce qui rend le reglage des sens praticable -- on inverse, on regarde,
        # on passe au suivant.
        self.create_timer(0.5, self._recharger_si_change)

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/leader/joint_states", self._cb, 20)
        hz = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self.create_timer(2.0, self._rapport_clamp)
        self.get_logger().info(
            "Teleop POSITION prete : %.0f Hz, lissage %.2f. Le guide doit etre LIBRE "
            "(couple coupe). Aucun message vers le vrai robot." % (hz, self.alpha))

    def _cb(self, msg):
        vus = dict(zip(msg.name, msg.position))
        for j in self.cal.joints:
            if j.nom_leader not in vus:
                continue
            cible, clampe = j.convertir(vus[j.nom_leader])
            if clampe:
                self._clamps.add(j.nom_urdf)
                self._n_clamp += 1
            prec = self.q.get(j.nom_urdf)
            self.q[j.nom_urdf] = (cible if prec is None
                                  else prec + (1 - self.alpha) * (cible - prec))
        self.dernier = self.get_clock().now()

    def _tick(self):
        if not self.q:
            return
        if self.dernier is not None:
            age = (self.get_clock().now() - self.dernier).nanoseconds / 1e9
            if age > self.timeout:
                # Le guide s'est tu : on GELE la derniere pose connue plutot que de
                # continuer a republier une donnee perimee comme si elle etait fraiche.
                return
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        for nom, v in self.q.items():
            m.name.append(nom)
            m.position.append(float(v))
        self.pub.publish(m)

    def _fichier_calib(self):
        if self._chemin:
            return os.path.expanduser(self._chemin)
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("roby_control"),
                            "config", "leader_calibration.yaml")

    def _mtime_calib(self):
        try:
            return os.path.getmtime(self._fichier_calib())
        except OSError:
            return None

    def _recharger_si_change(self):
        m = self._mtime_calib()
        if m is None or m == self._mtime:
            return
        try:
            self.cal = charger(self._chemin)
        except Exception as e:                      # fichier en cours d'ecriture
            self.get_logger().warn("calibration illisible, on garde l'ancienne : %s" % e)
            return
        self._mtime = m
        self.get_logger().warn(
            "calibration RECHARGEE : %s"
            % ", ".join("%s signe %+.0f echelle %.3f" % (j.nom_urdf, j.signe, j.echelle)
                        for j in self.cal.joints if j.nom_urdf.startswith("joint")))

    def _rapport_clamp(self):
        """Un clamp n'est jamais silencieux, mais on ne sature pas le log non plus."""
        if self._clamps:
            self.get_logger().warn(
                "butee atteinte (%d fois) : %s"
                % (self._n_clamp, ", ".join(sorted(self._clamps))))
            self._clamps.clear()
            self._n_clamp = 0


def main():
    rclpy.init()
    n = TeleopPos()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
