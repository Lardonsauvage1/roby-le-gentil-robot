#!/usr/bin/env python3
"""roby_test_securite.py — valide sur le MATERIEL les garde-fous de securite.

⚠️ A n'utiliser que MOTEURS DESOLIDARISES DU BRAS. Ces tests envoient VOLONTAIREMENT
des consignes hors butee : c'est tout l'objet de la mesure. Sur un bras assemble,
ils iraient taper la butee mecanique.

Pourquoi ce script existe : les correctifs de securite du 2026-07-20 (butees URDF
lues par le C++, vitesse du garde) n'avaient ete verifies qu'au DEMARRAGE (les bonnes
valeurs sont chargees) et en tests unitaires. Personne n'avait prouve qu'un mouvement
reel est effectivement borne. C'est ce que fait ce script.

  butees  : commande joint_3 a +2.0 rad (butee URDF +0.65) et mesure ou il s'arrete.
            Publie DIRECTEMENT au controleur, en contournant volontairement le
            controle Python de roby_goto_joints -> teste la DERNIERE barriere, le C++.
  vitesse : envoie un saut brutal via /guard/joint_trajectory et mesure la vitesse
            reelle en sortie. Doit etre bornee par --max-vel du garde.

L'axe 1 n'est jamais utilise. Sans --go : DRY, rien n'est envoye.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
LIM = {"joint_1": (-3.14159, 3.14159), "joint_2": (-1.6, 2.1),
       "joint_3": (-3.0, 0.65), "joint_4": (-3.1416, 3.1416),
       "joint_5": (-1.6, 1.6)}


class Tester(Node):
    def __init__(self, topic):
        super().__init__("roby_test_securite")
        self.cur = None
        self.trace = []          # (t, positions) pour mesurer la vitesse
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.pub = self.create_publisher(JointTrajectory, topic, 10)

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in J):
            self.cur = [float(d[j]) for j in J]
            self.trace.append((time.monotonic(), list(self.cur)))

    def wait_state(self, t=5.0):
        t0 = time.monotonic()
        while self.cur is None and time.monotonic() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.cur

    def wait_sub(self, t=10.0):
        """Attend que l'abonne soit DECOUVERT avant de publier.

        Sans cette attente, le message part dans le vide : la decouverte DDS n'est
        pas instantanee et un publisher tout juste cree n'a encore aucun abonne.
        Piege vecu : le test concluait "clamp effectif" alors que RIEN n'avait ete
        recu ni bouge -- un faux positif silencieux, exactement ce qu'on traque."""
        t0 = time.monotonic()
        while self.pub.get_subscription_count() == 0 and time.monotonic() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.pub.get_subscription_count()

    def send(self, target, secs):
        jt = JointTrajectory()
        jt.joint_names = list(J)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in target]
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        jt.points = [pt]
        self.pub.publish(jt)

    def observe(self, secs):
        self.trace.clear()
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.02)
        return list(self.trace)


def vmax_of(trace, idx):
    """vitesse max observee sur l'axe idx (rad/s), lissee sur 3 echantillons."""
    v = 0.0
    for k in range(3, len(trace)):
        dt = trace[k][0] - trace[k - 3][0]
        if dt > 1e-3:
            v = max(v, abs(trace[k][1][idx] - trace[k - 3][1][idx]) / dt)
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("test", choices=["butees", "vitesse"])
    ap.add_argument("--go", action="store_true", help="ENVOIE la consigne (sinon DRY)")
    ap.add_argument("--axe", type=int, default=3, help="axe teste (JAMAIS 1)")
    ap.add_argument("--secs", type=float, default=6.0, help="duree d'observation")
    a = ap.parse_args()
    if a.axe == 1:
        print("REFUS : l'axe 1 est exclu des tests."); return 1
    idx = a.axe - 1
    name = J[idx]
    lo, hi = LIM[name]

    topic = "/arm_controller/joint_trajectory" if a.test == "butees" else "/guard/joint_trajectory"
    rclpy.init()
    n = Tester(topic)
    cur = n.wait_state()
    if cur is None:
        print("❌ pas de /joint_states : la stack tourne-t-elle ?")
        n.destroy_node(); rclpy.shutdown(); return 1

    tgt = list(cur)
    if a.test == "butees":
        demande = hi + 1.35                      # franchement au-dela de la butee
        tgt[idx] = demande
        print(f"TEST BUTEES — {name}")
        print(f"  position actuelle : {cur[idx]:+.4f} rad")
        print(f"  butee URDF        : [{lo:+.3f}, {hi:+.3f}]")
        print(f"  consigne ENVOYEE  : {demande:+.4f} rad  ({demande - hi:+.2f} au-dela)")
        print(f"  topic             : {topic}  (contourne le controle Python : on teste le C++)")
        print(f"  ATTENDU           : le moteur s'arrete a {hi:+.3f}, PAS a {demande:+.3f}")
        print(f"  amplitude reelle  : {abs(hi - cur[idx]):.3f} rad si le clamp marche,"
              f" {abs(demande - cur[idx]):.3f} rad sinon")
    else:
        demande = min(hi - 0.05, cur[idx] + 1.2)
        tgt[idx] = demande
        print(f"TEST VITESSE — {name}")
        print(f"  position actuelle : {cur[idx]:+.4f} rad")
        print(f"  consigne          : {demande:+.4f} rad en 0.2 s  = saut brutal")
        print(f"  topic             : {topic}  (le garde doit brider)")
        print(f"  ATTENDU           : vitesse de sortie bornee par --max-vel du garde (1.0 rad/s)")

    if not a.go:
        print("\n[DRY] rien envoye. Ajoute --go pour EXECUTER (moteurs desolidarises !).")
        n.destroy_node(); rclpy.shutdown(); return 0

    nsub = n.wait_sub()
    if nsub == 0:
        print(f"\n❌ AUCUN abonne sur {topic} : la consigne partirait dans le vide.")
        n.destroy_node(); rclpy.shutdown(); return 1
    print(f"\n  abonne(s) detecte(s) sur {topic} : {nsub}")

    depart = n.cur[idx]
    secs = a.secs if a.test == "butees" else 0.2
    n.send(tgt, secs)
    trace = n.observe(a.secs)
    fin = n.cur[idx]
    bouge = abs(fin - depart)
    print(f"  ---> depart {depart:+.4f} -> final {fin:+.4f} rad  (deplacement {bouge:.4f} rad)")

    # Un test qui ne bouge pas ne prouve RIEN : on le dit, au lieu de conclure
    # "clampe" par defaut (faux positif rencontre au 1er essai).
    if bouge < 0.02:
        print("  ---> ⚠️ NON CONCLUANT : le moteur n'a pas bouge du tout.")
        print("       Le clamp n'a pas ete exerce ; ne pas interpreter comme un succes.")
        n.destroy_node(); rclpy.shutdown(); return 2

    if a.test == "butees":
        ok = fin <= hi + 0.02
        print(f"  ---> {'✅ CLAMP EFFECTIF' if ok else '❌ CLAMP INEFFICACE'} : "
              f"arret a {fin:+.4f} pour une consigne de {demande:+.4f}")
        print(f"       depassement de butee : {max(0.0, fin - hi):+.4f} rad")
    else:
        v = vmax_of(trace, idx)
        print(f"  ---> vitesse max observee : {v:.3f} rad/s")
        print(f"  ---> {'✅ BRIDEE' if v <= 1.15 else '❌ NON BRIDEE'} (seuil garde 1.0 rad/s)")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
