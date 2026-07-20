#!/usr/bin/env python3
"""Espion des consignes pince/verrou : horodate CHAQUE message /gripper et
/head_lock recu cote PC. But = detecter les consignes parasites (plusieurs
messages par clic) ou en retard.

Affiche pour chaque message : dt depuis le precedent, valeur, nb de publishers.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class Spy(Node):
    def __init__(self):
        super().__init__("roby_spy_gripper")
        self.t0 = {"/gripper": None, "/head_lock": None}
        self.n = {"/gripper": 0, "/head_lock": 0}
        self.create_subscription(Bool, "/gripper", lambda m: self._cb("/gripper", m), 10)
        self.create_subscription(Bool, "/head_lock", lambda m: self._cb("/head_lock", m), 10)
        self.create_timer(2.0, self._pubcount)
        print("Espion pret. Ouvre/ferme la pince (et le verrou) depuis le jog.\n"
              "  colonne dt = temps depuis le message precedent sur le MEME topic.\n")

    def _cb(self, topic, msg):
        now = time.monotonic()
        prev = self.t0[topic]
        dt = "" if prev is None else "  dt=%.3fs" % (now - prev)
        self.t0[topic] = now
        self.n[topic] += 1
        val = "FERME/VERROU(True)" if msg.data else "OUVRE/DEVERROU(False)"
        flag = ""
        if prev is not None and (now - prev) < 0.3:
            flag = "   <<< RAPPROCHE (parasite ?)"
        print("[%9.3f] %-11s = %-22s (#%d)%s%s"
              % (now, topic, val, self.n[topic], dt, flag))

    def _pubcount(self):
        g = self.count_publishers("/gripper")
        h = self.count_publishers("/head_lock")
        if g != 1 or h != 1:
            print("  ... publishers: /gripper=%d  /head_lock=%d  (attendu 1 chacun ; >1 = source de parasites)" % (g, h))


def main():
    import signal
    # Survivre aux signaux envoyes par l'orchestrateur : on veut capturer en
    # continu pendant que Sam clique. Kill par -9 pour arreter.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, signal.SIG_IGN)
        except Exception:
            pass
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init()
    n = Spy()
    while rclpy.ok():
        try:
            rclpy.spin_once(n, timeout_sec=0.5)
        except Exception:
            pass


if __name__ == "__main__":
    main()
