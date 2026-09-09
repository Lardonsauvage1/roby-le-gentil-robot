#!/usr/bin/env python3
"""roby_leader_panel.py — panneau du bras guide : un bouton pour le recentrer.

Pourquoi un panneau plutot qu'une ligne de commande : en teleoperation en POSITION,
la pose de depart du guide decale tout le suivi. On la reprend donc souvent, une main
sur le bras -- pas dans un terminal. Meme raison d'etre que roby_collect_panel.py et
roby_infer_panel.py.

Le bouton appelle /leader/recentrer : couple bas, retour au zero par le plus court
chemin, puis couple RECOUPE. L'etat de repos du guide reste "libre a la main".

    bash ~/roby_leader_panel.sh
"""
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger


class Panneau(Node):
    def __init__(self):
        super().__init__("roby_leader_panel")
        self.cli_recentrer = self.create_client(Trigger, "/leader/recentrer")
        self.cli_joystick = self.create_client(SetBool, "/leader/joystick")
        self.n_msgs = 0
        self.create_subscription(JointState, "/leader/joint_states", self._cb, 10)

    def _cb(self, _m):
        self.n_msgs += 1


def main():
    rclpy.init()
    n = Panneau()
    threading.Thread(target=lambda: rclpy.spin(n), daemon=True).start()

    root = tk.Tk()
    root.title("Bras guide")
    root.geometry("380x230")
    cadre = ttk.Frame(root, padding=14)
    cadre.pack(fill="both", expand=True)

    etat = tk.StringVar(value="connexion...")
    ttk.Label(cadre, textvariable=etat, wraplength=340,
              justify="left").pack(pady=(0, 12), anchor="w")

    def appeler(client, requete, libelle):
        etat.set("%s..." % libelle)
        root.update_idletasks()
        if not client.wait_for_service(timeout_sec=3.0):
            etat.set("service indisponible — leader_node tourne-t-il ?")
            return
        fut = client.call_async(requete)
        def attendre():
            if not fut.done():
                root.after(100, attendre)
                return
            r = fut.result()
            etat.set(("OK — " if r.success else "ECHEC — ") + r.message)
        root.after(100, attendre)

    b = ttk.Button(cadre, text="RECENTRER LE GUIDE",
                   command=lambda: appeler(n.cli_recentrer, Trigger.Request(),
                                           "recentrage en cours"))
    b.pack(fill="x", ipady=10)

    ligne = ttk.Frame(cadre)
    ligne.pack(fill="x", pady=(10, 0))
    ttk.Button(ligne, text="joystick ON",
               command=lambda: appeler(n.cli_joystick, SetBool.Request(data=True),
                                       "activation")).pack(side="left", expand=True,
                                                           fill="x", padx=(0, 4))
    ttk.Button(ligne, text="joystick OFF (libre)",
               command=lambda: appeler(n.cli_joystick, SetBool.Request(data=False),
                                       "coupure")).pack(side="left", expand=True,
                                                        fill="x", padx=(4, 0))

    def rafraichir():
        if n.n_msgs:
            etat_actuel = etat.get()
            if etat_actuel == "connexion...":
                etat.set("guide connecte (%d messages recus)" % n.n_msgs)
        root.after(500, rafraichir)
    rafraichir()

    try:
        root.mainloop()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
