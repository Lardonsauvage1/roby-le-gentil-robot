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
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger


class Panneau(Node):
    def __init__(self):
        super().__init__("roby_leader_panel")
        self.cli_recentrer = self.create_client(Trigger, "/leader/recentrer")
        self.cli_joystick = self.create_client(SetBool, "/leader/joystick")
        self.cli_embrayage = self.create_client(SetBool, "/teleop_cart/embrayage")
        self.cli_maintien = self.create_client(SetBool, "/leader/maintien")
        self.cli_pose = self.create_client(Trigger, "/teleop_cart/pose_travail")
        self.cli_reset = self.create_client(Trigger, "/guard/reset")
        self.garde = None
        self.t_garde = None
        self.create_subscription(String, "/guard/status", self._cb_garde, 10)
        self.cli_param = self.create_client(
            SetParameters, "/leader_teleop_cart/set_parameters")
        self.etat_cart = None
        self.create_subscription(Float64MultiArray, "/teleop_cart/etat",
                                 self._cb_cart, 10)
        self.n_msgs = 0
        self.create_subscription(JointState, "/leader/joint_states", self._cb, 10)

    def _cb(self, _m):
        self.n_msgs += 1

    def _cb_garde(self, m):
        self.garde = m.data
        self.t_garde = time.monotonic()

    def _cb_cart(self, m):
        self.etat_cart = list(m.data)

    def requete_echelle(self, k):
        v = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(k))
        return SetParameters.Request(parameters=[Parameter(name="echelle", value=v)])


def main():
    rclpy.init()
    n = Panneau()
    threading.Thread(target=lambda: rclpy.spin(n), daemon=True).start()

    root = tk.Tk()
    root.title("Bras guide")
    root.geometry("440x640")
    root.minsize(440, 480)
    # Redimensionnement VERTICAL autorise : la largeur est figee pour que les boutons
    # gardent leur place, mais brider la hauteur avait fini par couper les boutons du
    # bas des qu'on en ajoutait un. Une fenetre trop petite qui tronque son contenu
    # sans rien dire est pire qu'une fenetre qui s'etire.
    root.resizable(False, True)

    # DISPOSITION FIGEE. Les libellés d'état changent de longueur à chaque message ;
    # s'ils partagent le flux des boutons, la mise en page se recalcule et les boutons
    # se déplacent SOUS LE DOIGT. Ils sont donc relégués dans un bandeau du bas, de
    # hauteur imposée, avec propagation coupée : quoi qu'il s'y affiche, rien d'autre
    # ne bouge.
    bas = ttk.Frame(root, height=140)
    bas.pack(side="bottom", fill="x")
    bas.pack_propagate(False)

    cadre = ttk.Frame(root, padding=14)
    cadre.pack(side="top", fill="both", expand=True)

    etat = tk.StringVar(value="connexion...")
    suivi = tk.StringVar(value="")
    ttk.Separator(bas).pack(fill="x")
    ttk.Label(bas, textvariable=etat, wraplength=400, justify="left",
              anchor="nw").pack(fill="both", expand=True, padx=12, pady=(6, 0))
    ttk.Label(bas, textvariable=suivi, foreground="#555", wraplength=400,
              justify="left", anchor="sw").pack(fill="x", padx=12, pady=(0, 2))
    garde = tk.StringVar(value="")
    lab_garde = tk.Label(bas, textvariable=garde, wraplength=400, justify="left",
                         anchor="sw")
    lab_garde.pack(fill="x", padx=12, pady=(0, 8))

    def appeler(client, requete, libelle):
        etat.set("%s..." % libelle)
        root.update_idletasks()
        if not client.wait_for_service(timeout_sec=3.0):
            etat.set("service indisponible — le noeud concerne tourne-t-il ?")
            return
        fut = client.call_async(requete)

        def attendre():
            if not fut.done():
                root.after(100, attendre)
                return
            r = fut.result()
            msg = getattr(r, "message", "")
            ok = getattr(r, "success", True)
            if hasattr(r, "results"):          # SetParameters
                ok = all(x.successful for x in r.results)
                msg = "echelle appliquee" if ok else "echelle refusee"
            etat.set(("OK — " if ok else "ECHEC — ") + msg)
        root.after(100, attendre)

    def enchainer(etapes):
        """Appelle plusieurs services a la SUITE, en s'arretant au premier echec.

        Sert a EMBRAYER, qui doit d'abord activer le rappel elastique du guide. L'ordre
        n'est pas libre : si on embrayait AVANT d'activer le rappel, le guide partirait
        vers son zero une fois le couple mis, et le grand bras suivrait ce trajet. En
        activant le rappel d'abord, on laisse le guide se stabiliser, PUIS on pose les
        ancres.
        """
        if not etapes:
            return
        (client, requete, libelle), reste = etapes[0], etapes[1:]
        etat.set("%s..." % libelle)
        if not client.wait_for_service(timeout_sec=3.0):
            etat.set("service indisponible — le noeud concerne tourne-t-il ?")
            return
        fut = client.call_async(requete)

        def attendre():
            if not fut.done():
                root.after(100, attendre)
                return
            r = fut.result()
            ok = getattr(r, "success", True)
            if not ok:
                etat.set("ECHEC — " + getattr(r, "message", libelle))
                return
            etat.set("OK — " + getattr(r, "message", libelle))
            if reste:
                # Laisse le guide se stabiliser sous le rappel avant d'ancrer.
                root.after(600, lambda: enchainer(reste))
        root.after(100, attendre)

    ttk.Label(cadre, text="Bras guide").pack(anchor="w")
    ttk.Button(cadre, text="RECENTRER LE GUIDE",
               command=lambda: appeler(n.cli_recentrer, Trigger.Request(),
                                       "recentrage en cours")).pack(fill="x", ipady=10,
                                                                    pady=(4, 0))
    # Pas de bouton "joystick OFF" : le recentrage coupe le joystick lui-meme, et
    # c'etait le seul cas ou on avait besoin de l'eteindre a la main.
    ttk.Button(cadre, text="maintien du guide (tient sa pose)",
               command=lambda: appeler(n.cli_maintien, SetBool.Request(data=True),
                                       "maintien")).pack(fill="x", pady=(8, 0))
    ttk.Separator(cadre).pack(fill="x", pady=12)
    ttk.Label(cadre, text="Cartesien — rapport de mouvement").pack(anchor="w")
    ligne_k = ttk.Frame(cadre)
    ligne_k.pack(fill="x", pady=(4, 0))
    for libelle, k in (("1:1", 1.0), ("1:2", 0.5), ("1:5", 0.2), ("1:10", 0.1)):
        ttk.Button(ligne_k, text=libelle,
                   command=lambda kk=k: appeler(n.cli_param, n.requete_echelle(kk),
                                                "echelle")
                   ).pack(side="left", expand=True, fill="x", padx=2)

    ttk.Button(cadre, text="placer le bras en POSE DE TRAVAIL",
               command=lambda: appeler(n.cli_pose, Trigger.Request(),
                                       "placement")).pack(fill="x", pady=(8, 0))

    ligne_e = ttk.Frame(cadre)
    ligne_e.pack(fill="x", pady=(10, 0))
    ttk.Button(ligne_e, text="EMBRAYER",
               command=lambda: enchainer([
                   # MAINTIEN, pas rappel : en pilotage position, un guide qui revient
                   # vers son zero fait suivre le grand bras des qu'on reembraye.
                   (n.cli_maintien, SetBool.Request(data=True), "maintien du guide"),
                   (n.cli_embrayage, SetBool.Request(data=True), "embrayage"),
               ])).pack(side="left", expand=True, fill="x", ipady=8, padx=(0, 4))
    ttk.Button(ligne_e, text="DEBRAYER",
               command=lambda: appeler(n.cli_embrayage, SetBool.Request(data=False),
                                       "debrayage")).pack(side="left", expand=True,
                                                          fill="x", ipady=8, padx=(4, 0))
    # GARDE (vrai bras, ADR-003). Apres un gel -- plancher, collision --, le garde ne
    # laisse plus rien passer tant qu'on ne l'a pas rearme, et la teleop refuse
    # d'embrayer. Sans ce bouton il fallait un terminal : constate le 2026-09-13, le
    # bras paraissait mort. Rearmer ne fait rien bouger ; il faut ensuite EMBRAYER.
    ttk.Button(cadre, text="REARMER LE GARDE (apres un gel)",
               command=lambda: appeler(n.cli_reset, Trigger.Request(),
                                       "rearmement du garde")).pack(fill="x", pady=(8, 0))

    def rafraichir():
        if n.etat_cart and len(n.etat_cart) >= 4:
            k, emb, err, sig = n.etat_cart
            suivi.set("echelle 1:%.3g   %s   ecart orientation %.1f deg   "
                      "marge singularite %.3f"
                      % (1.0 / k if k else 0, "EMBRAYE" if emb > 0.5 else "debraye",
                         err, sig))
        age = None if n.t_garde is None else time.monotonic() - n.t_garde
        if n.garde is None or age is None or age > 2.0:
            garde.set("garde : absent (normal avec le bras simule seul, "
                      "obligatoire pour le vrai bras)")
            lab_garde.config(foreground="#555")
        elif n.garde.startswith("FROZEN"):
            raison = n.garde[len("FROZEN["):n.garde.rfind("]")]
            garde.set("GARDE GELE : %s -- verifier, puis REARMER LE GARDE" % raison)
            lab_garde.config(foreground="#c0392b")
        else:
            garde.set("garde : OK")
            lab_garde.config(foreground="#1e8449")
        if n.n_msgs and etat.get() == "connexion...":
            etat.set("guide connecte (%d messages recus)" % n.n_msgs)
        root.after(300, rafraichir)
    rafraichir()

    try:
        root.mainloop()
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
