#!/usr/bin/env python3
"""Jog fin du bras Roby + lecture TCP + capture de poses (calibration nid/dock).

But : piloter le VRAI robot avec une precision bien meilleure que le degre,
lire la position cartesienne du TCP (repere world = base robot), et capturer
des poses articulaires (nid, coins du plan de travail...) pour la calibration.

A LANCER AVEC LE PYTHON SYSTEME (pas pyenv) + ROS Jazzy + DDS deja configures :
    /usr/bin/python3 ~/roby_fine_jog.py
(voir le .sh compagnon roby_fine_jog.sh qui source tout)

Securite :
  - Ne bouge JAMAIS au demarrage. Les cibles sont init sur la pose mesuree.
  - Chaque mouvement = un clic explicite de Sam, a vitesse LENTE (0.13 rad/s).
  - Cibles bridees aux butees articulaires.
"""
import os
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import tf2_ros

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
LIMITS = {                       # butees position (rad), depuis l'URDF
    "joint_1": (-3.14159, 3.14159),
    "joint_2": (-1.6, 2.1),
    "joint_3": (-3.0, 0.65),
    "joint_4": (-3.1416, 3.1416),
    "joint_5": (-1.6, 1.6),
}
SPEED = 0.13            # rad/s — lent (open-loop, anti-a-coup)
MIN_DUR = 1.0          # duree mini d'un mouvement (s)
WORLD, TCP = "world", "tcp"
CAPTURE_FILE = os.path.expanduser("~/roby_poses.yaml")  # fichier UNIQUE faisant autorite
STEPS_DEG = [0.05, 0.1, 0.5, 1.0, 5.0, 10.0]   # pas de jog selectionnables (deg)

D2R = 3.141592653589793 / 180.0
R2D = 180.0 / 3.141592653589793


class FineJog(Node):
    def __init__(self, root):
        super().__init__("roby_fine_jog")
        self.root = root
        self.measured = None        # derniere pose mesuree (dict joint->rad)
        self.target = None          # vecteur cible interne (list rad, ordre JOINTS)
        self.tcp_xyz = None         # (x,y,z) m dans world

        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.lock_pub = self.create_publisher(Bool, "/head_lock", 10)
        self.grip_pub = self.create_publisher(Bool, "/gripper", 10)
        self.ac = ActionClient(self, FollowJointTrajectory,
                               "/arm_controller/follow_joint_trajectory")
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self._build_gui()

    # ---------- ROS callbacks ----------
    def _on_js(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(j in m for j in JOINTS):
            self.measured = {j: float(m[j]) for j in JOINTS}
            if self.target is None:          # init cibles = pose mesuree
                self.target = [self.measured[j] for j in JOINTS]
                self._sync_entries()

    def _read_tcp(self):
        try:
            t = self.tf_buf.lookup_transform(WORLD, TCP, rclpy.time.Time())
            tr = t.transform.translation
            self.tcp_xyz = (tr.x, tr.y, tr.z)
        except Exception:
            pass

    # ---------- GUI ----------
    def _build_gui(self):
        self.root.title("Roby — Jog fin / calibration nid")
        self.root.configure(padx=10, pady=10)
        big = ("TkDefaultFont", 11)

        info = tk.Label(self.root, fg="#a00", font=("TkDefaultFont", 10, "bold"),
                        text="Le robot bouge LENTEMENT a chaque clic. Repere world = base "
                             "robot (+X avant, +Y gauche, +Z haut).")
        info.grid(row=0, column=0, columnspan=7, sticky="w", pady=(0, 8))

        # --- entete lecture ---
        self.lbl_state = tk.Label(self.root, font=("TkFixedFont", 10), justify="left",
                                  text="en attente de /joint_states...", anchor="w")
        self.lbl_state.grid(row=1, column=0, columnspan=7, sticky="w")
        self.lbl_tcp = tk.Label(self.root, font=("TkFixedFont", 11, "bold"),
                                fg="#06c", anchor="w", text="TCP : --")
        self.lbl_tcp.grid(row=2, column=0, columnspan=7, sticky="w", pady=(0, 8))

        # --- selecteur de pas ---
        sel = tk.Frame(self.root); sel.grid(row=3, column=0, columnspan=7, sticky="w")
        tk.Label(sel, text="Pas de jog :", font=big).pack(side="left")
        self.step_var = tk.DoubleVar(value=0.5)
        for s in STEPS_DEG:
            ttk.Radiobutton(sel, text=f"{s}°", value=s,
                            variable=self.step_var).pack(side="left", padx=4)

        # --- lignes par joint ---
        self.entries = {}
        hdr = tk.Frame(self.root); hdr.grid(row=4, column=0, columnspan=7, pady=(8, 2), sticky="w")
        for c, txt in enumerate(["Axe", "mesure", "", "cible (deg)", "", "butees"]):
            tk.Label(hdr, text=txt, width=[6, 16, 4, 12, 4, 16][c],
                     font=("TkDefaultFont", 9, "italic")).grid(row=0, column=c)

        for i, j in enumerate(JOINTS):
            row = tk.Frame(self.root); row.grid(row=5 + i, column=0, columnspan=7, sticky="w", pady=1)
            tk.Label(row, text=j.replace("joint_", "A"), width=6, font=big).grid(row=0, column=0)
            lbl_m = tk.Label(row, text="--", width=16, font=("TkFixedFont", 10)); lbl_m.grid(row=0, column=1)
            tk.Button(row, text="−", width=2, command=lambda k=i: self._jog(k, -1)
                      ).grid(row=0, column=2)
            e = tk.Entry(row, width=10, font=("TkFixedFont", 10), justify="right"); e.grid(row=0, column=3)
            tk.Button(row, text="+", width=2, command=lambda k=i: self._jog(k, +1)
                      ).grid(row=0, column=4)
            lo, hi = LIMITS[j]
            tk.Label(row, text=f"[{lo*R2D:.0f}, {hi*R2D:.0f}]°", width=16,
                     font=("TkFixedFont", 9), fg="#888").grid(row=0, column=5)
            self.entries[j] = (e, lbl_m)

        # --- boutons action ---
        act = tk.Frame(self.root); act.grid(row=11, column=0, columnspan=7, pady=(10, 4), sticky="w")
        tk.Button(act, text="▶ Aller a la cible", font=big, bg="#cfe9cf",
                  command=self._goto).pack(side="left", padx=3)
        tk.Button(act, text="Recopier mesure → cible", command=self._sync_target_from_measure
                  ).pack(side="left", padx=3)

        # --- verrou / pince ---
        sv = tk.Frame(self.root); sv.grid(row=12, column=0, columnspan=7, pady=4, sticky="w")
        tk.Label(sv, text="Nid/pince :", font=big).pack(side="left")
        tk.Button(sv, text="Verrouiller (75°)", bg="#e9cfcf",
                  command=lambda: self._lock(True)).pack(side="left", padx=3)
        tk.Button(sv, text="Deverrouiller (50°)",
                  command=lambda: self._lock(False)).pack(side="left", padx=3)
        tk.Button(sv, text="Pince fermer", command=lambda: self._grip(True)).pack(side="left", padx=3)
        tk.Button(sv, text="Pince ouvrir", command=lambda: self._grip(False)).pack(side="left", padx=3)

        # --- capture ---
        cap = tk.Frame(self.root); cap.grid(row=13, column=0, columnspan=7, pady=(8, 2), sticky="w")
        tk.Label(cap, text="Capturer la pose sous le nom :", font=big).pack(side="left")
        self.cap_name = tk.Entry(cap, width=14); self.cap_name.pack(side="left", padx=4)
        self.cap_name.insert(0, "nid")
        tk.Button(cap, text="\U0001f4be Capturer", bg="#cfd9e9", font=big,
                  command=self._capture).pack(side="left", padx=4)
        self.lbl_log = tk.Label(self.root, text="", fg="#060", anchor="w", justify="left",
                                font=("TkFixedFont", 9))
        self.lbl_log.grid(row=14, column=0, columnspan=7, sticky="w", pady=(4, 0))

    # ---------- actions ----------
    def _clamp(self, j, v):
        lo, hi = LIMITS[j]
        return max(lo, min(hi, v))

    def _jog(self, idx, sign):
        if self.measured is None:
            return self._log("Pas encore de pose mesuree.", err=True)
        j = JOINTS[idx]
        step = self.step_var.get() * D2R
        # Jog RELATIF a la pose MESUREE courante (jamais une cible interne
        # accumulee : sinon, apres un restart de la stack, self.target reste
        # perime => saut dangereux vers l'ancienne pose, cf incident nid 2026-06).
        base = [self.measured[jj] for jj in JOINTS]
        base[idx] = self._clamp(j, base[idx] + sign * step)
        self.target = base
        self._sync_entries()
        self._send(self.target)

    def _goto(self):
        if self.target is None:
            return self._log("Pas encore de pose mesuree.", err=True)
        tgt = []
        for j in JOINTS:
            try:
                v = float(self.entries[j][0].get()) * D2R
            except ValueError:
                return self._log(f"Valeur invalide pour {j}.", err=True)
            tgt.append(self._clamp(j, v))
        self.target = tgt
        self._sync_entries()
        self._send(tgt)

    def _send(self, target_rad):
        if not self.ac.server_is_ready():
            if not self.ac.wait_for_server(timeout_sec=2.0):
                return self._log("arm_controller absent (stack Pi5 lancee ?).", err=True)
        base = [self.measured[j] for j in JOINTS] if self.measured else target_rad
        delta = max(abs(target_rad[i] - base[i]) for i in range(5))
        if delta > 0.35:  # garde-fou anti-saut (~20 deg) : refuse tout grand mouvement
            return self._log("REFUS: saut de %.0f deg > 20 deg (securite). "
                             "Jogge par petits pas." % (delta * R2D), err=True)
        dur = max(MIN_DUR, delta / SPEED)
        traj = JointTrajectory(); traj.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(); pt.positions = list(target_rad)
        pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
        self.ac.send_goal_async(goal)
        self._log("MOVE cible=[%s]  (%.1fs)" %
                  (", ".join("%.2f°" % (v * R2D) for v in target_rad), dur))

    def _sync_target_from_measure(self):
        if self.measured is None:
            return self._log("Pas de mesure.", err=True)
        self.target = [self.measured[j] for j in JOINTS]
        self._sync_entries()
        self._log("Cible recopiee sur la mesure.")

    def _sync_entries(self):
        if self.target is None:
            return
        for i, j in enumerate(JOINTS):
            e = self.entries[j][0]
            e.delete(0, tk.END)
            e.insert(0, "%.2f" % (self.target[i] * R2D))

    def _lock(self, v):
        self.lock_pub.publish(Bool(data=v))
        self._log("VERROU" if v else "DEVERROU")

    def _grip(self, close):
        self.grip_pub.publish(Bool(data=close))
        self._log("pince FERME" if close else "pince OUVRE")

    def _capture(self):
        if self.measured is None:
            return self._log("Rien a capturer (pas de mesure).", err=True)
        name = self.cap_name.get().strip() or "pose"
        vals = [self.measured[j] for j in JOINTS]
        xyz = self.tcp_xyz
        line = "%s: [%s]" % (name, ", ".join("%.4f" % v for v in vals))
        if xyz:
            line += "   # tcp_world_m=[%.4f, %.4f, %.4f]" % xyz
        # Remplace l'entree existante de meme nom (pas d'append => aucun doublon
        # de cle dans le fichier autoritaire) ; conserve commentaires et autres poses.
        old = []
        if os.path.exists(CAPTURE_FILE):
            old = [l for l in open(CAPTURE_FILE)
                   if l.split(":", 1)[0].strip() != name]
        old.append(line + "\n")
        with open(CAPTURE_FILE, "w") as f:
            f.write("".join(old))
        self._log("CAPTURE -> %s\n  %s" % (CAPTURE_FILE, line))

    def _log(self, msg, err=False):
        self.lbl_log.configure(text=msg, fg="#a00" if err else "#060")
        self.get_logger().info(msg)

    # ---------- refresh ----------
    def refresh(self):
        self._read_tcp()
        if self.measured is not None:
            for j in JOINTS:
                self.entries[j][1].configure(
                    text="%+7.2f° (%+.4f)" % (self.measured[j] * R2D, self.measured[j]))
            self.lbl_state.configure(
                text="mesure: " + "  ".join("%s=%+.2f°" % (j.replace("joint_", "A"),
                                                                self.measured[j] * R2D) for j in JOINTS))
        if self.tcp_xyz is not None:
            x, y, z = self.tcp_xyz
            self.lbl_tcp.configure(text="TCP world : x=%+.1f  y=%+.1f  z=%+.1f  (mm)"
                                        % (x * 1000, y * 1000, z * 1000))


def main():
    rclpy.init()
    root = tk.Tk()
    node = FineJog(root)

    def tick():
        rclpy.spin_once(node, timeout_sec=0.0)
        node.refresh()
        root.after(50, tick)

    def on_close():
        try:
            node.destroy_node(); rclpy.shutdown()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(50, tick)
    root.mainloop()


if __name__ == "__main__":
    main()
