#!/usr/bin/env python3
"""Panneau de jog manuel : verrou tete (CH2) + pince (CH3).
Publie /head_lock et /gripper (std_msgs/Bool) vers les noeuds du Pi5.
  /head_lock : true=VERROU(75 deg)  false=DEVERROU(50 deg)
  /gripper   : true=FERMER(55 deg)  false=OUVRIR(120 deg)
Aucun mouvement des axes : juste les 2 servos.
Lancer: ~/roby_gripper_jog.sh
"""
import threading
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class Pub(Node):
    def __init__(self):
        super().__init__("roby_gripper_jog")
        self.lock = self.create_publisher(Bool, "/head_lock", 10)
        self.grip = self.create_publisher(Bool, "/gripper", 10)

    def send(self, pub, val):
        m = Bool(); m.data = val
        # publier plusieurs fois (robustesse discovery DDS)
        for _ in range(3):
            pub.publish(m)


def main():
    rclpy.init()
    n = Pub()
    threading.Thread(target=lambda: rclpy.spin(n), daemon=True).start()

    root = tk.Tk()
    root.title("Roby - Jog verrou + pince")
    root.configure(padx=16, pady=16)
    status = tk.StringVar(value="Pret. (verrou tete + pince)")

    def act(pub, val, txt):
        n.send(pub, val)
        status.set(txt)

    big = dict(width=22, height=2, font=("Sans", 13, "bold"))

    tk.Label(root, text="VERROU TETE (CH2)", font=("Sans", 11, "bold")).grid(row=0, column=0, columnspan=2, pady=(0, 4))
    tk.Button(root, text="VERROUILLER (75)", bg="#c8e6c9",
              command=lambda: act(n.lock, True, "head_lock = VERROU (75 deg)"), **big).grid(row=1, column=0, padx=4, pady=4)
    tk.Button(root, text="DEVERROUILLER (50)", bg="#ffe0b2",
              command=lambda: act(n.lock, False, "head_lock = DEVERROU (50 deg)"), **big).grid(row=1, column=1, padx=4, pady=4)

    tk.Label(root, text="PINCE (CH3)", font=("Sans", 11, "bold")).grid(row=2, column=0, columnspan=2, pady=(12, 4))
    tk.Button(root, text="FERMER (55)", bg="#bbdefb",
              command=lambda: act(n.grip, True, "gripper = FERMER (55 deg)"), **big).grid(row=3, column=0, padx=4, pady=4)
    tk.Button(root, text="OUVRIR (120)", bg="#bbdefb",
              command=lambda: act(n.grip, False, "gripper = OUVRIR (120 deg)"), **big).grid(row=3, column=1, padx=4, pady=4)

    tk.Label(root, textvariable=status, fg="#333", font=("Sans", 10)).grid(row=4, column=0, columnspan=2, pady=(14, 0))

    root.mainloop()
    rclpy.shutdown()


main()
