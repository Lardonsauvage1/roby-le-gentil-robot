#!/usr/bin/env python3
"""roby_gripper_monitor.py — panneau de surveillance de la PINCE pendant l'inference.

Montre CE QUE LE MODELE DEMANDE, pas seulement ce que la pince fait :

  - la valeur BRUTE continue predite (7e composante de l'action), AVANT seuillage ;
  - le seuil 0.5 et de quel cote on est ;
  - une COURBE des 60 dernieres secondes : c'est la qu'on voit si la valeur flotte
    autour du seuil (= la pince claque) ou si elle est franche ;
  - la MARGE au seuil : < 0.1 = zone de claquement, signalee en orange/rouge ;
  - le nombre de BASCULES et le temps depuis la derniere ;
  - ce qui est REELLEMENT parti au robot (/guard/gripper) pour comparer.

Lecture : une commande saine = valeur collee a 0 ou a 1. Une valeur qui traine entre
0.4 et 0.6 = le modele est INDECIS, et le seuillage transforme cette indecision en
ouvertures/fermetures parasites.

NE COMMANDE RIEN : lecture seule. Marche aussi quand roby_infer_cart tourne en DRY.
Usage : bash ~/roby_gripper_monitor.sh
"""
import collections
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

import sys as _sys, os as _os
_sys.path.insert(0, _os.path.expanduser("~"))
from roby_gripper import GRIP_HAUT, GRIP_BAS, GRIP_MILIEU

WIN = 60.0          # secondes affichees
# La "bande" affichee est desormais EXACTEMENT l'hysteresis appliquee par les
# noeuds d'inference : ce qu'on voit a l'ecran est ce qui est reellement decide.
THR = GRIP_MILIEU
NEAR = (GRIP_HAUT - GRIP_BAS) / 2.0

BG = "#1e1e1e"
FG = "#e0e0e0"
GRID = "#333333"
C_OPEN = "#4caf50"
C_CLOSE = "#e53935"
C_WARN = "#ff9800"


class Mon(Node):
    def __init__(self):
        super().__init__("roby_gripper_monitor")
        self.lock = threading.Lock()
        self.hist = collections.deque()       # (t, valeur brute)
        self.raw = None
        self.fresh = False
        self.n_fresh = 0
        self.last_fresh = None
        self.sent = None                      # dernier /guard/gripper (ce qui part au robot)
        self.flips = 0
        self.t_flip = None
        self.state = None
        self.t0 = time.monotonic()
        self.create_subscription(Float32, "/roby_infer/gripper_raw", self._raw, 20)
        self.create_subscription(Bool, "/guard/gripper", self._sent, 20)

    def _raw(self, m):
        t = time.monotonic()
        v = float(m.data)
        # convention du noeud : valeur <0 = consigne TENUE (tampon vide), >=0 = prediction NEUVE
        fresh = v >= 0.0
        if not fresh:
            v = -v - 1e-6
        st = v > THR
        with self.lock:
            self.raw = v
            self.fresh = fresh
            if fresh:
                self.n_fresh += 1
                self.last_fresh = (t, v)
            self.hist.append((t, v, fresh))
            while self.hist and t - self.hist[0][0] > WIN:
                self.hist.popleft()
            if self.state is not None and st != self.state:
                self.flips += 1
                self.t_flip = t
            self.state = st

    def _sent(self, m):
        with self.lock:
            self.sent = bool(m.data)


class Panel:
    def __init__(self, node):
        self.n = node
        self.root = tk.Tk()
        self.root.title("Roby — consigne PINCE du modele")
        self.root.configure(bg=BG)
        self.root.geometry("760x520")

        tk.Label(self.root, text="CE QUE LE MODELE DEMANDE", bg=BG, fg="#888",
                 font=("DejaVu Sans", 10)).pack(pady=(10, 0))
        self.lbl_state = tk.Label(self.root, text="—", bg=BG, fg=FG,
                                  font=("DejaVu Sans", 40, "bold"))
        self.lbl_state.pack()
        self.lbl_val = tk.Label(self.root, text="valeur brute —", bg=BG, fg=FG,
                                font=("DejaVu Sans Mono", 20))
        self.lbl_val.pack()
        self.lbl_warn = tk.Label(self.root, text="", bg=BG, fg=C_WARN,
                                 font=("DejaVu Sans", 13, "bold"))
        self.lbl_warn.pack(pady=(4, 8))

        self.cv = tk.Canvas(self.root, width=720, height=250, bg="#111", highlightthickness=0)
        self.cv.pack(padx=20)
        tk.Label(self.root,
                 text=(f"60 dernieres secondes — pointilles fins = consigne TENUE (tampon vide) ; "
                       f"trait plein + point = prediction NEUVE — ligne blanche = seuil {THR}"),
                 bg=BG, fg="#777", font=("DejaVu Sans", 9)).pack()

        self.lbl_foot = tk.Label(self.root, text="", bg=BG, fg="#aaa",
                                 font=("DejaVu Sans Mono", 11), justify="left")
        self.lbl_foot.pack(pady=8)
        self.tick()

    def tick(self):
        with self.n.lock:
            raw = self.n.raw
            hist = list(self.n.hist)
            flips = self.n.flips
            t_flip = self.n.t_flip
            sent = self.n.sent

        if raw is None:
            self.lbl_state.config(text="EN ATTENTE", fg="#666")
            self.lbl_val.config(text="aucune consigne recue")
            self.lbl_foot.config(text="Le noeud roby_infer_cart doit tourner (DRY suffit).\n"
                                      "Topic attendu : /roby_infer/gripper_raw")
        else:
            closed = raw > THR
            marge = abs(raw - THR)
            col = C_CLOSE if closed else C_OPEN
            if marge < NEAR:
                col = C_WARN
            self.lbl_state.config(text="FERMER" if closed else "OUVRIR", fg=col)
            self.lbl_val.config(text=f"valeur brute {raw:.3f}   (seuil {THR})", fg=col)
            if marge < NEAR:
                self.lbl_warn.config(
                    text=f"⚠ INDECIS — a {marge:.3f} du seuil : c'est la que la pince claque")
            else:
                self.lbl_warn.config(text="")
            d = "—" if t_flip is None else f"{time.monotonic()-t_flip:.1f} s"
            envoye = "—" if sent is None else ("FERME" if sent else "OUVRE")
            # SEULES les predictions fraiches disent ce que le modele DECIDE :
            # les maintiens (tampon vide) repetent la derniere et fausseraient la stat.
            fr = [v for (_, v, f) in hist if f]
            near = sum(1 for v in fr if abs(v - THR) < NEAR)
            pct = 100.0 * near / max(len(fr), 1)
            self.lbl_foot.config(
                text=(f"bascules : {flips}      derniere : {d}\n"
                      f"envoye au robot (/guard/gripper) : {envoye}\n"
                      f"predictions NEUVES sur 60 s : {len(fr)}  (sur {len(hist)} points publies)\n"
                      f"parmi elles, pres du seuil : {pct:.0f} %  ({near}/{max(len(fr),1)})"))
        self.draw(hist)
        self.root.after(100, self.tick)

    def draw(self, hist):
        c = self.cv
        c.delete("all")
        W, H = 720, 250
        # bande d'indecision + grille
        y_thr = H - THR * H
        c.create_rectangle(0, H - (THR + NEAR) * H, W, H - (THR - NEAR) * H,
                           fill="#2a2216", outline="")
        for f in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = H - f * H
            c.create_line(0, y, W, y, fill=GRID)
            c.create_text(22, y - 8, text=f"{f:.2f}", fill="#666",
                          font=("DejaVu Sans Mono", 8))
        c.create_line(0, y_thr, W, y_thr, fill="#ffffff", width=1, dash=(4, 3))
        if len(hist) < 2:
            c.create_text(W / 2, H / 2, text="en attente de donnees…", fill="#555",
                          font=("DejaVu Sans", 12))
            return
        t_end = hist[-1][0]
        pts = []
        for (t, v, f) in hist:
            x = W - (t_end - t) / WIN * W
            pts.append((x, H - max(0.0, min(1.0, v)) * H, v, f))
        # trait fin = consigne TENUE (maintien) ; trait epais + point = prediction NEUVE
        for i in range(1, len(pts)):
            x0, y0, v0, f0 = pts[i - 1]
            x1, y1, v1, f1 = pts[i]
            col = C_CLOSE if v1 > THR else C_OPEN
            if abs(v1 - THR) < NEAR:
                col = C_WARN
            c.create_line(x0, y0, x1, y1, fill=col, width=2 if f1 else 1,
                          dash=() if f1 else (2, 4))
        for (x, y, v, f) in pts:
            if not f:
                continue
            col = C_CLOSE if v > THR else C_OPEN
            if abs(v - THR) < NEAR:
                col = C_WARN
            c.create_oval(x - 2.5, y - 2.5, x + 2.5, y + 2.5, fill=col, outline="")
        x, y, v, f = pts[-1]
        col = C_CLOSE if v > THR else C_OPEN
        c.create_oval(x - 5, y - 5, x + 5, y + 5, outline="#ffffff", width=1, fill=col)


def main():
    rclpy.init()
    node = Mon()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    p = Panel(node)
    try:
        p.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
