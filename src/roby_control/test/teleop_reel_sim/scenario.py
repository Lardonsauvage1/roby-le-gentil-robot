#!/usr/bin/env python3
"""Validation de la teleop du VRAI bras sur le FAUX robot (US-023, etape simulation).

Chaine complete, identique a la vraie stack sauf le materiel :

    faux bras guide (ce script) -> leader_teleop_reel -> roby_guard (+ MoveIt, scene)
        -> arm_controller (JTC open-loop) -> mock_components -> /joint_states

Mesures : retard du suivi, ecart au bras, latence du garde, consignes bridees par le
garde, vitesse articulaire. Pannes injectees : guide muet, descente vers la table
(le garde doit geler et la teleop debrayer), second emetteur vers le garde.

    ROS_DOMAIN_ID=43 ROBY_J3_SCALE=0.9299 python3 scenario.py [--sortie mesures.json]

Refuse le domaine 42.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

if os.environ.get("ROS_DOMAIN_ID", "0") == "42":
    sys.exit("domaine 42 = vraie stack : ce scenario ne tourne que sur le faux robot")

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray, String  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402

from roby_control.leader_mapping import charger  # noqa: E402
from roby_control.leader_teleop_cart import J  # noqa: E402
from roby_control.leader_teleop_reel import TOPIC_GARDE, vers_modele  # noqa: E402

sys.path.insert(0, os.path.expanduser("~/ros2_ws/tools/pc"))
from roby_tool_pickup import fkT  # noqa: E402
import roby_oracle as O  # noqa: E402

POSES = os.path.expanduser("~/roby_poses.yaml")


def tcp(q_robot):
    return fkT(vers_modele(q_robot))[:3, 3]


class Scenario(Node):
    def __init__(self):
        super().__init__("scenario_teleop_reel")
        self.cal = charger()
        self.lock = threading.Lock()
        self.js = []            # (t, q robot)
        self.js_stamp = []      # (horodatage du message, q) : pour les accelerations
        self.entree = []        # (t, msg) vers le garde
        self.sortie = []        # (t, msg) du garde vers le controleur
        self.statuts = []       # (t, str)
        self.suivi = []         # (t, [ecart_retard_mm, ecart_der_mm, age_js_ms, n])
        self.etat = None
        self.q_guide = None     # pose Roby equivalente au guide (espace modele)
        self.guide_on = False
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.create_subscription(JointTrajectory, TOPIC_GARDE,
                                 lambda m: self._log(self.entree, m), 50)
        self.create_subscription(JointTrajectory, "/arm_controller/joint_trajectory",
                                 lambda m: self._log(self.sortie, m), 50)
        self.create_subscription(String, "/guard/status",
                                 lambda m: self._log(self.statuts, m.data), 10)
        self.create_subscription(Float64MultiArray, "/teleop_cart/suivi",
                                 lambda m: self._log(self.suivi, list(m.data)), 50)
        self.create_subscription(Float64MultiArray, "/teleop_cart/etat", self._etat, 10)
        self.p_leader = self.create_publisher(JointState, "/leader/joint_states", 10)
        self.p_ctrl = self.create_publisher(JointTrajectory,
                                            "/arm_controller/joint_trajectory", 10)
        self.c_emb = self.create_client(SetBool, "/teleop_cart/embrayage")
        self.c_reset = self.create_client(Trigger, "/guard/reset")
        self.create_timer(0.01, self._leader)

    def _log(self, liste, x):
        with self.lock:
            liste.append((time.monotonic(), x))

    def _js(self, m):
        vus = dict(zip(m.name, m.position))
        if all(n in vus for n in J):
            q = np.array([vus[n] for n in J])
            self._log(self.js, q)
            with self.lock:
                self.js_stamp.append((time.monotonic(),
                                      m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, q))

    def _etat(self, m):
        self.etat = list(m.data)

    def _leader(self):
        if not self.guide_on or self.q_guide is None:
            return
        m = JointState()
        par = self.cal.par_nom
        for i, nom in enumerate(J):
            j = par[nom]
            m.name.append(j.nom_leader)
            m.position.append(float(j.convertir_inverse(float(self.q_guide[i]))[0]))
        self.p_leader.publish(m)

    # --------------------------------------------------------------- outils
    def q_mes(self):
        with self.lock:
            return None if not self.js else self.js[-1][1].copy()

    def embraye(self):
        return bool(self.etat and self.etat[1] > 0.5)

    def appeler(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        f = client.call_async(req)
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        return f.result()

    def embrayer(self, oui=True):
        return self.appeler(self.c_emb, SetBool.Request(data=oui))

    def statut(self):
        with self.lock:
            return self.statuts[-1][1] if self.statuts else None


def direction_descente(g, eps=1e-5):
    """Pas articulaire (joint_2, 3, 5) qui fait descendre le TCP sans changer
    l'orientation de l'outil. Norme 1."""
    from roby_tool_pickup import rotvec
    T0 = fkT(g)
    cols = []
    for i in (1, 2, 4):
        T = fkT(g + eps * np.eye(5)[i])
        cols.append(np.concatenate([[(T[2, 3] - T0[2, 3]) / eps],
                                    rotvec(T0[:3, :3].T @ T[:3, :3]) / eps]))
    A = np.array(cols).T                       # 4 x 3
    x = np.linalg.lstsq(A, np.array([-1.0, 0, 0, 0]), rcond=None)[0]
    d = np.zeros(5)
    d[[1, 2, 4]] = x / np.linalg.norm(x)
    return d


def attendre(pred, max_s, pas=0.02):
    t0 = time.monotonic()
    while time.monotonic() - t0 < max_s:
        if pred():
            return time.monotonic() - t0
        time.sleep(pas)
    return None


def compteurs(statut):
    out = {}
    for tok in (statut or "").split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            if v.isdigit():
                out[k] = int(v)
    return out


def fenetre(liste, t0, t1):
    return [(t, x) for t, x in liste if t0 <= t <= t1]


def analyser(s, t0, t1):
    """Retard, ecarts, latence du garde et vitesses sur la fenetre [t0, t1]."""
    with s.lock:
        ent = fenetre(s.entree, t0, t1)
        sor = fenetre(s.sortie, t0, t1)
        js = fenetre(s.js, t0, t1)
        sv = fenetre(s.suivi, t0, t1)
    r = {"n_envois": len(ent), "n_sorties_garde": len(sor)}
    # Latence du garde : chaque entree -> la premiere sortie qui la suit (il en publie
    # une par entree, dans l'ordre).
    ts = np.array([t for t, _ in sor])
    lat = []
    for te, _ in ent:
        k = int(np.searchsorted(ts, te))
        if k < len(ts) and ts[k] - te < 0.1:
            lat.append(1000 * (ts[k] - te))
    if lat:
        r["latence_garde_ms"] = {"mediane": float(np.median(lat)),
                                 "p95": float(np.percentile(lat, 95)),
                                 "max": float(np.max(lat))}
    # Retard : decalage qui aligne le mieux la trajectoire mesuree du TCP sur la
    # consigne la plus recente (dernier point de chaque envoi).
    if len(ent) > 20 and len(js) > 50:
        tc = np.array([t for t, _ in ent])
        pc = np.array([tcp(m.points[-1].positions) for _, m in ent])
        tm = np.array([t for t, _ in js])
        pm = np.array([tcp(q) for _, q in js])
        best = None
        for tau in np.arange(0.0, 0.6, 0.005):
            ok = (tm - tau >= tc[0]) & (tm - tau <= tc[-1])
            if ok.sum() < 20:
                continue
            ref = np.stack([np.interp(tm[ok] - tau, tc, pc[:, i]) for i in range(3)], 1)
            e = float(np.mean(np.linalg.norm(pm[ok] - ref, axis=1)))
            if best is None or e < best[1]:
                best = (tau, e)
        ref0 = np.stack([np.interp(tm, tc, pc[:, i]) for i in range(3)], 1)
        e0 = np.linalg.norm(pm - ref0, axis=1) * 1000
        r["retard_ms"] = 1000 * best[0] if best else None
        r["ecart_residuel_apres_retard_mm"] = 1000 * best[1] if best else None
        r["ecart_a_la_consigne_mm"] = {"mediane": float(np.median(e0)),
                                       "max": float(np.max(e0))}
        amp = np.ptp(pm, axis=0) * 1000
        r["amplitude_tcp_mm"] = [float(a) for a in amp]
        dt = np.diff(tm)
        qm = np.array([q for _, q in js])
        ok = dt > 1e-3
        v = np.abs(np.diff(qm, axis=0)[ok] / dt[ok, None])
        r["vitesse_art_max_rad_s"] = {J[i]: float(np.percentile(v[:, i], 99))
                                      for i in range(5)}
    # A-COUPS : acceleration articulaire de la consigne suivie par le controleur,
    # calculee sur les horodatages des messages (pas sur l'heure de reception).
    with s.lock:
        st = [(ts, q) for tr, ts, q in s.js_stamp if t0 <= tr <= t1]
    if len(st) > 50:
        ts = np.array([x[0] for x in st])
        qs = np.array([x[1] for x in st])
        _, idx = np.unique(ts, return_index=True)
        ts, qs = ts[idx], qs[idx]
        v = np.diff(qs, axis=0) / np.diff(ts)[:, None]
        acc = np.abs(np.diff(v, axis=0)) / np.diff(ts[1:])[:, None]
        r["acceleration_art_rad_s2"] = {J[i]: {"p50": float(np.percentile(acc[:, i], 50)),
                                               "p99": float(np.percentile(acc[:, i], 99))}
                                        for i in (0, 1, 2, 4)}
    if sv:
        e = np.array([x[0] for _, x in sv])
        r["suivi_ecart_retard_mm"] = {"mediane": float(np.median(e)), "max": float(e.max())}
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sortie", default="")
    ap.add_argument("--duree", type=float, default=20.0)
    a = ap.parse_args()

    rclpy.init()
    s = Scenario()
    ex = MultiThreadedExecutor()
    ex.add_node(s)
    threading.Thread(target=ex.spin, daemon=True).start()
    rapport = {"domaine": os.environ.get("ROS_DOMAIN_ID"),
               "ROBY_J3_SCALE": os.environ.get("ROBY_J3_SCALE")}

    def etape(nom, **kw):
        rapport.setdefault("etapes", []).append(dict(nom=nom, **kw))
        print("[%s] %s" % (nom, json.dumps(kw, ensure_ascii=False, default=float)),
              flush=True)

    try:
        # 1. Faux robot en pose de depart (directement au controleur : c'est le FAUX).
        assert attendre(lambda: s.q_mes() is not None, 30), "pas de /joint_states"
        depart = np.array(yaml.safe_load(open(POSES))["depart_infer_comp"], float)
        jt = JointTrajectory(joint_names=list(J))
        jt.points = [JointTrajectoryPoint(positions=[float(v) for v in depart],
                                          time_from_start=Duration(sec=4))]
        s.p_ctrl.publish(jt)
        assert attendre(lambda: np.allclose(s.q_mes(), depart, atol=1e-3), 8), \
            "faux robot pas en pose de depart"
        etape("pose_depart", q=depart.tolist(), tcp_m=tcp(depart).tolist())

        # 2. Guide immobile sur la pose du bras, garde pret.
        s.q_guide = vers_modele(s.q_mes())
        s.guide_on = True
        assert attendre(lambda: (s.statut() or "").startswith("OK"), 20), \
            "garde absent : %s" % s.statut()
        time.sleep(1.0)

        # 3. Second emetteur vers le garde -> embrayage refuse.
        autre = s.create_publisher(JointTrajectory, TOPIC_GARDE, 10)
        time.sleep(1.0)
        r = s.embrayer()
        etape("refus_second_emetteur", succes=r.success, message=r.message)
        s.destroy_publisher(autre)
        time.sleep(1.0)

        # 4. Embrayage : le bras ne doit pas bouger.
        q_avant = s.q_mes()
        r = s.embrayer()
        assert r and r.success, r
        time.sleep(1.0)
        etape("embrayage", message=r.message,
              deplacement_mm=1000 * float(np.linalg.norm(tcp(s.q_mes()) - tcp(q_avant))))

        # 5. Mouvement lent et ample en espace libre.
        g0 = s.q_guide.copy()
        st0 = compteurs(s.statut())
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.duree and s.embraye():
            t = time.monotonic() - t0
            s.q_guide = g0 + np.array([0.3 * math.sin(2 * math.pi * t / 8),
                                       0.15 * math.sin(2 * math.pi * t / 6),
                                       0.15 * math.sin(2 * math.pi * t / 5), 0.0, 0.0])
            time.sleep(0.01)
        t1 = time.monotonic()
        st1 = compteurs(s.statut())
        res = analyser(s, t0 + 1.0, t1)
        res.update(embraye_tout_du_long=s.embraye(), statut_garde=s.statut(),
                   bridages_garde=st1.get("clamp", 0) - st0.get("clamp", 0),
                   passages_garde=st1.get("pass", 0) - st0.get("pass", 0))
        etape("mouvement_libre", duree_s=round(t1 - t0, 1), **res)
        # la main revient doucement au point de depart
        g1 = s.q_guide.copy()
        for k in range(101):
            s.q_guide = g1 + (g0 - g1) * k / 100
            time.sleep(0.02)

        # 6. Debrayage : le bras s'arrete.
        r = s.embrayer(False)
        time.sleep(0.4)
        qa = s.q_mes()
        s.q_guide = g0 + np.array([0.2, 0, 0, 0, 0])
        time.sleep(1.0)
        etape("debrayage", message=r.message, bouge_apres_mm=1000 * float(
            np.linalg.norm(tcp(s.q_mes()) - tcp(qa))))
        s.q_guide = vers_modele(s.q_mes())
        time.sleep(0.5)

        # 7. Guide muet -> retour en LIBRE.
        assert s.embrayer().success
        time.sleep(0.5)
        s.guide_on = False
        tm = time.monotonic()
        dt = attendre(lambda: not s.embraye(), 3)
        # etat n'est plus publie sans guide : on relit apres reprise du flux
        s.q_guide = vers_modele(s.q_mes())
        s.guide_on = True
        time.sleep(0.3)
        etape("guide_muet", debraye=not s.embraye(),
              delai_s=None if dt is None else round(time.monotonic() - tm, 2))
        time.sleep(0.5)
        etape("pas_de_reprise_seule", embraye=s.embraye())

        # 8. Descente vers la table. Depuis le plancher virtuel de la teleop (2026-09-13),
        # le bras doit s'arreter AU-DESSUS du plancher du garde (+2,5 cm) sans le faire
        # geler : on descend tant que le bras descend encore, puis on mesure.
        assert s.embrayer().success
        time.sleep(0.5)
        # Descente a ORIENTATION CONSTANTE, comme une main qui descend la pince sans
        # la tourner. (Une premiere version inclinait le poignet du guide : joint_5
        # arrivait en butee et c'est le controle de saut qui debrayait, pas le garde.)
        g = s.q_guide.copy()
        t0 = time.monotonic()
        z_min, t_min = float("inf"), t0
        while s.embraye() and time.monotonic() - t0 < 40:
            g = g + direction_descente(g) * 0.1 * 0.02
            s.q_guide = g
            time.sleep(0.02)
            z = float(tcp(s.q_mes())[2])
            if z < z_min - 0.001:
                z_min, t_min = z, time.monotonic()
            elif time.monotonic() - t_min > 3.0:
                break                              # plus de descente depuis 3 s
        t1 = time.monotonic()
        qf = s.q_mes()
        p = tcp(qf)
        plancher_garde = float(O._z_pick(p[0], p[1])) - 0.03
        time.sleep(0.5)
        etape("descente_table", embraye=s.embraye(), apres_s=round(t1 - t0, 1),
              statut_garde=s.statut(), tcp_z_min_m=z_min,
              plancher_garde_m=plancher_garde,
              marge_au_plancher_garde_mm=1000 * (z_min - plancher_garde))
        s.embrayer(False)
        r = s.appeler(s.c_reset, Trigger.Request())
        etape("reset_garde", message=r.message if r else None)
    finally:
        s.guide_on = False
        if s.embraye():
            s.embrayer(False)
        if a.sortie:
            with open(a.sortie, "w") as f:
                json.dump(rapport, f, indent=2, ensure_ascii=False, default=float)
        ex.shutdown()
        s.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
