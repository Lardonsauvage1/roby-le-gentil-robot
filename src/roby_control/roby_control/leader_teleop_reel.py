"""Teleoperation CARTESIENNE du VRAI bras par le bras guide (US-023, ADR-003 option B).

Meme calcul que leader_teleop_cart (bras simule) : ancre incrementale, echelle, laisse,
IK DLS, debrayage. Seule la SORTIE change :

    bras guide -> ce noeud -> /guard/joint_trajectory -> roby_guard -> arm_controller

Le mode simulation n'est pas touche : `leader_teleop_cart` reste le noeud du bras simule,
celui-ci en herite et ne remplace que la sortie.

Ce qui differe du bras simule, et pourquoi :

- RIEN sur /joint_states. Sur la vraie stack, ce topic appartient au robot ; y publier une
  pose calculee ferait croire au garde et a MoveIt que le bras est ailleurs.
- La consigne part de la position REELLE. A l'embrayage, la pose de calcul est prise sur
  /joint_states (converti en espace modele) : le bras ne saute pas. Debraye, elle suit le
  bras en continu.
- HORIZON GLISSANT (ADR-003, precision 1). Un point par cycle au JTC donne un mouvement
  saccade : le controleur replanifie a chaque point. On envoie donc les `points_horizon`
  dernieres consignes, RETARDEES de `horizon_s`, remplacees a chaque envoi. Aucun point
  n'est extrapole : le bras suit le chemin reellement demande, avec `horizon_s` de retard.
  Si plus rien n'arrive, il s'arrete au plus `horizon_s` plus loin -- jamais au-dela de
  la derniere consigne calculee.
- GARDE OBLIGATOIRE (ADR-003, precision 3). Embrayage refuse, avec son motif, si le garde
  ne publie pas son etat, s'il est gele, s'il tourne sans MoveIt, ou si un autre noeud
  (le modele IA) publie deja vers lui.
- RETOUR EN LIBRE (US-023) : flux du guide perdu, /joint_states perdu, garde gele ou
  disparu, bras qui ne suit plus (derive), saut de consigne en un cycle (couture d'un axe
  qui fait le tour). Le suivi ne reprend jamais seul : il faut reembrayer.
- PLAFONDS : vitesse cartesienne <= `vitesse_max_reel_m_s`, vitesse de rotation de
  l'outil <= `vitesse_ori_max_rad_s` (l'orientation est recopiee a 1:1, l'echelle ne
  la reduit pas), vitesse articulaire <= `vitesse_art_max_rad_s` (la base, recopiee du
  guide, echappe a la limite cartesienne), echelle <= `echelle_max_reel`.
- PLANCHER VIRTUEL : la cible ne descend pas sous le plancher du garde + `plancher_marge_m`
  (meme point, meme formule que roby_guard). Le bras glisse au ras de la table au lieu
  de faire geler le garde ; le garde reste le dernier filet.
- Espace ROBOT vs MODELE : /joint_states et le garde parlent en consigne COMPENSEE
  (ROBY_J3_SCALE, meme convention que roby_infer_cart et roby_guard). L'IK travaille en
  espace modele. La conversion est faite a l'entree et a la sortie.

La pince n'est pas pilotee (le guide n'a pas d'axe pince dans ce noeud).

    bash ~/roby_leader_teleop_reel.sh
"""

from __future__ import annotations

import collections
import math
import os
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from roby_control.leader_teleop_cart import J, TeleopCart
from roby_tool_pickup import fkT, rotvec   # chemin ajoute par leader_teleop_cart
import roby_oracle as O                    # plancher : la meme fonction que le garde

TOPIC_GARDE = "/guard/joint_trajectory"

# Meme convention que roby_infer_cart / roby_guard : opt-in par variable d'environnement.
J3_SCALE = float(os.environ.get("ROBY_J3_SCALE", "0") or 0)
J3_REF = float(os.environ.get("ROBY_J3_REF", "0.5237"))


def vers_modele(q, scale=None, ref=None):
    """Consigne lue sur /joint_states (compensee) -> espace modele (celui de l'IK)."""
    scale = J3_SCALE if scale is None else scale
    ref = J3_REF if ref is None else ref
    q = np.array(q, float).copy()
    if scale > 0:
        q[2] = ref + scale * (q[2] - ref)
    return q


def vers_robot(q, scale=None, ref=None):
    """Espace modele -> consigne compensee a envoyer (inverse de vers_modele)."""
    scale = J3_SCALE if scale is None else scale
    ref = J3_REF if ref is None else ref
    q = np.array(q, float).copy()
    if scale > 0:
        q[2] = ref + (q[2] - ref) / scale
    return q


def _rodrigues(rv):
    """Vecteur rotation -> matrice."""
    a = float(np.linalg.norm(rv))
    if a < 1e-12:
        return np.eye(3)
    k = np.asarray(rv, float) / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def _enrouler(a):
    """Angle ramene dans [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def interp_hist(hist, t):
    """Consigne a l'instant t, interpolee dans l'historique [(t, q), ...].

    Avant le premier echantillon : le premier (l'ancre, donc la position reelle a
    l'embrayage). Apres le dernier : le dernier. Jamais d'extrapolation.
    """
    ts = np.fromiter((h[0] for h in hist), float)
    qs = np.array([h[1] for h in hist], float)
    return np.array([np.interp(t, ts, qs[:, i]) for i in range(qs.shape[1])])


def horizon(hist, t, horizon_s, n):
    """Points (time_from_start, q) de l'horizon glissant envoye a l'instant t.

    Le point j (1..n) est la consigne calculee a t - horizon_s + j*horizon_s/n, a
    atteindre a t + j*horizon_s/n. Le dernier est donc la consigne la plus recente, a
    `horizon_s` d'ici.
    """
    pas = horizon_s / n
    return [(j * pas, interp_hist(hist, t - horizon_s + j * pas)) for j in range(1, n + 1)]


def lire_garde(etat, age, timeout):
    """Motif de refus d'apres /guard/status, ou "" si le garde est utilisable."""
    if etat is None or age is None or age > timeout:
        return ("garde absent (aucun /guard/status depuis %s) : lancer roby_guard"
                % ("le demarrage" if age is None else "%.1f s" % age))
    if etat.startswith("FROZEN"):
        # Motif ENTIER : il contient des espaces ("plancher : pince z=..."), un
        # decoupage au premier espace le coupait au milieu.
        raison = etat[len("FROZEN["):etat.rfind("]")] if "]" in etat else etat
        return ("garde GELE [%s] : verifier, puis REARMER LE GARDE (panneau du bras "
                "guide, ou /guard/reset)" % raison)
    if "moveit=off" in etat:
        return "garde lance SANS MoveIt (--no-moveit) : interdit pour le vrai bras"
    if not etat.startswith("OK"):
        return "etat du garde illisible : %r" % etat
    return ""


class TeleopReel(TeleopCart):
    PUBLIE_JOINT_STATES = False
    MSG_SORTIE = ("VRAI BRAS : consignes vers %s, garde obligatoire, rien sur "
                  "/joint_states." % TOPIC_GARDE)

    def __init__(self):
        super().__init__()
        # 2 points sur 80 ms (mesures du 2026-09-13). Le retard du suivi vaut ~horizon
        # + 10 ms : 130 ms avec 3 points sur 120 ms, 45 ms avec 1 point sur 40 ms, a
        # douceur egale sur le faux robot. L'horizon ne sert qu'a absorber un message
        # en retard : sur le lien filaire PC -> Pi, la reception de /joint_states n'a
        # jamais depasse 13,8 ms d'intervalle (10 ms nominal, 1 500 messages). 80 ms
        # tolere 40 ms de retard, trois fois le pire mesure. A reverifier sur le vrai
        # bras : un point unique a donne des saccades le 2026-07-14 (ADR-003).
        self.declare_parameter("horizon_s", 0.08)
        self.declare_parameter("points_horizon", 2)
        self.declare_parameter("envoi_hz", 25.0)
        self.declare_parameter("vitesse_max_reel_m_s", 0.05)
        self.declare_parameter("echelle_max_reel", 1.0)
        self.declare_parameter("seuil_derive_m", 0.03)
        self.declare_parameter("duree_derive_s", 0.5)
        self.declare_parameter("saut_max_rad", 0.10)
        self.declare_parameter("vitesse_art_max_rad_s", 0.4)
        self.declare_parameter("vitesse_ori_max_rad_s", 0.4)
        # Plancher virtuel (2026-09-13) : sans lui, l'IK emmenait la pince sous le
        # plancher du garde, qui gelait ; apres rearmement le bras etait encore a 1 mm
        # de la limite et regelait au moindre geste. `plancher_marge_garde_m` doit
        # valoir le --floor-margin du garde (0,03 par defaut).
        self.declare_parameter("plancher_virtuel", True)
        self.declare_parameter("plancher_marge_garde_m", 0.03)
        self.declare_parameter("plancher_marge_m", 0.015)
        self.declare_parameter("js_timeout_s", 0.3)
        self.declare_parameter("garde_timeout_s", 1.5)
        self.declare_parameter("maintien_s", 0.3)
        g = self.get_parameter
        self.horizon_s = float(g("horizon_s").value)
        self.n_pts = max(1, int(g("points_horizon").value))
        self.echelle_max = float(g("echelle_max_reel").value)
        self.seuil_derive = float(g("seuil_derive_m").value)
        self.duree_derive = float(g("duree_derive_s").value)
        self.saut_max = float(g("saut_max_rad").value)
        self.vart = float(g("vitesse_art_max_rad_s").value)
        self.omega_max = float(g("vitesse_ori_max_rad_s").value)
        self.plancher_virtuel = bool(g("plancher_virtuel").value)
        self.marge_garde = float(g("plancher_marge_garde_m").value)
        self.marge_plancher = float(g("plancher_marge_m").value)
        self._au_plancher = False
        self.js_timeout = float(g("js_timeout_s").value)
        self.garde_timeout = float(g("garde_timeout_s").value)
        self.maintien = float(g("maintien_s").value)
        self._div_envoi = max(1, round(1.0 / (self.dt * float(g("envoi_hz").value))))

        # Plafonds : ils s'imposent aux reglages herites du mode simulation.
        cap_v = float(g("vitesse_max_reel_m_s").value)
        if self.vmax > cap_v:
            self.get_logger().warn("vitesse_max_m_s %.3f plafonnee a %.3f m/s (vrai bras)"
                                   % (self.vmax, cap_v))
            self.vmax = cap_v
        if self.k > self.echelle_max:
            self.get_logger().warn("echelle %.3g plafonnee a %.3g (vrai bras)"
                                   % (self.k, self.echelle_max))
            self.set_parameters([Parameter("echelle", value=self.echelle_max)])
            self.k = self.echelle_max
        if self.embraye:
            # Embrayer est un acte explicite, et ici il a des preconditions.
            self.embraye = False
            self.get_logger().warn("embraye_au_demarrage ignore : embrayer par le service")

        self.q_mes = None            # derniere /joint_states du robot (espace ROBOT)
        self.t_mes = None
        self.garde_etat = None
        self.t_garde = None
        self.hist = collections.deque(maxlen=200)
        self._n_tick = 0
        self._n_envois = 0
        self._derive_depuis = None
        self._ecarts = []
        self._autres = 0
        self._prec_directs = {}
        self._g_brut = None          # angles bruts du guide (axes recopies)
        self._g_lim = None           # les memes, brides en vitesse
        self._t_cb = None

        self.pub_garde = self.create_publisher(JointTrajectory, TOPIC_GARDE, 10)
        self.pub_suivi = self.create_publisher(Float64MultiArray, "/teleop_cart/suivi", 10)
        self.create_subscription(JointState, "/joint_states", self._cb_js, 20)
        self.create_subscription(String, "/guard/status", self._cb_garde, 10)

        self.get_logger().warn(
            "VRAI BRAS : echelle 1:%.3g (max %.3g), vitesse max %.3f m/s et %.2f rad/s, "
            "horizon %d points sur %.0f ms a %.0f Hz, ROBY_J3_SCALE=%s. DEBRAYE : "
            "embrayer par /teleop_cart/embrayage une fois le garde pret."
            % (1.0 / self.k if self.k else 0, self.echelle_max, self.vmax, self.vart,
               self.n_pts,
               1000 * self.horizon_s, 1.0 / (self.dt * self._div_envoi),
               J3_SCALE if J3_SCALE > 0 else "0 (desactivee)"))

    # ------------------------------------------------------------------ entrees
    def _cb_js(self, msg):
        vus = dict(zip(msg.name, msg.position))
        if not all(n in vus for n in J):
            return
        self.q_mes = np.array([vus[n] for n in J], float)
        self.t_mes = time.monotonic()

    def _cb(self, msg):
        """Pose du guide, puis bridage des axes RECOPIES (la base) a `vitesse_art_max`.

        La limite cartesienne ne voit pas ces axes : sans bridage, une main rapide
        ferait tourner la base du vrai bras a sa vitesse. On bride l'angle du GUIDE
        tel que le voit le calcul, avant l'IK : la base et le coude restent calcules
        ensemble. Un saut brut du codeur du guide, impossible a la main, fait debrayer.
        """
        super()._cb(msg)
        if self.guide_q is None or not self.directs:
            return
        t = time.monotonic()
        brut = {i: self.guide_q[i] for i in self.directs}
        if self._g_brut is not None and self.embraye and self._t_cb is not None:
            # 5 rad/s : hors de portee d'une main ; 0,3 rad minimum pour tolerer un
            # message perdu.
            seuil = max(3 * self.saut_max, 5.0 * (t - self._t_cb))
            for i in self.directs:
                dd = abs(_enrouler(brut[i] - self._g_brut.get(i, brut[i])))
                if dd > seuil:
                    self._debrayer("le codeur du guide %s a saute de %.2f rad en un message"
                                   % (J[i], dd))
        self._g_brut = brut
        if not self.embraye or self._g_lim is None or self._t_cb is None:
            self._g_lim = dict(brut)
        else:
            pas = self.vart / max(abs(self.k), 1e-3) * (t - self._t_cb)
            for i in self.directs:
                prec = self._g_lim.get(i, brut[i])
                dd = _enrouler(brut[i] - prec)
                self._g_lim[i] = _enrouler(prec + max(-pas, min(pas, dd)))
        self._t_cb = t
        for i in self.directs:
            self.guide_q[i] = self._g_lim[i]

    def _cb_garde(self, msg):
        self.garde_etat = msg.data
        self.t_garde = time.monotonic()
        if self.embraye and msg.data.startswith("FROZEN"):
            self._debrayer("garde GELE : %s" % msg.data, maintien=False)

    def _cb_recentrage(self, msg):
        avant = self.embraye
        super()._cb_recentrage(msg)
        if avant and not self.embraye:
            self._maintenir()

    # ------------------------------------------------------------------ etat
    def _age_js(self):
        return None if self.t_mes is None else time.monotonic() - self.t_mes

    def _js_frais(self):
        a = self._age_js()
        return a is not None and a <= self.js_timeout

    def _motif_garde(self):
        age = None if self.t_garde is None else time.monotonic() - self.t_garde
        return lire_garde(self.garde_etat, age, self.garde_timeout)

    def _autres_emetteurs(self):
        return max(0, self.count_publishers(TOPIC_GARDE) - 1)

    def _motif_refus(self):
        if self.guide is None:
            return "aucune donnee du bras guide (leader_node lance ?)"
        if not self._js_frais():
            a = self._age_js()
            return ("pas de /joint_states du robot %s (stack lancee ?)"
                    % ("recu" if a is None else "depuis %.2f s" % a))
        m = self._motif_garde()
        if m:
            return m
        n = self._autres_emetteurs()
        if n:
            return ("%d autre(s) noeud(s) publie(nt) deja vers %s (modele IA ?) : "
                    "l'arreter d'abord" % (n, TOPIC_GARDE))
        return ""

    def _motif_arret(self):
        if self.dernier is not None:
            age = (self.get_clock().now() - self.dernier).nanoseconds / 1e9
            if age > self.timeout:
                return "flux du bras guide perdu depuis %.2f s" % age
        if not self._js_frais():
            return "plus de /joint_states du robot depuis %.2f s" % (self._age_js() or 0)
        m = self._motif_garde()
        if m:
            return m
        if self._n_tick % 25 == 0:
            self._autres = self._autres_emetteurs()
        if self._autres:
            return "un autre noeud publie vers %s" % TOPIC_GARDE
        return ""

    # ------------------------------------------------------------------ services
    def _srv_embrayage(self, req, resp):
        if not req.data:
            avant = self.embraye
            resp = super()._srv_embrayage(req, resp)
            if avant:
                self._maintenir()
            return resp
        if self.embraye:
            return super()._srv_embrayage(req, resp)      # simple reancrage
        motif = self._motif_refus()
        if motif:
            resp.success = False
            resp.message = "embrayage REFUSE : " + motif
            self.get_logger().warn(resp.message)
            return resp
        # L'ancre est posee sur la position REELLE du bras.
        self.q = vers_modele(self.q_mes)
        self.hist.clear()
        self.hist.append((time.monotonic(), self.q.copy()))
        self._derive_depuis = None
        self._ecarts = []
        self._autres = 0
        return super()._srv_embrayage(req, resp)

    def _ancrer(self):
        super()._ancrer()
        # Reancrer ramene la cible des axes recopies sur la consigne : ce n'est pas
        # un saut du guide.
        self._prec_directs = {}

    def _dls_base_figee(self, j, target_p, target_R, iters=8):
        """IK du bras simule, avec la vitesse de ROTATION demandee plafonnee.

        La position est deja bridee (cible <= vitesse_max * dt de la pince), mais pas
        l'orientation : elle est recopiee du guide a 1:1, quelle que soit l'echelle. Le
        2026-09-13 au banc (echelle 1:10), la main inclinait le guide a 40-75 deg/s ;
        le plafond articulaire laissait alors la solution de l'IK s'eloigner un peu plus
        a chaque cycle de la consigne bridee, jusqu'au seuil de saut -- debrayage sans
        vrai saut. Bridee ici, la cible reste a portee du bras : il suit avec retard et
        rattrape, rien n'est perdu (l'orientation est incrementale depuis l'ancre).
        """
        R_act = fkT(np.asarray(j, float))[:3, :3]
        rv = rotvec(R_act.T @ target_R)
        a = float(np.linalg.norm(rv))
        amax = self.omega_max * self.dt
        if a > amax:
            R_lim = R_act @ _rodrigues(rv * (amax / a))
            # Le point COMMANDE (bout de pince) ne doit pas bouger pour autant :
            # link_gripper est recalcule pour l'orientation bridee.
            off = np.array([self.off, 0.0, 0.0])
            target_p = target_p + (target_R - R_lim) @ off
            target_R = R_lim
        if self.plancher_virtuel:
            # PLANCHER, meme point et meme formule que le garde (link_gripper,
            # z_pick(x, y) - marge), avec `plancher_marge_m` de mieux. Jamais au-dessus
            # de la hauteur actuelle : le plancher empeche de descendre, il ne souleve
            # pas le bras tout seul.
            borne = min(self._z_plancher(target_p[0], target_p[1]),
                        float(fkT(np.asarray(j, float))[2, 3]))
            if target_p[2] < borne:
                dz = borne - float(target_p[2])
                target_p = np.array(target_p, float)
                target_p[2] = borne
                # L'ancre remonte d'autant (z ne depend pas de la rotation de base) :
                # le geste sous le plancher est perdu, comme au-dela de la laisse.
                # Sans cela, la main devait remonter de toute la profondeur enfoncee
                # (jusqu'a la laisse, 6 cm) avant que le bras ne decolle.
                if self.ancre_p is not None:
                    self.ancre_p = np.array(self.ancre_p, float)
                    self.ancre_p[2] += dz
                if not self._au_plancher:
                    self._au_plancher = True
                    self.get_logger().info("plancher virtuel : le bras glisse au ras de "
                                           "la table (%.3f m)" % borne)
            else:
                self._au_plancher = False
        return super()._dls_base_figee(j, target_p, target_R, iters)

    def _z_plancher(self, x, y):
        """Hauteur mini de link_gripper imposee par la teleop."""
        return (float(O._z_pick(float(x), float(y))) - self.marge_garde
                + self.marge_plancher)

    def _srv_pose_travail(self, req, resp):
        resp.success = False
        resp.message = ("interdit sur le vrai bras : la consigne sauterait loin du bras. "
                        "Amener le bras avec la procedure de lancement, puis embrayer.")
        return resp

    def _sur_parametres(self, params):
        from rcl_interfaces.msg import SetParametersResult
        cap = getattr(self, "echelle_max", None)
        for p in params:
            if p.name == "echelle" and cap is not None and p.value > cap:
                return SetParametersResult(
                    successful=False,
                    reason="echelle %.3g > %.3g : plafond du vrai bras" % (p.value, cap))
        return super()._sur_parametres(params)

    # ------------------------------------------------------------------ boucle
    def _tick(self):
        t = time.monotonic()
        self._n_tick += 1
        if self.embraye:
            motif = self._motif_arret()
            if motif:
                self._debrayer(motif)
        if not self.embraye:
            if self._js_frais():
                self.q = vers_modele(self.q_mes)     # debraye : on suit le vrai bras
            super()._tick()
            return
        q_avant = np.array(self.q, float)
        super()._tick()
        if not self.embraye:
            return
        if self.plancher_virtuel:
            # Filet : l'IK ne tient pas toujours sa cible au millimetre. Un pas qui
            # descend a moins de 5 mm du plancher du GARDE est refuse -- sinon c'est
            # le garde qui gele.
            p0 = fkT(q_avant)[:3, 3]
            p1 = fkT(np.asarray(self.q, float))[:3, 3]
            lim = self._z_plancher(p1[0], p1[1]) - self.marge_plancher + 0.005
            if p1[2] < lim and p1[2] < p0[2]:
                self.q = q_avant
        q_new = np.asarray(self.q, float)
        d = q_new - q_avant
        # SAUT = discontinuite de la CIBLE d'un cycle a l'autre (couture d'un axe qui
        # fait le tour, codeur du guide qui saute, solveur). Pour les axes resolus par
        # l'IK, la cible part de la consigne precedente : c'est d. Pour les axes
        # RECOPIES, la cible vient du guide et peut etre en avance sur la consigne
        # (plafond de vitesse) : on compare donc la cible a la cible precedente, sinon
        # une main simplement rapide ferait debrayer.
        saut = np.abs(d)
        for i in self.directs:
            prec = self._prec_directs.get(i)
            saut[i] = 0.0 if prec is None else abs(q_new[i] - prec)
            self._prec_directs[i] = float(q_new[i])
        if float(saut.max()) > self.saut_max:
            i = int(np.argmax(saut))
            self.q = q_avant
            self._debrayer("saut de consigne %s %.2f rad en un cycle (max %.2f) -- "
                           "couture d'un axe, codeur du guide ou solveur"
                           % (J[i], saut[i], self.saut_max))
            return
        dq = np.abs(d)
        # VITESSE ARTICULAIRE plafonnee, filet de securite des axes resolus par l'IK
        # (la limite cartesienne les tient deja). Les axes RECOPIES sont brides en
        # amont, dans _cb : les brider ici donnerait a l'IK une base qui n'est pas
        # celle pour laquelle elle a calcule le coude -- constate au banc, joint_3
        # sautait de 0,11 rad des que la main tournait vite la base.
        pas = self.vart * self.dt
        actifs = self._actifs()
        if actifs and float(dq[actifs].max()) > pas:
            q_lim = np.array(self.q, float)
            q_lim[actifs] = q_avant[actifs] + np.clip(d[actifs], -pas, pas)
            self.q = q_lim
        self.hist.append((t, np.array(self.q, float)))
        if self._n_tick % self._div_envoi == 0:
            self._envoyer(t)
        self._surveiller_derive(t)

    def _envoyer(self, t):
        jt = JointTrajectory()
        jt.joint_names = list(J)
        for tfs, q in horizon(self.hist, t, self.horizon_s, self.n_pts):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in vers_robot(q)]
            p.time_from_start = Duration(sec=int(tfs), nanosec=int((tfs % 1) * 1e9))
            jt.points.append(p)
        self.pub_garde.publish(jt)
        self._n_envois += 1

    def _surveiller_derive(self, t):
        """Le bras doit etre la ou etait la consigne `horizon_s` plus tot."""
        q_mes = vers_modele(self.q_mes)
        p_mes = self._p_outil(q_mes)
        ecart = float(np.linalg.norm(p_mes - self._p_outil(interp_hist(self.hist,
                                                                       t - self.horizon_s))))
        ecart_der = float(np.linalg.norm(p_mes - self._p_outil(self.q)))
        self._ecarts.append(ecart)
        m = Float64MultiArray()
        m.data = [1000 * ecart, 1000 * ecart_der, 1000 * (self._age_js() or 0),
                  float(self._n_envois)]
        self.pub_suivi.publish(m)
        if ecart > self.seuil_derive:
            if self._derive_depuis is None:
                self._derive_depuis = t
            elif t - self._derive_depuis > self.duree_derive:
                self._debrayer("le bras ne suit plus : %.0f mm de la consigne pendant "
                               "%.1f s" % (1000 * ecart, t - self._derive_depuis))
        else:
            self._derive_depuis = None

    # ------------------------------------------------------------------ sorties
    def _debrayer(self, motif, maintien=True):
        if not self.embraye:
            return
        self.embraye = False
        self._derive_depuis = None
        self.get_logger().error("DEBRAYE (retour en LIBRE) : %s" % motif)
        if maintien:
            self._maintenir()

    def _maintenir(self):
        """Arrete le bras LA OU IL EST, via le garde.

        Sans /joint_states frais, on n'envoie rien : une position perimee ferait bouger
        le bras. La derniere trajectoire envoyee finit de toute facon au plus
        `horizon_s` plus loin.
        """
        if not self._js_frais() or self._motif_garde():
            return
        jt = JointTrajectory()
        jt.joint_names = list(J)
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in self.q_mes]
        p.time_from_start = Duration(sec=int(self.maintien),
                                     nanosec=int((self.maintien % 1) * 1e9))
        jt.points.append(p)
        self.pub_garde.publish(jt)

    def _rapport(self):
        super()._rapport()
        if self.embraye and self._ecarts:
            e = np.array(self._ecarts) * 1000
            self.get_logger().info(
                "suivi : ecart au bras median %.1f mm, max %.1f mm | %d envois | garde %s"
                % (float(np.median(e)), float(e.max()), self._n_envois, self.garde_etat))
            self._ecarts = []


def main():
    rclpy.init()
    n = TeleopReel()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.embraye:
            n._debrayer("arret du noeud")
            time.sleep(0.1)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
