"""Teleoperation CARTESIENNE a echelle variable, du bras simule par le bras guide (US-026).

La main commande une POSITION du TCP, pas des articulations, et un facteur d'echelle
arbitre entre amplitude et precision -- l'architecture des robots chirurgicaux.

    P_roby = P_ancre_roby + k * (P_guide - P_ancre_guide)

La forme INCREMENTALE n'est pas un detail : c'est elle qui rend possibles le debrayage
et le changement d'echelle a chaud. Changer k reancre au passage, donc rien ne saute.
En commande absolue, passer de 1:1 a 1:5 teleporterait le bras.

Debrayage : a 1:5, l'espace du guide ne couvre plus qu'un cinquieme de celui de Roby.
On relache, on ramene le guide au centre, on reembraye -- comme on souleve une souris.
Sans lui, les petits rapports sont inutilisables.

Orientation : Roby a 5 axes, donc 5 degres de liberte, alors que position (3) et
orientation (3) en demandent 6. Il en manque un, structurellement. `w_ori` arbitre :
bas = la position passe d'abord, l'orientation suit du mieux qu'elle peut. L'ecart
d'orientation restant est MESURE et publie -- un suivi qui ne suit plus doit se voir.

Ce noeud n'anime que le modele. Aucun message vers le vrai bras.

    bash ~/roby_leader_teleop_cart.sh
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import SetBool

from roby_control.leader_mapping import charger

sys.path.insert(0, os.path.expanduser("~/ros2_ws/tools/pc"))
from roby_tool_pickup import LIMITS, Rz, dls, fkT, jac, rotvec   # noqa: E402

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


class TeleopCart(Node):
    def __init__(self):
        super().__init__("leader_teleop_cart")
        self.declare_parameter("echelle", 1.0)
        # POSITION PRIORITAIRE (choix de Sam, 2026-09-09). Roby a 5 axes pour un
        # probleme qui en demande 6 : quelque chose doit ceder, et c'est l'orientation.
        # Une petite valeur plutot que zero : a zero exact le solveur ne tient plus du
        # tout l'orientation et le poignet peut partir n'importe ou.
        self.declare_parameter("w_ori", 0.05)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("timeout_s", 0.5)
        self.declare_parameter("vitesse_max_m_s", 0.25)
        self.declare_parameter("calib_file", "")
        # Depart DEBRAYE : embrayer est un acte explicite, jamais un etat par defaut.
        # Meme regle que le couple du guide (US-018).
        self.declare_parameter("embraye_au_demarrage", False)
        # Seuil de proximite de singularite (plus petite valeur singuliere de J).
        # L'amortissement `lam` du DLS masque les singularites en ralentissant en
        # silence : sans ce signal, l'operateur croit que le robot n'obeit plus.
        self.declare_parameter("seuil_singularite", 0.02)
        # BASE HORS CINEMATIQUE (idee de Sam, 2026-09-09) : joint_1 n'est plus resolu
        # par l'IK, sa valeur est recopiee de celle du guide avec le meme rapport de
        # mouvement. Deux gains : la base devient previsible (un degre de guide donne
        # k degres de robot, toujours), et elle sort du probleme sur-contraint -- il
        # reste 4 axes pour 3 positions + orientation au mieux, au lieu de 5 pour 6.
        self.declare_parameter("base_directe", True)
        # Axes RECOPIES du guide au lieu d'etre resolus par l'IK, numerotes 1..5.
        # Chacun retire une inconnue au solveur ET une demande : un axe recopie devient
        # previsible (un degre de guide = k degres de robot, toujours), et l'IK n'a plus
        # a arbitrer pour lui. Avec [1, 5] il reste 3 axes pour 3 positions.
        self.declare_parameter("axes_directs", [1, 5])
        # POINT DE COMMANDE, en metres le long de l'axe de l'outil depuis link_gripper.
        # fkT() s'arrete a link_gripper ; le bout de pince (frame `tcp`) est 10 cm plus
        # loin. Sans cet offset, l'operateur place la croix sur un objet alors que le
        # point reellement commande est 10 cm en arriere -- decalage constant constate
        # le 2026-09-09. Mettre 0.0 pour revenir au point de commande historique du
        # projet (link_gripper), celui qu'utilisent l'oracle et roby_tool_pickup.
        self.declare_parameter("offset_tcp_m", 0.10)
        # LAISSE : distance maximale entre la cible demandee et le TCP reellement
        # atteint. Sans elle, un bras bloque (butee, singularite) laisse la commande
        # INTEGRER indefiniment le geste : constate le 2026-09-09, joint_3 en butee et
        # une cible partie a 87 cm au-dessus de la pince. L'operateur perd alors toute
        # correspondance, et il n'y a aucun retour en arriere possible.
        # L'exces n'est pas accumule mais JETE : quand le bras se libere, le suivi
        # reprend depuis la ou il est, sans rattrapage brutal.
        self.declare_parameter("laisse_m", 0.06)

        self._chemin = self.get_parameter("calib_file").value or None
        self.cal = charger(self._chemin)
        self._mtime = self._mtime_calib()
        self.k = float(self.get_parameter("echelle").value)
        self.w_ori = float(self.get_parameter("w_ori").value)
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.vmax = float(self.get_parameter("vitesse_max_m_s").value)
        self.seuil_sing = float(self.get_parameter("seuil_singularite").value)
        self.base_directe = bool(self.get_parameter("base_directe").value)
        self.off = float(self.get_parameter("offset_tcp_m").value)
        self.laisse = float(self.get_parameter("laisse_m").value)
        self.directs = sorted({int(a) - 1 for a in
                               self.get_parameter("axes_directs").value
                               if 1 <= int(a) <= 5})

        self.q = np.array([0.0, 0.25, -1.175, 0.0, 0.0])   # pose simulee courante
        self.guide = None            # (p, R) du guide, derniere lecture
        self.guide_q1 = None         # angle de base du guide, converti
        self.guide_q = None          # les 5 angles du guide, convertis
        self.ancre_qg = {}           # angles du guide au moment de l'ancrage
        self.ancre_qr = {}           # angles du robot au moment de l'ancrage
        self.ancre_guide = None
        self.ancre_p = None
        self.ancre_q1_guide = None
        self.ancre_q1_roby = None
        self.ancre_R_guide = None
        self.ancre_R_roby = None
        self.ancre_aff = None
        self.dernier = None
        self.embraye = bool(self.get_parameter("embraye_au_demarrage").value)
        self._clamps = set()
        self._sing = False
        self._diverge = False
        self._tcp_ancre = None
        self._verif_saut = 0
        self._tcp_ancre = None
        self._verif_saut = 0

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_etat = self.create_publisher(Float64MultiArray,
                                              "/teleop_cart/etat", 10)
        # On publie sur /oracle_debug : c'est le topic de marqueurs deja branche dans
        # le viewer.rviz du projet. Rien a configurer cote RViz.
        # QoS DURABLE : l'affichage de marqueurs du viewer.rviz du projet est configure
        # en transient_local. Un editeur volatile est refuse en silence -- RViz ne dit
        # rien, il n'affiche simplement jamais rien. Le seul indice est un WARN cote
        # editeur : "requesting incompatible QoS".
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile
        self.pub_marq = self.create_publisher(
            MarkerArray, "/oracle_debug",
            QoSProfile(depth=5, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.cible_monde = None
        self.cible_brute = None
        self.create_subscription(JointState, "/leader/joint_states", self._cb, 20)
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile
        self.recentrage = False
        self.create_subscription(
            Bool, "/leader/recentrage", self._cb_recentrage,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SetBool, "/teleop_cart/embrayage", self._srv_embrayage)
        self.add_on_set_parameters_callback(self._sur_parametres)

        hz = float(self.get_parameter("publish_rate_hz").value)
        self.dt = 1.0 / hz
        self.create_timer(self.dt, self._tick)
        self.create_timer(2.0, self._rapport)
        self.create_timer(0.05, self._marqueurs)
        # Rechargement A CHAUD, comme le noeud position. Sans lui, tout reglage de la
        # calibration reste sans effet et on cherche la cause ailleurs -- vecu le
        # 2026-09-09 : trois offsets appliques d'affilee, aucun effet visible.
        self.create_timer(0.5, self._recharger_si_change)
        self.get_logger().info(
            "Teleop CARTESIENNE prete : echelle 1:%.3g, w_ori %.2f, %.0f Hz. "
            "%s. Aucun message vers le vrai robot."
            % (1.0 / self.k if self.k else 0, self.w_ori, hz,
               "EMBRAYE" if self.embraye else "DEBRAYE (embrayer pour piloter)"))

    # ------------------------------------------------------------------ entrees
    def _p_outil(self, q):
        """Position du POINT DE COMMANDE : link_gripper decale de `off` sur son axe x."""
        T = fkT(q)
        return T[:3, 3] + T[:3, :3] @ np.array([self.off, 0.0, 0.0])

    def _cb(self, msg):
        """Pose du guide -> (position, rotation) dans le repere de Roby.

        Source PROVISOIRE : angles guide -> angles Roby -> FK de Roby. Elle ne demande
        aucune mesure, mais le geste n'est pas geometriquement fidele -- 10 cm de main
        ne font pas 10 cm de reference, les deux bras n'ayant pas les memes proportions.
        A remplacer par la vraie FK du guide des que ses longueurs seront relevees :
        c'est le SEUL endroit a changer.
        """
        vus = dict(zip(msg.name, msg.position))
        angles = []
        for nom in J:
            j = next((x for x in self.cal.joints if x.nom_urdf == nom), None)
            if j is None or j.nom_leader not in vus:
                return
            angles.append(j.convertir(vus[j.nom_leader])[0])
        T = fkT(angles)
        # PAS d'offset d'outil du cote guide. L'appliquer aussi ici lui allonge le bras
        # de levier : une rotation du poignet du guide se traduirait alors en 10 cm de
        # translation demandee, et le tremblement de la main serait amplifie d'autant.
        # Le decalage n'a de sens que sur le point de commande du ROBOT ; la reference
        # du guide n'est qu'un parametrage, et sa constante est absorbee par l'ancre.
        p_out = T[:3, 3]
        self.guide_q1 = float(angles[0])
        self.guide_q = [float(a) for a in angles]
        if self.base_directe:
            # DANS LE PLAN DU BRAS. Quand la base est pilotee a part, la position de
            # reference doit etre exprimee APRES joint_1, sinon elle porte encore la
            # rotation de la base -- et comme le zero de la base est a -180 deg (guide
            # monte a l'envers), le repere de reference est retourne d'un demi-tour
            # alors que la base, elle, est forcee ailleurs. Les deux se contredisent et
            # TOUS LES AXES paraissent inverses. Vecu le 2026-09-09.
            Rmb = Rz(-self.guide_q1)
            self.guide = (Rmb @ p_out, Rmb @ T[:3, :3])
        else:
            self.guide = (p_out.copy(), T[:3, :3].copy())
        self.dernier = self.get_clock().now()

    def _cb_recentrage(self, msg):
        """Le guide rejoint sa pose de reference : on DEBRAYE.

        Recentrer est un geste de remise en ordre -- on repart d'un etat propre. Le
        grand bras ne doit donc pas suivre le trajet du petit, et il ne doit pas non
        plus repartir tout seul a la fin : c'est a l'operateur de decider quand le
        suivi reprend, en embrayant.

        Auparavant on se contentait de figer puis de reancrer ; le suivi reprenait
        alors sans que personne ne l'ait demande.
        """
        avant = self.recentrage
        self.recentrage = bool(msg.data)
        if self.recentrage and not avant:
            if self.embraye:
                self.embraye = False
                self.get_logger().warn(
                    "recentrage du guide : DEBRAYE (embrayer pour reprendre le suivi)")
            else:
                self.get_logger().warn("recentrage du guide : grand bras fige")

    # ------------------------------------------------------------------ services
    def _srv_embrayage(self, req, resp):
        if req.data:
            if self.guide is None:
                resp.success = False
                resp.message = "aucune donnee du guide : embrayage refuse"
                return resp
            self._ancrer()
            self._tcp_ancre = self._p_outil(self.q).copy()
            self._verif_saut = 12          # on surveille les 12 premiers cycles
            self.embraye = True
            resp.message = ("EMBRAYE a l'echelle 1:%.3g — les deux ancres sont posees, "
                            "le robot n'a pas bouge" % (1.0 / self.k if self.k else 0))
        else:
            self.embraye = False
            resp.message = "DEBRAYE : le robot est fige, le guide est libre"
        resp.success = True
        self.get_logger().warn("/teleop_cart/embrayage -> %s" % resp.message)
        return resp

    def _ancrer(self):
        """Repose les deux ancres sur l'etat courant. C'est TOUT le debrayage, et c'est
        aussi ce qui rend le changement d'echelle sans a-coup."""
        self.ancre_guide = self.guide[0].copy()
        if self.base_directe:
            # Meme repere des deux cotes : l'ancre du robot est prise APRES joint_1.
            self.ancre_p = Rz(-float(self.q[0])) @ self._p_outil(self.q)
        else:
            self.ancre_p = self._p_outil(self.q).copy()
        self.ancre_q1_guide = self.guide_q1
        self.ancre_q1_roby = float(self.q[0])
        # ORIENTATION ancree, comme la position. Sans cela elle reste ABSOLUE : tourner
        # le poignet du guide pendant un debrayage fait sauter le bras des qu'on
        # reembraye, puisque l'IK se met a rattraper une orientation qui a change sans
        # lui. Mesure du 2026-09-09 : 10,4 mm de saut sur un debrayage ordinaire.
        # DEUXIEME ANCRE, pour l'AFFICHAGE seulement. Celle de commande est deplacee
        # par la laisse pour empecher la divergence ; celle-ci ne bouge jamais entre
        # deux ancrages. La croix montre donc ou la MAIN demande, pas ou le bras a
        # bien voulu aller -- sans quoi elle reste collee au bras et ne signale plus
        # rien. Les deux besoins sont contradictoires avec une seule ancre.
        self.ancre_aff = self.ancre_p.copy()
        self.ancre_R_guide = self.guide[1].copy()
        T = fkT(self.q)
        self.ancre_R_roby = (Rz(-float(self.q[0])) @ T[:3, :3] if
                             (self.base_directe and 0 in self.directs)
                             else T[:3, :3].copy())
        self.ancre_qg = {i: self.guide_q[i] for i in self.directs}
        self.ancre_qr = {i: float(self.q[i]) for i in self.directs}

    def _sur_parametres(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "echelle":
                if not 0.01 <= p.value <= 5.0:
                    return SetParametersResult(successful=False,
                                               reason="echelle hors de 0,01..5")
                self.k = float(p.value)
                if self.embraye and self.guide is not None:
                    self._ancrer()      # sans reancrage, le bras SAUTE
                self.get_logger().warn("echelle -> 1:%.3g (reancre)" % (1.0 / self.k))
            elif p.name == "laisse_m":
                self.laisse = float(p.value)
                self.get_logger().warn("laisse -> %.3f m" % self.laisse)
            elif p.name == "axes_directs":
                self.directs = sorted({int(a) - 1 for a in p.value if 1 <= int(a) <= 5})
                if self.embraye and self.guide is not None:
                    self._ancrer()
                self.get_logger().warn(
                    "axes recopies du guide : %s (reancre)"
                    % (", ".join(J[i] for i in self.directs) or "aucun"))
            elif p.name == "base_directe":
                self.base_directe = bool(p.value)
                if self.embraye and self.guide is not None:
                    self._ancrer()
                self.get_logger().warn(
                    "base %s (reancre)"
                    % ("RECOPIEE directement du guide" if self.base_directe
                       else "resolue par l'IK"))
            elif p.name == "w_ori":
                self.w_ori = float(p.value)
                self.get_logger().warn("w_ori -> %.2f" % self.w_ori)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ boucle
    def _tick(self):
        if self.guide is None:
            return
        if self.dernier is not None:
            age = (self.get_clock().now() - self.dernier).nanoseconds / 1e9
            if age > self.timeout:
                # Flux perdu : on GELE. Poursuivre la derniere consigne ferait
                # continuer le bras tout seul.
                self._publier()
                return
        if self.embraye and self.ancre_guide is not None and not self.recentrage:
            p_guide, R_guide = self.guide
            cible = self.ancre_p + self.k * (p_guide - self.ancre_guide)
            if self.ancre_R_guide is not None:
                # Orientation INCREMENTALE : on applique au robot la rotation que le
                # guide a subie depuis l'ancrage, et non son orientation absolue.
                R_guide = self.ancre_R_roby @ (self.ancre_R_guide.T @ R_guide)

            depart = np.array(self.q, float)
            if self.base_directe and self.ancre_qg:
                for i in self.directs:
                    # Ecart deroule dans [-pi, pi] : les axes du guide franchissent la
                    # couture de leur codeur, une soustraction brute y donnerait
                    # presque un tour.
                    d = (self.guide_q[i] - self.ancre_qg[i] + math.pi) % (2 * math.pi) \
                        - math.pi
                    qi = self.ancre_qr[i] + self.k * d
                    lo, hi = LIMITS[J[i]]
                    qi = (qi + math.pi) % (2.0 * math.pi) - math.pi
                    depart[i] = min(hi, max(lo, qi))
                # La cible, calculee dans le plan, revient dans le monde par la base
                # REELLE du robot -- celle qu'on vient de fixer.
                if 0 in self.directs:
                    # La cible, calculee dans le plan, revient dans le monde par la base
                    # REELLE du robot -- celle qu'on vient de fixer.
                    Rb = Rz(depart[0])
                    cible = Rb @ cible
                    R_guide = Rb @ R_guide

            actuel = self._p_outil(self.q)

            # LAISSE. Si le bras ne suit pas, on ne laisse pas l'ecart grandir : on
            # deplace l'ANCRE pour absorber l'exces. Le geste au-dela de la laisse est
            # simplement perdu, comme quand on pousse une souris contre le bord de
            # l'ecran -- et la correspondance reste vraie a tout instant.
            # Cible d'AFFICHAGE : meme formule, mais depuis l'ancre qui ne bouge pas.
            if self.ancre_aff is not None:
                aff = self.ancre_aff + self.k * (p_guide - self.ancre_guide)
                if self.base_directe and 0 in self.directs:
                    aff = Rz(depart[0]) @ aff
                self.cible_brute = np.array(aff, float)

            ec = cible - actuel
            n_ec = float(np.linalg.norm(ec))
            if self.laisse > 0 and n_ec > self.laisse:
                exces = ec * (1.0 - self.laisse / n_ec)
                if self.base_directe and 0 in self.directs:
                    self.ancre_p = self.ancre_p + Rz(-depart[0]) @ exces
                else:
                    self.ancre_p = self.ancre_p + exces
                cible = actuel + ec * (self.laisse / n_ec)

            # Limitation de vitesse cartesienne : une main peut bouger bien plus vite
            # que ce que le bras encaisse, et l'echelle > 1 amplifie encore.
            d = cible - actuel
            dmax = self.vmax * self.dt
            n = float(np.linalg.norm(d))
            if n > dmax:
                cible = actuel + d * (dmax / n)

            # dls() vise link_gripper : on retire l'offset de l'outil, exprime dans
            # l'orientation demandee. Le point de commande reste le bout de pince.
            cible_lg = cible - R_guide @ np.array([self.off, 0.0, 0.0])
            q = (self._dls_base_figee(depart, cible_lg, R_guide) if self.base_directe
                 else dls(depart, cible_lg, R_guide, w_ori=self.w_ori))
            for i, nom in enumerate(J):
                lo, hi = LIMITS[nom]
                if hi - lo >= 2.0 * math.pi - 1e-3:
                    # Axe qui fait le TOUR : ses bornes sont une couture, pas un
                    # obstacle. Clamper y bloque le suivi alors que rien ne l'empeche
                    # mecaniquement -- et c'est exactement ce qui arrive a la base,
                    # dont le repos est pose sur la couture (-180 deg) parce qu'elle
                    # est montee a l'envers sur le guide. On ENROULE.
                    c = (lo + hi) / 2.0
                    q[i] = c + (q[i] - c + math.pi) % (2.0 * math.pi) - math.pi
                    continue
                if q[i] < lo or q[i] > hi:
                    self._clamps.add(nom)
                    q[i] = min(hi, max(lo, q[i]))
            if not np.all(np.isfinite(q)):
                # On garde la pose precedente -- publier des NaN fige RViz et tue le
                # noeud au calcul suivant -- MAIS on REANCRE aussitot. Sans ce
                # reancrage, le tick suivant repart de la meme pose avec la meme cible
                # et rediverge : le bras se fige alors definitivement, alors que la
                # croix continue de suivre la main. Symptome vecu le 2026-09-09 :
                # "la croix suit le petit mais le gros ne cherche plus a aller vers".
                if not self._diverge:
                    self._diverge = True
                    self.get_logger().error(
                        "solveur divergent -> REANCRAGE. cible=%s R_fini=%s "
                        "depart=%s axes_figes=%s"
                        % (np.round(cible, 3), bool(np.all(np.isfinite(R_guide))),
                           np.round(depart, 3), [J[i] for i in self.directs]))
                self._ancrer()
                self._publier()
                return
            self._diverge = False
            self.cible_monde = np.array(cible, float)
            self.q = q
            if self._verif_saut > 0:
                self._verif_saut -= 1
                if self._verif_saut == 0 and self._tcp_ancre is not None:
                    d = float(np.linalg.norm(self._p_outil(self.q) - self._tcp_ancre))
                    # Deux appels DISTINCTS : rclpy memorise la severite par site
                    # d'appel et refuse qu'elle change ("Logger severity cannot be
                    # changed between calls"). Un ternaire sur le logger fait donc
                    # planter le noeud des que la condition bascule.
                    if d > 0.01:
                        self.get_logger().error(
                            "saut a l'embrayage : %.1f mm (attendu : ~0)" % (1000 * d))
                    else:
                        self.get_logger().info(
                            "embrayage sans saut : %.1f mm" % (1000 * d))
            self._surveiller_singularite()
        self._publier()

    def _dls_base_figee(self, j, target_p, target_R, lam=0.06, iters=8):
        """DLS a axes FIGES : meme algorithme, colonnes des axes recopies annulees.

        Annuler la colonne suffit : dq = J^T (J J^T + lam^2 I)^-1 e, donc une colonne
        nulle donne un dq nul sur cet axe. Inutile de reecrire un solveur reduit pour
        chaque combinaison d'axes figes.
        """
        j = np.array(j, float)
        for _ in range(iters):
            T = fkT(j)
            e = np.concatenate([target_p - T[:3, 3],
                                self.w_ori * (T[:3, :3] @ rotvec(T[:3, :3].T @ target_R))])
            Jm = jac(j)
            for i in self.directs:
                Jm[:, i] = 0.0
            dq = Jm.T @ np.linalg.inv(Jm @ Jm.T + lam ** 2 * np.eye(6)) @ e
            # Pas borne : sans cela, une cible hors d'atteinte fait diverger le solveur
            # en quelques iterations, la FK deborde et self.q part en NaN -- le noeud
            # meurt alors sur "SVD did not converge", loin de la vraie cause.
            n = float(np.linalg.norm(dq))
            if n > 0.5:
                dq = dq * (0.5 / n)
            j = j + dq
            # Bornes appliquees A CHAQUE iteration, pas seulement a la fin : le solveur
            # ne doit pas explorer des poses impossibles pour y calculer sa jacobienne.
            for i, nom in enumerate(J):
                lo, hi = LIMITS[nom]
                if hi - lo < 2.0 * math.pi - 1e-3:
                    j[i] = min(hi, max(lo, j[i]))
        return j

    def _surveiller_singularite(self):
        try:
            s = np.linalg.svd(jac(self.q), compute_uv=False)
        except np.linalg.LinAlgError:
            return          # diagnostic seulement : jamais une cause d'arret
        proche = bool(s[-1] < self.seuil_sing)
        if proche != self._sing:
            self._sing = proche
            if proche:
                self.get_logger().warn(
                    "SINGULARITE proche (sigma_min %.4f < %.4f) : le bras va sembler "
                    "mou dans une direction. L'amortissement du DLS le fait ralentir "
                    "SANS erreur -- c'est ce message qui l'annonce."
                    % (s[-1], self.seuil_sing))
            else:
                self.get_logger().info("sortie de la zone de singularite")

    def _publier(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(J)
        m.position = [float(v) for v in self.q]
        self.pub.publish(m)
        if self.guide is not None:
            T = fkT(self.q)
            err = float(np.linalg.norm(rotvec(T[:3, :3].T @ self.guide[1])))
            e = Float64MultiArray()
            try:
                sig = float(np.linalg.svd(jac(self.q), compute_uv=False)[-1])
            except np.linalg.LinAlgError:
                sig = float("nan")
            e.data = [float(self.k), 1.0 if self.embraye else 0.0,
                      math.degrees(err), sig]
            self.pub_etat.publish(e)

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
            cal = charger(self._chemin)
        except Exception as e:
            self.get_logger().warn("calibration illisible, on garde l'ancienne : %s" % e)
            return
        self.cal, self._mtime = cal, m
        # La reference du guide vient de changer : sans reancrage, le grand bras
        # sauterait de l'ecart introduit par le reglage.
        if self.embraye and self.guide is not None:
            self._ancrer()
        self.get_logger().warn(
            "calibration RECHARGEE (reancre) : %s"
            % ", ".join("%s zero_urdf %+.0f deg" % (j.nom_urdf,
                                                    math.degrees(j.zero_urdf))
                        for j in self.cal.joints if j.nom_urdf.startswith("joint")))

    def _marqueurs(self):
        """Cible demandee (vert) et TCP atteint (bleu), avec une croix de reperage.

        La croix n'est pas cosmetique : quand le suivi est bon, la cible tombe A
        L'INTERIEUR du maillage de la pince et devient invisible -- constate le
        2026-09-09, les deux points etaient a 5 mm l'un de l'autre et noyes dans le
        modele 3D. Les trois branches de 16 cm depassent toujours du bras.

        Les deux reperes ensemble disent ce qu'aucun chiffre ne montre aussi vite : si
        le vert s'ecarte du bleu, le bras ne suit plus -- butee, singularite ou vitesse
        plafonnee. Un ecart nul et un bras bloque se ressemblent autrement.
        """
        if self.cible_brute is None:
            return
        arr = MarkerArray()
        tcp = fkT(self.q)[:3, 3]
        # Rouge des que le bras ne rejoint plus ce qu'on lui demande : c'est le signal
        # qu'on sort de l'atteignable (butee, singularite, ou geste trop rapide).
        ecart = float(np.linalg.norm(self.cible_brute - tcp))
        vert = (0.1, 0.9, 0.2)
        rouge = (0.95, 0.25, 0.15)
        teinte = rouge if ecart > 0.03 else vert
        maintenant = self.get_clock().now().to_msg()

        def base(i, type_):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = maintenant
            m.ns = "oracle_D"
            m.id = i
            m.type = type_
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        for i, (p, rgba, taille) in enumerate((
                (self.cible_brute, teinte + (0.85,), 0.030),
                (tcp, (0.2, 0.5, 1.0, 0.55), 0.022))):
            m = base(i, Marker.SPHERE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
            m.scale.x = m.scale.y = m.scale.z = taille
            (m.color.r, m.color.g, m.color.b, m.color.a) = rgba
            arr.markers.append(m)

        croix = base(2, Marker.LINE_LIST)
        croix.scale.x = 0.004
        croix.color.r, croix.color.g, croix.color.b = teinte
        croix.color.a = 0.9
        from geometry_msgs.msg import Point
        c, L = self.cible_brute, 0.08
        for axe in range(3):
            for signe in (-1.0, 1.0):
                d = np.zeros(3)
                d[axe] = signe * L
                pt = Point()
                pt.x, pt.y, pt.z = map(float, c)
                croix.points.append(pt)
                pt2 = Point()
                pt2.x, pt2.y, pt2.z = map(float, c + d)
                croix.points.append(pt2)
        arr.markers.append(croix)

        # Trait cible -> TCP : l'ecart cesse d'etre un chiffre et devient une longueur.
        trait = base(3, Marker.LINE_LIST)
        trait.scale.x = 0.006
        trait.color.r, trait.color.g, trait.color.b = teinte
        trait.color.a = 0.9
        for p in (self.cible_brute, tcp):
            pt = Point()
            pt.x, pt.y, pt.z = map(float, p)
            trait.points.append(pt)
        arr.markers.append(trait)
        self.pub_marq.publish(arr)

    def _rapport(self):
        if self._clamps:
            self.get_logger().warn("butee atteinte : %s" % ", ".join(sorted(self._clamps)))
            self._clamps.clear()


def main():
    rclpy.init()
    n = TeleopCart()
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
