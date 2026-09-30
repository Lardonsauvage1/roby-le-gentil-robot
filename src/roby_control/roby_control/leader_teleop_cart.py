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

POINT COMMANDE = CENTRE DU POIGNET (Niels, 2026-09-15). Roby a 5 axes pour un probleme
qui en demande 6 : position et orientation de la pince ne peuvent pas etre tenues
ensemble. On les DECOUPLE, comme sur un bras industriel a poignet : la base et les deux
axes du poignet (joint_4, joint_5) sont RECOPIES du bras guide, et l'IK ne place que le
centre du poignet (origine de joint_5), avec joint_2 et joint_3. Ce centre ne dependant
ni de joint_4 ni de joint_5, tourner le poignet du guide ne deplace plus la cible : il
oriente la pince autour d'un point fixe. Il reste 2 axes pour 2 coordonnees dans le plan
du bras -- un probleme bien pose, singulier seulement coude tendu ou replie a fond.
Avant, l'IK placait un point de la pince (11 cm devant joint_5) avec 4 axes et une
orientation faiblement ponderee : le poignet se tordait pour aider la position.

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
from std_srvs.srv import SetBool, Trigger

from roby_control.leader_mapping import charger
from roby_control.sim_joint_states import GardeJointStates

# tools/pc de la MEME copie du depot que ce fichier (paquet installe en --symlink-install :
# le chemin reel est dans src/). Repli sur le depot en service sinon.
_TOOLS = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", "tools", "pc")
sys.path.insert(0, os.path.normpath(_TOOLS) if os.path.isdir(_TOOLS)
                else os.path.expanduser("~/ros2_ws/tools/pc"))
from roby_tool_pickup import LIMITS, Rz, fk_poignet, fkT   # noqa: E402

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
# joint_4, joint_5 : TOUJOURS recopies du bras guide -- ils portent l'ORIENTATION.
# ⚠️ Depuis l'URDF mesuree du 2026-09-20 (ADR-005), joint_4 deplace aussi le centre du
# poignet (~10 cm/rad) : ce n'est pas une derive, le guide a la meme geometrie et le robot
# reproduit son deplacement a 1:1. Mais l'affirmation « ils ne deplacent pas le point »
# n'est plus vraie, et le choix du point commande est a reprendre (consequence de l'ADR).
POIGNET = (3, 4)

# Pose de travail du reseau BC, mesuree sur le vrai robot (TCP a z = 0,33 m).
POSE_TRAVAIL = [-0.2991, 0.8560, -0.4835, -0.0492, 1.3045]


class TeleopCart(Node):
    # Sortie du noeud. Ici le bras SIMULE : la pose calculee est publiee sur
    # /joint_states. La sous-classe du vrai bras (leader_teleop_reel) n'y publie rien
    # -- sur la vraie stack, ce topic appartient au robot.
    PUBLIE_JOINT_STATES = True
    MSG_SORTIE = "Aucun message vers le vrai robot."

    def __init__(self):
        super().__init__("leader_teleop_cart")
        self.declare_parameter("echelle", 1.0)
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
        # AMORTISSEMENT ADAPTATIF. Un lambda fixe est un compromis unique applique
        # partout : assez fort pour survivre pres d'une singularite, donc bien trop
        # fort loin d'elle, ou il n'y a rien a amortir. Le bras traine alors en
        # permanence pour se proteger d'un danger presque toujours absent.
        # Ici lambda vaut ZERO tant que la plus petite valeur singuliere est
        # confortable, et ne monte que lorsqu'elle s'effondre.
        self.declare_parameter("lam_max", 0.10)
        self.declare_parameter("lam_seuil", 0.08)
        self.declare_parameter("ik_tolerance_m", 1e-4)
        # BASE HORS CINEMATIQUE (idee de Sam, 2026-09-09) : joint_1 n'est plus resolu
        # par l'IK, sa valeur est recopiee de celle du guide avec le meme rapport de
        # mouvement. Deux gains : la base devient previsible (un degre de guide donne
        # k degres de robot, toujours), et elle sort du probleme sur-contraint -- il
        # reste 4 axes pour 3 positions + orientation au mieux, au lieu de 5 pour 6.
        self.declare_parameter("base_directe", True)
        # Axes RECOPIES du guide au lieu d'etre resolus par l'IK, numerotes 1..5.
        # joint_4 et joint_5 le sont TOUJOURS (ajoutes d'office) : ils ne deplacent pas le
        # centre du poignet, l'IK ne pourrait rien en faire. La base (1) est au choix :
        # recopiee, l'IK place le poignet dans le plan du bras avec joint_2 et joint_3 ;
        # retiree, elle rejoint l'IK, qui place alors le poignet en 3D avec joint_1..3.
        # Le 2026-09-09, figer joint_5 creait une quasi-singularite : la cible etait
        # alors un point de la PINCE, que joint_2/3 ne placent pas seuls. Elle ne l'est
        # plus -- c'est le centre du poignet, que joint_2/3 placent exactement.
        self.declare_parameter("axes_directs", [1, 4, 5])
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
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.vmax = float(self.get_parameter("vitesse_max_m_s").value)
        self.seuil_sing = float(self.get_parameter("seuil_singularite").value)
        self.base_directe = bool(self.get_parameter("base_directe").value)
        self.laisse = float(self.get_parameter("laisse_m").value)
        self.directs = self._lire_directs(self.get_parameter("axes_directs").value)

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
        self.ancre_aff = None
        self.dernier = None
        self.embraye = bool(self.get_parameter("embraye_au_demarrage").value)
        self._clamps = set()
        self._sing = False
        self._diverge = False
        self._tcp_ancre = None
        self._verif_saut = 0
        # Reglages qui changent la FACON dont le guide est exprime (axes recopies, base
        # directe, calibration) : appliques au PROCHAIN message du guide, avec le
        # reancrage. Reancrer tout de suite posait l'ancre sur une pose du guide calculee
        # avec l'ANCIEN reglage ; au message suivant, l'ecart devenait un vrai mouvement
        # (13 cm de TCP pour +10 deg sur le zero de la base, guide immobile ; revue du
        # 2026-09-13).
        self._attente = {}
        self._reancrer = False

        self.pub = (self.create_publisher(JointState, "/joint_states", 10)
                    if self.PUBLIE_JOINT_STATES else None)
        # Jamais a cote d'un vrai robot (BUG-008) : muet si un autre publisher existe.
        self._garde_js = GardeJointStates(self, self.pub) if self.pub is not None else None
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
        self.create_service(Trigger, "/teleop_cart/pose_travail", self._srv_pose_travail)
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
            "Teleop CARTESIENNE prete : echelle 1:%.3g, point commande = CENTRE DU "
            "POIGNET, axes recopies %s, %.0f Hz. %s. %s"
            % (1.0 / self.k if self.k else 0, [J[i] for i in self.directs], hz,
               "EMBRAYE" if self.embraye else "DEBRAYE (embrayer pour piloter)",
               self.MSG_SORTIE))

    # ------------------------------------------------------------------ geometrie
    @staticmethod
    def _lire_directs(valeur):
        """Axes recopies (indices 0..4) : ceux demandes (numerotes 1..5) + le poignet."""
        return sorted({int(a) - 1 for a in valeur if 1 <= int(a) <= 5} | set(POIGNET))

    def _echelle_axe(self, i):
        """Rapport guide -> robot d'un axe RECOPIE. La base suit l'echelle : elle deplace
        le poignet lateralement, comme une translation. Le poignet reste a 1:1 : il
        oriente la pince, et l'orientation n'etait deja pas reduite par l'echelle."""
        return self.k if i == 0 else 1.0

    def _p_poignet(self, q):
        """POINT COMMANDE : centre du poignet (origine de joint_5)."""
        return fk_poignet(q)

    def _p_pince(self, q):
        """link_gripper : le point que protege le garde (plancher), et celui ou l'on
        mesure saut et derive -- il porte aussi les erreurs du poignet."""
        return fkT(q)[:3, 3]

    def _jac_poignet(self, q, axes, eps=1e-6):
        """Jacobienne en position du centre du poignet, colonnes `axes` (3 x n).

        `axes` ne contient que les axes rendus a l'IK. Celle de joint_5 serait nulle ; celle
        de joint_4 ne l'est PLUS depuis l'URDF du 2026-09-20 (ADR-005), mais joint_4 reste
        recopie du guide, donc hors de `axes`."""
        q = np.asarray(q, float)
        p0, Jm = fk_poignet(q), np.zeros((3, len(axes)))
        for c, i in enumerate(axes):
            dq = np.zeros(len(J))
            dq[i] = eps
            Jm[:, c] = (fk_poignet(q + dq) - p0) / eps
        return Jm

    # ------------------------------------------------------------------ entrees

    def _cb(self, msg):
        """Pose du guide -> position de SON centre de poignet, dans le repere de Roby.

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
        if self._attente:
            self.directs = self._attente.get("directs", self.directs)
            self.base_directe = self._attente.get("base_directe", self.base_directe)
            self._attente = {}
        # MEME point des deux cotes : le centre du poignet. Le poignet du guide peut
        # tourner sans deplacer la reference -- il n'oriente que la pince.
        p_poignet = fk_poignet(angles)
        self.guide_q1 = float(angles[0])
        self.guide_q = [float(a) for a in angles]
        if self._plan():
            # DANS LE PLAN DU BRAS. Quand la base est pilotee a part, la position de
            # reference doit etre exprimee APRES joint_1, sinon elle porte encore la
            # rotation de la base -- et comme le zero de la base est a -180 deg (guide
            # monte a l'envers), le repere de reference est retourne d'un demi-tour
            # alors que la base, elle, est forcee ailleurs. Les deux se contredisent et
            # TOUS LES AXES paraissent inverses. Vecu le 2026-09-09.
            self.guide = Rz(-self.guide_q1) @ p_poignet
        else:
            self.guide = p_poignet.copy()
        self.dernier = self.get_clock().now()
        if self._reancrer:
            self._reancrer = False
            if self.embraye:
                self._ancrer()

    def _plan(self):
        """Position du guide exprimee DANS LE PLAN DU BRAS (apres joint_1) ?

        Seulement si la base est recopiee du guide : c'est la base reelle du robot qui
        ramene ensuite la cible dans le monde (_tick). Avec base_directe mais joint_1
        hors des axes recopies, l'ancre etait dans le plan et la cible jamais ramenee :
        6 cm de mouvement des l'embrayage, guide immobile (revue du 2026-09-13)."""
        return 0 in self._recopies()

    def _recopies(self):
        """Axes EFFECTIVEMENT recopies du guide (indices). La base n'en fait partie que
        si `base_directe` ; le poignet, toujours. Tout le reste est a l'IK."""
        return [i for i in self.directs if i != 0 or self.base_directe]

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
            self._tcp_ancre = self._p_pince(self.q).copy()
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

    def _srv_pose_travail(self, req, resp):
        """Place le bras SIMULE dans la pose de travail.

        Necessaire parce que la commande cartesienne est INCREMENTALE : elle ne compare
        que des ecarts depuis l'ancre, donc changer la correspondance neutre ne deplace
        pas le bras d'un millimetre. Il reste la ou il etait -- y compris dans une
        region mal conditionnee dont on ne sort plus, puisque la laisse borne la cible
        a quelques centimetres.

        Pose choisie : celle d'ou part le reseau BC, mesuree sur le vrai robot. TCP a
        z = 0,33 m au-dessus de la zone du dataset, sigma_min = 0,18.
        """
        if self.embraye:
            resp.success = False
            resp.message = "debrayez d'abord : deplacer le bras embraye ferait un saut"
            return resp
        self.q = np.array(POSE_TRAVAIL, float)
        self._publie = {}
        self.cible_brute = None
        self.cible_monde = None
        p = self._p_poignet(self.q)
        resp.success = True
        resp.message = ("bras place en pose de travail, centre du poignet a z = %.3f m "
                        "(embrayez pour reprendre)" % p[2])
        self.get_logger().warn("/teleop_cart/pose_travail -> %s" % resp.message)
        return resp

    def _ancrer(self):
        """Repose les deux ancres sur l'etat courant. C'est TOUT le debrayage, et c'est
        aussi ce qui rend le changement d'echelle sans a-coup."""
        self.ancre_guide = self.guide.copy()
        if self._plan():
            # Meme repere des deux cotes : l'ancre du robot est prise APRES joint_1.
            self.ancre_p = Rz(-float(self.q[0])) @ self._p_poignet(self.q)
        else:
            self.ancre_p = self._p_poignet(self.q).copy()
        self.ancre_q1_guide = self.guide_q1
        self.ancre_q1_roby = float(self.q[0])
        # DEUXIEME ANCRE, pour l'AFFICHAGE seulement. Celle de commande est deplacee
        # par la laisse pour empecher la divergence ; celle-ci ne bouge jamais entre
        # deux ancrages. La croix montre donc ou la MAIN demande, pas ou le bras a
        # bien voulu aller -- sans quoi elle reste collee au bras et ne signale plus
        # rien. Les deux besoins sont contradictoires avec une seule ancre.
        self.ancre_aff = self.ancre_p.copy()
        # Axes recopies (base, poignet) ancres eux aussi : INCREMENTAUX depuis
        # l'embrayage. Absolus, tourner le poignet du guide pendant un debrayage ferait
        # sauter le bras a l'embrayage suivant (10,4 mm mesures le 2026-09-09).
        self.ancre_qg = {i: self.guide_q[i] for i in self._recopies()}
        self.ancre_qr = {i: float(self.q[i]) for i in self._recopies()}

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
                nouveaux = self._lire_directs(p.value)
                if self.embraye and self.guide is not None:
                    self._attente["directs"] = nouveaux     # au prochain message du guide
                    self._reancrer = True
                else:
                    self.directs = nouveaux
                self.get_logger().warn(
                    "axes recopies du guide : %s (reancre)"
                    % (", ".join(J[i] for i in nouveaux) or "aucun"))
            elif p.name == "base_directe":
                nouvelle = bool(p.value)
                if self.embraye and self.guide is not None:
                    self._attente["base_directe"] = nouvelle
                    self._reancrer = True
                else:
                    self.base_directe = nouvelle
                self.get_logger().warn(
                    "base %s (reancre)"
                    % ("RECOPIEE directement du guide" if nouvelle
                       else "resolue par l'IK"))
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
            p_guide = self.guide
            cible = self.ancre_p + self.k * (p_guide - self.ancre_guide)

            # Axes RECOPIES d'abord : ils fixent la base (donc le plan du bras) et
            # l'orientation de la pince ; l'IK travaille ensuite a axes recopies figes.
            depart = np.array(self.q, float)
            for i in self._recopies():
                if i not in self.ancre_qg:
                    continue
                # Ecart deroule dans [-pi, pi] : les axes du guide franchissent la
                # couture de leur codeur, une soustraction brute y donnerait
                # presque un tour.
                d = (self.guide_q[i] - self.ancre_qg[i] + math.pi) % (2 * math.pi) \
                    - math.pi
                qi = self.ancre_qr[i] + self._echelle_axe(i) * d
                lo, hi = LIMITS[J[i]]
                qi = (qi + math.pi) % (2.0 * math.pi) - math.pi
                depart[i] = min(hi, max(lo, qi))
            if self._plan():
                # La cible, calculee dans le plan, revient dans le monde par la base
                # REELLE du robot -- celle qu'on vient de fixer.
                cible = Rz(depart[0]) @ cible

            # Point ACTUEL pris avec les axes recopies deja appliques : la laisse et la
            # limite de vitesse ne portent que sur ce que l'IK doit accomplir. La base a
            # sa propre bride (vitesse articulaire) ; le poignet ne deplace pas ce point.
            actuel = self._p_poignet(depart)

            # LAISSE. Si le bras ne suit pas, on ne laisse pas l'ecart grandir : on
            # deplace l'ANCRE pour absorber l'exces. Le geste au-dela de la laisse est
            # simplement perdu, comme quand on pousse une souris contre le bord de
            # l'ecran -- et la correspondance reste vraie a tout instant.
            # Cible d'AFFICHAGE : meme formule, mais depuis l'ancre qui ne bouge pas.
            if self.ancre_aff is not None:
                aff = self.ancre_aff + self.k * (p_guide - self.ancre_guide)
                if self._plan():
                    aff = Rz(depart[0]) @ aff
                self.cible_brute = np.array(aff, float)

            ec = cible - actuel
            n_ec = float(np.linalg.norm(ec))
            if self.laisse > 0 and n_ec > self.laisse:
                # L'ancre est REPOSITIONNEE, pas incrementee. Lui AJOUTER l'exces etait
                # faux : au tick suivant l'exces se recalcule depuis la nouvelle ancre
                # et s'ajoute encore, si bien que la cible reste eternellement une
                # laisse devant la pince -- meme main immobile. Le bras poursuivait
                # alors une carotte qui recule, jusqu'a la butee. Signale par Sam le
                # 2026-09-09 : "il continue d'y aller jusqu'a plus pouvoir".
                #
                # On impose donc directement : cible = actuel + laisse * direction,
                # d'ou l'ancre se deduit. Main immobile => cible fixe => le bras la
                # rejoint et s'arrete.
                voulu = actuel + ec * (self.laisse / n_ec)
                delta = self.k * (p_guide - self.ancre_guide)
                if self._plan():
                    self.ancre_p = Rz(-depart[0]) @ voulu - delta
                else:
                    self.ancre_p = voulu - delta
                cible = voulu

            # Limitation de vitesse cartesienne : une main peut bouger bien plus vite
            # que ce que le bras encaisse, et l'echelle > 1 amplifie encore.
            d = cible - actuel
            dmax = self.vmax * self.dt
            n = float(np.linalg.norm(d))
            if n > dmax:
                cible = actuel + d * (dmax / n)

            q = self._ik_poignet(depart, cible)
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
                        "solveur divergent -> REANCRAGE. cible=%s depart=%s "
                        "axes_recopies=%s"
                        % (np.round(cible, 3), np.round(depart, 3),
                           [J[i] for i in self._recopies()]))
                self._ancrer()
                self._publier()
                return
            self._diverge = False
            self.cible_monde = np.array(cible, float)
            self.q = q
            if self._verif_saut > 0:
                self._verif_saut -= 1
                if self._verif_saut == 0 and self._tcp_ancre is not None:
                    d = float(np.linalg.norm(self._p_pince(self.q) - self._tcp_ancre))
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

    def _actifs(self):
        """Indices des axes que le solveur a le droit de bouger."""
        recopies = self._recopies()
        return [i for i in range(len(J)) if i not in recopies]

    def _ik_poignet(self, j, cible, iters=8):
        """IK differentielle amortie du CENTRE DU POIGNET, position seule.

        Les axes recopies (base eventuelle, joint_4, joint_5) gardent la valeur recue ;
        le solveur ne bouge que les autres : joint_2 et joint_3, plus joint_1 si la base
        n'est pas recopiee. Base recopiee, la cible est dans le plan du bras et 2 axes
        placent 2 coordonnees : aucun compromis, plus d'orientation a ponderer.

        Choix conserves de la version precedente :

        1. On resout dans le SOUS-ESPACE ACTIF. Annuler des colonnes laissait une
           matrice de rang deficient dont la plus petite valeur singuliere valait
           toujours zero -- tout critere de conditionnement calcule dessus etait donc
           faux, et l'amortissement adaptatif aurait ete au maximum en permanence.
        2. lambda ADAPTATIF : nul quand le bras est bien conditionne, il ne monte
           qu'a l'approche d'une singularite (coude tendu ou replie a fond).
        3. Forme NORMALE (Ja^T Ja + lam^2 I) de taille n x n.
        """
        j = np.array(j, float)
        act = self._actifs()
        if not act:
            return j
        lam_max = float(self.get_parameter("lam_max").value)
        seuil = max(1e-6, float(self.get_parameter("lam_seuil").value))
        tol = float(self.get_parameter("ik_tolerance_m").value)
        I = np.eye(len(act))
        for _ in range(iters):
            e = np.asarray(cible, float) - fk_poignet(j)
            if float(np.linalg.norm(e)) < tol:
                break                      # deja au but : ne pas iterer pour rien
            Ja = self._jac_poignet(j, act)
            try:
                smin = float(np.linalg.svd(Ja, compute_uv=False)[-1])
            except np.linalg.LinAlgError:
                smin = 0.0
            lam2 = 0.0 if smin >= seuil else lam_max ** 2 * (1.0 - (smin / seuil) ** 2)
            try:
                dqa = np.linalg.solve(Ja.T @ Ja + (lam2 + 1e-12) * I, Ja.T @ e)
            except np.linalg.LinAlgError:
                break
            dq = np.zeros(len(J))
            dq[act] = dqa
            n = float(np.linalg.norm(dq))
            if not np.isfinite(n):
                break
            if n < 1e-9:
                # Reste d'erreur hors d'atteinte des axes actifs (composante hors du
                # plan du bras quand la base est recopiee) : rien de plus a gagner.
                break
            if n > 0.5:
                dq = dq * (0.5 / n)        # pas borne : une cible hors d'atteinte
            j = j + dq                     # faisait diverger le solveur en 8 pas
            for i, nom in enumerate(J):
                lo, hi = LIMITS[nom]
                if hi - lo < 2.0 * math.pi - 1e-3:
                    j[i] = min(hi, max(lo, j[i]))
        return j

    def _surveiller_singularite(self):
        act = self._actifs()
        if not act:
            return
        try:
            # Sur les colonnes ACTIVES, et pour le point que resout le solveur : tout
            # autre conditionnement ne correspond a rien de ce qu'il peut faire.
            s = np.linalg.svd(self._jac_poignet(self.q, act), compute_uv=False)
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
        if self._garde_js is not None:
            self._garde_js.publier(m)
        if self.guide is not None:
            # [echelle, embraye, ECART DU POIGNET A SA CIBLE (mm), sigma_min]. Le 3e champ
            # etait l'ecart d'orientation : l'orientation est maintenant recopiee, elle ne
            # peut plus deriver. Ce qui peut ne plus suivre, c'est le poignet (butee,
            # singularite, geste trop rapide).
            ecart = (0.0 if self.cible_monde is None else 1000.0 * float(
                np.linalg.norm(self.cible_monde - self._p_poignet(self.q))))
            e = Float64MultiArray()
            try:
                act = self._actifs()
                sig = float(np.linalg.svd(self._jac_poignet(self.q, act),
                                          compute_uv=False)[-1]) if act else 0.0
            except np.linalg.LinAlgError:
                sig = float("nan")
            e.data = [float(self.k), 1.0 if self.embraye else 0.0, ecart, sig]
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
        # sauterait de l'ecart introduit par le reglage. Reancrage au PROCHAIN message du
        # guide, converti avec la NOUVELLE calibration (cf. self._attente).
        if self.embraye and self.guide is not None:
            self._reancrer = True
        self.get_logger().warn(
            "calibration RECHARGEE (reancre) : %s"
            % ", ".join("%s zero_urdf %+.0f deg" % (j.nom_urdf,
                                                    math.degrees(j.zero_urdf))
                        for j in self.cal.joints if j.nom_urdf.startswith("joint")))

    def _marqueurs(self):
        """Cible demandee (vert) et centre du poignet atteint (bleu), avec une croix.

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
        # Le POINT COMMANDE, comme la croix : le centre du poignet. Un autre point
        # laisserait les deux reperes a distance constante meme avec un suivi parfait,
        # et la croix virerait au rouge (seuil 3 cm) en permanence (vecu le 2026-09-13).
        tcp = self._p_poignet(self.q)
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
