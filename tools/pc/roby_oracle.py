#!/usr/bin/env python3
"""
roby_oracle.py — Boucle ORACLE de collecte de dataset (imitation learning).

Le robot (pince) genere des episodes varies en boucle. Fenetre ENREGISTREE = START->STOP :
  [SETUP] prendre l'objet a D -> R aleatoire -> poser -> aerien A
  [REC]   A -> R (connu) -> prendre -> D -> lacher
  (l'objet revient a D -> reboucle)
  OBJET MANIPULE = pomme blanche imprimee en 3D (ex-cone bleu).

CARTESIEN + ancre sur les JOINTS de D :
  Le jog lit le repere 'tcp' (bout de pince), mais le DLS travaille en repere FK
  (link_gripper+6cm) -> ecart ~100mm. Donc on ancre sur D_JOINTS et on calcule
  D_XYZ = fk_pos(D_JOINTS) : exact et coherent avec le DLS. L'orientation de prise
  R_GRASP = fkT(D_JOINTS)[:3,:3] (top-down) est gardee pour tous les points.

PRISE/DEPOSE (demande Sam) : TOUJOURS un point d'approche 10cm AU-DESSUS, AVANT et APRES.
  transit haut = MoveIt libre (anti-collision) ; descente/remontee = LIGNE DROITE DLS verticale.

Modes :
  --mode dry   : logique pure, AUCUNE ROS (log).
  --mode plan  : IK reelle (DLS) validee sur chaque point, AUCUN mouvement (log). Test geometrie.
  --mode real  : bras REEL (exige --go + presence Sam).
"""
import argparse
import os
import random
import sys
import time

import numpy as np

# ================= Kinematique (repere link_gripper, copie de roby_tool_pickup) =================
def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def H(R, t): T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


def fkT(j):
    j1, j2, j3, j4, j5 = j
    T = np.eye(4)
    T = T @ H(Rz(j1), [0, 0, 0.02])
    T = T @ H(Ry(j2), [0.024031, 0, 0.202992])
    T = T @ H(Ry(j3), [-0.015224, 0, 0.441653])
    T = T @ H(Rx(j4), [0.119473, 0, 0.029716])
    T = T @ H(Ry(j5), [0.321516, 0, 0])
    T = T @ H(np.eye(3), [0.06, 0, 0])
    return T


def fk_pos(j):
    return fkT(j)[:3, 3]


# ================= Configuration =================
J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
LIMITS = {"joint_1": (-3.14159, 3.14159), "joint_2": (-1.6, 2.1),
          "joint_3": (-3.0, 0.65), "joint_4": (-3.1416, 3.1416),
          "joint_5": (-1.6, 1.6)}

# Point de depose D — RELEVE SUR LE VRAI ROBOT DANS LA CUISINE (2026-09-07, demande Sam).
# Bras amene au-dessus de la zone, puis verrou de tete ouvert 5 s et referme pour que
# l'outil se rassoie dans son cone avant la lecture => la pose lue fait foi.
#   joints [-0.2991, 0.8560, -0.4835, -0.0492, 1.3045]  ->  xyz (+0.728, -0.228, +0.332)
# L'ANCIEN D (joints [0.5649, 1.2491, -0.5935, -0.0557, 1.0070], xyz (+0.678, +0.426, +0.072))
# venait de l'atelier et tombait dans l'emprise de la PLAQUE DE CUISSON (x 0.30..0.88,
# y +0.10..+0.61 dans cuisine.yaml) : la collecte y aurait depose la pomme. Le nouveau D
# est DANS la zone de tirage declaree. Sauvegarde : roby_oracle.py.avant_D_cuisine
# ⚠️ Z_PICK_CUISINE (ci-dessous) vaut toujours 0.2551, soit 7.6 cm PLUS BAS que ce D.
# Sam a constate a 0.2551 que la pince entrerait dans la table => hauteur de PRISE encore
# a mesurer sur cette surface (le +8 cm qui a donne ce D est un degagement, pas une mesure).
D_JOINTS = [-0.2991, 0.8560, -0.4835, -0.0492, 1.3045]
# Decalage de D du au REDRESSEMENT de l'outil (2026-09-07, demande Sam).
# D_JOINTS a ete releve avec la pince PENCHEE de 6.6 deg. En redressant l'orientation de
# prise (cf. _redresse_vertical), le bout des doigts se deplace de ~1.2 cm horizontalement
# alors que fk_pos, lui, ne bouge pas. On decale donc D de l'oppose pour que la prise
# retombe au meme point physique.
#   deplacement calcule : dx=+1.1 cm dy=+0.1 cm, pour une longueur d'outil de 10 cm sous
#   fk_pos (= l'ecart tcp_world/fk_pos documente sur D_pose_cone).
# ⚠️ Ces 10 cm sont une SUPPOSITION tiree d'un seul releve. Si la prise accroche mal,
# c'est ce chiffre qu'il faut corriger -- ou simplement poser la pomme sous les doigts.
D_OFFSET_OUTIL = (-0.0115, -0.0014, 0.0)
D_XYZ = tuple(float(v) + o for v, o in zip(fk_pos(D_JOINTS), D_OFFSET_OUTIL))
def _redresse_vertical(R):
    """Ramene l'axe de l'outil (x local) exactement sur la verticale descendante.

    L'orientation de prise etait jusqu'ici reprise TELLE QUELLE de la pose D, elle-meme
    heritee de D_pose_cone capturee au jog dans l'atelier : elle penchait de 6.6 deg, et
    ces 6.6 deg se propageaient a TOUTES les prises (le grasp-azimut ne fait que tourner
    cette orientation autour de la verticale). Constat Sam 2026-09-07 : la pince n'est
    jamais orientee vers la table. Or l'intention du grasp-azimut est justement que la
    verticale exacte soit atteignable partout -- verifie : 27 points sur 28 de la zone,
    inclinaison residuelle 0.3 deg.
    """
    a = R[:, 0]
    b = np.array([0.0, 0.0, -1.0])
    v = np.cross(a, b)
    sn = float(np.linalg.norm(v))
    cs = float(a @ b)
    if sn < 1e-9:
        return R.copy()
    K = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return (np.eye(3) + K + K @ K * ((1.0 - cs) / sn ** 2)) @ R


R_GRASP = _redresse_vertical(fkT(D_JOINTS)[:3, :3])    # prise TOP-DOWN exacte (0.3 deg)
_AZI_D = float(np.arctan2(D_XYZ[1], D_XYZ[0]))         # azimut base->D (repere du grasp-azimut)

# ---------------------------------------------------------------------------
# ZONE DE TRAVAIL — CUISINE (2026-09-06). Remplace l'ancienne table de l'atelier.
# Bornee par trois obstacles reels de la scene `cuisine` :
#   X : bord de la structure sous le robot (+0.19) -> aplomb du refrigerateur (+0.89)
#   Y : bord du refrigerateur (-0.56)              -> bord de la plaque de cuisson (+0.10)
# Retrait supplementaire de 5 cm cote robot, puis TABLE_MARGIN de 10 cm sur les 4 bords
# (demande de Sam) => zone effective 45 x 46 cm : X +0.34..+0.79, Y -0.46..0.00.
# Ancienne zone atelier (conservee pour memoire) : x (-0.258, 1.043), y (-0.218, 0.543).
TABLE_PHYS = {"x": (0.24, 0.89), "y": (-0.56, 0.10)}
# Zone de tirage = TOUTE la table sauf : marge aux bords, disque autour de la base
# robot, et disque d'exclusion autour du point de depose D (demande Sam 2026-07-11).
TABLE_MARGIN = 0.10               # marge a chaque bord (m) — 10 cm demandes par Sam pour la
                                  # zone cuisine, contre 5 cm sur l'ancienne table.
D_KEEPOUT = 0.05                  # rayon d'exclusion autour de D (m) : R jamais < 5cm de D
# Randomisation du point de DEPOSE (demande Sam 2026-09-07). A chaque episode la pomme
# est reposee dans un disque de ce rayon autour de D, au lieu de toujours au meme point.
# But : que le reseau n'apprenne pas UNE position de depose par coeur.
# L'episode suivant reprend la pomme LA OU ELLE A ETE POSEE, pas a D.
D_JITTER = 0.02                   # rayon de randomisation de la depose (m)
BASE_KEEPOUT = 0.30               # rayon d'exclusion autour de la base robot (origine)
# Empreinte au sol du SUPPORT sous le bras -> exclue du tirage.
# Cuisine (2026-09-06) : support reel mesure, X -0.247..+0.248, Y -0.287..+0.559.
# (atelier, pour memoire : centre (0.00, 0.0925), demi-dimensions 0.2575 x 0.4375)
BLOCK_MARGIN = 0.05
_BLK_C = (0.0005, 0.136)
_BLK_HALF = (0.4950 / 2, 0.846 / 2)
LIFT = 0.05                       # point d'approche 5cm AU-DESSUS de la CIBLE (avant : 10cm)
AERIEN_DZ = (0.18, 0.30)          # hauteur aerienne = z de D + [min,max]

# --- Scenario RATTRAPAGE (--recovery, 2026-07-22, demande Sam) ---
# Episodes ou le bras DEMARRE a cote de la pomme, AU RAS de la table, pince FERMEE
# sur du vide (la prise vient d'echouer : on ferme AVANT de descendre, ce qui rend le
# ratage realiste au lieu d'etre simule apres coup), puis se releve un peu, se recale,
# ramasse, et amene a la cible. Apprend la RECUPERATION : le reseau finit souvent
# JUSTE a cote de la pomme (variabilite observee le 2026-07-22) => sans demo de
# rattrapage il reste bloque a cote.
# decalage horizontal "rate" vs la pomme (m), tire uniforme. Reglable via
# ROBY_RECOVERY_OFFSET="min,max" (ex : "0.10,0.20" pour des ratages plus lointains).
RECOVERY_OFFSET = tuple(float(v) for v in
                        os.environ.get("ROBY_RECOVERY_OFFSET", "0.02,0.08").split(","))
RECOVERY_LIFT = 0.05              # "se lever un peu" avant de se recaler (m au-dessus du beside)
# 1 cm -> 0.5 cm (2026-09-07). Le 1 cm datait de juillet, quand la hauteur de prise avait
# une autre reference. Depuis, la pince n'est plus qu'a 1 cm de la table a D, et la correction
# radiale descend encore de 1.5 cm vers le bord proche : descendre d'1 cm de plus mettait les
# doigts EN APPUI sur le plateau. Reglable par ROBY_RECOVERY_EXTRA_DOWN.
RECOVERY_EXTRA_DOWN = float(os.environ.get("ROBY_RECOVERY_EXTRA_DOWN", "0.005"))
REC_SETTLE = 1.5                  # s : attente apres rec.start (le bag doit s'abonner AVANT
                                  # que le mouvement enregistre commence, sinon le debut manque)
N_TRY_VALID = 80                  # + de tirages : grande zone + rejet (keep-out/atteignabilite)
W_ORI_DESCENT = 0.5               # poids orientation des lignes droites. Avec le GRASP-AZIMUT
                                  # (cf Motion.ik) l'outil vertical est ATTEIGNABLE partout, donc
                                  # on peut tenir l'orientation fermement (0.5) => outil bien normal
                                  # au plan sur toute la zone, remontee qui ne diverge plus.


# ---- Correction d'inclinaison de table (calibree 2026-07-12, mesures Sam) ----
# La table reelle est inclinee vs le modele plat (z=D.z partout). Mesures :
#   - a D (rayon base->D ~= 0.80 m) : hauteur de prise CORRECTE (erreur 0)
#   - au plus proche (rayon 0.447 m, R ep.1 du run seed 2109787852) : la pince
#     s'arrete 5 cm TROP HAUT (la table reelle y est ~5 cm plus basse qu'a D).
# => plan incline RADIAL : la prise descend d'autant plus qu'on s'approche de la
# base. Nul a D, negatif (descend plus) pres de la base, positif (descend moins)
# au-dela de D. Affiner = changer TILT_DZ_NEAR / TILT_R_NEAR (ou re-sonder).
_R_D = float(np.hypot(D_XYZ[0], D_XYZ[1]))     # rayon base->D (~0.801 m)
# RE-CALIBRE le 2026-09-07 sur la cuisine, APRES compensation de l'echelle joint_3.
# Sam : la hauteur est bonne a la depose (= a D), mais il faut descendre 1.5 cm de plus
# au coin le plus proche du robot. C'est donc une correction RADIALE, nulle a D.
TILT_R_NEAR = 0.572                            # rayon du coin proche-droit mesure (m)
TILT_DZ_NEAR = -0.015                          # correction z a ce rayon : 1.5 cm plus bas
_TILT_SLOPE = TILT_DZ_NEAR / (TILT_R_NEAR - _R_D)   # ~0.142 m/m
# Bornes serrees : la calibration ne couvre que r = 0.572 .. 0.763. Au-dela on
# extrapolerait sans mesure (jusqu'a -3.3 cm au bord proche de la zone). On borne donc,
# quitte a s'arreter un peu trop HAUT loin de la bande calibree -- rater une pomme est
# recuperable, appuyer sur la table ne l'est pas.
TILT_DZ_CLAMP = (-0.020, 0.010)                # correction bornee (m)


def _table_dz(x, y):
    """Correction de hauteur (m) au point (x,y) : plan incline radial, nul a D."""
    r = float(np.hypot(x, y))
    dz = _TILT_SLOPE * (r - _R_D)
    lo, hi = TILT_DZ_CLAMP
    return float(min(hi, max(lo, dz)))


# Hauteur de prise sur la zone CUISINE — MESUREE SUR LE VRAI ROBOT (2026-09-07, Sam).
# Bras amene au-dessus de la zone : a fk z=+0.3316 la pince est a 1 cm de toucher la table.
# => surface reelle a fk z=+0.3216, prise a +0.3316 (1 cm de garde, doigts autour de la
#    pomme posee sans taper le plateau). Coincide avec le z du nouveau D, comme a l'atelier
#    ou Z_PICK valait deja exactement le z de D.
#
# ⚠️ POURQUOI L'ANCIENNE VALEUR ETAIT FAUSSE (0.2551, soit 6.6 cm SOUS la table) : elle
# venait du calcul "surface -0.007 + 26.21 cm de degagement releve a l'atelier". Ces 26.21 cm
# n'ont aucune realite physique — c'etait la difference entre une surface exprimee dans le
# repere MONDE (-0.190) et une prise exprimee dans le repere FK (+0.0721), deux reperes
# decales de ~10 cm en z. Le calcul melangeait donc deux referentiels, et le -0.007 lu dans
# cuisine.yaml (sommet des meubles, repere monde) a ete resservi tel quel en repere FK.
# LECON : cette hauteur se MESURE sur le robot, elle ne se deduit pas d'une autre scene.
# ⚠️ CES VALEURS SUPPOSENT LA COMPENSATION joint_3 ACTIVE (ROBY_J3_SCALE=0.9299).
# Sans elle, la hauteur n'est juste qu'au rayon de D et derive de +-2 cm ailleurs.
# Historique : 0.3316 avait ete calibre a D *avec* l'erreur d'echelle dedans ; une fois
# l'erreur compensee, le modele rejoint le reel et cette valeur devient 4 cm trop haute.
# Les deux points de mesure (D et coin proche-droit) donnent tous deux, independamment,
# le modele a 3.9 cm au-dessus de la table -- c'est cette coincidence qui valide le modele.
# Voir NOTES_echelle_joint3.md.
Z_SURFACE_CUISINE = 0.2916        # table, repere modele, compensation active
# Le -0.5 cm global essaye plus tot est ANNULE : Sam a precise que la hauteur est deja
# bonne a la depose. Le besoin n'etait pas uniforme -> il est traite par _table_dz.
CLEARANCE_PRISE = 0.010           # garde au contact a D
Z_PICK_CUISINE = Z_SURFACE_CUISINE + CLEARANCE_PRISE     # = +0.3016 m a D


def _z_pick(x=None, y=None):
    """Hauteur de prise sur la zone cuisine.

    REACTIVEE le 2026-09-07, re-calibree sur la cuisine (voir TILT_R_NEAR/TILT_DZ_NEAR) :
    nulle a D, -1.5 cm au coin proche-droit. Sans x,y (appel par defaut) on rend la
    hauteur a D, ce qui est exact puisque la correction y vaut zero.
    """
    if x is None or y is None:
        return Z_PICK_CUISINE
    return Z_PICK_CUISINE + _table_dz(x, y)


def _z_lift(z_target):
    """Hauteur du point d'approche = LIFT au-dessus de la CIBLE visee (relatif a chaque
    point, avant : LIFT au-dessus de D => plus que LIFT pour les points corriges bas)."""
    return z_target + LIFT


def _sampling_bounds():
    """Bornes externes du tirage = table retrecie de TABLE_MARGIN a chaque bord."""
    (xlo, xhi) = TABLE_PHYS["x"]
    (ylo, yhi) = TABLE_PHYS["y"]
    return ((xlo + TABLE_MARGIN, xhi - TABLE_MARGIN),
            (ylo + TABLE_MARGIN, yhi - TABLE_MARGIN))


def _zone_ok(x, y):
    """Point de tirage valide : hors keep-out D et hors keep-out base robot."""
    if np.hypot(x - D_XYZ[0], y - D_XYZ[1]) < D_KEEPOUT:   # trop pres de la depose
        return False
    if np.hypot(x, y) < BASE_KEEPOUT:                       # trop pres de la base robot
        return False
    if (abs(x - _BLK_C[0]) <= _BLK_HALF[0] + BLOCK_MARGIN   # dans l'empreinte du socle sous le bras
            and abs(y - _BLK_C[1]) <= _BLK_HALF[1] + BLOCK_MARGIN):
        return False
    return True


def _window():   # bornes externes (compat : print run() + markers RViz)
    return _sampling_bounds()


# Marge INTERIEURE aux butees (2026-09-07). Avant, le test tolerait 0.02 rad AU-DELA
# des butees : l'oracle validait une pose hors limite, le controleur la clampait a la
# butee exacte, et MoveIt refusait ensuite toute planification depuis cet etat
# ("Start state out of bounds") -- bras bloque, impossible d'en sortir par MoveIt.
# Vecu le 2026-09-07 apres 46 episodes : joint_5 fige a 1.6000000000000005 pour une
# butee a 1.6. On exige desormais de rester A L'INTERIEUR.
LIMIT_MARGIN = 0.05               # rad


def _in_limits(j):
    for k, jn in enumerate(J):
        lo, hi = LIMITS[jn]
        if not (lo + LIMIT_MARGIN <= j[k] <= hi - LIMIT_MARGIN):
            return False
    return True


# ================= Couche mouvement =================
class Motion:
    def __init__(self, mode, vel=0.30, cart_speed=0.02):
        self.mode = mode
        self.vel = vel                 # vitesse libre MoveIt (facteur d'echelle)
        self.cart_speed = cart_speed   # vitesse cartesienne des lignes droites (m/s)
        self.pk = None
        self.dls = None
        self.w_ori = 1.0
        if mode in ("plan", "real"):
            self._ros_init()

    def _ros_init(self):
        sys.path.insert(0, os.path.expanduser("~"))
        import roby_tool_pickup
        self.dls = roby_tool_pickup.dls
        # Descente = position prioritaire (5 DOF) : vaut pour l'atteignabilite (ik)
        # ET les lignes droites (roby_tool_pickup.straight lit DESCENT_W_ORI).
        roby_tool_pickup.DESCENT_W_ORI = W_ORI_DESCENT
        self.w_ori = W_ORI_DESCENT
        if self.mode == "real":
            import rclpy
            roby_tool_pickup.FREE_VEL = self.vel          # libre plus rapide (+ fluide : accel monte avec)
            roby_tool_pickup.CART_SPEED = self.cart_speed # lignes droites plus lentes (prehension propre)
            if not rclpy.ok():
                rclpy.init()
            self.pk = roby_tool_pickup.Pickup(dry=False)
            self.pk.wait_state()
            print(f"    [real] libre={self.vel} ligne_droite={self.cart_speed}m/s ; bras a {['%.2f'%v for v in (self.pk.cur or [])]}")

    def ik(self, xyz):
        """cartesien -> joints via DLS, GRASP-AZIMUT (2026-07-12).
        L'outil vise la VERTICALE (normal au plan) mais le LACET suit l'azimut du
        point : la base fait FACE au point => le vertical est atteignable PARTOUT,
        meme cross-body (y<0). Avant (R_GRASP fixe = lacet de D), aux azimuts
        eloignes, tenir ce lacet forcait une pose VRILLEE (jusqu'a 18 deg) qui
        faisait DIVERGER la remontee (abort 30% des points !). Ici : seed j1 vers
        le point + cible = R_GRASP tournee de dazi autour de la verticale.
        None si hors butees."""
        p = np.array(xyz, float)
        dazi = float(np.arctan2(p[1], p[0]) - _AZI_D)
        seed = np.array(D_JOINTS, float)
        seed[0] = D_JOINTS[0] + dazi                 # base face au point
        target_R = Rz(dazi) @ R_GRASP                # verticale, lacet suivant l'azimut
        j = self.dls(seed, p, target_R, iters=20, w_ori=self.w_ori)
        return j if _in_limits(j) else None

    def reachable(self, xyz):
        if self.mode == "dry":
            return True
        return self.ik(xyz) is not None

    def _track(self, jstart, pa, pb, keepR):
        """Suit la ligne droite pa->pb en DLS (tient keepR), comme straight().
        Retourne (pire_suivi_m, j_final) ou (None, j) si hors butees en route."""
        import roby_tool_pickup as tp
        j = np.array(jstart, float)
        N = max(2, int(np.ceil(np.linalg.norm(pb - pa) / tp.STEP)))
        worst = 0.0
        for i in range(1, N + 1):
            wp = pa + (i / N) * (pb - pa)
            j = self.dls(j, wp, keepR, w_ori=self.w_ori)
            if not _in_limits(j):
                return None, j
            worst = max(worst, np.linalg.norm(fk_pos(j) - wp))
        return worst, j

    def descent_ok(self, xyz):
        """True si descente (z_lift->xyz) ET remontee (xyz->z_lift) tiennent le suivi
        < 10mm. Reproduit le garde-fou de straight() DANS LES DEUX SENS => un R
        accepte ici ne peut plus avorter (la remontee cross-body etait le trou du
        garde-fou : on validait la descente mais pas la remontee)."""
        if self.mode == "dry":
            return True
        japp = self.ik((xyz[0], xyz[1], _z_lift(xyz[2])))
        if japp is None:
            return False
        above = fk_pos(np.array(japp, float))
        low = np.array([xyz[0], xyz[1], xyz[2]], float)
        keepR = fkT(np.array(japp, float))[:3, :3]
        d, jlow = self._track(japp, above, low, keepR)          # descente
        if d is None or d > 0.010:
            return False
        keepR2 = fkT(jlow)[:3, :3]
        r, _ = self._track(jlow, low, above, keepR2)            # remontee
        return r is not None and r <= 0.010

    def move_free(self, name, xyz):          # transit haut = MoveIt libre (anti-collision)
        if self.mode in ("dry", "plan"):
            tag = "[transit]" if self.mode == "dry" else "[transit] IK=%s" % ("OK" if self.reachable(xyz) else "HORS-BUTEE")
            print(f"    {tag:22s} {name:16s} -> (x={xyz[0]:+.3f} y={xyz[1]:+.3f} z={xyz[2]:+.3f})")
            return self.reachable(xyz)
        j = self.ik(xyz)
        if j is None:
            print(f"    [transit] {name}: IK hors butee"); return False
        return self.pk.free_to(list(j), name)

    def descend(self, name, xyz):            # descente/remontee = LIGNE DROITE DLS verticale
        if self.mode in ("dry", "plan"):
            print(f"    {'[ligne droite]':22s} {name:16s} -> (x={xyz[0]:+.3f} y={xyz[1]:+.3f} z={xyz[2]:+.3f})")
            return True
        return self.pk.straight(np.array(xyz, float), name)

    def gripper(self, close):
        if self.mode in ("dry", "plan"):
            print(f"    [pince] {'FERME' if close else 'OUVRE'}")
            return
        self.pk.grip(close)

    def pre_record_settle(self, close=False):
        """Debut de fenetre enregistree : (1) laisse le bag s'abonner (REC_SETTLE)
        pour ne pas manquer le debut ; (2) RE-AFFIRME l'etat pince initial => la
        consigne pince est DANS le bag (topic evenementiel : sinon le 1er /gripper du
        bag serait le prochain changement, l'etat de depart serait perdu).
        close=False (normal, pince ouverte prete a saisir) ; close=True (RATTRAPAGE :
        pince FERMEE sur du vide = la prise vient d'echouer)."""
        etat = "FERMEE" if close else "OUVERTE"
        if self.mode in ("dry", "plan"):
            print(f"    [REC] settle + re-affirme pince {etat}")
            return
        time.sleep(REC_SETTLE)
        self.gripper(close=close)   # etat initial capture dans le bag
        time.sleep(0.3)

    # --- prise/depose : approche 5cm AVANT + descente droite + APRES remontee 5cm ---
    def pick_at(self, name, xyz):
        above = (xyz[0], xyz[1], _z_lift(xyz[2]))
        if not self.move_free(f"approche_{name}", above): return False   # AVANT : 5cm au-dessus
        if not self.descend(name, xyz): return False                    # descente droite DLS
        self.gripper(close=True)
        if not self.descend(f"remonte_{name}", above): return False      # APRES : remontee 5cm
        return True

    def place_at(self, name, xyz):
        above = (xyz[0], xyz[1], _z_lift(xyz[2]))
        if not self.move_free(f"approche_{name}", above): return False   # AVANT
        if not self.descend(name, xyz): return False
        self.gripper(close=False)
        if not self.descend(f"remonte_{name}", above): return False      # APRES
        return True


# ================= Tirages =================
def sample_table(motion, rng):
    (xlo, xhi), (ylo, yhi) = _sampling_bounds()
    for _ in range(N_TRY_VALID):
        x, y = rng.uniform(xlo, xhi), rng.uniform(ylo, yhi)
        if not _zone_ok(x, y):
            continue
        xyz = (x, y, _z_pick(x, y))                            # z corrige de l'inclinaison
        if motion.reachable(xyz) and motion.descent_ok(xyz):   # R garanti sans abort
            return xyz
    return None


def sample_depose(motion, rng):
    """Point de depose tire uniformement dans un disque de rayon D_JITTER autour de D.

    Meme validation qu'un point de prise (atteignabilite + descente/remontee), donc une
    depose ne peut pas avorter en cours d'episode. Retombe sur D si aucun tirage ne passe
    (D lui-meme est valide par construction : c'est une pose relevee sur le robot).
    """
    for _ in range(N_TRY_VALID):
        a = rng.uniform(0.0, 2.0 * np.pi)
        r = D_JITTER * np.sqrt(rng.uniform(0.0, 1.0))    # sqrt => uniforme en SURFACE
        px, py = D_XYZ[0] + r * np.cos(a), D_XYZ[1] + r * np.sin(a)
        xyz = (px, py, _z_pick(px, py))
        if motion.reachable(xyz) and motion.descent_ok(xyz):
            return xyz
    return (D_XYZ[0], D_XYZ[1], _z_pick(D_XYZ[0], D_XYZ[1]))


def sample_beside(motion, R, rng):
    """Scenario rattrapage : pose 'ratee' a cote de la pomme R, AU RAS de la table,
    decalee horizontalement de RECOVERY_OFFSET dans une direction aleatoire. Meme
    validation qu'un point de prise (zone + atteignabilite + descente/remontee), donc
    le rattrapage ne peut pas avorter au demarrage. None si aucun essai ne passe."""
    for _ in range(N_TRY_VALID):
        ang = rng.uniform(-np.pi, np.pi)
        mag = rng.uniform(*RECOVERY_OFFSET)
        x = R[0] + mag * np.cos(ang)
        y = R[1] + mag * np.sin(ang)
        if not _zone_ok(x, y):
            continue
        xyz = (x, y, _z_pick(x, y) - RECOVERY_EXTRA_DOWN)      # 1cm SOUS la prise (pince au ras table)
        if motion.reachable(xyz) and motion.descent_ok(xyz):
            return xyz
    return None


def sample_aerien(motion, rng):
    (xlo, xhi), (ylo, yhi) = _sampling_bounds()
    for _ in range(N_TRY_VALID):
        x, y = rng.uniform(xlo, xhi), rng.uniform(ylo, yhi)
        if not _zone_ok(x, y):
            continue
        xyz = (x, y, D_XYZ[2] + rng.uniform(*AERIEN_DZ))
        if motion.reachable(xyz):
            return xyz
    return None



def _angles_pince_reels():
    """Lit les angles de pince DANS LA CONFIG CHARGEE, au lieu de les recopier.

    Avant (2026-07-20), ces valeurs etaient ecrites en dur dans la fiche d'episode :
    110/70, alors que le xacro reellement charge disait 110/75 depuis le reglage du
    serrage. Les fiches affirmaient donc un angle de fermeture qui n'avait jamais ete
    execute -- exactement le genre de detail qui fait chercher au mauvais endroit des
    mois plus tard. En cas d'echec de lecture on renvoie None : une fiche qui dit
    "je ne sais pas" vaut mieux qu'une fiche qui se trompe.
    """
    import re
    for base in (os.path.expanduser("~/rlgr"), os.path.expanduser("~/ros2_ws")):
        f = os.path.join(base, "src/roby_hardware/config/"
                               "roby_hardware_steppers_only.ros2_control.xacro")
        try:
            txt = open(f, encoding="utf-8").read()
        except OSError:
            continue
        o = re.search(r'name="gripper_open_deg">([0-9.]+)<', txt)
        c = re.search(r'name="gripper_closed_deg">([0-9.]+)<', txt)
        if o and c:
            return {"open_deg_stack": float(o.group(1)),
                    "closed_deg_stack": float(c.group(1)),
                    "source": f}
    return {"open_deg_stack": None, "closed_deg_stack": None,
            "source": "NON LU - ne pas se fier a ces valeurs"}

def _episode_meta(i, seed, mode, R, vel, cart_speed, recovery=False, beside=None, depose=None):
    """Fiche d'infos d'un episode enregistre (ecrite a cote du bag : <ep>.meta.json)."""
    import datetime
    (xw, yw) = _window()
    meta = {
        "date": datetime.datetime.now().isoformat(timespec="seconds"),
        "batch_seed": seed,                 # identifie la serie (rejouable via --seed)
        "episode": i,
        "mode": mode,
        "scenario": "recovery" if recovery else "normal",
        "objet_manipule": "pomme blanche imprimee en 3D",
        "fenetre_enregistree": (
            ("RATTRAPAGE : depart A COTE de la pomme au ras de la table, pince FERMEE sur du vide "
             "(la prise vient d'echouer) -> se releve un peu -> ROUVRE -> se recale au-dessus de R -> "
             "prend -> depose a D -> lache. Le placement de l'objet a R ET la mise en position 'ratee' "
             "(setup) ne sont PAS enregistres.")
            if recovery else
            ("A_aerien -> prise de l'objet a R -> depose a D -> lache. "
             "Le PLACEMENT de l'objet a R (setup) n'est PAS enregistre.")),
        "pick_point_R": {"x": round(float(R[0]), 4), "y": round(float(R[1]), 4), "z": round(float(R[2]), 4)},
        # Point de depose REEL de cet episode (randomise dans un disque de D_JITTER
        # autour de D). C'est la que la pomme se trouvera au debut de l'episode suivant.
        "place_point_D": {"x": round(float((depose or D_XYZ)[0]), 4),
                          "y": round(float((depose or D_XYZ)[1]), 4),
                          "z": round(float((depose or D_XYZ)[2]), 4)},
        "place_randomisation_rayon_m": D_JITTER,
        "place_point_D_nominal": {"x": round(float(D_XYZ[0]), 4), "y": round(float(D_XYZ[1]), 4),
                                  "z": round(float(D_XYZ[2]), 4)},
        "generation_cible": {
            "methode": ("tirage uniforme (x,y) sur la table, rejets keep-out (bords/D/base/socle), "
                        "z corrige par plan incline radial, IK grasp-azimut (outil vertical, lacet "
                        "suivant l'azimut), validation descente ET remontee"),
            "fenetre_tirage_x": [round(xw[0], 3), round(xw[1], 3)],
            "fenetre_tirage_y": [round(yw[0], 3), round(yw[1], 3)],
            "z_correction": {"pente_m_par_m": round(_TILT_SLOPE, 4), "r_D": round(_R_D, 3),
                             "cal_r_proche": TILT_R_NEAR, "cal_dz_proche": TILT_DZ_NEAR},
            "w_ori_descente": W_ORI_DESCENT,
            "lift_m": LIFT,
        },
        "pince": dict({"topic": "/gripper Bool (true=FERME, false=OUVRE)"},
                       **_angles_pince_reels()),
        "vitesses": {"libre_moveit": vel, "ligne_droite_m_s": cart_speed},
    }
    if recovery and beside is not None:
        meta["start_beside_rate"] = {"x": round(float(beside[0]), 4), "y": round(float(beside[1]), 4),
                                     "z": round(float(beside[2]), 4),
                                     "decalage_vs_R_m": round(float(np.hypot(beside[0] - R[0],
                                                                             beside[1] - R[1])), 4)}
        meta["recovery_params"] = {"offset_m": list(RECOVERY_OFFSET), "lift_m": RECOVERY_LIFT}
    return meta


def make_recorder(mode, out_dir, no_record=False):
    if mode in ("dry", "plan") or no_record:
        class _RecDry:
            def start(self, name, meta=None):
                extra = f"  (R={meta['pick_point_R']})" if meta else ""
                print(f"    [REC] start -> {name}{extra}"); return name
            def stop(self): print("    [REC] stop")
        return _RecDry()
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from roby_recorder import Recorder, TOPICS
    # Selection des cameras via env ROBY_CAMS : "left" (exterieure seule),
    # "right" (poignet seule), "both" (defaut). Le reste des topics est garde.
    cams = os.environ.get("ROBY_CAMS", "both").lower()
    if cams == "left":
        topics = [t for t in TOPICS if "head_camera/right" not in t]
    elif cams == "right":
        topics = [t for t in TOPICS if "head_camera/left" not in t]
    else:
        topics = TOPICS
    print(f"    [REC] cameras={cams}  ({len(topics)} topics)")
    return Recorder(out_dir, topics=topics)


def run(mode, n_episodes, out_dir, seed, vel, cart_speed, no_record=False, recovery=False):
    # seed None => graine OS (vraiment aleatoire, differente a chaque run).
    # On l'imprime : si une serie est bonne, on la rejoue avec --seed <valeur>.
    if seed is None:
        seed = random.SystemRandom().randint(0, 2**31 - 1)
    print(f"[seed] {seed}   (pour rejouer cette meme serie : --seed {seed})")
    rng = random.Random(seed)
    motion = Motion(mode, vel, cart_speed)
    # Sous-dossier de run (batch) : noms d'episodes uniques => plus de collision
    # ep_000 entre deux runs (ros2 bag record REFUSE un dossier deja existant).
    import datetime
    _pref = "batch_recovery" if recovery else "batch"
    batch_dir = os.path.join(out_dir, f"{_pref}_{seed}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")
    rec = make_recorder(mode, batch_dir, no_record)
    if mode == "real":
        print("*** --no-record : le bras BOUGE mais AUCUN bag n'est enregistre (verif) ***"
              if no_record else f"Bags -> {batch_dir}")

    (xw, yw) = _window()
    print(f"=== ORACLE mode={mode}  episodes={n_episodes} ===")
    print(f"D (repere FK) = ({D_XYZ[0]:.3f}, {D_XYZ[1]:.3f}, {D_XYZ[2]:.3f})   z_pick={_z_pick():.3f}  lift=+{LIFT:.2f} (relatif cible)")
    print(f"Fenetre tirage: x{tuple(round(v,3) for v in xw)} y{tuple(round(v,3) for v in yw)}   aerien z=D.z+{AERIEN_DZ}")
    print(f"Depose RANDOMISEE dans un disque de {D_JITTER*100:.0f} cm de rayon autour de D ; "
          f"l'episode suivant reprend la pomme la ou elle a ete posee.")
    print("L'objet (pomme blanche imprimee 3D) doit etre a D au depart.\n")

    motion.gripper(close=False)   # demarre pince OUVERTE (prete a saisir l'objet a D)

    # Ou se trouve REELLEMENT la pomme. Au depart : a D, posee la par Sam. Ensuite : au
    # dernier point de depose randomise. Sans ce suivi, l'episode suivant irait prendre a D
    # une pomme qui est jusqu'a D_JITTER plus loin -> il se refermerait sur du vide.
    # ⚠️ z = _z_pick(), PAS D_XYZ[2] : D_XYZ vient de la pose articulaire relevee, dont le z
    # (0.332) est anterieur a la correction de hauteur de prise (0.302). Prendre D_XYZ[2] tel
    # quel rendait la TOUTE PREMIERE prise 3 cm trop haute -- la pince se refermait sur du vide.
    d_cur = (D_XYZ[0], D_XYZ[1], _z_pick(D_XYZ[0], D_XYZ[1]))

    for i in range(n_episodes):
        print(f"--- Episode {i:03d} ---")
        if not motion.pick_at("cone_D", d_cur):
            print("  !! ECHEC prise D -> ARRET"); break
        R = sample_table(motion, rng)
        if R is None:
            print("  !! pas de R atteignable, saut"); continue
        if not motion.place_at("R_aleatoire", R):
            print("  !! ECHEC pose R -> ARRET"); break
        if recovery:
            # --- SCENARIO RATTRAPAGE ---
            B = sample_beside(motion, R, rng)
            if B is None:
                print("  !! pas de pose 'ratee' atteignable, saut"); continue
            B_above = (B[0], B[1], B[2] + RECOVERY_LIFT)      # 'se lever un peu' au-dessus du beside
            # SETUP (NON enregistre) : aller au-dessus, COMMENCER A FERMER la pince, PUIS descendre
            # sur du vide = la prise vient d'echouer (pince deja en fermeture en descendant, demande Sam).
            if not motion.move_free("rate_approche", B_above):
                print("  !! ECHEC approche pose ratee -> ARRET"); break
            motion.gripper(close=True)                        # commence a fermer AVANT de descendre
            if not motion.descend("rate_beside", B):
                print("  !! ECHEC descente pose ratee -> ARRET"); break
            d_next = sample_depose(motion, rng)
            rec.start(f"ep_{i:03d}", _episode_meta(i, seed, mode, R, vel, cart_speed,
                                                   recovery=True, beside=B, depose=d_next))
            motion.pre_record_settle(close=True)   # etat initial DANS le bag : pince FERMEE (rate), a cote
            # RATTRAPAGE enregistre : se lever -> ROUVRIR -> se recaler+ramasser -> amener a D -> lacher
            ok = motion.descend("releve", B_above)         # se lever un peu (pince encore fermee)
            if ok:
                motion.gripper(close=False)                # ROUVRIR la pince (prete a re-saisir)
                ok = motion.pick_at("R_connu", R) and motion.place_at("cone_D", d_next)
            rec.stop()
        else:
            A = sample_aerien(motion, rng)
            if A is None:
                print("  !! pas de A atteignable, saut"); continue
            if not motion.move_free("aerien_A", A):
                print("  !! ECHEC aerien -> ARRET"); break
            d_next = sample_depose(motion, rng)
            rec.start(f"ep_{i:03d}", _episode_meta(i, seed, mode, R, vel, cart_speed, depose=d_next))
            motion.pre_record_settle()   # bag pret + etat pince initial (OUVERT) dans le bag
            ok = motion.pick_at("R_connu", R) and motion.place_at("cone_D", d_next)
            rec.stop()
        if not ok:
            print("  !! ECHEC pendant enregistrement -> ARRET"); break
        # La depose a reussi : la pomme est maintenant a d_next. En cas d'echec on ne met
        # PAS a jour d_cur -- on ignore ou la pomme a atterri, et la boucle s'arrete.
        d_cur = tuple(d_next)
        # ⚠️ La chaine "OK (objet revenu" est un CONTRAT avec roby_collect_panel.py, qui
        # detecte la reussite d'un episode en la cherchant dans cette sortie. La reformuler
        # fait jeter automatiquement TOUS les episodes (vecu le 2026-09-07). Ne pas y toucher.
        print(f"  episode {i:03d} OK (objet revenu a D, depose a "
              f"{d_next[0]:+.3f},{d_next[1]:+.3f} = D{np.hypot(d_next[0]-D_XYZ[0], d_next[1]-D_XYZ[1])*100:+.1f}cm)\n")

    print("=== fin ===")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["dry", "plan", "real"], default="dry")
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--out", default="~/roby_datasets")
    ap.add_argument("--seed", type=int, default=None,
                    help="graine du tirage (defaut: aleatoire OS, imprimee au demarrage)")
    ap.add_argument("--go", action="store_true", help="requis pour --mode real (bras reel)")
    ap.add_argument("--vel", type=float, default=0.30, help="vitesse libre MoveIt (defaut 0.30)")
    ap.add_argument("--cart-speed", type=float, default=0.02, help="vitesse ligne droite m/s (defaut 0.02)")
    ap.add_argument("--no-record", action="store_true", help="bouge le bras SANS enregistrer (verif)")
    ap.add_argument("--recovery", action="store_true",
                    help="scenario RATTRAPAGE : depart A COTE de la pomme au ras de la table (rate), "
                         "se releve, se recale, ramasse, depose. Batch dedie batch_recovery_*.")
    args = ap.parse_args()
    if args.mode == "real" and float(os.environ.get("ROBY_J3_SCALE", "0") or 0) <= 0:
        # Z_SURFACE_CUISINE et les corrections de table SUPPOSENT la compensation joint_3
        # active. Sans elle, la prise a D reste juste mais derive de +-2 cm ailleurs,
        # jusque DANS la table au coin proche. Aucun lanceur ne la posait (revue du
        # 2026-09-13) : on refuse plutot que de collecter dans la table.
        sys.exit("REFUS : --mode real sans ROBY_J3_SCALE. Les hauteurs de prise supposent "
                 "la compensation joint_3 : export ROBY_J3_SCALE=0.9299 (valeur du panneau "
                 "d'inference), cf. NOTES_echelle_joint3.md.")
    if args.mode == "real" and not args.go:
        print("REFUS : --mode real bouge le BRAS REEL. Relance avec --go (Sam present).")
        return
    run(args.mode, args.episodes, os.path.expanduser(args.out), args.seed, args.vel, args.cart_speed,
        args.no_record, recovery=args.recovery)


if __name__ == "__main__":
    main()
