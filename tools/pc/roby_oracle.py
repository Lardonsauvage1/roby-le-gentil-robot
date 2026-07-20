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

# Point de depose D : ancre sur les JOINTS captures (2026-07-11, roby_poses.yaml:D_pose_cone).
# Reprise : joint_5 redresse (0.33 -> 1.01) pour une prise QUASI TOP-DOWN
# (axe doigts ~6 deg de la verticale, vs 32 deg avant).
D_JOINTS = [0.5649, 1.2491, -0.5935, -0.0557, 1.0070]
D_XYZ = tuple(float(v) for v in fk_pos(D_JOINTS))     # en repere FK (coherent DLS)
R_GRASP = fkT(D_JOINTS)[:3, :3]                        # orientation de prise (top-down)
_AZI_D = float(np.arctan2(D_XYZ[1], D_XYZ[0]))         # azimut base->D (repere du grasp-azimut)

# Plan physique (roby_environments/atelier_actuel.yaml) : borne dure anti-hors-table.
TABLE_PHYS = {"x": (-0.258, 1.043), "y": (-0.218, 0.543)}
# Zone de tirage = TOUTE la table sauf : marge aux bords, disque autour de la base
# robot, et disque d'exclusion autour du point de depose D (demande Sam 2026-07-11).
TABLE_MARGIN = 0.05               # marge a chaque bord de table (m) : jamais colle au bord
D_KEEPOUT = 0.05                  # rayon d'exclusion autour de D (m) : R jamais < 5cm de D
BASE_KEEPOUT = 0.30               # rayon d'exclusion autour de la base robot (origine)
# Socle 'base_robot' (atelier_actuel.yaml) : box size[0.515,0.875] centre (0.0,0.0925).
# = le bloc juste sous le bras -> on exclut son empreinte au sol + marge.
BLOCK_MARGIN = 0.05
_BLK_C = (0.00, 0.0925)
_BLK_HALF = (0.515 / 2, 0.875 / 2)
LIFT = 0.05                       # point d'approche 5cm AU-DESSUS de la CIBLE (avant : 10cm)
AERIEN_DZ = (0.18, 0.30)          # hauteur aerienne = z de D + [min,max]
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
TILT_R_NEAR = 0.447                            # rayon du point de calibration proche (m)
TILT_DZ_NEAR = -0.05                           # correction z a ce rayon (m) : 5 cm plus bas
_TILT_SLOPE = TILT_DZ_NEAR / (TILT_R_NEAR - _R_D)   # ~0.142 m/m
TILT_DZ_CLAMP = (-0.09, 0.05)                  # garde-fou anti-crash : correction bornee (m)


def _table_dz(x, y):
    """Correction de hauteur (m) au point (x,y) : plan incline radial, nul a D."""
    r = float(np.hypot(x, y))
    dz = _TILT_SLOPE * (r - _R_D)
    lo, hi = TILT_DZ_CLAMP
    return float(min(hi, max(lo, dz)))


def _z_pick(x=None, y=None):
    """Hauteur de prise. Sans (x,y) = z de D (retro-compat print/aerien).
    Avec (x,y) = z de D + correction d'inclinaison de table au point vise."""
    if x is None:
        return D_XYZ[2]
    return D_XYZ[2] + _table_dz(x, y)


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


def _in_limits(j):
    for k, jn in enumerate(J):
        lo, hi = LIMITS[jn]
        if not (lo - 0.02 <= j[k] <= hi + 0.02):
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

    def pre_record_settle(self):
        """Debut de fenetre enregistree : (1) laisse le bag s'abonner (REC_SETTLE)
        pour ne pas manquer le debut ; (2) RE-AFFIRME l'etat pince OUVERTE => la
        consigne pince initiale est DANS le bag (topic evenementiel : sinon le 1er
        /gripper du bag serait la fermeture a R, l'etat de depart serait perdu)."""
        if self.mode in ("dry", "plan"):
            print("    [REC] settle + re-affirme pince OUVERTE")
            return
        time.sleep(REC_SETTLE)
        self.gripper(close=False)   # etat initial OUVERT, capture dans le bag
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

def _episode_meta(i, seed, mode, R, vel, cart_speed):
    """Fiche d'infos d'un episode enregistre (ecrite a cote du bag : <ep>.meta.json)."""
    import datetime
    (xw, yw) = _window()
    return {
        "date": datetime.datetime.now().isoformat(timespec="seconds"),
        "batch_seed": seed,                 # identifie la serie (rejouable via --seed)
        "episode": i,
        "mode": mode,
        "objet_manipule": "pomme blanche imprimee en 3D",
        "fenetre_enregistree": ("A_aerien -> prise de l'objet a R -> depose a D -> lache. "
                                "Le PLACEMENT de l'objet a R (setup) n'est PAS enregistre."),
        "pick_point_R": {"x": round(float(R[0]), 4), "y": round(float(R[1]), 4), "z": round(float(R[2]), 4)},
        "place_point_D": {"x": round(float(D_XYZ[0]), 4), "y": round(float(D_XYZ[1]), 4), "z": round(float(D_XYZ[2]), 4)},
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


def run(mode, n_episodes, out_dir, seed, vel, cart_speed, no_record=False):
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
    batch_dir = os.path.join(out_dir, f"batch_{seed}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")
    rec = make_recorder(mode, batch_dir, no_record)
    if mode == "real":
        print("*** --no-record : le bras BOUGE mais AUCUN bag n'est enregistre (verif) ***"
              if no_record else f"Bags -> {batch_dir}")

    (xw, yw) = _window()
    print(f"=== ORACLE mode={mode}  episodes={n_episodes} ===")
    print(f"D (repere FK) = ({D_XYZ[0]:.3f}, {D_XYZ[1]:.3f}, {D_XYZ[2]:.3f})   z_pick={_z_pick():.3f}  lift=+{LIFT:.2f} (relatif cible)")
    print(f"Fenetre tirage: x{tuple(round(v,3) for v in xw)} y{tuple(round(v,3) for v in yw)}   aerien z=D.z+{AERIEN_DZ}")
    print("L'objet (pomme blanche imprimee 3D) doit etre a D au depart ; il revient a D a chaque fin d'episode.\n")

    motion.gripper(close=False)   # demarre pince OUVERTE (prete a saisir l'objet a D)

    for i in range(n_episodes):
        print(f"--- Episode {i:03d} ---")
        if not motion.pick_at("cone_D", D_XYZ):
            print("  !! ECHEC prise D -> ARRET"); break
        R = sample_table(motion, rng)
        if R is None:
            print("  !! pas de R atteignable, saut"); continue
        if not motion.place_at("R_aleatoire", R):
            print("  !! ECHEC pose R -> ARRET"); break
        A = sample_aerien(motion, rng)
        if A is None:
            print("  !! pas de A atteignable, saut"); continue
        if not motion.move_free("aerien_A", A):
            print("  !! ECHEC aerien -> ARRET"); break
        rec.start(f"ep_{i:03d}", _episode_meta(i, seed, mode, R, vel, cart_speed))
        motion.pre_record_settle()   # bag pret + etat pince initial (OUVERT) dans le bag
        ok = motion.pick_at("R_connu", R) and motion.place_at("cone_D", D_XYZ)
        rec.stop()
        if not ok:
            print("  !! ECHEC pendant enregistrement -> ARRET"); break
        print(f"  episode {i:03d} OK (objet revenu a D)\n")

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
    args = ap.parse_args()
    if args.mode == "real" and not args.go:
        print("REFUS : --mode real bouge le BRAS REEL. Relance avec --go (Sam present).")
        return
    run(args.mode, args.episodes, os.path.expanduser(args.out), args.seed, args.vel, args.cart_speed,
        args.no_record)


if __name__ == "__main__":
    main()
