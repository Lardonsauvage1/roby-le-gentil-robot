# Roby le gentil robot

Bras robotique 5 axes DIY pour l'apprentissage par imitation (Diffusion Policy, LeRobot),
piloté par ROS 2 Jazzy, ros2_control et MoveIt 2. État du dépôt au 2026-09-13.

## Architecture (« B », deux machines)

| Machine | Rôle | Réseau |
|---|---|---|
| **Pi5** (`roby-desktop`) | temps réel : `ros2_control` (`RobySystem`, 100 Hz), `robot_state_publisher`, caméras de la tête | `192.168.2.37` |
| **PC** (`sam-AtomMan`) | MoveIt `move_group` + RViz, scène de collision, garde, inférence (iGPU, OpenVINO), bras guide, outils | `192.168.2.95` |

CycloneDDS des deux côtés, `ROS_DOMAIN_ID=42`, réseau dédié `192.168.2.x`.
Reconstruire une machine : [`deploy/pi5/README.md`](deploy/pi5/README.md),
[`deploy/pc/README.md`](deploy/pc/README.md).

## Contenu du dépôt

| Chemin | Contenu |
|---|---|
| `src/neuroneimitationcarote_description` | URDF/xacro du bras + maillages STL (« Roby » dans la doc, `neuroneimitationcarote` dans le code) |
| `src/neuroneimitationcarote_moveit_config` | configuration MoveIt 2 ; `pc_moveit.launch.py` = côté PC de l'architecture B |
| `src/roby_hardware` | plugin C++ ros2_control `RobySystem` : steppers en GPIO (axes 1-3), servos PCA9685 (axes 4-5, verrou de tête, pince), joint BLDC optionnel (`wrist:=bldc`) ; `robot_control.launch.py` = côté Pi5 |
| `src/roby_environments` | scènes de collision MoveIt (`cuisine`, 35 obstacles ; `atelier_actuel`) |
| `src/roby_control` | bras guide SO-ARM 101 (`leader_node`, correspondance, joystick) et téléopérations : bras **simulé** (`leader_teleop_sim`, `_pos`, `_cart`) et **vrai** bras via le garde (`leader_teleop_reel`) ; nœuds ArUco / suivi visuel (non modifiés depuis juin 2026) |
| `src/roby_wrist_bldc` | poignet axe 5 BLDC (carte B-G431B-ESC1 + SimpleFOC) : driver série, nœud ROS, pont vers `RobySystem` — **pas encore monté sur le robot** |
| `firmware/wrist_bldc_simplefoc` | firmware PlatformIO de la carte du poignet BLDC — **seule copie** |
| `tools/pc`, `tools/pi5` | outillage opérationnel de chaque machine, appelé par des liens depuis le home — voir [`tools/README.md`](tools/README.md) |
| `CABLAGE.md` | **fiche unique** du câblage et de l'adressage (GPIO, I2C, USB, réseau) |
| `deploy/` | ce qu'il faut sur chaque machine en dehors du workspace (DDS, système, udev…) |
| `hot_reload_urdf.py` | outil de mise au point de l'URDF : historique |
| `launch_sim.sh` | MoveIt seul avec un robot simulé (`demo.launch.py`), sur le domaine **43** |

## Lancer le vrai bras

La procédure fait foi : skill **`/roby-lancer-bras`** du dépôt
[`roby-specs`](https://github.com/Lardonsauvage1/roby-specs) (nettoyage des processus,
stack Pi5, vérifications, MoveIt côté PC, scène de collision, sortie du nid). En résumé :

```bash
# Pi5, tete posee dans le nid
ros2 launch roby_hardware robot_control.launch.py          # wrist:=bldc : poignet BLDC
bash ~/launch_cams.sh
# PC
ros2 launch neuroneimitationcarote_moveit_config pc_moveit.launch.py
ros2 run roby_environments scene_loader --env cuisine
```

`robot_full.launch.py` est **obsolète** (move_group sur le Pi5, course « mock »).

## Règles de sécurité

- **Aucun mouvement des moteurs sans le feu vert explicite de Sam.**
- Tête posée **dans le nid** avant de lancer la stack : la référence est en boucle ouverte.
- **Un seul** publisher de `/joint_states` et de `/robot_description` : ceux du Pi5
  (BUG-008). Toute simulation se fait sur un autre domaine (`ROS_DOMAIN_ID=43`).
- Scène de collision **chargée** avant tout mouvement planifié ou gardé.
- Tout producteur d'angles (modèle IA, téléopération du vrai bras) passe par le **garde**
  (`tools/pc/roby_guard.py`, `/guard/joint_trajectory`) : butées, vitesse, plancher,
  collision MoveIt. Le jog fin (`roby_fine_jog`) envoie directement à `arm_controller`.

## Tests (sans matériel)

Voir [`deploy/pc/README.md`](deploy/pc/README.md#tests-sans-matériel) : `roby_control`
(bras guide, téléopération, 97 tests), `roby_wrist_bldc` (67 tests, carte simulée), pont
BLDC de `roby_hardware` (3 tests) et gtests de `roby_hardware` (66 tests).

## Dépôts liés

- [`roby-specs`](https://github.com/Lardonsauvage1/roby-specs) : spécifications, user
  stories, ADR, bugs et procédures (`/roby-*`).
- `lerobot-experiments` : entraînement et évaluation des modèles (LeRobot).
- `NIC_forge` (organisation) : expériences (EXP) et décisions, qui citent ce dépôt par
  commit et empreinte.
