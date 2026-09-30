# Câblage et adressage de Roby — fiche unique

**Seule fiche de câblage du projet.** Tout autre document renvoie ici au lieu de recopier.
Les valeurs logicielles (broches, canaux, ports) sont lues par le code dans
[`src/roby_hardware/config/roby_hardware_steppers_only.ros2_control.xacro`](src/roby_hardware/config/roby_hardware_steppers_only.ros2_control.xacro)
(inclus par `roby_motor1_test.urdf.xacro`, chargé par `robot_control.launch.py`) : **si ce
fichier et cette fiche divergent, le xacro fait foi et cette fiche est à corriger.**

Dernière vérification sur le robot : **2026-09-30**, après recâblage (lecture seule).

## Machines et réseau

Sous-réseau dédié `192.168.2.x` (routeur Edimax, `192.168.2.1`).

| Machine | Nom | Adresse | Utilisateur |
|---|---|---|---|
| Raspberry Pi 5 (temps réel, caméras de tête) | `roby-desktop` | `192.168.2.37` | `roby` (clé ed25519) |
| Raspberry Pi 3 (caméra USB extérieure) | `roby-cam` | `192.168.2.4` | `sam` |
| PC (MoveIt, RViz, inférence) | `sam-AtomMan` | `192.168.2.95` (câble sur `enp86s0`) | `sam` |

DDS : `deploy/pc/cyclone_config.xml` → `~/cyclone_config.xml` (PC),
`deploy/pi5/cyclone_config.xml` → `~/cyclone_config.xml` (Pi5), `ROS_DOMAIN_ID=42`.
Adresses dans le code : `src/roby_control/roby_control/bringup/stack_spec.py` (`PI5`),
`tools/pc/roby_env.sh`.

## Pi 5 — GPIO (contrôleur `/dev/gpiochip4`, le RP1 ; **pas** gpiochip0)

| Axe | Moteur / driver | STEP | DIR | Réduction |
|---|---|---|---|---|
| 1 (base) | stepper classique, boucle ouverte | GPIO17 (broche 11) | GPIO27 (broche 13) | 16/85 |
| 2 (épaule) | NEMA 34 12 Nm à encodeur intégré + driver **CL86Y** boucle fermée | GPIO22 (broche 15) | GPIO23 (broche 16) | 15/44 |
| 3 (coude) | NEMA 34 12 Nm à encodeur intégré + driver **CL86Y** boucle fermée | GPIO24 (broche 18) | GPIO25 (broche 22) | 300/1408, inversé |

- 12 800 impulsions/tour (DIP des drivers CL86Y réglés pareil). PUL-/DIR- des CL86Y au GND du Pi.
- Couplage mécanique axes 2/3 : `coupling_ratio_m2` = -6000/45056.
- La boucle fermée est **interne aux CL86Y** : aucun retour de position vers le Pi. La
  position publiée est le compteur de pas.
- Les encodeurs externes RS-485 AS5048A (UART0 `/dev/ttyAMA0`, DE/RE GPIO26) sont **retirés
  du robot et du code** depuis 2026-09-30.

## Pi 5 — I2C bus 1 (`/dev/i2c-1`, GPIO2/3) : PCA9685 en `0x40`

(`0x70` visible dans `i2cdetect` = adresse « all-call » du PCA9685, normal.)

| Canal | Fonction | Réglages | Code |
|---|---|---|---|
| CH0 | axe 4, roulis poignet (servo MG996R) | 0–270°, init 135° | xacro `joint_4_*`, `tools/pi5/axe4_zero.py` |
| CH1 | axe 5, tangage poignet — servo **provisoire** (`wrist:=servo`) | 0–180°, init 41,3°, inversé | xacro `joint_5_*` |
| CH2 | verrou de tête (changeur d'outil) | **50° = verrouillé, 75° = déverrouillé** | `tools/pi5/head_lock_node.py`, `lock_servo.py` |
| CH3 | pince | fermée 75°, ouverte 110° | xacro `gripper_*_deg`, `tools/pi5/gripper_node.py` |
| CH7 | électrovanne ventouse | — | `tools/pi5/roby_ventouse.py` |
| CH8 | pompe ventouse | — | `tools/pi5/roby_ventouse.py` |

Réveil manuel des sorties : `tools/pi5/pca_wake.sh`.

## Pi 5 — USB

| Périphérique | Lien udev | Détail | Code |
|---|---|---|---|
| Carte poignet BLDC axe 5 (`wrist:=bldc`) — B-G431B-ESC1, ST-LINK VID `0483` | `/dev/roby_wrist` | 115200 bauds, LA8308 KV90 + réducteur planétaire 9:1, AS5600, alim 12 V | `src/roby_wrist_bldc/` (règle `udev/99-roby-wrist.rules`), firmware `firmware/wrist_bldc_simplefoc/` |

Sans la règle udev la carte apparaît en `/dev/ttyACM*` ; `roby_wrist_bldc/ports.py` la
retrouve alors par son VID.

## Pi 5 — caméras CSI

2 × ov5647 (`dtoverlay=ov5647,cam0` et `cam1`, voir `deploy/pi5/systeme/config.txt.extrait`) :
`left` = poignet (`i2c@88000`), `right` = vue extérieure (`i2c@80000`). Code :
`tools/pi5/cam_pub_pi2_dual.py`, `tools/pi5/launch_cams.sh`.

## PC — USB

| Périphérique | Lien udev | Détail | Code |
|---|---|---|---|
| Bras guide SO-ARM 101 (servos STS3215, adaptateur CH343 VID `1a86`) | `/dev/roby_leader` | 1 Mbaud, ids 1–6 | `src/roby_control/roby_control/leader_bus.py`, règle `tools/pc/udev/99-roby-leader.rules` |
| Webcam PC (ArUco) | — | index 0 | `src/roby_control/roby_control/aruco_node.py` |

## Pi 3 `roby-cam`

Caméra USB OV4689 « AK-Camera » (VID `2bcf`), serveur `~/roby_cam/roby_cam_usb.py --lan`
→ `http://192.168.2.4:8090/`. Code source : `tools/pc/roby_cam_usb*.py`. Alimentation
insuffisante (sous-tension) au 2026-09-30.

## État constaté le 2026-09-30

| Élément | Résultat |
|---|---|
| Pi 5 réseau / alim | ✅ `192.168.2.37`, `throttled=0x0` |
| PCA9685 `0x40` | ✅ répond |
| Carte BLDC axe 5 | ✅ vue en USB (derrière un hub), trames ~55 Hz, pas de défaut ; position figée à `0.0000` (AS5600 à vérifier) ; règle udev non installée |
| Caméras CSI | ❌ aucun capteur détecté — à revoir |
| Pi 3 caméra USB | ✅ flux 720p ~10 fps, ⚠️ sous-tension |
