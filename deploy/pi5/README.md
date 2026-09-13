# Pi5 — reconstruire la machine temps réel de Roby

État relevé sur le Pi5 en service le 2026-09-13 (lecture seule, rien n'a été modifié).
Remplace l'ancienne version de ce fichier, qui décrivait l'architecture d'avant juin 2026
(encodeurs AS5048A, esclaves Arduino RS-485, `robot_full.launch.py`) : **tout cela est
retiré du projet**.

## Rôle du Pi5 (architecture B)

Le Pi5 ne fait que le **temps réel** et les **caméras** :

- `robot_control.launch.py` : `robot_state_publisher` (**seul** publisher de
  `/robot_description`), `ros2_control_node` (`RobySystem` + `arm_controller` +
  `joint_state_broadcaster`), TF `world → base_link` ;
- `cam_pub_pi2_dual.py` : les deux caméras CSI de la tête.

MoveIt (`move_group`), RViz, l'inférence, le garde et les outils tournent sur le **PC**
(`pc_moveit.launch.py`, voir `deploy/pc/README.md`). Procédure de lancement complète,
avec ses vérifications : skill `/roby-lancer-bras` du dépôt `roby-specs`.

## Matériel piloté par le Pi5

| Élément | Interface |
|---|---|
| Axe 1 (base) | stepper, step/dir en GPIO (`/dev/gpiochip4`, le RP1 du Pi5 — **pas** gpiochip0) |
| Axes 2 et 3 | NEMA 34 12 Nm, drivers **CL86Y** boucle fermée, step/dir en GPIO |
| Axe 4 (roulis poignet) | servo, PCA9685 (I2C bus 1, adresse 0x40) canal **CH0** |
| Axe 5 (tangage poignet) | servo **provisoire**, PCA9685 **CH1** — remplaçant BLDC (carte B-G431B-ESC1 en USB, `roby_wrist_bldc`) prêt mais **non monté** |
| Verrou de tête (changeur d'outil) | PCA9685 **CH2** |
| Pince | PCA9685 **CH3** |
| Caméras | 2 × ov5647 CSI (`cam0`, `cam1`) : `left` = **poignet** (`i2c@88000`), `right` = **vue extérieure** (`i2c@80000`) — vérifié sur les images le 2026-09-13 |

Alimentation des moteurs **séparée** du Pi5 (coupure physique d'urgence). Pas de fin de
course : la référence est le **nid** (voir « Pièges »).

## 1. Système

- Ubuntu **24.04.4 LTS** (noyau 6.8 raspi), ROS 2 **Jazzy**.
- Paquets présents sur la machine en service :
  `ros-jazzy-ros-base ros-jazzy-ros2-control ros-jazzy-ros2-controllers
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-xacro ros-jazzy-robot-state-publisher
  ros-jazzy-moveit i2c-tools gpiod libgpiod-dev python3-serial`.
  (`ros-jazzy-moveit` n'est plus utilisé à l'exécution sur le Pi5.)
- Python utilisateur (`pip3 install --user`) : `picamera2` 0.3.36, `gpiod` 2.4.1,
  `av`, `simplejpeg`.
- Groupes de l'utilisateur `roby` : `gpio`, `dialout`, `video`.

Fichiers système, à copier depuis `systeme/` :

| Fichier du dépôt | Destination | Rôle |
|---|---|---|
| `systeme/99-gpio.rules` | `/etc/udev/rules.d/` | GPIO sans sudo (`sudo groupadd -f gpio`) |
| `systeme/99-roby-rt.conf` | `/etc/security/limits.d/` | rtprio 98 + memlock illimité pour `roby` (boucle 100 Hz) |
| `systeme/cpu-performance.service` | `/etc/systemd/system/` puis `systemctl enable` | gouverneur CPU `performance` au démarrage (anti-overrun) |
| `systeme/config.txt.extrait` | lignes à reporter dans `/boot/firmware/config.txt` | UART0, I2C, SPI, et **les deux ov5647** (`camera_auto_detect=0` + `dtoverlay=ov5647,cam0/cam1`) |

## 2. Réseau et DDS

- Sous-réseau dédié `192.168.2.x` : Pi5 = `192.168.2.37`, PC = `192.168.2.95`.
- `~/.bashrc` : `source /opt/ros/jazzy/setup.bash`, `ROS_DOMAIN_ID=42`,
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI=file:///home/roby/cyclone_config.xml`.
  Il ne source **pas** le workspace : la stack le source elle-même au lancement.
- `cyclone_config.xml` (ce dossier) → `~/cyclone_config.xml`. Identique à celui en service :
  multicast coupé, pairs `localhost` et `192.168.2.95`.

## 3. Workspace

```bash
git clone https://github.com/Lardonsauvage1/roby-le-gentil-robot.git ~/rlgr
cd ~/rlgr
colcon build --packages-select roby_hardware roby_environments \
    neuroneimitationcarote_description neuroneimitationcarote_moveit_config
# + roby_wrist_bldc si le poignet BLDC est monte (wrist:=bldc)
```

- Le workspace canonique du Pi5 s'appelle **`~/rlgr`** (build par copie, pas en symlink).
- `roby_control` est un paquet **PC** (bras guide, téléopération) : inutile sur le Pi5.
- Les scripts de `tools/pi5/` sont appelés par des **liens** dans `~` :
  `for f in ~/rlgr/tools/pi5/*.py ~/rlgr/tools/pi5/*.sh; do ln -sfn "$f" ~/; done`.

## 4. Caméras (picamera2 + libcamera compilée)

`launch_cams.sh` lance `cam_pub_pi2_dual.py` (**un seul** processus pour les deux
caméras : deux processus cassent le verrouillage de l'ISP ; un verrou `flock` empêche une
seconde instance). Il a besoin de :

- **libcamera 0.5.2 compilée depuis les sources** : paquet source Debian
  `libcamera_0.5.2+rpt20250903` dans `~/lc_src/`, compilé dans
  `~/lc_src/libcamera-0.5.2+rpt20250903/build`. Le binding Python est pris dans
  `build/src/py` via `PYTHONPATH` (voir `tools/pi5/launch_cams.sh`). Les commandes exactes
  de cette compilation n'ont pas été conservées : **lacune connue**.
- **`pystubs/pykms.py`** (ce dossier) → `~/pystubs/pykms.py` : picamera2 importe `pykms`
  (aperçu DRM) qu'on n'utilise pas ; ce stub rend l'import inoffensif.

Topics : `/head_camera/{left,right}/image_raw/compressed`, 15 Hz. Mode **BRUT** par
défaut depuis le 2026-09-06 (exposition auto, balance des blancs coupée) ;
`ROBY_CAM_AUTO=1` et `ROBY_CAM_FIGE=1` sont en opt-in.

## 5. Lancement

Toujours **tête posée dans le nid** avant de lancer (le compteur de pas part de
`initial_positions.yaml` = pose du nid).

```bash
# stack temps reel (ce que fait l'etape 1 de /roby-lancer-bras)
source /opt/ros/jazzy/setup.bash && source ~/rlgr/install/setup.bash
ros2 launch roby_hardware robot_control.launch.py            # wrist:=bldc pour le poignet BLDC
# cameras
bash ~/launch_cams.sh
```

`robot_full.launch.py` est **obsolète** : il lance aussi un `move_group` sur le Pi5, avec
une configuration MoveIt ancienne restée dans `install/` (compilée le 2026-05-31) ; un
second publisher de `/robot_description` provoque la course « mock » décrite dans
`/roby-lancer-bras`. Ne pas l'utiliser.

## 6. Après une coupure d'alimentation des servos

`tools/pi5/pca_wake.sh` réveille le PCA9685 (bit SLEEP, 50 Hz) puis **réécrit les quatre
canaux** à des valeurs fixes (axe 4 135°, axe 5 125,8°, verrou 50°, pince ouverte 110°).
⚠️ **Les servos bougent** : ne l'exécuter qu'avec le feu vert de Sam, bras dégagé.

## Pièges connus

- **Un seul publisher de `/joint_states`** (le `joint_state_broadcaster`). Un nœud de
  simulation lancé sur le domaine 42 a fait sauter l'axe 1 de 38° (BUG-008). Les nœuds de
  simulation du bras guide se taisent désormais s'ils voient un autre publisher ; simuler
  sur `ROS_DOMAIN_ID=43`.
- **Un seul `robot_state_publisher`**, celui du Pi5.
- Référence **par le nid**, en boucle ouverte : après un choc ou un doute, moteurs coupés,
  tête reposée dans le nid, stack relancée.
- `/dev/gpiochip4` et non `gpiochip0` (RP1).
- `~/dual_node.log` grossit sans rotation (4 Mo au 2026-09-13).
- Anciens dossiers inutilisés sur la machine, sans effet sur la stack :
  `~/ros2_ws_ABANDONNE_20260422`, `~/libcamera_*_INUTILISE_*`, `~/pigpio_INUTILISE_*`,
  `~/_archive_*_20260720`, `~/launch_stack.sh` (lance `robot_full`, obsolète).
