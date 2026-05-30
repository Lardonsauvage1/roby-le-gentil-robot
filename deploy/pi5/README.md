# Setup Pi5 — Roby le gentil robot

Procédure pour reconstruire un Pi5 fonctionnel à partir d'un Raspberry Pi OS Ubuntu 24.04 fraîchement installé.

## Hardware

- Raspberry Pi 5 (8GB recommandé)
- Drivers stepper DM860I sur axes 1-3 (NEMA via GPIO, alim 24V séparée)
- 3× Arduino Nano (esclaves RS-485) sur motors 1/2/3
- 3× AS5048A (PWM, sur arbres moteurs)
- 1× MAX485 maître (alimenté **5V** + level shifter 4 canaux vers Pi5 3.3V)
- 1× PCA9685 I2C (servo MG996R axe 4)
- 2× caméra CSI OV5647 (tête robot)

Voir aussi : memory `project_encodeurs_rs485.md` et `project_architecture_materielle.md`.

## 1. OS + ROS2

```bash
# Ubuntu 24.04 Noble pour Pi5 → ROS2 Jazzy
# Suivre https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt update && sudo apt install -y ros-jazzy-desktop \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-rmw-cyclonedds-cpp

# Dépendances Python encodeurs
sudo apt install -y python3-serial python3-gpiozero python3-yaml
```

## 2. Activation UART0 (RS-485 maître)

Dans `/boot/firmware/config.txt`, ajouter (ou s'assurer présent) :

```
dtparam=uart0=on
dtparam=i2c_arm=on
dtparam=spi=on
```

Puis reboot. `/dev/ttyAMA0` doit apparaître.

## 3. Accès GPIO sans sudo

```bash
sudo groupadd -f gpio
sudo usermod -aG gpio,dialout $USER
```

Créer `/etc/udev/rules.d/99-gpio.rules` :

```
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"
```

Puis `sudo udevadm control --reload && sudo udevadm trigger`. Logout/login pour appliquer les groupes.

GPIO chip : **`/dev/gpiochip4`** (RP1 sur Pi5, PAS gpiochip0).

## 4. Variables environnement (~/.bashrc)

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/$USER/cyclone_config.xml
```

## 5. Config DDS (CycloneDDS)

Copier `cyclone_config.xml` (présent dans ce dossier) dans `~/cyclone_config.xml`.

**Adapter les `<Peer>`** selon les IPs des autres machines (PC, Pi4) si elles changent. Le `autodetermine="true"` choisit l'interface réseau active automatiquement (pas besoin de fixer `enp87s0` etc. — c'était la cause de BUG-004).

## 6. Workspace ROS2

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Lardonsauvage1/roby-le-gentil-robot.git .

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 7. Calibration encodeurs

`encoder_calibration.yaml` est livré dans `src/roby_hardware/config/` (zeros pour la pose physique "bras plié 90° + axe 1 aligné", joints URDF tous à 0).

**Si l'aimant d'un AS5048A est démonté/remplacé**, refaire un snapshot :
1. Placer manuellement le bras à la pose initiale
2. Sur Pi5 : lancer un script lecteur (cf `scripts/encoder_publisher.py` qui charge le YAML — pour un nouveau snapshot, adapter `snapshot_zeros.py` documenté en mémoire `project_encodeurs_rs485.md`)
3. Remplacer les valeurs dans `encoder_calibration.yaml`
4. `colcon build` pour réinstaller

## 8. Firmware Arduino esclave

Code C++ documenté dans la mémoire `project_encodeurs_rs485.md`. Reflasher chaque Arduino Nano avec un `MY_ID` unique (1, 2 ou 3) correspondant au numéro de motor. Câblage MAX485 esclave : DI←D1, DE+RE←D4, RO→D0. AS5048A : signal P (PWM) → D3. Alim esclave en **5V**.

## 9. Lancement

```bash
ros2 launch roby_hardware robot_full.launch.py
```

Et en parallèle, monitoring encodeurs :

```bash
python3 src/roby_hardware/scripts/encoder_publisher.py
```

## Pièges connus

- MAX485 maître **DOIT** être alimenté en 5V (3.3V ne produit pas un différentiel suffisant). Niveau logique 5V → Pi5 protégé par level shifter.
- GND commun obligatoire entre Pi5, MAX485 maître, et alim 5V esclaves.
- Ne PAS appeler `tcdrain()` puis basculer DE/RE immédiatement → ajouter `time.sleep(0.001)` après `flush()`.
- Côté Arduino esclave : `delay(3)` après réception de l'ID avant `DE_RE HIGH` (laisse le maître commuter en RX).
- `pulseIn()` côté Arduino peut renvoyer occasionnellement des valeurs aberrantes (2-8% selon capteur) → filtre médian glissant côté maître absorbe.

Détails complets : memory `project_encodeurs_rs485.md`.
