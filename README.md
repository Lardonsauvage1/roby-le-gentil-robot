# Roby le gentil robot

Bras robotique 5 axes DIY pour imitation learning (behavioral cloning) avec ROS2 Jazzy et MoveIt2.

## Architecture

Le projet est distribué sur 3 machines :

| Machine | Rôle | IP |
|---------|------|----|
| **PC** (sam-AtomMan) | Planification MoveIt, RViz, simulation, entraînement | 192.168.1.95 |
| **Pi5** (roby-arm) | Contrôle moteurs (steppers + servos) | 192.168.1.37 |
| **Pi4** (roby-cam) | Vision caméra + ArUco tracking | 192.168.1.28 |

## Packages ROS2

| Package | Description |
|---------|-------------|
| `neuroneimitationcarote_description` | URDF/Xacro du bras + meshes STL |
| `neuroneimitationcarote_moveit_config` | Configuration MoveIt2 (planification, controllers, RViz) |

## Lancer la simulation

```bash
# Prérequis : ROS2 Jazzy + MoveIt2 installés
cd ~/ros2_ws
colcon build --symlink-install
./launch_sim.sh
```

Dans RViz :
- Déplacer les **marqueurs orange** pour choisir la pose cible
- Panneau **MotionPlanning** > **Planning** > **Plan & Execute**

### Trajectoire en boucle (test)

```bash
# Dans un 2e terminal
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
python3 src/neuroneimitationcarote_moveit_config/scripts/loop_trajectory.py
```

## Stack technique

- **ROS2** Jazzy Jalisco
- **MoveIt2** (OMPL/RRTConnect, CHOMP, Pilz, STOMP)
- **ros2_control** avec mock_components (simulation) / gpiod + PCA9685 (hardware)
- **DDS** : CycloneDDS (PC + Pi5), FastDDS (Pi4)
- **Apprentissage** : LeRobot (behavioral cloning, CNN+MLP ResNet18) — futur

## Repo lié

Les spécifications, user stories et suivi de bugs sont dans un repo séparé :
[roby-specs](https://github.com/Lardonsauvage1/roby-specs)
