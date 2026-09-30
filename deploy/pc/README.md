# PC — poste de pilotage de Roby

État relevé sur le PC en service (`sam-AtomMan`, `192.168.2.95`) le 2026-09-13.

## Rôle du PC (architecture B)

- `pc_moveit.launch.py` : `move_group` + RViz (`wrist:=bldc` pour le poignet BLDC).
- Scène de collision : `ros2 run roby_environments scene_loader --env cuisine`
  (**obligatoire** : sans elle, MoveIt et le garde ne voient aucun obstacle).
- Garde (`tools/pc/roby_guard.py`) : tout producteur d'angles (modèle IA, téléopération du
  vrai bras) passe par `/guard/joint_trajectory`.
- Panneaux : modèle sur le bras (`roby_infer_panel.py`), jog (`roby_fine_jog`), bras guide
  (`roby_leader_panel.py`).
- Bras guide SO-ARM 101 (`roby_control` : `leader_node`, téléopérations).
- Inférence LeRobot sur l'iGPU (OpenVINO), entraînement : dépôt `lerobot-experiments`,
  environnement `~/lerobot-experiments/venv`.

Procédure de lancement complète : skill `/roby-lancer-bras` du dépôt `roby-specs`.

## Système

- Ubuntu **24.04.4 LTS**, ROS 2 **Jazzy**, CycloneDDS, `ROS_DOMAIN_ID=42`.
- Intel Core Ultra 9 185H (CPU 0-11 = P-cores, 12-19 = E-cores) + iGPU Intel Arc.
- **Piège pyenv / linuxbrew** : le `python3` du shell n'est pas celui de ROS. Toujours
  `source ~/roby_env.sh` (lien vers `tools/pc/roby_env.sh`), qui remet `/usr/bin` en tête,
  et compiler avec `PATH=/usr/bin:$PATH colcon build --symlink-install`.

## Fichiers de ce dossier

| Fichier | Destination | Rôle |
|---|---|---|
| `cyclone_config.xml` | `~/cyclone_config.xml` | DDS : interface `enp86s0`, pairs `localhost` et `192.168.2.37` (Pi5), multicast coupé. Le Pi5 a sa propre version (`deploy/pi5/`). |
| `roby_leader_calib.yaml` | `~/roby_leader_calib.yaml` | bornes mesurées des servos du bras guide (US-017), lues par `leader_bus.py` à cet emplacement. Copie de l'état du 2026-09-13 : **le fichier du home fait foi**, celui-ci est une sauvegarde. |

Règle udev du bras guide : `tools/pc/udev/99-roby-leader.rules` (carte identifiée par son
numéro de série, lien `/dev/roby_leader`). Poignet BLDC :
`src/roby_wrist_bldc/udev/99-roby-wrist.rules`.

## Workspace et scripts

```bash
git clone https://github.com/Lardonsauvage1/roby-le-gentil-robot.git ~/ros2_ws
cd ~/ros2_ws && PATH=/usr/bin:$PATH colcon build --symlink-install
# les scripts sont appeles par des liens dans le home :
for f in ~/ros2_ws/tools/pc/*.sh ~/ros2_ws/tools/pc/*.py ~/ros2_ws/tools/pc/*.yaml; do ln -sfn "$f" ~/; done
```

Jusqu'au 2026-09-13, trois lanceurs du bras guide n'existaient que dans le home
(`roby_leader_teleop_cart.sh`, `roby_leader_teleop_pos.sh`, `roby_leader_panel.sh`) : ils
sont désormais dans `tools/pc/`. Sur le PC en service, ce sont encore des fichiers
ordinaires du home ; les remplacer par des liens.

## Tests (sans matériel)

```bash
source ~/roby_env.sh && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/roby_control    && python3 -m pytest test   # bras guide, teleoperation
cd ~/ros2_ws/src/roby_wrist_bldc && python3 -m pytest test   # poignet BLDC (carte simulee)
cd ~/ros2_ws/src/roby_hardware   && python3 -m pytest test/test_bldc_bridge_sim.py
# gtests roby_hardware : build/roby_hardware/test_*
```

Les tests ROS utilisent chacun leur domaine DDS (87, 88, 89 ; `ROBY_TEST_DOMAIN_ID` pour
isoler des séries parallèles) et refusent 42 (vraie stack) et 43 (simulation isolée).
`src/roby_hardware/test/test_simulation.py` et `test_stepper_integration.py` (avril 2026)
supposent une stack lancée : ils ne font pas partie de cette liste.

Validation de la téléopération du vrai bras sur un faux robot complet (domaine 43) :
`src/roby_control/test/teleop_reel_sim/`.
