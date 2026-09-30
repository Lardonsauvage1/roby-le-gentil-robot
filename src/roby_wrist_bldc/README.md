# roby_wrist_bldc — axe 5 (poignet) BLDC

Moteur LA8308 KV90 (20 paires de pôles) + réducteur planétaire 9:1, AS5600 (I2C) sur l'arbre
moteur, carte B-G431B-ESC1 sous SimpleFOC (firmware : projet PlatformIO `test_bldc`), bus 24 V,
USB série 115200 bauds vers le Pi5. **Toutes les positions sont en rad de l'axe du bras.**

Spec : `brasRobot/.claude/specifications/in-progress/spec-poignet-axe5-bldc.md` — ADR-004.

## Contenu

| Module | Rôle |
|---|---|
| `protocol.py` | encodage des commandes P/Z/S/E/R/?, décodage des trames `S <pos> <I> <défaut>` et réponses |
| `limiter.py` | rampe en ligne 100 Hz (vitesse + accélération bornées, freinage anticipé, feedforward) |
| `ports.py` | détection du port : `/dev/roby_wrist` (udev) > ST-LINK VID 0483 > unique ttyACM (jamais le CH343 du bras guide) |
| `driver.py` | `WristBldcDriver` : connexion/reconnexion, thread de lecture, état thread-safe, commandes acquittées, streaming 100 Hz, sécurité |
| `sim.py` | carte simulée fidèle au firmware (tests, `simulate:=true`) |
| `node.py` | nœud ROS 2 pont ros2_control ↔ carte (`/roby/wrist_bldc/*`) |
| `cli.py` | console de banc sans ROS (`wrist_cli shell`) |
| `bench.py` | procédure de réception au montage T1..T10 + rapport JSON (`wrist_bench`) |

## Utilisation du driver (Python pur)

```python
from roby_wrist_bldc.driver import WristBldcDriver, DriverConfig

with WristBldcDriver(DriverConfig(port="auto")) as wrist:
    wrist.initialize_at_nest(0.2009)        # tête posée dans le nid : Z, vérif, E
    wrist.move_to(0.8)                      # non bloquant, consignes P à 100 Hz en rampe
    wrist.wait_until_reached(timeout=5.0)
    position, courant, defaut = wrist.get_state()
```

Sécurité : sur défaut (trame `défaut=1`, `FAULT STALL`, ou écart consigne/mesure > 0,35 rad
pendant 0,5 s), le streaming s'arrête, `move_to`/`set_position` lèvent `FaultActiveError`, et
l'appelant décide (`reset_fault()`). Mouvement refusé aussi si : liaison perdue, position non
recalée (démarrage, reboot carte détecté), moteur désactivé.

## Tests

```bash
cd src/roby_wrist_bldc && python3 -m pytest -q test/     # ~15 s, sans matériel
ros2 run roby_wrist_bldc wrist_bench --simulate --yes     # procédure de réception à blanc
```

Au montage (stack **arrêtée**, tête au nid) : `ros2 run roby_wrist_bldc wrist_bench --nest 0.2009`.

## ROS

```bash
ros2 launch roby_wrist_bldc wrist_bldc.launch.py simulate:=true              # nœud seul
ros2 launch roby_hardware robot_control.launch.py wrist:=bldc                # Pi5 : stack RT
ros2 launch neuroneimitationcarote_moveit_config pc_moveit.launch.py wrist:=bldc   # PC
ros2 service call /roby/wrist_bldc/reset_fault std_srvs/srv/Trigger
ros2 topic echo /roby/wrist_bldc/status
```

`wrist:=bldc` est le **défaut** depuis le 2026-09-30 (poignet BLDC monté sur le bras).
`wrist:=servo` reste disponible pour l'ancien servo provisoire PCA9685 CH1, démonté.

Test du pont complet sans matériel (JTC → RobySystem → nœud → carte simulée) :
`python3 -m pytest -q src/roby_hardware/test/test_bldc_bridge_sim.py`.
