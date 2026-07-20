# `attic/` — scripts Pi5 d'une phase revolue

Conserves pour l'historique, **plus utilises**. Rien dans le projet vivant ne les
appelle (verifie par `grep` sur les deux arbres et sur `.claude/`). Ils restent
versionnes : si l'un redevient utile, il suffit de le remonter d'un niveau.

## Encodeurs RS-485 — le sous-systeme a ete RETIRE du projet

Les encodeurs absolus lus par des Arduino esclaves en RS-485 ont ete abandonnes au
profit des encodeurs integres aux drivers CL86Y et du referencement par le nid. Le
xacro charge declare `encoder_enabled=false`, donc rien de tout ceci ne tourne.

| Fichier | Role |
|---|---|
| `rs485_master.py` | lecture brute du bus RS-485 |
| `test_100.py` | 100 lectures d'affilee (fiabilite) |
| `multi_test.py` | N series x M lectures |
| `snapshot_zeros.py` | calcul des offsets zero encodeurs |
| `encoder_to_joints.py` | conversion valeur encodeur -> angle articulaire |
| `encoder_publisher.py` | publiait `/joint_states_measured` |
| `trajectory_test*.py` | mesure du decalage open-loop vs encodeurs — ils lisent `/joint_states_measured`, topic qui n'est plus publie |

⚠️ Le chemin encodeur du C++ (`encoder_driver`, `outlier_filter`, `pid`) est lui
**conserve** dans `roby_hardware` : il est inerte a l'execution mais teste, et les
encodeurs integres aux drivers le reutiliseront.

## Demo "lanceurs du Bureau" — abandonnee

`demo_orchestrator.py`, `demo_prep.py`, `capture_pose.py` (qui alimentait
`demo_poses.yaml`). Remplacee par les skills `/roby-lancer-bras` et `/roby-nid`.

## Correctifs ponctuels deja appliques

- `fix_ax1_accel.py` : editait en place `robot_full.launch.py` et
  `move_group_fast.launch.py` pour injecter l'acceleration dediee de l'axe 1.
  **Non rejouable** (il s'arrete sur des ancres deja consommees), et les deux
  fichiers qu'il modifiait ne sont plus sur le chemin vivant — l'axe 1 dedie n'est
  donc plus effectif, ce qui reste **a arbitrer**.
- `pi5_timeecho.py` : mesure du decalage d'horloge PC/Pi5, remplace par NTP.
