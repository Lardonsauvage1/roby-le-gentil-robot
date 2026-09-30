# `attic/` — scripts Pi5 d'une phase revolue

Conserves pour l'historique, **plus utilises**. Rien dans le projet vivant ne les
appelle. Si l'un redevient utile, il suffit de le remonter d'un niveau.

| Fichier | Role |
|---|---|
| `fix_ax1_accel.py` | ancien correctif d'acceleration de l'axe 1 |
| `pi5_timeecho.py` | echo d'horloge pour mesurer la derive PC/Pi5 |

Supprimes le 2026-09-30 (nettoyage cablage, recuperables dans l'historique git) :
les scripts des **encodeurs RS-485 AS5048A**, retires du robot (`rs485_master.py`,
`test_100.py`, `multi_test.py`, `snapshot_zeros.py`, `encoder_to_joints.py`,
`encoder_publisher.py`, `trajectory_test*.py`) et la **demo de juin 2026**
(`demo_orchestrator.py`, `demo_prep.py`, `capture_pose.py`).
