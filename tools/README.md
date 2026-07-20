# `tools/` — outillage opérationnel du robot

Scripts qui font tourner Roby au quotidien. Jusqu'au 2026-07-20 ils vivaient **uniquement
dans les répertoires personnels** des deux machines, sans aucun contrôle de version : leur
seul filet de sécurité était une quarantaine de fichiers `.bak_*` horodatés. Trois semaines
de travail n'existaient qu'en un seul exemplaire, sur un seul disque.

```
tools/
├── pc/     exécuté sur le PC (sam-AtomMan) — MoveIt, inférence, collecte, visualisation
└── pi5/    exécuté sur le Pi5 (roby-desktop) — caméras, servos, tests bas niveau
```

La séparation est **par machine**, pas par thème : ces scripts contiennent des chemins et
des dépendances propres à leur hôte (`/home/roby`, bindings libcamera compilés, `venv`
lerobot du PC…). Un script `pi5/` ne fonctionne pas sur le PC et réciproquement.

## Les scripts restent utilisables depuis le home

Chaque fichier est **lié symboliquement** depuis le répertoire personnel :

```
~/roby_infer.py -> ~/ros2_ws/tools/pc/roby_infer.py
```

Les habitudes ne changent donc pas (`bash ~/roby_infer.sh` marche toujours), mais **le
fichier réel est versionné**. Les imports Python continuent de fonctionner : Python résout
le lien et place le **vrai** dossier en tête de `sys.path`, donc `from roby_oracle import fkT`
trouve son voisin dans `tools/pc/`.

⚠️ **Ne jamais remplacer un lien par une copie** (`cp` par-dessus, éditeur qui écrit un
nouveau fichier) : la modification sortirait du dépôt sans être vue. Éditer le fichier dans
`tools/`, ou vérifier avec `ls -l ~/roby_*.py` que ce sont toujours des liens.

## Points d'entrée principaux

| Script | Rôle |
|---|---|
| `pc/roby_infer.py` / `.sh` | inférence réseau, modèles **joint** (5 articulations + pince) |
| `pc/roby_infer_cart.py` / `.sh` | inférence réseau, modèles **cartésiens** (pose TCP 6D + pince) |
| `pc/roby_vision.py` | prétraitement image **partagé** par les deux — ne pas dupliquer |
| `pc/roby_guard.py` / `.sh` | garde du corps : butées, vitesse, plancher, collision MoveIt |
| `pc/roby_oracle.py` | oracle de collecte (prise/dépose scriptée) |
| `pc/roby_collect_panel.py` / `.sh` | panneau de collecte du dataset (garder / jeter / relance) |
| `pc/roby_tool_pickup.py` | IK amortie DLS maison + séquence de prise |
| `pc/roby_sortie_nid.sh` + `.yaml` | trajectoire nid ↔ sortie, rejeu lent et sûr |
| `pc/roby_cam_view.py` / `.sh` | visualisation des 2 caméras (relais MJPEG, port 8081) |
| `pi5/cam_pub_pi2_dual.py` | **nœud caméra** : les 2 capteurs dans UN process (ISP partagé) |
| `pi5/launch_cams.sh` | lancement des caméras, avec garde anti-double-lancement |

## Volontairement non versionnés

- `~/launch_stack.sh` (Pi5) et `~/start_robot.sh` (PC) : lanceurs **périmés**, documentés
  comme à ne plus utiliser (architecture A qui provoque la course au mock, ancienne IP).
- Les `*.bak_*` : remplacés par l'historique git (voir `.gitignore`).
