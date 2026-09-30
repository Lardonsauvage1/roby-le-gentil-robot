# Firmware de la carte du poignet BLDC (axe 5) — B-G431B-ESC1 / SimpleFOC

**Seule copie du firmware.** Ouvrir ce dossier dans PlatformIO pour compiler/flasher.
L'ancien projet `~/Documents/PlatformIO/Projects/test_bldc` du PC a été rapatrié ici le
2026-09-30 puis supprimé (version du 2026-09-24 : réducteur 9:1, alimentation 12 V, traces de
démarrage `BOOT` / `initFOC=`), c'est celle qui a été flashée et testée sur la carte ce jour-là.

## Matériel

LA8308 KV90 (20 paires de pôles) + réducteur **planétaire 9:1**, AS5600 en I2C sur l'arbre moteur
(PB7/PB8, 400 kHz), carte B-G431B-ESC1 (STM32G431), bus **12 V** (à 24 V la carte chauffait trop),
USB (ST-LINK VCP) 115200 bauds. Vitesse maxi : 15 rad/s moteur = 1,67 rad/s bras.

## Compiler / flasher

```bash
cd firmware/wrist_bldc_simplefoc
pio run                 # compile
pio run -t upload       # flashe par le ST-LINK embarqué
pio device monitor      # 115200 : trames "S <pos> <courant> <défaut>" à ~50 Hz
```

## Protocole

Positions en **rad de l'axe du bras** (la carte applique ×9). `P<angle>` consigne, `Z<angle>`
recalage sans mouvement, `S` stop, `E` enable, `R` reset défaut, `?` position. Réponses :
`RECALE`, `POS`, `ENABLED`, `DISABLED`, `RESET`, `FAULT STALL`, `READY`.
Côté Pi : `src/roby_wrist_bldc` (driver, simulateur fidèle à ce firmware, nœud ROS).

## Test d'endurance

`logs/endurance.py [port]` (défaut `/dev/roby_wrist`) — **fait bouger l'axe** ; journal du
2026-09-24 dans `logs/endurance_20260924.log`.

## ⚠️ Défauts connus — NON corrigés dans ce code

Relevés à la lecture le 2026-09-12 (détail : spec `spec-poignet-axe5-bldc`, US-032). Aucun n'a
été reproduit sur la carte.

| # | Gravité | Défaut |
|---|---|---|
| F1 | haute | `atof()` sans contrôle : `P` ou `Pabc` → consigne **0 rad** (mouvement). |
| F2 | haute | Aucune butée côté carte : `P100` → 100 rad (16 tours d'axe). |
| F3 | moyenne | `current_limit = 3 A` sans effet : en couple « voltage » sans `phase_resistance`, SimpleFOC borne par `voltage_limit` (6 V). |
| F4 | moyenne | Aucun indicateur « recalé » dans la trame : le Pi ne peut pas savoir avec certitude si la carte a redémarré. |
| F5 | moyenne | `initFOC()` aligne le capteur au boot (le moteur bouge d'environ 1° en sortie) : risque d'échec si la tête est tenue par le nid. |
| F6 | moyenne | Défaut blocage = `motor.disable()` : le poignet part en roue libre, la tête peut tomber. |
| F7 | basse | `E` accepté pendant un défaut (répond `ENABLED` mais tension nulle). |

Pas de watchdog de communication : si le Pi s'arrête, la carte tient la dernière consigne. C'est
voulu (le poignet porte la tête).
