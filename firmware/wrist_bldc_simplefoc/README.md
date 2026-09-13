# Firmware de la carte du poignet BLDC (axe 5) — B-G431B-ESC1 / SimpleFOC

Copie **telle quelle** du projet PlatformIO `~/Documents/PlatformIO/Projects/test_bldc` du PC
(`src/main.cpp` du 2026-09-12 01:40, md5 `baa213a3cef034ed39b81f21d42399c8`), versionnée ici le
2026-09-12 pour que la spec et le driver puissent y renvoyer. **Aucune modification.**

- Le code collé par Sam dans la session du 2026-09-12 correspond à ce fichier, vérifié sur les
  passages clés (`RECALE`, `offset_bras`, `STALL_CURRENT`, `velocity_limit`).
- Que ce soit exactement la version **flashée** sur la carte n'a **pas** été vérifié.
- D'après Sam, la carte fonctionne en boucle fermée. Claude ne l'a pas vue tourner : l'axe n'était
  pas monté.

## Matériel

LA8308 KV90 (20 paires de pôles) + réducteur cycloïdal 20:1, AS5600 en I2C sur l'arbre moteur
(PB7/PB8, 400 kHz), carte B-G431B-ESC1 (STM32G431), bus 24 V, USB (ST-LINK VCP) 115200 bauds.

## Compiler / flasher

```bash
cd firmware/wrist_bldc_simplefoc
pio run                 # compile
pio run -t upload       # flashe par le ST-LINK embarqué
pio device monitor      # 115200 : trames "S <pos> <courant> <défaut>" à ~50 Hz
```

## Protocole

Positions en **rad de l'axe du bras** (la carte applique ×20). `P<angle>` consigne, `Z<angle>`
recalage sans mouvement, `S` stop, `E` enable, `R` reset défaut, `?` position. Réponses :
`RECALE`, `POS`, `ENABLED`, `DISABLED`, `RESET`, `FAULT STALL`, `READY`.
Côté Pi : `src/roby_wrist_bldc` (driver, simulateur fidèle à ce firmware, nœud ROS).

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
