# Erreur d'échelle de joint_3 — mesures et calculs (2026-09-07)

**État : diagnostiqué, NON réparé.** Une compensation temporaire opt-in existe
(`ROBY_J3_SCALE`), désactivée par défaut. La vraie réparation est dans la config du
driver et reste à faire.

## Le symptôme

Le bras réel est **plus bas** que le modèle affiché par RViz. Sam a comparé en prenant le
**changeur d'outil** comme repère (pas la pince, dont il sait qu'elle est absente de l'URDF).

L'erreur est **nulle au nid** et grandit avec la course. Ce n'est donc pas un décalage de
référence (un ré-étiquetage se verrait partout, nid compris), ni une perte de pas : le bras
est revenu au nid en fin de séance et l'erreur y était de nouveau nulle — une perte de pas
ne se répare pas toute seule.

C'est aussi **exclu que ce soit de la souplesse mécanique** : Sam l'a écarté explicitement.

## Les mesures (écart pince / table, bras à la hauteur de prise commandée z=0.3316)

| point | x | y | rayon | joint_3 | course j3 depuis le nid | écart mesuré |
|---|---|---|---|---|---|---|
| D | +0.728 | -0.228 | 0.763 | -0.4835 | -1.007 rad | **1.0 cm** |
| coin proche-droit | +0.34 | -0.46 | 0.572 | +0.1753 | -0.359 rad | **3.0 cm** ⚠️ |
| coin loin-gauche +5cm | +0.79 | 0.00 | 0.790 | -0.6716 | -1.195 rad | **5.0 cm** |

⚠️ **Le coin proche a été estimé deux fois : d'abord 4.0 cm, puis 3.0 cm.** Écart de 1 cm
entre deux estimations à l'œil du même point. C'est la principale incertitude de tout ce
qui suit. **À re-mesurer à la règle ou au pied à coulisse.**

Référence du nid utilisée comme ancre : `joint_3 = 0.5237` (pose `approche_nid` atteinte).

## Le calcul

Modèle : joint_3 parcourt une fraction `f` de la course commandée, comptée depuis le nid.

    joint_3_réel = J3_REF + f × (joint_3_commandé − J3_REF)

On cale `f` sur deux points, en imposant que l'écart pince/table prédit reproduise l'écart
mesuré entre eux.

| mesures utilisées | f obtenu | dents équivalentes (2e étage 20/N) |
|---|---|---|
| D=1.0 cm, coin=**3.0** cm | **0.9299** | N = 34.4 |
| D=1.0 cm, coin=4.0 cm | 0.8935 | N = 35.8 |

**N n'est pas entier**, donc on ne peut PAS conclure à un nombre de dents faux. Avec la
mesure à 4.0 cm, N≈36 tombait juste (à 0.24 %) et l'explication « poulie 36 dents au lieu
de 32 » était séduisante — la mesure à 3.0 cm la casse. **Ne pas retenir cette piste sans
compter physiquement les dents** (Sam n'a pas pu le faire ce soir).

Valeur retenue pour la compensation : **f = 0.9299**.

### Hypothèse alternative non écartée

Une erreur de **couplage 2→3** (erreur ∝ course de joint_2) ajuste les mêmes mesures. Sur
la zone de travail les deux modèles ne diffèrent **jamais de plus de 2 mm** — indiscernables
en pratique, mais **la réparation ne serait pas au même endroit** :

- échelle joint_3 → `joint_3_gear_ratio_num/den`
- couplage → `coupling_ratio_m2_num/den` (ou `coupling_ratio_m3_`)

Pour trancher il faut un point où les deux divergent, or la zone n'en offre pas : le coin
proche-droit est déjà le plus éloigné en course de joint_3 que le bras puisse atteindre.
**Le moyen sûr reste l'inclinomètre** : commander joint_3 seul à deux angles connus et
mesurer l'angle réel.

## La compensation temporaire (opt-in)

Dans `roby_tool_pickup.py`, méthode `Pickup._compense()`, appliquée aux **deux** points
d'envoi : `_exec_traj()` (lignes droites DLS) et `free_to()` (buts articulaires MoveIt).

    export ROBY_J3_SCALE=0.9299        # active
    export ROBY_J3_REF=0.5237          # ancre (défaut)

**Sans la variable, elle ne fait rien.** Un redémarrage sans elle rend le comportement
d'origine — c'est voulu, rien n'est écrit en dur.

Sauvegarde : `roby_tool_pickup.py.avant_compensation_j3`, retiree du depot le 2026-09-13 ; la ressortir par `git show fa2fe67:tools/pc/roby_tool_pickup.py.avant_compensation_j3`.

### ⚠️ Effet de bord sur le dataset

Compensation active, `/joint_states` renvoie la consigne **compensée**, pas la valeur
« modèle ». Une FK appliquée telle quelle (`roby_dataset_to_cartesian.py`) donnera un
cartésien **faux**. Pour régénérer un cartésien juste à partir des bags :

    joint_3_modèle = J3_REF + (joint_3_enregistré − J3_REF) × 0.9299

Les bags restent donc exploitables — c'est justement pour ça qu'on enregistre les commandes
moteur et pas le cartésien. Mais la conversion doit défaire la compensation d'abord.

## Ce qui reste à faire

1. **Re-mesurer** l'écart au coin proche-droit à la règle (les deux estimations diffèrent de 1 cm).
2. **Compter les dents** de la poulie de sortie de joint_3 (config : 20/32).
3. **Inclinomètre** : joint_3 seul, deux angles, pour séparer échelle et couplage.
4. Corriger la **config du driver** (`roby_hardware_steppers_only.ros2_control.xacro`) plutôt que ce
   contournement — ça corrigerait aussi MoveIt, RViz, le jog et le garde.
5. Après correction : **re-référencer au nid** et re-vérifier les poses capturées avec
   l'ancien rapport (`changeur_outil`, `D_pose_cone`, `nid`).

## Poses relevées ce soir

    D_cuisine          [-0.2991, 0.8560, -0.4835, -0.0492, 1.3045]   x=+0.728 y=-0.228 z=+0.332
    coin_proche_droit  [-0.9376, 0.4542,  0.1736, -0.0512, 1.0492]   x=+0.34  y=-0.46
    coin_loin_gauche   [ 0.0049, 0.9265, -0.6092, -0.0489, 1.3597]   x=+0.79  y= 0.00

Les 2 autres coins de la zone (proche-gauche r=0.34, loin-droit r=0.914) sont **hors portée**.

## Vérification de la compensation (2026-09-07, sur le vrai robot)

Compensation appliquée au coin proche-droit → pince mesurée à **4 cm** de la table.
C'est bien le résultat attendu, et non un échec : les deux points de mesure donnent
chacun, indépendamment, **le modèle à 3.9 cm au-dessus de la table**.

    à D     le réel est 2.9 cm sous le modèle, pince mesurée à 1.0 cm → modèle à 3.9 cm
    au coin le réel est 0.9 cm sous le modèle, pince mesurée à 3.0 cm → modèle à 3.9 cm

La concordance des deux valeurs n'était pas garantie : c'est elle qui valide le modèle.

Compensation active, le réel rejoint le modèle → la pince se place à 3.9 cm.
D'où la correction de `Z_PICK_CUISINE` : **0.3316 → 0.3016** (table à 0.2916).
L'ancienne valeur avait été calibrée à D *avec* l'erreur d'échelle dedans ; supprimer
l'erreur décalibrait la référence du même coup.

**Cette hauteur vaut désormais sur toute la zone**, plus seulement au rayon de D —
à condition que `ROBY_J3_SCALE` soit exporté.
