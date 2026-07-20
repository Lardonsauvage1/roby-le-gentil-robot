#!/usr/bin/env python3
"""roby_gripper.py — seuillage de la commande de pince, avec HYSTERESIS.

Le modele sort une valeur CONTINUE pour la pince. Jusqu'ici on la seuillait
brutalement a 0.5 : toute oscillation autour de ce seuil devenait un ordre
d'ouverture ou de fermeture.

Mesure sur le robot le 2026-07-20 (b2cart_small_96, 30 s de pilotage) :
**14 changements d'etat**, dont des paires separees de **65 ms** --
    291.640 OUVRE -> 291.706 FERME   (66 ms)
    292.442 OUVRE -> 292.506 FERME   (64 ms)
Le servo, provisoire et sous-dimensionne, etait cycle a ~15 Hz. C'est le
"la pince bugue un peu" rapporte par Sam, et c'est purement notre seuillage :
le modele, lui, n'a rien fait d'aberrant.

Principe : deux seuils au lieu d'un.
    valeur >= HAUT  -> FERMER
    valeur <= BAS   -> OUVRIR
    entre les deux  -> GARDER L'ETAT COURANT
Une valeur qui flotte autour de 0.5 ne provoque donc plus rien.

⚠️ Contrepartie assumee : si le modele restait durablement DANS la bande
(par ex. 0.55) au moment de la prise, la pince ne se fermerait pas. Les
observations disent qu'il est franc quand il decide (0.9996 releve en
conditions reelles), donc la bande est sure -- mais si une prise echouait
sans que la pince bouge, c'est la premiere chose a verifier.
"""

GRIP_HAUT = 0.6    # au-dessus : fermer
GRIP_BAS = 0.4     # en dessous : ouvrir
GRIP_MILIEU = 0.5  # seuil simple, utilise seulement au tout premier appel


def fermer(valeur, etat_courant):
    """-> True s'il faut FERMER la pince, False s'il faut l'ouvrir.

    `etat_courant` : dernier etat commande (True=ferme, False=ouvert), ou None
    au premier appel -- dans ce cas on retombe sur le seuil simple, faute d'etat
    a conserver.
    """
    v = float(valeur)
    if etat_courant is None:
        return v > GRIP_MILIEU
    if v >= GRIP_HAUT:
        return True
    if v <= GRIP_BAS:
        return False
    return bool(etat_courant)      # dans la bande : on ne bouge pas


def dans_la_bande(valeur):
    """La valeur est-elle dans la zone d'indecision ? (diagnostic)"""
    return GRIP_BAS < float(valeur) < GRIP_HAUT
