"""Mode JOYSTICK du bras guide (idee de Sam, 2026-09-05) — sans ROS, testable seul.

Changement de paradigme par rapport a US-019/US-020 : le leader ne commande plus une
POSITION mais une VITESSE.

  - chaque axe est rappele vers son zero, avec un couple qui CROIT avec l'ecart :
    quasi nul pres du zero, au maximum en bout de course ;
  - l'ecart au zero devient une consigne de vitesse pour le vrai bras : a gauche du
    zero -> Roby tourne a gauche, et plus le guide est loin, plus Roby va vite ;
  - une ZONE MORTE autour du zero evite de vibrer et de deriver a l'arret.

Deux consequences heureuses :
  - l'affaissement du leader cesse d'etre un probleme : le rappel le tient. Il devient
    meme le mecanisme du retour au neutre.
  - les ~124 deg de la base inatteignables en 1:1 (US-019) disparaissent : en vitesse,
    tout l'espace de Roby devient accessible, quelle que soit la course du guide.
"""

from __future__ import annotations


def commande(ecart, demi_course, zone_morte, couple_max, couple_min=0):
    """Traduit un ecart au zero en (consigne_vitesse, couple_de_rappel).

    `ecart`       : ecart deroule au zero (rad, signe)
    `demi_course` : ecart au-dela duquel on est "a fond" (rad)
    `zone_morte`  : |ecart| en deca duquel on ne fait RIEN (rad)
    `couple_max`  : Torque_Limit au plein rappel
    `couple_min`  : Torque_Limit des la SORTIE de la zone morte (0 = ancien comportement)

    Retourne (v, couple), v dans [-1, 1] et couple dans [0, couple_max].

    Deux regimes distincts, et c'est voulu :
      - DANS la zone morte : couple exactement ZERO. Un rappel residuel ferait osciller
        le bras autour du neutre en permanence — c'est ce que la zone morte empeche.
      - DES LA SORTIE : le couple part directement de `couple_min` et non de zero. Sans
        cela, il existe une bande ou le rappel est theoriquement actif mais trop faible
        pour vaincre le moindre frottement : le bras y stagne (constate le 2026-09-05).
    Il y a donc une DISCONTINUITE volontaire au bord de la zone morte. C'est le prix a
    payer pour que le rappel morde des qu'il s'active.
    """
    a = abs(ecart)
    if a <= zone_morte:
        return 0.0, 0
    utile = max(1e-9, demi_course - zone_morte)
    frac = min(1.0, (a - zone_morte) / utile)
    signe = 1.0 if ecart > 0 else -1.0
    cmin = min(couple_min, couple_max)
    return signe * frac, int(round(cmin + frac * (couple_max - cmin)))


def commande_axe(mapping, position, zone_morte, couple_max, couple_min=0):
    """Comme `commande`, mais a partir d'un JointMapping et d'une position brute.

    La consigne est exprimee dans le repere de ROBY : le signe de la calibration est
    applique, pour que "guide a gauche" fasse bien tourner Roby a gauche et non l'inverse.
    """
    ecart = mapping.ecart_deroule(position)
    demi = (mapping.course_leader / 2.0) if mapping.course_leader else 3.14159
    v, couple = commande(ecart, demi, zone_morte, couple_max, couple_min)
    return mapping.signe * v, couple


RESOLUTION = 4096


def pas_court(depuis, vers, resolution=RESOLUTION):
    """Ecart en pas vers `vers`, par le plus court chemin (bouclage compris).

    Le servo, lui, interpole betement en NUMERO DE PAS : pour aller de 3803 a 883 il
    descend par 3000, 2000, 1000... soit 71 % de tour, au lieu de franchir 4095->0 en
    1176 pas. Vecu le 2026-09-05 sur l'epaule et la rotation du poignet, qui partaient
    tous deux a l'envers.
    """
    demi = resolution // 2
    return (vers - depuis + demi) % resolution - demi


def cible_de_rappel(actuel, zero, pas_max, zone_morte_pas):
    """Cible a ecrire dans Goal_Position pour tirer vers le zero SANS faire le tour.

    On ne vise jamais le zero absolu : on vise un point proche, decale d'au plus
    `pas_max` dans la bonne direction, recalcule a chaque cycle. La cible suit donc le
    bras. Deux effets :
      - le servo ne peut jamais tenter un grand saut ni partir par le chemin long ;
      - c'est stable : chaque correction reduit l'ecart au zero, sans emballement.
    Dans la zone morte on vise la position actuelle : aucun rappel, donc aucune vibration.
    """
    ecart = pas_court(actuel, zero)
    if abs(ecart) <= zone_morte_pas:
        return actuel % RESOLUTION
    borne = max(-pas_max, min(pas_max, ecart))
    return (actuel + borne) % RESOLUTION
