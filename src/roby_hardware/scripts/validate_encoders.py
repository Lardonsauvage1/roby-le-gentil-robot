#!/usr/bin/env python3
"""Validation GO/NO-GO des 3 encodeurs RS-485 (AS5048A), BRAS IMMOBILE.

Read-only : interroge les esclaves 1/2/3, ne pilote AUCUN moteur. A lancer
APRES toute intervention hardware sur les encodeurs, pour decider si le
closed-loop peut etre reactive (cf. piege #8 aimant desaligne).

Critere : un encodeur immobile sain doit etre stable. On mesure, par esclave :
  - taux de reponses valides (vs 0xFFFE / timeout)
  - dispersion (ecart-type des residus, en tenant compte du wrap 0/360)
  - taux d'outliers (sauts vs la valeur centrale)

Usage :  python3 validate_encoders.py [n_lectures]   (defaut 2500)
Affiche les stats sur TOUT l'echantillon + les SHOW_VALUES premieres lectures
brutes par encodeur (pour diagnostiquer en live un correctif physique).
Quitte avec code 0 si TOUS GO, 1 sinon (utilisable en script).
"""
import sys
import time
import statistics

import serial
from gpiozero import DigitalOutputDevice

# --- Liaison RS-485 (identique a encoder_publisher.py) ---
DE_RE_PIN = 26
PORT = "/dev/ttyAMA0"
BAUD = 115200
TIMEOUT = 0.03
SLAVES = (1, 2, 3)

# --- Seuils GO/NO-GO (bras immobile) ---
MIN_VALID_RATE = 0.90   # >= 90% de reponses valides
MAX_SPREAD_DEG = 3.0    # ecart-type des residus <= 3 deg (slave 1 sain ~1.4)
OUTLIER_DEG = 5.0       # un residu > 5 deg = lecture aberrante (bras immobile)
MAX_OUTLIER_RATE = 0.05  # <= 5% d'outliers
SHOW_VALUES = 200       # nb de lectures brutes affichees par encodeur (diagnostic)
MEDIAN_WIN = 5          # fenetre du filtre median (= celui applique cote Pi)
MAX_STEP_DEG = 20.0     # rejet : saut > ce seuil vs derniere ACCEPTEE = aberrant
MAX_REJECT = 5          # porte de sortie : apres N rejets consecutifs, on accepte


def wrap180(d):
    return ((d + 180.0) % 360.0) - 180.0


def rate_reject(values, max_step=MAX_STEP_DEG, max_consec=MAX_REJECT):
    """Etage 1 : rejet par limitation de vitesse. Une valeur trop loin (> max_step)
    de la DERNIERE ACCEPTEE est ignoree (on tient la derniere bonne). Apres
    max_consec rejets consecutifs, on accepte quand meme (porte de sortie : evite
    de rester bloque si le capteur a vraiment change de valeur). Sortie alignee
    sur l'entree (valeur tenue a chaque pas)."""
    out = []
    last_acc = None
    consec = 0
    for v in values:
        if last_acc is None:
            last_acc, consec = v, 0
        elif abs(wrap180(v - last_acc)) > max_step:
            consec += 1
            if consec >= max_consec:
                last_acc, consec = v, 0   # porte de sortie
            # sinon : on tient last_acc (rejet)
        else:
            last_acc, consec = v, 0
        out.append(last_acc)
    return out


def median_filter(values, win=MEDIAN_WIN):
    """Etage 2 : filtre median glissant (un pic isole est elimine tant que
    < win/2 echantillons consecutifs sont aberrants)."""
    out = []
    for i in range(len(values)):
        window = values[max(0, i - win + 1):i + 1]
        out.append(statistics.median(window))
    return out


def pi_filter(resid):
    """Pipeline complet cote Pi : rejet de vitesse PUIS mediane."""
    return median_filter(rate_reject(resid))


def query(ser, de_re, sid):
    ser.reset_input_buffer()
    de_re.on(); time.sleep(0.001)
    ser.write(bytes([sid])); ser.flush(); time.sleep(0.001)
    de_re.off()
    t = time.time()
    while time.time() - t < TIMEOUT:
        b = ser.read(1)
        if b and b[0] == 0xFF:
            c = ser.read(1)
            if c and c[0] == sid:
                d = ser.read(2)
                if len(d) == 2:
                    v = (d[0] << 8) | d[1]
                    return None if v == 0xFFFE else (v / 65535.0) * 360.0
            return None
    return None


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 2500
    de_re = DigitalOutputDevice(DE_RE_PIN)
    de_re.off()
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.1)

    data = {s: [] for s in SLAVES}
    errors = {s: 0 for s in SLAVES}
    seqlog = {s: [] for s in SLAVES}   # toutes les lectures dans l'ordre (None inclus)
    print(f"Lecture {n} echantillons/esclave, BRAS IMMOBILE...")
    for _ in range(n):
        for s in SLAVES:
            v = query(ser, de_re, s)
            seqlog[s].append(v)
            if v is None:
                errors[s] += 1
            else:
                data[s].append(v)
        time.sleep(0.005)
    de_re.close(); ser.close()

    # Colonnes BRUT (avant filtre, pour diagnostic) et FILTRE (apres median N=5,
    # = ce que le Pi utilise reellement). Le VERDICT porte sur le FILTRE : un pic
    # isole eliminable par la mediane ne doit pas faire echouer le test.
    print(f"\n{'enc':>3} | {'valides':>8} | {'centre':>7} | "
          f"{'spr.brut':>8} {'out.brut':>8} | {'spr.FILT':>8} {'out.FILT':>8} | verdict")
    print("-" * 78)
    all_go = True
    for s in SLAVES:
        d = data[s]
        total = len(d) + errors[s]
        valid_rate = len(d) / total if total else 0.0
        if d:
            center = statistics.median(d)
            resid = [wrap180(v - center) for v in d]
            # BRUT
            spread_raw = statistics.pstdev(resid) if len(resid) > 1 else 0.0
            out_raw = sum(1 for r in resid if abs(r) > OUTLIER_DEG) / len(d)
            # FILTRE (pipeline Pi complet : rejet de vitesse PUIS mediane)
            filt = pi_filter(resid)
            spread_f = statistics.pstdev(filt) if len(filt) > 1 else 0.0
            out_f = sum(1 for r in filt if abs(r) > OUTLIER_DEG) / len(filt)
        else:
            center = float("nan")
            spread_raw = out_raw = spread_f = out_f = float("inf")

        # Verdict sur le signal FILTRE
        go = (valid_rate >= MIN_VALID_RATE and spread_f <= MAX_SPREAD_DEG
              and out_f <= MAX_OUTLIER_RATE)
        all_go = all_go and go
        print(f"{s:>3} | {valid_rate*100:6.1f}% | {center:6.1f}d | "
              f"{spread_raw:7.2f}d {out_raw*100:6.1f}% | "
              f"{spread_f:7.2f}d {out_f*100:6.1f}% | "
              f"{'✅ GO' if go else '❌ NO-GO'}")

    print("-" * 78)
    print(f"Verdict sur FILTRE (pipeline Pi = rejet vitesse >{MAX_STEP_DEG:.0f}d "
          f"puis median N={MEDIAN_WIN}). Seuils : valides>={MIN_VALID_RATE*100:.0f}%  "
          f"spr.FILT<={MAX_SPREAD_DEG}d  out.FILT(>{OUTLIER_DEG}d)<={MAX_OUTLIER_RATE*100:.0f}%")

    # --- Dump des SHOW_VALUES premieres lectures, dans l'ordre (None = ERR) ---
    # (stats calculees sur les {n} lectures ci-dessus ; on n'affiche que le
    #  debut pour rester lisible et permettre un diagnostic visuel.)
    for s in SLAVES:
        seq = seqlog[s][:SHOW_VALUES]
        print(f"\n=== enc {s} : {SHOW_VALUES} premieres lectures / {len(seqlog[s])} "
              f"(dans l'ordre, ERR=pas de reponse) ===")
        cells = ["ERR" if v is None else f"{v:6.1f}" for v in seq]
        for i in range(0, len(cells), 10):
            print("  " + " ".join(cells[i:i + 10]))
    if all_go:
        print("\n>>> TOUS GO — feedback fiable, closed-loop reactivable.")
        sys.exit(0)
    else:
        print("\n>>> NO-GO — NE PAS reactiver le closed-loop. Verifier aimant/cablage "
              "des encodeurs en echec (piege #8).")
        sys.exit(1)


if __name__ == "__main__":
    main()
