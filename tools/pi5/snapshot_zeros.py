"""Snapshot des zeros encodeurs.

Lit les 3 encodeurs pendant 10s sans intervention, calcule moyenne / min / max
/ ecart-type / taux d'echec, et propose les valeurs zero a sauvegarder.

Ne sauvegarde RIEN automatiquement — affiche juste le rapport. L'ecriture
du YAML est faite separement apres validation.
"""

import math
import statistics
import time

import serial
from gpiozero import DigitalOutputDevice

DE_RE_PIN = 26
PORT = "/dev/ttyAMA0"
BAUD = 115200
DURATION_S = 10.0

de_re = DigitalOutputDevice(DE_RE_PIN)
de_re.off()
ser = serial.Serial(PORT, BAUD, timeout=0.03)
time.sleep(0.1)


def query(sid):
    ser.reset_input_buffer()
    de_re.on(); time.sleep(0.001)
    ser.write(bytes([sid])); ser.flush(); time.sleep(0.001)
    de_re.off()
    t = time.time()
    while time.time() - t < 0.03:
        b = ser.read(1)
        if b and b[0] == 0xFF:
            c = ser.read(1)
            if c and c[0] == sid:
                d = ser.read(2)
                if len(d) == 2:
                    v = (d[0] << 8) | d[1]
                    return "NM" if v == 0xFFFE else (v / 65535.0) * 360.0
            return None
    return None


samples = {1: [], 2: [], 3: []}
fails = {1: 0, 2: 0, 3: 0}

print(f"Lecture pendant {DURATION_S}s — NE TOUCHE A RIEN...")
t0 = time.time()
while time.time() - t0 < DURATION_S:
    for sid in (1, 2, 3):
        v = query(sid)
        if isinstance(v, float):
            samples[sid].append(v)
        else:
            fails[sid] += 1

print()
print(f"{'Motor':>6} | {'count':>6} | {'fails':>6} | {'min':>7} | {'max':>7} | {'mean':>7} | {'stdev':>6} | {'rate%':>5}")
print("-" * 80)
zeros = {}
for sid in (1, 2, 3):
    s = samples[sid]
    n = len(s)
    f = fails[sid]
    total = n + f
    rate = 100.0 * n / total if total > 0 else 0.0
    if n >= 2:
        sd = statistics.stdev(s)
        m = statistics.mean(s)
        mn = min(s)
        mx = max(s)
        # Gestion wrap : si l'ecart-type est gigantesque (>50 deg) c'est
        # probablement parce que les valeurs traversent 360->0. On detecte.
        if mx - mn > 180:
            # Tente unwrap : shift les valeurs < 180 de +360
            unwrapped = [v + 360 if v < 180 else v for v in s]
            if max(unwrapped) - min(unwrapped) < 180:
                m = statistics.mean(unwrapped) % 360
                sd = statistics.stdev(unwrapped)
                mn = min(unwrapped) % 360
                mx = max(unwrapped) % 360
        print(f"{sid:>6} | {n:>6} | {f:>6} | {mn:>7.2f} | {mx:>7.2f} | {m:>7.2f} | {sd:>6.3f} | {rate:>5.1f}")
        zeros[sid] = m
    else:
        print(f"{sid:>6} | {n:>6} | {f:>6} | (pas assez de samples)")

print()
print("Valeurs zero proposees (moyenne sur les lectures reussies) :")
for sid, v in zeros.items():
    print(f"  motor_{sid}: {v:.4f}")

de_re.close()
ser.close()
