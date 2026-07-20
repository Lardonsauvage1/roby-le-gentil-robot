"""Lance N runs de M lectures, calcule mediane + nb outliers par run,
affiche un tableau comparatif pour juger la fiabilite.

Sam ne touche a rien pendant tout le test. Si les medianes sont coherentes
d'un run a l'autre, les valeurs sont fiables.
"""

import statistics
import time

import serial
from gpiozero import DigitalOutputDevice

DE_RE_PIN = 26
PORT = "/dev/ttyAMA0"
BAUD = 115200
NUM_RUNS = 5
NUM_READS = 100
OUTLIER_THRESHOLD_DEG = 5.0  # ecart a la mediane pour qualifier d'outlier
PAUSE_BETWEEN_RUNS_S = 1.0

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
                    return None if v == 0xFFFE else (v / 65535.0) * 360.0
            return None
    return None


def angular_diff(a, b):
    """Distance circulaire en deg entre a et b dans [0, 180]."""
    d = abs(a - b) % 360
    return min(d, 360 - d)


def analyze(values):
    """Retourne (mediane, n_inliers, n_outliers, n_fails) en gerant le wrap autour de la mediane."""
    n_fails = sum(1 for v in values if v is None)
    floats = [v for v in values if isinstance(v, float)]
    if len(floats) < 3:
        return (None, 0, 0, n_fails)
    med = statistics.median(floats)
    inliers = [v for v in floats if angular_diff(v, med) < OUTLIER_THRESHOLD_DEG]
    outliers = [v for v in floats if angular_diff(v, med) >= OUTLIER_THRESHOLD_DEG]
    # Recalcul mediane sur inliers
    med_clean = statistics.median(inliers) if inliers else med
    return (med_clean, len(inliers), len(outliers), n_fails)


print(f"Lancement de {NUM_RUNS} runs de {NUM_READS} lectures par capteur.")
print(f"Ne touche a rien pendant ~{NUM_RUNS * NUM_READS * 0.05 + NUM_RUNS * PAUSE_BETWEEN_RUNS_S:.0f}s.")
print()

# Resultats par run : medianes par capteur
all_medians = {1: [], 2: [], 3: []}

print(f"{'Run':>3} | {'M1 med':>8} {'in':>3} {'out':>3} {'fail':>4} | {'M2 med':>8} {'in':>3} {'out':>3} {'fail':>4} | {'M3 med':>8} {'in':>3} {'out':>3} {'fail':>4}")
print("-" * 100)

for run in range(1, NUM_RUNS + 1):
    samples = {1: [], 2: [], 3: []}
    for _ in range(NUM_READS):
        for sid in (1, 2, 3):
            samples[sid].append(query(sid))
    cells = []
    for sid in (1, 2, 3):
        med, n_in, n_out, n_fail = analyze(samples[sid])
        all_medians[sid].append(med)
        cells.append(f"{(f'{med:8.2f}' if med is not None else '   ----  '):>8} {n_in:>3} {n_out:>3} {n_fail:>4}")
    print(f"{run:>3} | {cells[0]} | {cells[1]} | {cells[2]}")
    time.sleep(PAUSE_BETWEEN_RUNS_S)

print()
print("=== Coherence inter-runs ===")
for sid in (1, 2, 3):
    meds = [m for m in all_medians[sid] if m is not None]
    if len(meds) < 2:
        print(f"  motor_{sid}: pas assez de runs valides")
        continue
    spread = max(meds) - min(meds)
    overall_med = statistics.median(meds)
    print(f"  motor_{sid}: medianes par run = {[f'{m:.2f}' for m in meds]} | spread = {spread:.3f} deg | mediane globale = {overall_med:.4f}")

de_re.close()
ser.close()
