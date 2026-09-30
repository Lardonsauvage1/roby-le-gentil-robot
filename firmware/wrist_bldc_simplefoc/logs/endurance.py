import serial, time, os, sys
STOP = os.path.join(os.path.dirname(__file__), "STOP")
LOG = open(os.path.join(os.path.dirname(__file__), "endurance_20260924.log"), "a", buffering=1)
def log(m): LOG.write(time.strftime("%H:%M:%S ") + m + "\n")
# Port en argument (defaut : lien udev de la carte, cf. src/roby_wrist_bldc/udev/)
s = serial.Serial(sys.argv[1] if len(sys.argv) > 1 else "/dev/roby_wrist", 115200, timeout=0.05)
time.sleep(0.3); s.reset_input_buffer()
s.write(b"E\n")
t_start = time.time(); last_rx = time.time(); pos = None; imax_total = 0
def poll():
    global last_rx, pos, imax_total
    l = s.readline().decode(errors="replace").strip()
    if not l: return None
    if not l.startswith("S "): log("msg: " + l); return ("msg", l)
    try: p, i, f = l.split()[1:4]; p = float(p); i = float(i); f = int(f)
    except: return None
    last_rx = time.time(); pos = p; imax_total = max(imax_total, abs(i))
    return ("S", p, i, f)
def go(target, tmax=4.0):
    s.write(f"P{target:.4f}\n".encode()); t0 = time.time(); imax = 0; settled = None
    while time.time() - t0 < tmax:
        r = poll()
        if os.path.exists(STOP): return "STOP demande"
        if time.time() - last_rx > 1.0: return "plus de donnees de la carte"
        if r and r[0] == "msg" and "FAULT" in r[1]: return "FAULT carte: " + r[1]
        if r and r[0] == "S":
            _, p, i, f = r; imax = max(imax, abs(i))
            if f: return "FAULT carte (flag defaut)"
            if abs(i) > 5.0: log(f"pic {i:+.2f} A a pos {p:+.4f} (cible {target:+.4f}, t={time.time()-t0:.2f}s)")
            if abs(i) > 40.0: s.write(b"S\n"); return f"courant {i:.2f} A > 40 A"
            if abs(p - target) < 0.005 and settled is None: settled = time.time() - t0
            if settled is not None and time.time() - t0 > settled + 0.3: return ("ok", imax, p)
    return ("ok", imax, pos)
while pos is None and time.time() - t_start < 3: poll()
if pos is None: log("FIN: aucune donnee au depart"); sys.exit()
p0 = pos; log(f"DEBUT va-et-vient p0={p0:+.4f} amplitude +0.5 rad, seuil 40 A")
cycle = 0; reason = None
while True:
    if time.time() - t_start > 1800: reason = "duree max 30 min atteinte"; break
    r1 = go(p0 + 0.5)
    if not isinstance(r1, tuple): reason = r1; break
    r2 = go(p0)
    if not isinstance(r2, tuple): reason = r2; break
    cycle += 1
    if cycle % 10 == 0:
        log(f"cycle {cycle}: aller {r1[2]:+.4f} ({r1[1]:.2f} A)  retour {r2[2]:+.4f} ({r2[1]:.2f} A)  t={time.time()-t_start:.0f}s")
log(f"FIN apres {cycle} cycles, {time.time()-t_start:.0f}s: {reason}. dernier pos={pos}, courant max total {imax_total:.2f} A")
print(f"FIN apres {cycle} cycles, {time.time()-t_start:.0f}s: {reason}. courant max total {imax_total:.2f} A")
