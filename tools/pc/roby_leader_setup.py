#!/usr/bin/env python3
"""roby_leader_setup.py -- attribution des IDs + banc du bus chaine (US-017).

Complement de roby_leader_bench.py (US-016), qui lui n'ecrit JAMAIS. Ce script-ci
ecrit, mais **uniquement le registre ID**, et sous conditions strictes.

Commandes :

  set-id --to N     attribue l'ID N au servo SEUL sur le bus.
                    DRY-RUN par defaut : n'ecrit rien tant que --go n'est pas donne.

  syncread          lecture SYNCHRONE des positions (une transaction pour tous les
                    servos, pas une par servo), mesure la frequence et compte les
                    erreurs. --duration 60 pour le test d'acceptation.

  current           releve tension / courant / charge / temperature par servo.

Pourquoi un servo a la fois : les STS3215 sortent d'usine **tous en ID 1**. Deux
servos ensemble repondent simultanement au meme ping -> collision, bus illisible.

Garde-fous de set-id (tous bloquants) :
  - EXACTEMENT UN servo doit repondre sur le bus ;
  - son Model_Number doit valoir 777 (STS3215) ;
  - son couple doit etre OFF (on ne touche pas a un servo sous couple) ;
  - rien n'est ecrit sans --go.

Sequence d'ecriture : Lock=0 (deverrouille l'EEPROM) -> ID=N -> Lock=1 adresse au
NOUVEL ID -> relecture de controle. Lock est a l'adresse 55 pour la serie STS/SMS
(verifie dans lerobot/motors/feetech/tables.py ; 48 est la serie SCS, pas la notre).
"""

import argparse
import os
import statistics
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from roby_leader_bench import (  # noqa: E402
    ADDR_ID,
    RESOLUTION,
    ADDR_MODEL_NUMBER,
    ADDR_PRESENT_LOAD,
    ADDR_PRESENT_POSITION,
    ADDR_PRESENT_TEMPERATURE,
    ADDR_TORQUE_ENABLE,
    ADDR_PRESENT_VOLTAGE,
    DEFAULT_BAUD,
    MODEL_STS3215,
    decode_load,
    find_port,
    open_bus,
    read_reg,
    steps_to_deg,
)

try:
    from scservo_sdk import COMM_SUCCESS, GroupSyncRead
except ImportError:
    sys.exit("scservo_sdk introuvable : lancer via bash ~/roby_leader_setup.sh ...")

ADDR_HOMING_OFFSET = (31, 2)   # EEPROM, sign-magnitude bit 11
ADDR_LOCK = (55, 1)          # EEPROM lock, serie STS/SMS
ADDR_PRESENT_CURRENT = (69, 2)
# Fenetre de tension acceptable pour ECRIRE en EEPROM (dixiemes de volt).
# Une ecriture EEPROM sous tension insuffisante peut CORROMPRE la memoire du
# servo. Les STS3215 7,4 V lisent 73-74 en fonctionnement sain.
VOLT_MIN_WRITE = 65  # 6,5 V
VOLT_MAX_WRITE = 84  # 8,4 V
MAX_ID = 20  # plage utile (IDs vises 1..6) ; --max-id 253 pour un balayage exhaustif


def scan_bus(ph, port, max_id):
    return [sid for sid in range(0, max_id + 1) if ph.ping(port, sid)[1] == COMM_SUCCESS]


def write_reg(ph, port, sid, reg, value):
    addr, size = reg
    if size == 1:
        comm, err = ph.write1ByteTxRx(port, sid, addr, value)
    else:
        comm, err = ph.write2ByteTxRx(port, sid, addr, value)
    return comm == COMM_SUCCESS and err == 0


# ---------------------------------------------------------------- set-id
def cmd_set_id(args):
    target = args.to
    if not 0 <= target <= 253:
        sys.exit("ID cible hors plage : %d" % target)

    port_name = find_port(args.port)
    port, ph = open_bus(port_name, args.baud)
    print("Port     : %s" % port_name)
    print("ID cible : %d" % target)
    print("Mode     : %s\n" % ("ECRITURE (--go)" if args.go else "DRY-RUN (rien ne sera ecrit)"))

    print("Balayage du bus (0..%d)..." % args.max_id)
    found = scan_bus(ph, port, args.max_id)
    print("  servo(s) qui repondent : %s\n" % (found if found else "AUCUN"))

    if len(found) == 0:
        port.closePort()
        sys.exit(
            "REFUS : aucun servo ne repond.\n"
            "  - alimentation 7,4 V presente AU CONNECTEUR du servo ?\n"
            "  - cable sur le bornier TTL BUS (pas RS485, pas l'USB) ?\n"
            "  - polarite verifiee ?"
        )
    if len(found) > 1:
        port.closePort()
        sys.exit(
            "REFUS : %d servos repondent (IDs %s).\n"
            "L'attribution se fait UN SERVO A LA FOIS, seul sur le bus.\n"
            "Debrancher les autres avant de recommencer." % (len(found), found)
        )

    sid = found[0]
    model = read_reg(ph, port, sid, ADDR_MODEL_NUMBER)
    torque = read_reg(ph, port, sid, ADDR_TORQUE_ENABLE)
    volt = read_reg(ph, port, sid, ADDR_PRESENT_VOLTAGE)
    pos = read_reg(ph, port, sid, ADDR_PRESENT_POSITION)
    print("Servo trouve : ID %d" % sid)
    print("  Model_Number : %s" % model)
    print("  couple       : %s" % ("OFF" if torque == 0 else "ON  <-- PROBLEME"))
    print("  tension      : %s" % ("%.1f V" % (volt / 10.0) if volt is not None else "?"))
    print("  position     : %s pas (%s)\n" % (pos, "%.1f deg" % steps_to_deg(pos) if pos is not None else "?"))

    if model != MODEL_STS3215:
        port.closePort()
        sys.exit("REFUS : Model_Number %s != %d (STS3215)." % (model, MODEL_STS3215))
    if torque != 0:
        port.closePort()
        sys.exit("REFUS : le couple est ACTIF. On n'ecrit pas dans un servo sous couple.")
    if volt is None or not VOLT_MIN_WRITE <= volt <= VOLT_MAX_WRITE:
        port.closePort()
        sys.exit(
            "REFUS : tension %s hors de la fenetre d'ecriture [%.1f V .. %.1f V].\n"
            "Une ecriture EEPROM sous tension insuffisante peut CORROMPRE le servo.\n"
            "Verifier sur l'alimentation :\n"
            "  - le courant : bloque a la limite (mode CC) => le servo consomme trop ;\n"
            "  - la tension de consigne : toujours reglee sur 7,4 V ?\n"
            "  - le cablage : chute de tension, mauvais contact, polarite."
            % ("%.1f V" % (volt / 10.0) if volt is not None else "illisible",
               VOLT_MIN_WRITE / 10.0, VOLT_MAX_WRITE / 10.0)
        )
    if sid == target:
        port.closePort()
        print("Rien a faire : ce servo porte deja l'ID %d." % target)
        return 0

    print("Action : ID %d -> ID %d  (Lock=0, ecriture, Lock=1, relecture)" % (sid, target))
    if not args.go:
        port.closePort()
        print("\nDRY-RUN : rien n'a ete ecrit. Relancer avec --go pour appliquer.")
        return 0

    if not write_reg(ph, port, sid, ADDR_LOCK, 0):
        port.closePort()
        sys.exit("ECHEC : deverrouillage EEPROM (Lock=0) refuse. Rien n'a ete modifie.")
    # /!\ Le code retour de cette ecriture n'est PAS fiable (constate le 2026-09-02) :
    # des que l'ID change, le servo renvoie son accuse avec le NOUVEL ID alors que le
    # SDK attend une reponse de l'ANCIEN -> il rapporte un echec de comm sur une
    # ecriture qui a REUSSI. Seule la relecture du bus fait foi.
    write_reg(ph, port, sid, ADDR_ID, target)
    time.sleep(0.3)

    moved = ph.ping(port, target)[1] == COMM_SUCCESS
    still_old = ph.ping(port, sid)[1] == COMM_SUCCESS
    if not moved:
        if still_old:
            write_reg(ph, port, sid, ADDR_LOCK, 1)
            port.closePort()
            sys.exit(
                "ECHEC : l'ID n'a pas change, le servo repond toujours en %d.\n"
                "EEPROM reverrouillee, rien n'est reste ouvert." % sid
            )
        port.closePort()
        sys.exit(
            "ECHEC : plus aucun servo ne repond (ni %d ni %d).\n"
            "Verifier l'alimentation et le cablage avant de recommencer." % (sid, target)
        )
    if not write_reg(ph, port, target, ADDR_LOCK, 1):
        print("[!] Reverrouillage EEPROM (Lock=1) echoue sur l'ID %d." % target)
        print("    L'ID est probablement ecrit ; verifier ci-dessous et relancer si besoin.")

    print("\n--- relecture de controle ---")
    after = scan_bus(ph, port, args.max_id)
    model2 = read_reg(ph, port, target, ADDR_MODEL_NUMBER)
    id2 = read_reg(ph, port, target, ADDR_ID)
    lock2 = read_reg(ph, port, target, ADDR_LOCK)
    print("  servo(s) sur le bus : %s" % after)
    print("  ID relu             : %s" % id2)
    print("  Model_Number        : %s" % model2)
    print("  Lock                : %s (1 = EEPROM reverrouillee)" % lock2)
    port.closePort()

    ok = after == [target] and id2 == target and model2 == MODEL_STS3215
    print("\n%s" % ("OK : le servo porte maintenant l'ID %d." % target if ok
                    else "ATTENTION : la relecture ne confirme pas l'attribution."))
    print("Etiqueter physiquement ce servo '%d' avant de le debrancher." % target)
    return 0 if ok else 1


# ---------------------------------------------------------------- syncread
def cmd_syncread(args):
    port_name = find_port(args.port)
    port, ph = open_bus(port_name, args.baud)
    ids = args.ids or scan_bus(ph, port, args.max_id)
    if not ids:
        port.closePort()
        sys.exit("Aucun servo sur le bus.")

    addr, size = ADDR_PRESENT_POSITION
    group = GroupSyncRead(port, ph, addr, size)
    for sid in ids:
        group.addParam(sid)

    print("Port %s | IDs %s | lecture SYNCHRONE (1 transaction pour %d servos)"
          % (port_name, ids, len(ids)))
    print("Duree %.0f s | lecture seule, aucun couple envoye\n" % args.duration)

    periods, errors, missing, n = [], 0, {sid: 0 for sid in ids}, 0
    last_show = 0.0
    t_end = time.perf_counter() + args.duration
    t_prev = time.perf_counter()
    while time.perf_counter() < t_end:
        comm = group.txRxPacket()
        now = time.perf_counter()
        periods.append(now - t_prev)
        t_prev = now
        n += 1
        if comm != COMM_SUCCESS:
            errors += 1
            continue
        vals = {}
        for sid in ids:
            if group.isAvailable(sid, addr, size):
                vals[sid] = group.getData(sid, addr, size)
            else:
                missing[sid] += 1
        if args.show and now - last_show > 0.5:
            last_show = now
            print("  " + "  ".join("ID%d=%6.1f deg" % (s, steps_to_deg(v))
                                   for s, v in sorted(vals.items())))

    port.closePort()
    if not periods:
        sys.exit("Aucune lecture effectuee.")
    hz = [1.0 / p for p in periods if p > 0]
    print("\n--- resultats ---")
    print("  lectures        : %d" % n)
    print("  erreurs de comm : %d" % errors)
    print("  frequence  min  : %.1f Hz" % min(hz))
    print("  frequence  moy  : %.1f Hz" % statistics.mean(hz))
    print("  frequence  max  : %.1f Hz" % max(hz))
    for sid in ids:
        if missing[sid]:
            print("  [!] ID %d absent de %d trames" % (sid, missing[sid]))
    verdict_hz = "OK (>= 50 Hz)" if statistics.mean(hz) >= 50 else "INSUFFISANT (< 50 Hz)"
    verdict_err = "OK (0 erreur)" if errors == 0 and not any(missing.values()) else "ECHEC"
    print("\n  critere frequence : %s" % verdict_hz)
    print("  critere erreurs   : %s" % verdict_err)
    return 0


# ---------------------------------------------------------------- current
def cmd_current(args):
    port_name = find_port(args.port)
    port, ph = open_bus(port_name, args.baud)
    ids = args.ids or scan_bus(ph, port, args.max_id)
    if not ids:
        port.closePort()
        sys.exit("Aucun servo sur le bus.")
    print("Port %s | IDs %s | lecture seule\n" % (port_name, ids))
    print("%-5s %10s %10s %9s %8s %8s" % ("ID", "pos(deg)", "courant", "charge", "temp", "tension"))
    for sid in ids:
        pos = read_reg(ph, port, sid, ADDR_PRESENT_POSITION)
        cur = read_reg(ph, port, sid, ADDR_PRESENT_CURRENT)
        load = decode_load(read_reg(ph, port, sid, ADDR_PRESENT_LOAD))
        temp = read_reg(ph, port, sid, ADDR_PRESENT_TEMPERATURE)
        volt = read_reg(ph, port, sid, ADDR_PRESENT_VOLTAGE)
        print("%-5d %10s %10s %9s %8s %8s" % (
            sid,
            "%.1f" % steps_to_deg(pos) if pos is not None else "?",
            cur if cur is not None else "?",
            load if load is not None else "?",
            "%d C" % temp if temp is not None else "?",
            "%.1f V" % (volt / 10.0) if volt is not None else "?",
        ))
    port.closePort()
    print("\n[!] 'courant' est la valeur BRUTE du registre Present_Current (unite non")
    print("    confirmee pour ce modele). La mesure qui FAIT FOI pour dimensionner")
    print("    l'alimentation est celle de l'afficheur de l'alim de labo.")
    return 0


# ---------------------------------------------------------------- lock
def cmd_lock(args):
    """Lit (ou remet) le verrou EEPROM. Lock=1 = protegee, Lock=0 = ecriture permise."""
    port_name = find_port(args.port)
    port, ph = open_bus(port_name, args.baud)
    ids = args.ids or scan_bus(ph, port, args.max_id)
    if not ids:
        port.closePort()
        sys.exit("Aucun servo sur le bus.")
    for sid in ids:
        cur = read_reg(ph, port, sid, ADDR_LOCK)
        etat = "?" if cur is None else ("VERROUILLEE" if cur == 1 else "DEVERROUILLEE")
        print("ID %-3d Lock=%s  (%s)" % (sid, cur, etat))
        if args.set is not None and cur != args.set:
            if not args.go:
                print("      -> passerait a %d  (dry-run, ajouter --go)" % args.set)
                continue
            write_reg(ph, port, sid, ADDR_LOCK, args.set)
            time.sleep(0.1)
            print("      -> relu : %s" % read_reg(ph, port, sid, ADDR_LOCK))
    port.closePort()
    return 0


# ---------------------------------------------------------------- calib
CALIB_FILE = os.path.expanduser("~/roby_leader_calib.yaml")
RESOLUTION = 4096
EDGE = 0.10  # a moins de 10 % d'un bord -> risque de bouclage 4095->0


def _load_calib():
    if not os.path.exists(CALIB_FILE):
        return {"servos": {}}
    with open(CALIB_FILE, encoding="utf-8") as f:
        return yaml.safe_load(f) or {"servos": {}}


def _mean_pos(ph, port, sid, n=5):
    vals = [read_reg(ph, port, sid, ADDR_PRESENT_POSITION) for _ in range(n)]
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, None
    return round(statistics.mean(vals)), (max(vals) - min(vals))


def span_and_dir(mn, mx, mid):
    """Course reelle min->max et sens, leve l'ambiguite du bouclage 4095->0.

    Un servo dont la course franchit le zero donne min > max en valeur brute : la
    soustraction directe est alors FAUSSE. La position INTERMEDIAIRE tranche, car
    elle n'est situee que sur le vrai trajet.
    Retourne (course_en_pas, sens, position_du_mid_sur_la_course) ou None si le mid
    n'est sur aucun des deux trajets (mesure incoherente).
    """
    if mn is None or mx is None:
        return None
    up, mid_up = (mx - mn) % RESOLUTION, None
    dn, mid_dn = (mn - mx) % RESOLUTION, None
    if mid is None:
        return None
    mid_up = (mid - mn) % RESOLUTION
    mid_dn = (mn - mid) % RESOLUTION
    if mid_up <= up and not (mid_dn <= dn):
        return up, "croissant", mid_up
    if mid_dn <= dn and not (mid_up <= up):
        return dn, "decroissant", mid_dn
    return None


def cmd_calib(args):
    data = _load_calib()
    servos = data.setdefault("servos", {})

    if args.show:
        if not servos:
            print("Aucune borne enregistree (%s absent)." % CALIB_FILE)
            return 0
        print("Fichier : %s\n" % CALIB_FILE)
        print("%-4s %-14s %12s %12s %11s" % ("ID", "articulation", "min(pas/deg)", "max(pas/deg)", "amplitude"))
        for sid in sorted(servos, key=int):
            e = servos[sid]
            mn, mx = e.get("min_steps"), e.get("max_steps")
            col = lambda v: "%5s/%6.1f" % (v, v * 360.0 / RESOLUTION) if v is not None else "     -/     -"
            if e.get("continuous"):
                print("%-4s %-14s %12s %12s %11s"
                      % (sid, e.get("joint", "?"), "  sans", " butee", "360.0 deg"))
                print("     rotation CONTINUE : course = tour complet (4096 pas). Pas de")
                print("     normalisation min/max ; le point de reference se fixe au")
                print("     montage des zeros (US-019).")
                continue
            res = span_and_dir(mn, mx, e.get("mid_steps"))
            amp = "%.1f deg" % (res[0] * 360.0 / RESOLUTION) if res else "?"
            print("%-4s %-14s %12s %12s %11s" % (sid, e.get("joint", "?"), col(mn), col(mx), amp))
            if res:
                span, sens, at = res
                boucle = ((mx - mn) % RESOLUTION != mx - mn) if sens == "croissant" else ((mn - mx) % RESOLUTION != mn - mx)
                print("     sens min->max : %s%s | mid a %d pas (%.0f %% de la course)"
                      % (sens, "  [BOUCLE par 0]" if boucle else "", at, 100.0 * at / span))
            elif mn is not None and mx is not None:
                print("     [!] amplitude INDECIDABLE : mesurer la position intermediaire")
                print("         (--as mid), sinon impossible de savoir si la course boucle par 0.")
        return 0

    if args.id is not None and args.continuous:
        port_name = find_port(args.port)
        port, ph = open_bus(port_name, args.baud)
        if ph.ping(port, args.id)[1] != COMM_SUCCESS:
            port.closePort()
            sys.exit("L'ID %d ne repond pas sur le bus." % args.id)
        pos, _ = _mean_pos(ph, port, args.id)
        port.closePort()
        print("ID %d : position %d pas (%.1f deg) -- axe SANS BUTEE (rotation continue)"
              % (args.id, pos, pos * 360.0 / RESOLUTION))
        if not args.go:
            print("\nDRY-RUN : rien enregistre. Ajouter --go.")
            return 0
        e = servos.setdefault(str(args.id), {})
        if args.joint:
            e["joint"] = args.joint
        e["continuous"] = True
        for k in ("min_steps", "min_deg", "max_steps", "max_deg", "mid_steps", "mid_deg"):
            e.pop(k, None)
        e["updated"] = time.strftime("%Y-%m-%dT%H:%M")
        with open(CALIB_FILE, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=True, allow_unicode=True)
        print("\nEnregistre : ID %d = rotation continue (course = 4096 pas / 360 deg)" % args.id)
        return 0

    if args.id is None or args.as_ is None:
        sys.exit("Preciser --id N et --as min|max|mid, ou --continuous  (ou --show)")

    port_name = find_port(args.port)
    port, ph = open_bus(port_name, args.baud)
    if ph.ping(port, args.id)[1] != COMM_SUCCESS:
        port.closePort()
        sys.exit("L'ID %d ne repond pas sur le bus." % args.id)
    pos, spread = _mean_pos(ph, port, args.id)
    torque = read_reg(ph, port, args.id, ADDR_TORQUE_ENABLE)
    port.closePort()
    if pos is None:
        sys.exit("Lecture de position impossible.")

    print("ID %d : position %d pas (%.1f deg), dispersion %d pas sur 5 lectures, couple %s"
          % (args.id, pos, pos * 360.0 / RESOLUTION, spread, "OFF" if torque == 0 else "ON"))
    if spread > 5:
        print("[!] Dispersion elevee : le servo bouge encore. Le laisser immobile et refaire.")
    if not args.go:
        print("\nDRY-RUN : rien enregistre. Ajouter --go pour ecrire dans %s" % CALIB_FILE)
        return 0

    e = servos.setdefault(str(args.id), {})
    if args.joint:
        e["joint"] = args.joint
    e["%s_steps" % args.as_] = pos
    e["%s_deg" % args.as_] = round(pos * 360.0 / RESOLUTION, 2)
    e["updated"] = time.strftime("%Y-%m-%dT%H:%M")
    with open(CALIB_FILE, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=True, allow_unicode=True)
    print("\nEnregistre : ID %d  %s = %d pas  ->  %s" % (args.id, args.as_, pos, CALIB_FILE))
    return 0


# ---------------------------------------------------------------- recentrer
CENTRE = RESOLUTION // 2   # 2048


def encode_sm(v, bit=11):
    """Sign-magnitude, comme le firmware Feetech l'attend (bit 11 = signe)."""
    mag = abs(v)
    if mag > (1 << bit) - 1:
        raise ValueError("decalage %d trop grand (max %d)" % (v, (1 << bit) - 1))
    return ((1 if v < 0 else 0) << bit) | mag


def decode_sm(v, bit=11):
    return -(v & ((1 << bit) - 1)) if (v >> bit) & 1 else v & ((1 << bit) - 1)


def cmd_recentrer(args):
    """Decale le zero du codeur pour que la course ne franchisse plus le bouclage.

    Pourquoi : un servo interpole en NUMERO DE PAS. Si sa course utile franchit
    4095->0, il ne peut pas traverser proprement : il part par le chemin long, ou
    reste bloque. Vecu le 2026-09-05 sur l'epaule, qui n'atteignait jamais son neutre.

    En amenant le milieu de course a 2048, la course s'etend de part et d'autre du
    centre et ne touche plus jamais les extremites. Le bouclage disparait pour de bon.

    `Present_Position = Actual_Position - Homing_Offset` (driver Feetech) donc :
        Homing_Offset = position_du_milieu - 2048
    """
    import yaml

    calib = os.path.expanduser(args.calib)
    if not os.path.exists(calib):
        sys.exit("calibration introuvable : %s" % calib)
    servos = yaml.safe_load(open(calib, encoding="utf-8"))["servos"]

    port_name = find_port(args.port)
    port, ph = open_bus(port_name, args.baud)
    print("Port     : %s" % port_name)
    print("Mode     : %s\n" % ("ECRITURE (--go)" if args.go else "DRY-RUN (rien ne sera ecrit)"))

    sid = args.id
    if ph.ping(port, sid)[1] != COMM_SUCCESS:
        port.closePort()
        sys.exit("L'ID %d ne repond pas." % sid)
    if str(sid) not in servos:
        port.closePort()
        sys.exit("Aucune borne mesuree pour l'ID %d dans %s" % (sid, calib))

    e = servos[str(sid)]
    if e.get("continuous"):
        port.closePort()
        sys.exit("L'ID %d est un axe SANS BUTEE : pas de milieu de course a centrer." % sid)

    offset_actuel = decode_sm(read_reg(ph, port, sid, ADDR_HOMING_OFFSET) or 0)
    pos = read_reg(ph, port, sid, ADDR_PRESENT_POSITION)
    torque = read_reg(ph, port, sid, ADDR_TORQUE_ENABLE)
    volt = read_reg(ph, port, sid, ADDR_PRESENT_VOLTAGE)

    mn, mx, mid = e["min_steps"], e["max_steps"], e["mid_steps"]
    up, dn = (mx - mn) % RESOLUTION, (mn - mx) % RESOLUTION
    mu, md = (mid - mn) % RESOLUTION, (mn - mid) % RESOLUTION
    if mu <= up and not md <= dn:
        sens, course = +1, up
    elif md <= dn and not mu <= up:
        sens, course = -1, dn
    else:
        port.closePort()
        sys.exit("Course indecidable pour l'ID %d (mid incoherent)." % sid)
    milieu = (mn + sens * course / 2.0) % RESOLUTION

    # Le milieu est exprime dans le repere ACTUEL (donc deja decale de offset_actuel).
    nouveau = int(round(milieu + offset_actuel - CENTRE))
    bornes = sorted(((mn + offset_actuel - nouveau) % RESOLUTION,
                     (mx + offset_actuel - nouveau) % RESOLUTION))

    print("ID %d   couple=%s   tension=%s" % (
        sid, "OFF" if torque == 0 else "ON  <-- PROBLEME",
        "%.1f V" % (volt / 10.0) if volt is not None else "?"))
    print("  position lue        : %d" % pos)
    print("  Homing_Offset actuel: %d" % offset_actuel)
    print("  milieu de course    : %.0f  (min %d, max %d, course %d pas)"
          % (milieu, mn, mx, course))
    print("  -> Homing_Offset    : %d   (pour amener le milieu a %d)" % (nouveau, CENTRE))
    print("  course apres        : %d .. %d  %s"
          % (bornes[0], bornes[1],
             "OK, ne touche plus les extremites"
             if 0 < bornes[0] and bornes[1] < RESOLUTION - 1 else "<-- ATTENTION"))

    if torque != 0:
        port.closePort()
        sys.exit("REFUS : couple ACTIF. Couper le couple avant de toucher a l'EEPROM.")
    if volt is None or not 65 <= volt <= 84:
        port.closePort()
        sys.exit("REFUS : tension hors plage — ecriture EEPROM interdite.")
    if not args.go:
        port.closePort()
        print("\nDRY-RUN : rien n'a ete ecrit. Relancer avec --go.")
        return 0

    write_reg(ph, port, sid, ADDR_LOCK, 0)
    ok = write_reg(ph, port, sid, ADDR_HOMING_OFFSET, encode_sm(nouveau))
    time.sleep(0.2)
    write_reg(ph, port, sid, ADDR_LOCK, 1)

    relu = decode_sm(read_reg(ph, port, sid, ADDR_HOMING_OFFSET) or 0)
    pos2 = read_reg(ph, port, sid, ADDR_PRESENT_POSITION)
    lock = read_reg(ph, port, sid, ADDR_LOCK)
    port.closePort()
    print("\n--- relecture de controle ---")
    print("  Homing_Offset relu  : %d  (%s)" % (relu, "OK" if relu == nouveau else "ECHEC"))
    print("  position maintenant : %d  (etait %d, decalage attendu %d)"
          % (pos2, pos, -(nouveau - offset_actuel)))
    print("  Lock                : %s" % lock)
    return 0 if relu == nouveau else 1


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--port", default=None, help="force le port (defaut : /dev/roby_leader)")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--max-id", type=int, default=MAX_ID)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("set-id", help="attribue un ID au servo SEUL sur le bus")
    s.add_argument("--to", type=int, required=True, help="ID a attribuer (1..6)")
    s.add_argument("--go", action="store_true", help="ECRIT reellement (sinon dry-run)")
    s.set_defaults(func=cmd_set_id)

    r = sub.add_parser("syncread", help="lecture synchrone + frequence + erreurs")
    r.add_argument("--ids", type=int, nargs="+", default=None)
    r.add_argument("--duration", type=float, default=60.0)
    r.add_argument("--show", action="store_true", help="affiche les positions")
    r.set_defaults(func=cmd_syncread)

    lk = sub.add_parser("lock", help="lit / remet le verrou EEPROM (Lock)")
    lk.add_argument("--ids", type=int, nargs="+", default=None)
    lk.add_argument("--set", type=int, choices=[0, 1], default=None)
    lk.add_argument("--go", action="store_true", help="ECRIT reellement (sinon dry-run)")
    lk.set_defaults(func=cmd_lock)

    cal = sub.add_parser("calib", help="releve les bornes mecaniques (min/max) d'un servo")
    cal.add_argument("--id", type=int, default=None)
    cal.add_argument("--as", dest="as_", choices=["min", "max", "mid"], default=None,
                 help="mid = position intermediaire, indispensable si la course boucle")
    cal.add_argument("--joint", default=None, help="libelle de l'articulation")
    cal.add_argument("--continuous", action="store_true",
                     help="axe SANS butee (rotation continue) : pas de min/max")
    cal.add_argument("--show", action="store_true", help="affiche le tableau enregistre")
    cal.add_argument("--go", action="store_true", help="ENREGISTRE (sinon dry-run)")
    cal.set_defaults(func=cmd_calib)

    rc = sub.add_parser("recentrer",
                        help="decale le zero du codeur pour supprimer le bouclage")
    rc.add_argument("--id", type=int, required=True)
    rc.add_argument("--calib", default="~/roby_leader_calib.yaml")
    rc.add_argument("--go", action="store_true", help="ECRIT en EEPROM (sinon dry-run)")
    rc.set_defaults(func=cmd_recentrer)

    c = sub.add_parser("current", help="tension / courant / charge / temperature")
    c.add_argument("--ids", type=int, nargs="+", default=None)
    c.set_defaults(func=cmd_current)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
