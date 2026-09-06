#!/usr/bin/env python3
"""roby_leader_bench.py -- banc LECTURE SEULE du bras guide (Feetech STS3215).

US-016 (spec-teleoperation-bras-guide) : prouver que l'alimentation, la polarite et
le lien USB sont sains AVANT de risquer six moteurs.

*** CE SCRIPT N'ECRIT JAMAIS DANS UN SERVO. ***
Pas de couple, pas d'ID, pas de baudrate, pas de position cible. Uniquement des
lectures. Les servos doivent rester libres a la main pendant tout le test.
(L'ecriture d'ID viendra dans US-017, avec un autre script.)

Commandes :

  scan            ping les IDs et affiche Model_Number (attendu 777 = STS3215)
                  --max-id N     : borne haute du balayage (defaut 20)
                  --full         : balaye 0..253
                  --all-bauds    : reessaye sur tous les baudrates usuels
                                   (a utiliser si rien ne repond a 1 Mbaud)

  read            lecture continue : position, tension, temperature, charge, couple
                  --ids 1 2 3    : limite aux IDs donnes (defaut : ceux trouves au scan)
                  --hz 2         : cadence d'affichage
                  --once         : une seule lecture puis sortie

Lecture des resultats :
  - Model_Number 777      => STS3215 reconnu, bus et alimentation sains.
  - Couple = OFF          => le servo est libre a la main, c'est l'etat attendu.
  - Tension ~7.4 V        => alimentation correcte AU SERVO (pas seulement au bornier).
  - Position qui suit la main => encodeur et communication OK.
  - Aucune reponse        => voir le diagnostic imprime en fin de scan.

Usage : bash ~/roby_leader_bench.sh scan
        bash ~/roby_leader_bench.sh read
"""

import argparse
import fcntl
import glob
import os
import sys
import time

try:
    from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler
except ImportError:
    sys.exit(
        "scservo_sdk introuvable.\n"
        "Ce script doit tourner dans le venv lerobot : bash ~/roby_leader_bench.sh ..."
    )

# --- Table de registres STS/SMS (adresse, taille). Source : lerobot/motors/feetech/tables.py
ADDR_MODEL_NUMBER = (3, 2)
ADDR_ID = (5, 1)
ADDR_BAUD_RATE = (6, 1)
ADDR_TORQUE_ENABLE = (40, 1)
ADDR_PRESENT_POSITION = (56, 2)
ADDR_PRESENT_LOAD = (60, 2)
ADDR_PRESENT_VOLTAGE = (62, 1)
ADDR_PRESENT_TEMPERATURE = (63, 1)

PROTOCOL_END = 0  # STS/SMS
MODEL_STS3215 = 777
RESOLUTION = 4096
DEFAULT_BAUD = 1_000_000
USUAL_BAUDS = [1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400]

SERIAL_LEADER = "5B79030022"  # numero de serie du pont CH343 de la carte du leader


def find_port(explicit=None):
    """Retourne le chemin du port serie de la carte du leader.

    Ordre : --port explicite, puis /dev/roby_leader (regle udev), puis recherche
    par NUMERO DE SERIE dans /dev/serial/by-id. On ne devine jamais un ttyACMx :
    /dev/ttyACM0 (Winbond) preexiste sur ce PC et n'est pas la carte.
    """
    if explicit:
        return explicit
    if os.path.exists("/dev/roby_leader"):
        return "/dev/roby_leader"
    for path in glob.glob("/dev/serial/by-id/*"):
        if SERIAL_LEADER in path:
            print(
                "[!] /dev/roby_leader absent : la regle udev n'est pas installee.\n"
                "    Repli sur %s (fonctionne, mais le nom peut changer).\n"
                "    Installer la regle :\n"
                "      sudo cp ~/ros2_ws/tools/pc/udev/99-roby-leader.rules /etc/udev/rules.d/\n"
                "      sudo udevadm control --reload-rules && sudo udevadm trigger"
                " --subsystem-match=tty\n" % path
            )
            return path
    sys.exit(
        "Carte du leader introuvable (serie %s).\n"
        "Verifier qu'elle est branchee : ls -l /dev/serial/by-id/" % SERIAL_LEADER
    )


# Descripteurs des verrous poses : gardes vivants pour toute la duree du process.
# Si on les laissait etre ramasses par le GC, le verrou serait relache aussitot.
_VERROUS = []


def prendre_le_bus(port_name):
    """Verrou EXCLUSIF sur le port, comme le fait leader_node.

    Sans ce verrou, un outil de banc peut lire le bus pendant que leader_node le lit
    aussi : deux maitres sur le meme port serie. Vecu le 2026-09-05 -> le noeud est mort
    sur `multiple access on port`, SANS passer par sa procedure d'arret, laissant un
    servo sous couple. Un /dev/ttyACM* s'ouvre deux fois sous Linux : rien ne l'empeche
    a part ce flock.
    """
    fd = os.open(port_name, os.O_RDWR)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        sys.exit(
            "Le bus %s est DEJA DETENU par un autre processus (leader_node ?).\n"
            "Deux maitres sur le meme port serie font planter celui qui tourne.\n"
            "  - pour LIRE l'etat pendant que le noeud tourne : passer par ses topics ROS\n"
            "      ros2 topic echo /leader/joint_states\n"
            "      ros2 topic echo /leader/telemetry\n"
            "  - pour utiliser ce banc : arreter d'abord leader_node." % port_name
        )
    _VERROUS.append(fd)


def open_bus(port_name, baud):
    prendre_le_bus(port_name)
    port = PortHandler(port_name)
    if not port.openPort():
        sys.exit("Ouverture impossible : %s" % port_name)
    if not port.setBaudRate(baud):
        port.closePort()
        sys.exit("Baudrate %d refuse sur %s" % (baud, port_name))
    return port, PacketHandler(PROTOCOL_END)


def read_reg(ph, port, sid, reg):
    """Lit un registre (adresse, taille). Retourne None si pas de reponse."""
    addr, size = reg
    try:
        if size == 1:
            value, comm, err = ph.read1ByteTxRx(port, sid, addr)
        else:
            value, comm, err = ph.read2ByteTxRx(port, sid, addr)
    except IndexError:
        # Le SDK indexe la reponse sans verifier qu'elle est arrivee : une lecture vide
        # le fait planter au lieu de renvoyer une erreur. Cela se produit juste apres
        # une ecriture EEPROM, pendant que le servo est occupe (vecu le 2026-09-05 : la
        # relecture de controle plantait alors que l'ecriture avait REUSSI).
        return None
    if comm != COMM_SUCCESS or err != 0:
        return None
    return value


def decode_load(raw):
    """Charge STS : magnitude sur 10 bits + bit de signe en position 10."""
    if raw is None:
        return None
    sign = -1 if raw & (1 << 10) else 1
    return sign * (raw & 0x3FF)


def steps_to_deg(raw):
    return None if raw is None else raw * 360.0 / RESOLUTION


def cmd_scan(args):
    port_name = find_port(args.port)
    bauds = USUAL_BAUDS if args.all_bauds else [args.baud]
    max_id = 253 if args.full else args.max_id

    print("Port   : %s" % port_name)
    print("IDs    : 0..%d" % max_id)
    print("Bauds  : %s" % ", ".join(str(b) for b in bauds))
    print("Ecriture : AUCUNE (lecture seule)\n")

    found_any = []
    for baud in bauds:
        port, ph = open_bus(port_name, baud)
        found = []
        for sid in range(0, max_id + 1):
            model, comm, err = ph.ping(port, sid)
            if comm != COMM_SUCCESS:
                continue
            found.append((sid, model))
        port.closePort()

        if not found:
            print("%9d bauds : rien" % baud)
            continue

        print("%9d bauds : %d servo(s)" % (baud, len(found)))
        port, ph = open_bus(port_name, baud)
        for sid, model in found:
            declared = read_reg(ph, port, sid, ADDR_MODEL_NUMBER)
            torque = read_reg(ph, port, sid, ADDR_TORQUE_ENABLE)
            volt = read_reg(ph, port, sid, ADDR_PRESENT_VOLTAGE)
            ok = "OK" if declared == MODEL_STS3215 else "?? attendu %d" % MODEL_STS3215
            print(
                "    ID %-3d  Model_Number=%-5s %s   couple=%s   %s"
                % (
                    sid,
                    declared,
                    ok,
                    "OFF" if torque == 0 else "*** ON ***" if torque else "?",
                    "%.1f V" % (volt / 10.0) if volt else "tension ?",
                )
            )
            if torque:
                print(
                    "    /!\\ ID %d a du COUPLE : il n'est pas libre a la main." % sid
                )
        port.closePort()
        found_any = found
        if found:
            break

    print()
    if not found_any:
        print("Aucun servo n'a repondu. Dans l'ordre, verifier :")
        print("  1. L'alimentation est-elle sur le bornier du bus TTL (pas RS485) ?")
        print("  2. Y a-t-il bien 7.4 V mesures AU CONNECTEUR SERVO ?")
        print("  3. La polarite du bornier est-elle respectee ?")
        print("  4. Le servo est-il branche sur le connecteur du bus TTL ?")
        print("  5. Relancer avec --all-bauds (baudrate d'usine different ?)")
        return 1
    if len(found_any) > 1 and all(sid == found_any[0][0] for sid, _ in found_any):
        print("/!\\ Plusieurs servos repondent au MEME ID : collision de bus.")
        print("    Les brancher UN PAR UN (US-017 renumerote la chaine).")
    return 0


def cmd_read(args):
    port_name = find_port(args.port)
    port, ph = open_bus(port_name, args.baud)

    ids = args.ids
    if not ids:
        ids = [
            sid
            for sid in range(0, args.max_id + 1)
            if ph.ping(port, sid)[1] == COMM_SUCCESS
        ]
        if not ids:
            port.closePort()
            sys.exit(
                "Aucun servo trouve. Lancer d'abord : bash ~/roby_leader_bench.sh scan"
            )

    print(
        "Port %s  |  IDs %s  |  lecture seule, aucun couple envoye" % (port_name, ids)
    )
    print("Bouger le bras a la main : la position doit suivre. Ctrl-C pour arreter.\n")
    header = "%-5s %10s %9s %8s %7s %8s" % (
        "ID",
        "pos(pas)",
        "pos(deg)",
        "charge",
        "temp",
        "tension",
    )
    period = 1.0 / max(args.hz, 0.1)

    try:
        while True:
            print(header)
            for sid in ids:
                pos = read_reg(ph, port, sid, ADDR_PRESENT_POSITION)
                load = decode_load(read_reg(ph, port, sid, ADDR_PRESENT_LOAD))
                temp = read_reg(ph, port, sid, ADDR_PRESENT_TEMPERATURE)
                volt = read_reg(ph, port, sid, ADDR_PRESENT_VOLTAGE)
                if pos is None:
                    print("%-5d %10s" % (sid, "pas de reponse"))
                    continue
                deg = steps_to_deg(pos)
                flag = ""
                if temp is not None and temp >= 55:
                    flag += "  /!\\ CHAUD"
                if volt is not None and (volt < 60 or volt > 84):
                    flag += "  /!\\ TENSION"
                print(
                    "%-5d %10d %9.1f %8s %7s %8s%s"
                    % (
                        sid,
                        pos,
                        deg,
                        load if load is not None else "?",
                        "%d C" % temp if temp is not None else "?",
                        "%.1f V" % (volt / 10.0) if volt is not None else "?",
                        flag,
                    )
                )
            if args.once:
                break
            print()
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nArret.")
    finally:
        port.closePort()
    return 0


def main():
    p = argparse.ArgumentParser(
        description="Banc LECTURE SEULE du bras guide Feetech STS3215 (US-016).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Ce script n'ecrit JAMAIS dans un servo : ni couple, ni ID, ni baudrate.",
    )
    p.add_argument(
        "--port", help="force le port (defaut : /dev/roby_leader puis by-id)"
    )
    p.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD, help="defaut %d" % DEFAULT_BAUD
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", help="ping les IDs et affiche Model_Number")
    s.add_argument("--max-id", type=int, default=20)
    s.add_argument("--full", action="store_true", help="balaye 0..253")
    s.add_argument(
        "--all-bauds", action="store_true", help="essaye tous les baudrates usuels"
    )
    s.set_defaults(func=cmd_scan)

    r = sub.add_parser(
        "read", help="lecture continue position/tension/temperature/charge"
    )
    r.add_argument(
        "--ids", type=int, nargs="+", help="IDs a lire (defaut : ceux trouves)"
    )
    r.add_argument("--max-id", type=int, default=20)
    r.add_argument("--hz", type=float, default=2.0)
    r.add_argument("--once", action="store_true")
    r.set_defaults(func=cmd_read)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
