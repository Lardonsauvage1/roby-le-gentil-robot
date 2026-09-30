#!/usr/bin/env python3
"""Banc prehenseur VENTOUSE : pilotage d une entree PWM via le PCA9685.

Meme methode que lock_test.py (dont la garde anti-contention est reprise telle quelle) :
100% standalone, a lancer stack COUPEE, sinon 2 maitres sur le bus I2C.

Convention doc du module : 100 deg = ACTIVE, 0 deg = COUPE.
A 50 Hz : pulse = 500 + (deg/180)*2000 us  ->  0 deg = 500 us, 100 deg = 1611 us.

Usage :
  ventouse_test.py --ch 7 --deg 100 --duree 5      # active 5 s puis coupe
  ventouse_test.py --ch 7 --deg 0                  # coupe et sort
  ventouse_test.py --ch 7 --lecture                # lit l etat sans rien changer
"""
import argparse
import fcntl
import os
import subprocess
import sys
import time

ADDR, BUS = 0x40, "/dev/i2c-1"


def refuse_si_stack_active():
    """Anti-contention PCA9685 : si la stack RT tourne, elle possede deja le bus."""
    if subprocess.run(["pgrep", "-f", "ros2_control_node"],
                      stdout=subprocess.DEVNULL).returncode == 0:
        sys.stderr.write(
            "REFUS: la stack RT (ros2_control) possede deja le PCA9685.\n"
            "  -> couper la stack RT d abord (sinon 2 maitres I2C = servos qui deconnent).\n")
        raise SystemExit(1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ch", type=int, default=7, help="canal PCA9685 (0-15)")
    p.add_argument("--deg", type=float, default=None, help="100 = active, 0 = coupe")
    p.add_argument("--duree", type=float, default=0.0,
                   help="secondes d activation avant coupure auto (0 = laisse en l etat)")
    p.add_argument("--lecture", action="store_true", help="lit l etat, n ecrit rien")
    p.add_argument("--alterne", type=int, default=0,
                   help="nb de cycles haut/bas (0 = pas d alternance)")
    p.add_argument("--t-haut", type=float, default=1.0, help="secondes a l etat actif")
    p.add_argument("--t-bas", type=float, default=1.0, help="secondes a l etat coupe")
    p.add_argument("--no-reset", action="store_true",
                   help="ne pas reinitialiser le PCA (utile si d autres servos sont alimentes)")
    a = p.parse_args()

    if a.ch in (0, 1, 2, 3):
        sys.stderr.write(f"REFUS: CH{a.ch} est deja utilise "
                         "(0=axe4, 1=axe5, 2=verrou tete, 3=pince).\n")
        raise SystemExit(1)

    refuse_si_stack_active()
    fd = os.open(BUS, os.O_RDWR)
    fcntl.ioctl(fd, 0x0703, ADDR)

    def reg(r, v):
        os.write(fd, bytes([r, v]))

    def rd(r):
        os.write(fd, bytes([r]))
        return os.read(fd, 1)[0]

    def lire_ch(ch):
        b = 0x06 + 4 * ch
        off = rd(b + 2) | (rd(b + 3) << 8)
        return off, off / 4096.0 * 20000.0

    def set_deg(ch, deg):
        pulse = 500.0 + (deg / 180.0) * 2000.0
        off = int((pulse / 20000.0) * 4096.0)
        b = 0x06 + 4 * ch
        os.write(fd, bytes([b, 0, 0, off & 0xFF, (off >> 8) & 0x0F]))
        back = rd(b + 2) | (rd(b + 3) << 8)
        etat = "OK" if back == off else "!! MISMATCH"
        print(f"  CH{ch} -> {deg:5.1f} deg | pulse {pulse:4.0f} us | "
              f"off ecrit {off}, relu {back} {etat}", flush=True)
        return back == off

    if not a.no_reset:
        print(">>> reset PCA9685 (prescale 121 = 50 Hz)")
        reg(0x00, 0x10)
        reg(0xFE, 121)
        reg(0x00, 0x20)
        time.sleep(0.001)
        reg(0x00, 0xA0)
        time.sleep(0.1)
    print(f"    MODE1=0x{rd(0x00):02X} PRESCALE={rd(0xFE)} (121 attendu)")

    off, us = lire_ch(a.ch)
    print(f">>> etat initial CH{a.ch} : off={off} soit ~{us:.0f} us")
    if a.lecture or a.deg is None:
        os.close(fd)
        return

    # on part TOUJOURS d un etat coupe et connu
    print(">>> mise a l etat COUPE avant tout")
    set_deg(a.ch, 0.0)
    time.sleep(0.3)

    if a.alterne > 0:
        haut = a.deg if a.deg is not None else 100.0
        print(f">>> ALTERNANCE {a.alterne} cycles : {haut:.0f} deg pendant {a.t_haut:.1f} s, "
              f"puis 0 deg pendant {a.t_bas:.1f} s")
        try:
            for i in range(a.alterne):
                print(f"--- cycle {i+1}/{a.alterne} : HAUT")
                set_deg(a.ch, haut)
                time.sleep(a.t_haut)
                print(f"--- cycle {i+1}/{a.alterne} : BAS")
                set_deg(a.ch, 0.0)
                time.sleep(a.t_bas)
        except KeyboardInterrupt:
            print("\n>>> interrompu -> coupure de securite")
        finally:
            set_deg(a.ch, 0.0)  # on ne laisse JAMAIS la sortie active en sortant
        os.close(fd)
        print("termine (sortie coupee).")
        return

    print(f">>> ACTIVATION a {a.deg:.0f} deg")
    set_deg(a.ch, a.deg)

    if a.duree > 0:
        print(f"    maintien {a.duree:.1f} s ...", flush=True)
        time.sleep(a.duree)
        print(">>> COUPURE")
        set_deg(a.ch, 0.0)

    os.close(fd)
    print("termine.")


if __name__ == "__main__":
    main()
