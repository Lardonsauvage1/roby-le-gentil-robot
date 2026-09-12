#!/usr/bin/env python3
"""Prehenseur VENTOUSE : sequences de SAISIE et de LACHER.

Cablage (verifie sur le banc le 2026-07-28) :
    CH7 = ELECTROVANNE de relachement   (impulsion pour casser le vide)
    CH8 = POMPE a vide                  (maintenue pendant le transport)

Convention du module : 100 deg = ACTIF, 0 deg = COUPE.
A 50 Hz : pulse = 500 + (deg/180)*2000 us.

Le PCA9685 CONSERVE sa consigne apres la fin du process : --saisir laisse donc la pompe
en marche, et le bras peut se deplacer entre les deux commandes.

ORDRE AU LACHER, non negociable : on coupe la POMPE **avant** d ouvrir la vanne. Sinon la
pompe aspire l air par la vanne ouverte, le vide ne casse pas franchement et elle tourne
a vide pour rien.

La vanne est commandee par IMPULSION, jamais en continu : une fois le vide casse, la
maintenir alimentee ne fait que chauffer la bobine.

A lancer stack RT COUPEE (sinon 2 maitres sur le bus I2C).

Usage :
    roby_ventouse.py --saisir        # vanne fermee, pompe ON, attend le vide -> TIENT
    roby_ventouse.py --lacher        # pompe OFF, impulsion vanne -> LACHE, tout coupe
    roby_ventouse.py --stop          # tout couper (securite)
    roby_ventouse.py --etat          # lire les 2 canaux sans rien changer
    roby_ventouse.py --lacher-sans-vanne   # mesure : lacher SANS la vanne (comparaison)
"""
import argparse
import fcntl
import os
import subprocess
import sys
import time

ADDR, BUS = 0x40, "/dev/i2c-1"
CH_VANNE, CH_POMPE = 7, 8


def refuse_si_stack_active():
    if subprocess.run(["pgrep", "-f", "ros2_control_node"],
                      stdout=subprocess.DEVNULL).returncode == 0:
        sys.stderr.write("REFUS: la stack RT possede deja le PCA9685 (2 maitres I2C).\n")
        raise SystemExit(1)


class Pca:
    def __init__(self, reset=False):
        self.fd = os.open(BUS, os.O_RDWR)
        fcntl.ioctl(self.fd, 0x0703, ADDR)
        if reset:
            for r, v in ((0x00, 0x10), (0xFE, 121), (0x00, 0x20)):
                os.write(self.fd, bytes([r, v]))
            time.sleep(0.001)
            os.write(self.fd, bytes([0x00, 0xA0]))
            time.sleep(0.1)

    def _rd(self, r):
        os.write(self.fd, bytes([r]))
        return os.read(self.fd, 1)[0]

    def lire(self, ch):
        b = 0x06 + 4 * ch
        off = self._rd(b + 2) | (self._rd(b + 3) << 8)
        return off

    def set_deg(self, ch, deg, nom=""):
        pulse = 500.0 + (deg / 180.0) * 2000.0
        off = int((pulse / 20000.0) * 4096.0)
        b = 0x06 + 4 * ch
        os.write(self.fd, bytes([b, 0, 0, off & 0xFF, (off >> 8) & 0x0F]))
        back = self.lire(ch)
        ok = "OK" if back == off else "!! MISMATCH"
        print(f"    CH{ch} {nom:12s} -> {deg:5.1f} deg ({pulse:4.0f} us)  {ok}", flush=True)
        return back == off

    def close(self):
        os.close(self.fd)


def main():
    p = argparse.ArgumentParser()
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--saisir", action="store_true")
    g.add_argument("--lacher", action="store_true")
    g.add_argument("--lacher-sans-vanne", action="store_true",
                   help="mesure : coupe la pompe SANS ouvrir la vanne (pour chiffrer l apport de la vanne)")
    g.add_argument("--stop", action="store_true")
    g.add_argument("--etat", action="store_true")
    p.add_argument("--t-vide", type=float, default=1.0, help="s d attente pour etablir le vide")
    p.add_argument("--t-souffle", type=float, default=0.3, help="s d ouverture de la vanne")
    p.add_argument("--ch-vanne", type=int, default=CH_VANNE)
    p.add_argument("--ch-pompe", type=int, default=CH_POMPE)
    a = p.parse_args()

    refuse_si_stack_active()
    pca = Pca(reset=False)

    if a.etat:
        for ch, nom in ((a.ch_vanne, "VANNE"), (a.ch_pompe, "POMPE")):
            off = pca.lire(ch)
            us = off / 4096.0 * 20000.0
            etat = "jamais pilote" if off == 4096 else ("ACTIF" if off > 200 else "coupe")
            print(f"  CH{ch} {nom:6s} : off={off:5d} (~{us:5.0f} us) -> {etat}")
        pca.close()
        return

    if a.stop:
        print(">>> ARRET COMPLET")
        pca.set_deg(a.ch_pompe, 0.0, "POMPE")
        pca.set_deg(a.ch_vanne, 0.0, "VANNE")
        pca.close()
        return

    if a.saisir:
        print(">>> SAISIE")
        pca.set_deg(a.ch_vanne, 0.0, "VANNE")        # etancheite d abord
        pca.set_deg(a.ch_pompe, 100.0, "POMPE")      # le vide monte
        print(f"    etablissement du vide ({a.t_vide:.1f} s) ...", flush=True)
        time.sleep(a.t_vide)
        print("    OBJET TENU — la pompe RESTE en marche (le bras peut se deplacer)")
        pca.close()
        return

    if a.lacher or a.lacher_sans_vanne:
        print(">>> LACHER" + (" (SANS vanne — mesure)" if a.lacher_sans_vanne else ""))
        pca.set_deg(a.ch_pompe, 0.0, "POMPE")        # TOUJOURS avant la vanne
        if a.lacher_sans_vanne:
            print("    pompe coupee, vanne laissee FERMEE : chronometre la chute de l objet")
        else:
            time.sleep(0.1)
            pca.set_deg(a.ch_vanne, 100.0, "VANNE")  # casse le vide
            time.sleep(a.t_souffle)
            pca.set_deg(a.ch_vanne, 0.0, "VANNE")    # impulsion : on ne maintient pas
            print("    OBJET LACHE — tout est coupe")
        pca.close()
        return


if __name__ == "__main__":
    main()
