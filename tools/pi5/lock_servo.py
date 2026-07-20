#!/usr/bin/env python3
"""Pilote le servo de verrouillage sur le canal 2 du PCA9685 (0x40, i2c-1).
Reproduit EXACTEMENT la logique de ServoDriver (50Hz, pulse=500+angle/180*2000us).
Usage: python3 lock_servo.py <angle_deg>
Clamp de securite: [0, 45] deg (verrou attendu ~10 deg). Channel modifiable en tete.
"""
import fcntl, os, sys, time

I2C_SLAVE = 0x0703
ADDR = 0x40
BUS  = "/dev/i2c-1"
CH   = 2                  # canal du servo de verrouillage
SAFE_MIN, SAFE_MAX = -40.0, 95.0   # deverrouille=50deg, recherche verrou plus haut

def _refuse_si_stack_active():
    """Anti-contention PCA9685 : si la stack RT (ros2_control) tourne, elle
    possede deja le bus I2C. Ecrire ici en plus = 2 maitres = servos qui
    deconnent. On refuse donc de demarrer tant que la stack est active."""
    import subprocess, sys
    if subprocess.run(["pgrep", "-f", "ros2_control_node"],
                      stdout=subprocess.DEVNULL).returncode == 0:
        sys.stderr.write(
            "REFUS: la stack RT (ros2_control) possede deja le PCA9685.\n"
            "  -> pilote le servo via son topic (la stack le gere deja), ou\n"
            "  -> coupe la stack RT d'abord (evite la contention I2C).\n")
        raise SystemExit(1)


def main():
    if len(sys.argv) != 2:
        print("usage: lock_servo.py <angle_deg>  (clamp %.0f..%.0f)" % (SAFE_MIN, SAFE_MAX)); return 1
    ang = float(sys.argv[1])
    clamped = max(SAFE_MIN, min(SAFE_MAX, ang))
    if clamped != ang:
        print("ANGLE %.1f hors garde-fou -> clamp a %.1f" % (ang, clamped))
    ang = clamped
    _refuse_si_stack_active()
    fd = os.open(BUS, os.O_RDWR); fcntl.ioctl(fd, I2C_SLAVE, ADDR)
    def reg(r, v): os.write(fd, bytes([r, v]))
    # init PCA9685 50Hz (idempotent ; safe meme si deja init)
    reg(0x00, 0x10); reg(0xFE, 121); reg(0x00, 0x20); time.sleep(0.001); reg(0x00, 0xA0)
    pulse_us = 500.0 + (ang/180.0)*2000.0
    off = int((pulse_us/20000.0)*4096.0)
    base = 0x06 + 4*CH
    os.write(fd, bytes([base, 0x00, 0x00, off & 0xFF, (off >> 8) & 0x0F]))
    os.close(fd)
    print("canal %d -> %.1f deg  (pulse %.0f us, off_tick %d)" % (CH, ang, pulse_us, off))
    return 0

sys.exit(main())
