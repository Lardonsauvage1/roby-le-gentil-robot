#!/usr/bin/env python3
"""Pilote le servo de PINCE sur le canal 3 du PCA9685 (0x40, i2c-1).
NE REINITIALISE PAS le PCA s il est deja a 50Hz (sinon on couperait CH2=verrou
tete => perte alim pince). Meme conversion que ServoDriver.
Usage: python3 gripper_servo.py <angle_deg>   (clamp de securite ci-dessous)
"""
import fcntl, os, sys, time

I2C_SLAVE = 0x0703
ADDR, BUS, CH = 0x40, "/dev/i2c-1", 3
SAFE_MIN, SAFE_MAX = 0.0, 160.0      # pince: fermee=55deg, recherche ouvert

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
        print("usage: gripper_servo.py <angle_deg>  (clamp %.0f..%.0f)" % (SAFE_MIN, SAFE_MAX)); return 1
    ang = float(sys.argv[1]); c = max(SAFE_MIN, min(SAFE_MAX, ang))
    if c != ang: print("ANGLE %.1f hors garde-fou -> clamp %.1f" % (ang, c))
    ang = c
    _refuse_si_stack_active()
    fd = os.open(BUS, os.O_RDWR); fcntl.ioctl(fd, I2C_SLAVE, ADDR)
    def wr(r, v): os.write(fd, bytes([r, v]))
    # init CONDITIONNEL : seulement si pas deja 50Hz (preserve CH2 verrou)
    os.write(fd, bytes([0xFE])); pres = os.read(fd, 1)[0]
    if pres != 121:
        wr(0x00, 0x10); wr(0xFE, 121); wr(0x00, 0x20); time.sleep(0.001); wr(0x00, 0xA0)
        print("PCA init 50Hz (etait %d)" % pres)
    else:
        print("PCA deja 50Hz, init saute (CH2 verrou preserve)")
    pulse = 500.0 + (ang/180.0)*2000.0; off = int((pulse/20000.0)*4096.0)
    base = 0x06 + 4*CH
    os.write(fd, bytes([base, 0, 0, off & 0xFF, (off >> 8) & 0x0F])); os.close(fd)
    print("CH%d (pince) -> %.1f deg  (pulse %.0f us, off_tick %d)" % (CH, ang, pulse, off))
    return 0

sys.exit(main())
