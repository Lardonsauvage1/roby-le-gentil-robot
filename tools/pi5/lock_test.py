#!/usr/bin/env python3
"""Test MINIMAL verrou changeur d outil (CH2 du PCA9685), 100% standalone.
A lancer SEUL, stack coupee => aucune collision I2C. Reset PCA propre puis
verrouille/deverrouille lentement pour observation."""
import fcntl, os, time
ADDR, BUS, CH = 0x40, "/dev/i2c-1", 2
LOCK, UNLOCK = 50.0, 75.0   # 2026-07-07 : 50=verrouille
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


_refuse_si_stack_active()
fd = os.open(BUS, os.O_RDWR); fcntl.ioctl(fd, 0x0703, ADDR)
def reg(r, v): os.write(fd, bytes([r, v]))
def rd(r): os.write(fd, bytes([r])); return os.read(fd, 1)[0]
def set_deg(a):
    pulse = 500.0 + (a/180.0)*2000.0; off = int((pulse/20000.0)*4096.0)
    b = 0x06 + 4*CH
    os.write(fd, bytes([b, 0, 0, off & 0xFF, (off >> 8) & 0x0F]))
    back = rd(b+2) | (rd(b+3) << 8)
    print("  CH%d -> %5.1f deg | pulse %4.0f us | off ecrit %d, relu %d %s"
          % (CH, a, pulse, off, back, "OK" if back == off else "!! MISMATCH"))
print(">>> reset PCA9685 propre")
reg(0x00,0x10); reg(0xFE,121); reg(0x00,0x20); time.sleep(0.001); reg(0x00,0xA0); time.sleep(0.1)
print("    MODE1=0x%02X PRESCALE=%d" % (rd(0x00), rd(0xFE)))
for i in range(3):
    print("--- cycle %d ---" % (i+1))
    print(" VERROUILLE"); set_deg(LOCK);   time.sleep(2.0)
    print(" DEVERROUILLE"); set_deg(UNLOCK); time.sleep(2.0)
print(">>> fin: remise en VERROUILLE"); set_deg(LOCK)
os.close(fd); print("termine.")
