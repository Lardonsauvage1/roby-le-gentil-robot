#!/usr/bin/env python3
"""Testeur VENTOUSE : pilote un canal du PCA9685 (0x40, i2c-1).
Sert a trouver a quelle IMPULSION la pompe/vanne du kit commute, et a identifier
lequel des 2 cables fait quoi.

Usage:
  python3 ventouse_test.py <ch> on|off        # 100deg (1611us) / 0deg (500us)
  python3 ventouse_test.py <ch> us <microsec> # impulsion BRUTE (ex: 1500)
  python3 ventouse_test.py <ch> sweep         # balaye 600..2500us, 1.5s/pas
  python3 ventouse_test.py alloff             # CH3 et CH4 -> off (500us)

⚠️ pompe alimentee par le rail V+ partage : teste COURT, surveille verrou/pince.
Garde-fou : refuse si la stack RT (ros2_control) tourne. STACK COUPEE requise.
"""
import fcntl, os, sys, time

I2C_SLAVE = 0x0703
ADDR, BUS = 0x40, "/dev/i2c-1"
VENTOUSE_CHANNELS = (3, 4)
SWEEP_US = [600, 900, 1100, 1300, 1500, 1700, 1900, 2100, 2300, 2500]


def _refuse_si_stack_active():
    import subprocess
    if subprocess.run(["pgrep", "-f", "ros2_control_node"],
                      stdout=subprocess.DEVNULL).returncode == 0:
        sys.stderr.write("REFUS: la stack RT (ros2_control) possede deja le PCA9685.\n"
                         "  -> coupe la stack RT d'abord (evite la contention I2C).\n")
        raise SystemExit(1)


def us_to_off(us):
    return int((us / 20000.0) * 4096.0)


def open_bus():
    fd = os.open(BUS, os.O_RDWR); fcntl.ioctl(fd, I2C_SLAVE, ADDR)
    os.write(fd, bytes([0xFE])); pres = os.read(fd, 1)[0]
    if pres != 121:
        for r, v in ((0x00, 0x10), (0xFE, 121), (0x00, 0x20)):
            os.write(fd, bytes([r, v]))
        time.sleep(0.001); os.write(fd, bytes([0x00, 0xA0]))
        print("PCA init 50Hz (etait %d)" % pres)
    return fd


def set_us(fd, ch, us):
    off = us_to_off(us); base = 0x06 + 4 * ch
    os.write(fd, bytes([base, 0, 0, off & 0xFF, (off >> 8) & 0x0F]))
    print("  CH%d -> pulse %4.0f us (off_tick %d)" % (ch, us, off))


def set_duty(fd, ch, pct):
    """Rapport cyclique 0..100 % (pas un signal servo). 100 = full ON, 0 = full OFF."""
    pct = max(0.0, min(100.0, pct)); base = 0x06 + 4 * ch
    if pct >= 100.0:
        os.write(fd, bytes([base, 0x00, 0x10, 0x00, 0x00]))   # bit full-ON
        print("  CH%d -> duty 100%% (full ON)" % ch)
    elif pct <= 0.0:
        os.write(fd, bytes([base, 0x00, 0x00, 0x00, 0x10]))   # bit full-OFF
        print("  CH%d -> duty 0%% (full OFF)" % ch)
    else:
        off = int(pct / 100.0 * 4095)
        os.write(fd, bytes([base, 0x00, 0x00, off & 0xFF, (off >> 8) & 0x0F]))
        print("  CH%d -> duty %.0f%% (off_tick %d)" % (ch, pct, off))


def main():
    args = sys.argv[1:]
    _refuse_si_stack_active()

    if args == ["alloff"]:
        fd = open_bus()
        for ch in VENTOUSE_CHANNELS:
            set_us(fd, ch, 500)
        os.close(fd); return 0

    if len(args) < 2:
        print(__doc__); return 1
    ch = int(args[0]); mode = args[1]
    fd = open_bus()

    if mode == "on":
        set_us(fd, ch, 1611)
    elif mode == "off":
        set_us(fd, ch, 500)
    elif mode == "us" and len(args) == 3:
        set_us(fd, ch, float(args[2]))
    elif mode == "duty" and len(args) == 3:
        set_duty(fd, ch, float(args[2]))
    elif mode == "sweep":
        print("SWEEP CH%d (repere a quelle valeur la pompe demarre) :" % ch)
        for us in SWEEP_US:
            set_us(fd, ch, us)
            time.sleep(1.5)
        set_us(fd, ch, 500)   # remet off a la fin
        print("  -> remis a 500us (off)")
    else:
        os.close(fd); print(__doc__); return 1

    os.close(fd)
    return 0


sys.exit(main())
