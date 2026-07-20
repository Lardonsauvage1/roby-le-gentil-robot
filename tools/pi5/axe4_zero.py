#!/usr/bin/env python3
"""Met l'axe 4 (servo PCA9685 canal 0) a joint_4 = 0 rad = 135 deg (offset/centre).
Meme conversion que ServoDriver : pulse_us = 500 + (ang/180)*2000 ; off = pulse/20000*4096.
Refuse si la stack RT (ros2_control) tourne (contention I2C). Init PCA conditionnel (preserve les
autres canaux s'ils sont deja a 50Hz)."""
import fcntl, os, sys, subprocess, time
I2C_SLAVE = 0x0703
ADDR, BUS, CH = 0x40, "/dev/i2c-1", 0     # axe 4 = canal 0
ANGLE = 135.0                              # joint_4 = 0 rad = angle_init/offset
if subprocess.run(["pgrep", "-f", "ros2_control_node"], stdout=subprocess.DEVNULL).returncode == 0:
    sys.stderr.write("REFUS: stack RT active (elle possede le PCA). Coupe-la d'abord.\n"); raise SystemExit(1)
fd = os.open(BUS, os.O_RDWR); fcntl.ioctl(fd, I2C_SLAVE, ADDR)
def wr(r, v): os.write(fd, bytes([r, v]))
os.write(fd, bytes([0xFE])); pres = os.read(fd, 1)[0]
if pres != 121:
    wr(0x00, 0x10); wr(0xFE, 121); wr(0x00, 0x20); time.sleep(0.001); wr(0x00, 0xA0)
    print(f"PCA init 50Hz (prescale etait {pres})")
else:
    print("PCA deja a 50Hz (init saute)")
pulse = 500.0 + (ANGLE / 180.0) * 2000.0
off = int((pulse / 20000.0) * 4096.0)
base = 0x06 + 4 * CH
os.write(fd, bytes([base, 0, 0, off & 0xFF, (off >> 8) & 0x0F])); os.close(fd)
print(f"CH{CH} (axe 4) -> {ANGLE:.1f} deg  (= joint_4 0 rad)  pulse {pulse:.0f} us  off_tick {off}")
