"""Meme logique que rs485_master.py mais affiche les 100 dernieres lectures
brut-par-brut (au lieu d'agreger toutes les secondes)."""

import serial
import time
from gpiozero import DigitalOutputDevice

DE_RE_PIN = 26
PORT = '/dev/ttyAMA0'
BAUD = 115200

de_re = DigitalOutputDevice(DE_RE_PIN)
de_re.off()

ser = serial.Serial(PORT, BAUD, timeout=0.03)
time.sleep(0.1)


def interroger(id):
    ser.reset_input_buffer()

    de_re.on()
    time.sleep(0.001)
    ser.write(bytes([id]))
    ser.flush()
    time.sleep(0.001)
    de_re.off()

    t = time.time()
    while True:
        if time.time() - t > 0.03:
            return None
        b = ser.read(1)
        if b and b[0] == 0xFF:
            break

    confirmed = ser.read(1)
    if not confirmed or confirmed[0] != id:
        return None

    data = ser.read(2)
    if len(data) < 2:
        return None

    val = (data[0] << 8) | data[1]
    if val == 0xFFFE:
        return "no_measure"
    return (val / 65535.0) * 360.0


def fmt(angle):
    if angle is None:
        return " TIMEOUT"
    elif angle == "no_measure":
        return " no_meas"
    else:
        return f"{angle:8.2f}"


try:
    print(f"{'#':>4} | {'N1':>8} | {'N2':>8} | {'N3':>8}")
    print("-" * 40)
    for i in range(100):
        a1 = interroger(1)
        a2 = interroger(2)
        a3 = interroger(3)
        print(f"{i+1:>4} | {fmt(a1)} | {fmt(a2)} | {fmt(a3)}")
except KeyboardInterrupt:
    pass
finally:
    de_re.close()
    ser.close()
