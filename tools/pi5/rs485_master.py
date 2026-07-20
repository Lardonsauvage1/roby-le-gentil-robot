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


def format_angle(angle):
    if angle is None:
        return "TIMEOUT"
    elif angle == "no_measure":
        return "no_meas"
    else:
        return f"{angle:.2f}"


try:
    nb_cycles = 0
    nb_echecs = [0, 0, 0, 0]
    t_debut = time.time()
    derniers = [None, None, None, None]

    while True:
        for i in range(1, 4):
            angle = interroger(i)
            derniers[i] = angle
            if angle is None or angle == "no_measure":
                nb_echecs[i] += 1

        nb_cycles += 1

        if time.time() - t_debut >= 1.0:
            freq = nb_cycles / (time.time() - t_debut)
            print(f"Freq: {freq:.1f} Hz | N1: {format_angle(derniers[1])} ({nb_echecs[1]} ech) | N2: {format_angle(derniers[2])} ({nb_echecs[2]} ech) | N3: {format_angle(derniers[3])} ({nb_echecs[3]} ech)")
            nb_cycles = 0
            nb_echecs = [0, 0, 0, 0]
            t_debut = time.time()
except KeyboardInterrupt:
    de_re.close()
    ser.close()
