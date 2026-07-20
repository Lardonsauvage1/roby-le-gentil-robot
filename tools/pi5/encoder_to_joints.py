"""POC: lire les 3 encodeurs RS-485 et sortir les angles joint (rad/deg).

Sur Pi5. Fork de rs485_master.py + conversion encodeur -> joint (avec multi-tour
et couplage axes 2/3).

Au boot :
  - Si encoder_calibration.yaml existe : charge encoder_raw_init_deg (zero offset)
  - Sinon : utilise la premiere lecture comme zero (suppose que le bras est deja
    dans la pose initiale ; ne pas faire confiance aux angles tant que la vraie
    calibration n'a pas ete faite)

Pose initiale URDF = "bras plie 90 + axe 1 aligne" = tous joints a 0.
"""

import math
import os
import statistics
import sys
import time
from collections import deque

import serial
import yaml
from gpiozero import DigitalOutputDevice

MEDIAN_FILTER_SIZE = 5

# ---------------------------------------------------------------------------
# RS-485 config (copie de rs485_master.py)
# ---------------------------------------------------------------------------
DE_RE_PIN = 26
PORT = "/dev/ttyAMA0"
BAUD = 115200
QUERY_TIMEOUT_S = 0.03

# ---------------------------------------------------------------------------
# Parametres mecaniques (depuis roby_hardware.ros2_control.xacro + roby_system.cpp)
# joint = motor_axis * gear_num / gear_den * (inverted ? -1 : 1)
# ---------------------------------------------------------------------------
JOINT_PARAMS = {
    1: {"gear_num": 16, "gear_den": 85, "inverted": False},
    2: {"gear_num": 15, "gear_den": 44, "inverted": True},
    3: {"gear_num": 300, "gear_den": 1408, "inverted": True},
}
# Couplage axes 2/3 (depuis roby_system.cpp:78-82) :
#   joint_3 = motor_3_axis + joint_2 * (coupling_m2 / coupling_m3)
COUPLING_M2 = 6000.0 / 45056.0
COUPLING_M3 = (15.0 * 20.0) / (44.0 * 32.0)
COUPLING_J2_TO_J3 = COUPLING_M2 / COUPLING_M3  # ~0.6248

CALIB_PATH = os.path.expanduser("~/encoder_calibration.yaml")


# ---------------------------------------------------------------------------
# RS-485 master (logique de rs485_master.py)
# ---------------------------------------------------------------------------
class RS485Master:
    def __init__(self, port=PORT, baud=BAUD, de_re_pin=DE_RE_PIN):
        self.de_re = DigitalOutputDevice(de_re_pin)
        self.de_re.off()
        self.ser = serial.Serial(port, baud, timeout=QUERY_TIMEOUT_S)
        time.sleep(0.1)

    def query(self, slave_id):
        """Retourne angle deg [0, 360), 'no_measure' (capteur ko), ou None (timeout)."""
        self.ser.reset_input_buffer()
        self.de_re.on()
        time.sleep(0.001)
        self.ser.write(bytes([slave_id]))
        self.ser.flush()
        time.sleep(0.001)
        self.de_re.off()

        t0 = time.time()
        while True:
            if time.time() - t0 > QUERY_TIMEOUT_S:
                return None
            b = self.ser.read(1)
            if b and b[0] == 0xFF:
                break

        confirmed = self.ser.read(1)
        if not confirmed or confirmed[0] != slave_id:
            return None
        data = self.ser.read(2)
        if len(data) < 2:
            return None
        val = (data[0] << 8) | data[1]
        if val == 0xFFFE:
            return "no_measure"
        return (val / 65535.0) * 360.0

    def close(self):
        try:
            self.de_re.close()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Conversion raw -> joint avec multi-tour
# ---------------------------------------------------------------------------
def wrap_to_180(deg):
    """Ramene deg dans ]-180, 180]."""
    return ((deg + 180.0) % 360.0) - 180.0


class MotorTracker:
    """Tient l'angle moteur "unwrapped" en degres signes a partir du raw absolu mono-tour.

    Au premier read : initialise unwrapped a wrap_to_180(raw - raw_init).
    Sur reads suivants : detecte les wrap (saut > 180 entre 2 reads) et accumule.

    Applique un filtre median glissant (N = MEDIAN_FILTER_SIZE) sur l'unwrapped
    pour absorber les outliers d'acquisition (pulseIn corrompu cote Arduino).
    """

    def __init__(self, raw_init_deg, filter_size=MEDIAN_FILTER_SIZE):
        self.raw_init = raw_init_deg
        self.last_raw = None
        self.unwrapped = None  # valeur instantanee (peut contenir des outliers)
        self.buffer = deque(maxlen=filter_size)  # buffer pour filtre median

    def update(self, raw_deg):
        if raw_deg is None or raw_deg == "no_measure":
            return self.filtered()
        if self.last_raw is None:
            self.unwrapped = wrap_to_180(raw_deg - self.raw_init)
        else:
            diff = wrap_to_180(raw_deg - self.last_raw)
            self.unwrapped += diff
        self.last_raw = raw_deg
        self.buffer.append(self.unwrapped)
        return self.filtered()

    def filtered(self):
        """Retourne la mediane du buffer (robuste aux outliers ponctuels)."""
        if not self.buffer:
            return None
        return statistics.median(self.buffer)


def motor_to_joint_rad(motor_unwrapped_deg, params):
    """Convertit l'angle moteur (axis-side equivalent) en angle joint en rad."""
    if motor_unwrapped_deg is None:
        return None
    motor_rad = math.radians(motor_unwrapped_deg)
    joint = motor_rad * params["gear_num"] / params["gear_den"]
    if params["inverted"]:
        joint = -joint
    return joint


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def load_calibration():
    if not os.path.exists(CALIB_PATH):
        return None
    with open(CALIB_PATH) as f:
        data = yaml.safe_load(f) or {}
    raw_init = data.get("encoder_raw_init_deg", {})
    if not all(k in raw_init for k in ("motor_1", "motor_2", "motor_3")):
        return None
    return {1: raw_init["motor_1"], 2: raw_init["motor_2"], 3: raw_init["motor_3"]}


def main():
    master = RS485Master()
    calib = load_calibration()

    if calib is None:
        print(
            f"[WARN] Pas de calibration trouvee a {CALIB_PATH}.\n"
            "       La premiere lecture stable sera utilisee comme zero.\n"
            "       Les angles joint NE SONT FIABLES QUE si le bras est\n"
            "       deja dans la pose 'bras plie 90 + axe 1 aligne'.\n"
        )
        # Acquerir une premiere lecture stable pour servir de zero (3 essais)
        calib = {}
        for sid in (1, 2, 3):
            for _ in range(20):
                v = master.query(sid)
                if isinstance(v, float):
                    calib[sid] = v
                    break
            else:
                print(f"[ERR] Impossible de lire l'encodeur {sid} pour init.")
                master.close()
                sys.exit(1)
        print(f"[INIT] Zero pris sur la position actuelle : {calib}")
    else:
        print(f"[OK] Calibration chargee depuis {CALIB_PATH} : {calib}")

    trackers = {sid: MotorTracker(calib[sid]) for sid in (1, 2, 3)}

    t_print = time.time()
    nb_cycles = 0
    try:
        while True:
            raws = {}
            for sid in (1, 2, 3):
                raws[sid] = master.query(sid)
                trackers[sid].update(raws[sid])
            nb_cycles += 1

            if time.time() - t_print >= 1.0:
                # Utilise l'unwrapped FILTRE (mediane glissante) pour la conversion joint
                motor_filtered = {sid: trackers[sid].filtered() for sid in (1, 2, 3)}
                joints_rad = {}
                for sid in (1, 2, 3):
                    joints_rad[sid] = motor_to_joint_rad(
                        motor_filtered[sid], JOINT_PARAMS[sid]
                    )
                if joints_rad[2] is not None and joints_rad[3] is not None:
                    joints_rad[3] = joints_rad[3] + joints_rad[2] * COUPLING_J2_TO_J3

                def fmt_joint(j):
                    if j is None:
                        return "  ----  "
                    return f"{math.degrees(j):+7.2f}d ({j:+6.3f}r)"

                def fmt_raw(r):
                    if r is None:
                        return "TIMEOUT"
                    if r == "no_measure":
                        return "no_meas"
                    return f"{r:6.2f}"

                def fmt_motor(v):
                    if v is None:
                        return "  ----  "
                    return f"{v:+7.2f}"

                freq = nb_cycles / (time.time() - t_print)
                print(
                    f"f={freq:4.1f}Hz | "
                    f"raw1={fmt_raw(raws[1])} m1f={fmt_motor(motor_filtered[1])} j1={fmt_joint(joints_rad[1])} | "
                    f"raw2={fmt_raw(raws[2])} m2f={fmt_motor(motor_filtered[2])} j2={fmt_joint(joints_rad[2])} | "
                    f"raw3={fmt_raw(raws[3])} m3f={fmt_motor(motor_filtered[3])} j3={fmt_joint(joints_rad[3])}"
                )
                t_print = time.time()
                nb_cycles = 0
    except KeyboardInterrupt:
        pass
    finally:
        master.close()


if __name__ == "__main__":
    main()
