#!/usr/bin/env python3
"""Node ROS2 qui publie les angles joints mesures par les encodeurs sur
`/joint_states_measured` (sensor_msgs/JointState) a 50 Hz.

Pas de closed-loop ici : monitoring seulement. Permet de comparer en live
avec `/joint_states` (open-loop, publie par ros2_control) pour quantifier
le decalage de BUG-005.

Encodeurs concernes : motor_1, motor_2, motor_3 (axes steppers).
Joints 4 et 5 (servos) ne sont pas publies — non equipes d'encodeurs.
"""

import math
import os
import statistics
import time
from collections import deque

import rclpy
import serial
import yaml
from gpiozero import DigitalOutputDevice
from rclpy.node import Node
from sensor_msgs.msg import JointState


# RS-485
DE_RE_PIN = 26
PORT = "/dev/ttyAMA0"
BAUD = 115200
QUERY_TIMEOUT_S = 0.03

# Filtre median glissant
MEDIAN_FILTER_SIZE = 5

# Calibration
CALIB_PATH = os.path.expanduser("~/encoder_calibration.yaml")

# Parametres mecaniques (depuis roby_hardware.ros2_control.xacro)
JOINT_PARAMS = {
    1: {"gear_num": 16, "gear_den": 85, "inverted": False, "name": "joint_1"},
    2: {"gear_num": 15, "gear_den": 44, "inverted": True, "name": "joint_2"},
    3: {"gear_num": 300, "gear_den": 1408, "inverted": True, "name": "joint_3"},
}
COUPLING_M2 = 6000.0 / 45056.0
COUPLING_M3 = (15.0 * 20.0) / (44.0 * 32.0)
COUPLING_J2_TO_J3 = COUPLING_M2 / COUPLING_M3


def wrap_to_180(deg):
    return ((deg + 180.0) % 360.0) - 180.0


class RS485Master:
    def __init__(self):
        self.de_re = DigitalOutputDevice(DE_RE_PIN)
        self.de_re.off()
        self.ser = serial.Serial(PORT, BAUD, timeout=QUERY_TIMEOUT_S)
        time.sleep(0.1)

    def query(self, sid):
        self.ser.reset_input_buffer()
        self.de_re.on(); time.sleep(0.001)
        self.ser.write(bytes([sid])); self.ser.flush(); time.sleep(0.001)
        self.de_re.off()
        t = time.time()
        while time.time() - t < QUERY_TIMEOUT_S:
            b = self.ser.read(1)
            if b and b[0] == 0xFF:
                c = self.ser.read(1)
                if c and c[0] == sid:
                    d = self.ser.read(2)
                    if len(d) == 2:
                        v = (d[0] << 8) | d[1]
                        return None if v == 0xFFFE else (v / 65535.0) * 360.0
                return None
        return None

    def close(self):
        try: self.de_re.close()
        except Exception: pass
        try: self.ser.close()
        except Exception: pass


class MotorTracker:
    def __init__(self, raw_init_deg, filter_size=MEDIAN_FILTER_SIZE):
        self.raw_init = raw_init_deg
        self.last_raw = None
        self.unwrapped = None
        self.buffer = deque(maxlen=filter_size)

    def update(self, raw_deg):
        if raw_deg is None:
            return self.filtered()
        if self.last_raw is None:
            self.unwrapped = wrap_to_180(raw_deg - self.raw_init)
        else:
            self.unwrapped += wrap_to_180(raw_deg - self.last_raw)
        self.last_raw = raw_deg
        self.buffer.append(self.unwrapped)
        return self.filtered()

    def filtered(self):
        if not self.buffer:
            return None
        return statistics.median(self.buffer)


def motor_to_joint_rad(motor_unwrapped_deg, params):
    if motor_unwrapped_deg is None:
        return None
    j = math.radians(motor_unwrapped_deg) * params["gear_num"] / params["gear_den"]
    return -j if params["inverted"] else j


def load_calibration():
    with open(CALIB_PATH) as f:
        data = yaml.safe_load(f) or {}
    raw = data["encoder_raw_init_deg"]
    return {1: raw["motor_1"], 2: raw["motor_2"], 3: raw["motor_3"]}


class EncoderPublisher(Node):
    def __init__(self):
        super().__init__("encoder_publisher")
        self.pub = self.create_publisher(JointState, "/joint_states_measured", 10)
        self.master = RS485Master()
        calib = load_calibration()
        self.get_logger().info(f"Calibration chargee : {calib}")
        self.trackers = {sid: MotorTracker(calib[sid]) for sid in (1, 2, 3)}

        # Polling RS-485 et publication a ~50 Hz
        self.create_timer(1.0 / 50.0, self.tick)

        # Stats pour log periodique
        self._tick_count = 0
        self._last_log = time.time()

    def tick(self):
        # Polling sync des 3 encodeurs (~18ms total a 56Hz)
        raws = {}
        for sid in (1, 2, 3):
            raws[sid] = self.master.query(sid)
            self.trackers[sid].update(raws[sid])

        # Conversion en angles joint
        motor_filtered = {sid: self.trackers[sid].filtered() for sid in (1, 2, 3)}
        joints_rad = {
            sid: motor_to_joint_rad(motor_filtered[sid], JOINT_PARAMS[sid])
            for sid in (1, 2, 3)
        }
        if joints_rad[2] is not None and joints_rad[3] is not None:
            joints_rad[3] = joints_rad[3] + joints_rad[2] * COUPLING_J2_TO_J3

        # Publier (uniquement les joints qui ont une valeur)
        if all(joints_rad[sid] is not None for sid in (1, 2, 3)):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [JOINT_PARAMS[sid]["name"] for sid in (1, 2, 3)]
            msg.position = [float(joints_rad[sid]) for sid in (1, 2, 3)]
            self.pub.publish(msg)

        self._tick_count += 1
        now = time.time()
        if now - self._last_log >= 5.0:
            freq = self._tick_count / (now - self._last_log)
            self.get_logger().info(
                f"Publication a {freq:.1f} Hz | "
                f"j1={math.degrees(joints_rad[1] or 0):+6.2f}d "
                f"j2={math.degrees(joints_rad[2] or 0):+6.2f}d "
                f"j3={math.degrees(joints_rad[3] or 0):+6.2f}d"
            )
            self._tick_count = 0
            self._last_log = now

    def destroy_node(self):
        self.master.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
