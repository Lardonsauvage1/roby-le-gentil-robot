#!/usr/bin/env python3
"""Noeud ROS de la pince (PCA9685 canal 3).
Ecoute /gripper (std_msgs/Bool) : true=FERMER (55deg), false=OUVRIR (120deg).
N alimente que tete verrouillee (CH2). Init CONDITIONNEL (ne touche pas le 50Hz
deja regle -> preserve CH2 verrou + axes 4/5). Demarre OUVERT (pret au clipsage).
Lance: python3 ~/gripper_node.py
"""
import fcntl, os, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

I2C_SLAVE = 0x0703
ADDR, BUS, CH = 0x40, "/dev/i2c-1", 3
OPEN_DEG, CLOSE_DEG = 120.0, 55.0      # ouvert / ferme (calibres 2026-06-02)

def angle_to_off(a):
    pulse = 500.0 + (a / 180.0) * 2000.0
    return int((pulse / 20000.0) * 4096.0)

class Gripper(Node):
    def __init__(self):
        super().__init__("gripper")
        self.fd = os.open(BUS, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, ADDR)
        self._ensure_50hz()
        self.set_angle(OPEN_DEG)            # demarre ouvert (clipsage)
        self.create_subscription(Bool, "/gripper", self.cb, 10)
        self.get_logger().info("gripper pret. /gripper: true=FERMER(55) false=OUVRIR(120). Etat=OUVERT")

    def _wr(self, reg, val):
        os.write(self.fd, bytes([reg, val]))

    def _ensure_50hz(self):
        os.write(self.fd, bytes([0xFE]))
        pres = os.read(self.fd, 1)[0]
        if pres != 121:
            self._wr(0x00, 0x10); self._wr(0xFE, 121); self._wr(0x00, 0x20)
            time.sleep(0.001); self._wr(0x00, 0xA0)
            self.get_logger().info("PCA9685 init 50Hz (prescale etait %d)" % pres)
        else:
            self.get_logger().info("PCA9685 deja 50Hz, init saute (CH2 verrou + axes4/5 preserves)")

    def set_angle(self, a):
        off = angle_to_off(a); base = 0x06 + 4 * CH
        os.write(self.fd, bytes([base, 0, 0, off & 0xFF, (off >> 8) & 0x0F]))

    def cb(self, msg):
        a = CLOSE_DEG if msg.data else OPEN_DEG
        self.set_angle(a)
        self.get_logger().info("%s -> %.0f deg" % ("FERMER" if msg.data else "OUVRIR", a))

def main():
    rclpy.init()
    n = Gripper()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

main()
