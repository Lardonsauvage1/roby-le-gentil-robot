#!/usr/bin/env python3
"""Noeud ROS du servo de verrouillage de tete (PCA9685 canal 2).
Ecoute /head_lock (std_msgs/Bool) : true=VERROU (75deg), false=DEVERROU (50deg).
Tourne A COTE de la stack arm : ecrit uniquement le CH2 (registres distincts des
axes 4/5 sur CH0/CH1). N init le PCA QUE s il n est pas deja a 50Hz (sinon on
couperait brievement tous les servos). Lance: python3 ~/head_lock_node.py
"""
import fcntl, os, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

I2C_SLAVE = 0x0703
ADDR, BUS, CH = 0x40, "/dev/i2c-1", 2
UNLOCK_DEG, LOCK_DEG = 50.0, 75.0

def angle_to_off(a):
    pulse = 500.0 + (a / 180.0) * 2000.0
    return int((pulse / 20000.0) * 4096.0)

class HeadLock(Node):
    def __init__(self):
        super().__init__("head_lock")
        self.fd = os.open(BUS, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, ADDR)
        self._ensure_50hz()
        self.set_angle(UNLOCK_DEG)          # repos = deverrouille
        self.create_subscription(Bool, "/head_lock", self.cb, 10)
        self.get_logger().info("head_lock pret. /head_lock: true=VERROU(75) false=DEVERROU(50). Etat=DEVERROU")

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
            self.get_logger().info("PCA9685 deja a 50Hz, init saute (coexistence axes 4/5 OK)")

    def set_angle(self, a):
        off = angle_to_off(a); base = 0x06 + 4 * CH
        os.write(self.fd, bytes([base, 0, 0, off & 0xFF, (off >> 8) & 0x0F]))

    def cb(self, msg):
        a = LOCK_DEG if msg.data else UNLOCK_DEG
        self.set_angle(a)
        self.get_logger().info("%s -> %.0f deg" % ("VERROU" if msg.data else "DEVERROU", a))

def main():
    rclpy.init()
    n = HeadLock()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

main()
