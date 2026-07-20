#!/usr/bin/env python3
"""Script de PREPARATION a la demo (dry-run, sans prise d objet).
Depart init (pince ouverte, deverrouille) -> pose de montage -> attente 10s
(montage tete) -> verrou -> D (5s) -> E (5s) -> A -> deverrou -> init.
Reutilise ~/demo_poses.yaml. Lance: python3 ~/demo_prep.py
"""
import os, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
POSES_FILE = os.path.expanduser("~/demo_poses.yaml")
SPEED = 0.13
MIN_DUR = 4.0

def load_poses():
    p = {}
    for line in open(POSES_FILE):
        line = line.strip()
        if not line or ":" not in line:
            continue
        name, rest = line.split(":", 1)
        p[name.strip()] = [float(x) for x in rest.strip().strip("[]").split(",")]
    return p

class Prep(Node):
    def __init__(self):
        super().__init__("demo_prep")
        self.ac = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.lock_pub = self.create_publisher(Bool, "/head_lock", 10)
        self.grip_pub = self.create_publisher(Bool, "/gripper", 10)
        self.cur = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.poses = load_poses()

    def _js(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(j in m for j in JOINTS):
            self.cur = [m[j] for j in JOINTS]

    def wait_current(self):
        t0 = time.time()
        while self.cur is None and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.2)

    def log(self, m):
        self.get_logger().info(m)

    def move(self, name):
        target = self.poses[name]
        self.wait_current()
        delta = max(abs(target[i] - (self.cur[i] if self.cur else 0.0)) for i in range(5))
        dur = max(MIN_DUR, delta / SPEED)
        self.log("MOVE -> %s (delta=%.2f rad, duree=%.1f s)" % (name, delta, dur))
        if not self.ac.wait_for_server(timeout_sec=10):
            self.log("ERREUR: action server arm_controller absent"); return False
        traj = JointTrajectory(); traj.joint_names = JOINTS
        pt = JointTrajectoryPoint(); pt.positions = target
        pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
        gh_fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_fut)
        gh = gh_fut.result()
        if not gh.accepted:
            self.log("goal refuse"); return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.log("  arrive a %s" % name)
        return True

    def lock(self, v):
        self.lock_pub.publish(Bool(data=v)); self.log("VERROU" if v else "DEVERROU"); self._spin(1.5)

    def grip(self, close):
        self.grip_pub.publish(Bool(data=close)); self.log("pince FERME" if close else "pince OUVRE"); self._spin(1.5)

    def _spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait(self, s, label):
        self.log("=== ATTENTE %.0fs : %s ===" % (s, label)); self._spin(s)

    def run(self):
        self.log("==== PREPARATION DEMO ====")
        self.move("init")
        self.grip(False)                  # pince ouverte
        self.lock(False)                  # deverrouille
        self.move("mount")                # position actuelle (montage)
        self.wait(10.0, "montage de la tete")
        self.lock(True)                   # verrouille
        self.move("D")
        self.wait(5.0, "verif D")
        self.move("E")
        self.wait(5.0, "verif E")
        self.move("A")
        self.wait(30.0, "avant deverrouillage (retire la tete)")
        self.lock(False)                  # deverrouille la tete
        self.move("init")
        self.log("==== FIN PREPARATION ====")

def main():
    rclpy.init(); n = Prep()
    try:
        n.run()
    finally:
        try:
            n.destroy_node(); rclpy.shutdown()
        except Exception:
            pass
    os._exit(0)   # sortie immediate (evite le hang rclpy au shutdown)

main()
