#!/usr/bin/env python3
"""Exécute une séquence de poses via MoveIt (planification ANTI-COLLISION).

Lit ~/roby_poses.yaml (nom: [j1,j2,j3,j4,j5] en radians) et exécute, dans
l'ordre, les poses passées en argument. move_group planifie en évitant les
obstacles de la planning scene (charge-les avant : scene_loader) puis exécute
via arm_controller. Vitesse réduite pour la sécurité / l'anti-overrun RT.

Usage :
  roby_moveit_seq.py --list                 # liste les poses connues
  roby_moveit_seq.py nid A B C              # va à nid, puis A, puis B, puis C
  roby_moveit_seq.py --vel 0.05 A B         # vitesse encore plus lente

À lancer avec le Python SYSTÈME + ROS + DDS (voir roby_moveit_seq.sh).
LE ROBOT BOUGE — lancer en surveillant, prêt à couper.
"""
import sys
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions)

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
GROUP = "arm"
# Source UNIQUE de poses (fichier faisant autorité). Capture via le teach
# pendant (roby_fine_jog, bouton Capturer) ou capture_to_poses.py.
POSES_FILES = [os.path.expanduser("~/roby_poses.yaml")]
DEFAULT_VEL = 0.10   # facteur d'échelle vitesse (10% du max) — lent
DEFAULT_ACC = 0.10


def load_poses():
    poses = {}
    for path in POSES_FILES:
        if not os.path.exists(path):
            continue
        try:
            data = yaml.safe_load(open(path)) or {}
        except Exception:
            continue
        for k, v in data.items():
            if isinstance(v, (list, tuple)) and len(v) == 5:
                poses[k] = [float(x) for x in v]   # le dernier fichier l'emporte
    return poses


class Seq(Node):
    def __init__(self, vel, acc, dry=False):
        super().__init__("roby_moveit_seq")
        self.vel = vel
        self.acc = acc
        self.dry = dry
        self.ac = ActionClient(self, MoveGroup, "/move_action")

    def move_to(self, name, joints):
        if not self.ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("move_group absent (stack Pi5 lancee ?)")
            return False
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = self.vel
        req.max_acceleration_scaling_factor = self.acc
        c = Constraints()
        for jn, val in zip(JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        opt = PlanningOptions()
        opt.plan_only = self.dry                     # --dry : planifie SEULEMENT (ne bouge pas)
        opt.planning_scene_diff.is_diff = True       # utilise la scene courante (solides)
        opt.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options = opt

        self.get_logger().info("MoveIt -> %s : planification + execution (vel=%.2f)..."
                               % (name, self.vel))
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("  goal refuse"); return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        code = rf.result().result.error_code.val
        ok = (code == 1)   # MoveItErrorCodes.SUCCESS = 1
        self.get_logger().info("  %s : %s (code=%d)"
                               % (name, "OK" if ok else "ECHEC", code))
        return ok


def main():
    args = sys.argv[1:]
    vel, acc = DEFAULT_VEL, DEFAULT_ACC
    dry = False
    if "--dry" in args:
        dry = True; args.remove("--dry")
    if "--vel" in args:
        i = args.index("--vel"); vel = float(args[i + 1]); del args[i:i + 2]
    poses = load_poses()
    if not args or args[0] == "--list":
        print("Poses connues (%s) :" % ", ".join(POSES_FILES))
        for k, v in poses.items():
            print("  %-12s [%s]" % (k, ", ".join("%.3f" % x for x in v)))
        if not poses:
            print("  (aucune — capture des poses d'abord)")
        return
    unknown = [a for a in args if a not in poses]
    if unknown:
        print("Poses inconnues :", unknown, "\nDispo :", ", ".join(poses)); return

    rclpy.init()
    n = Seq(vel, acc, dry=dry)
    if dry:
        print("*** MODE DRY : planification seule, AUCUN mouvement ***")
    try:
        for name in args:
            if not n.move_to(name, poses[name]):
                print("ARRET : echec/refus sur '%s'." % name); break
            time.sleep(0.5)
    finally:
        n.destroy_node()
        rclpy.shutdown()
    os._exit(0)


main()
