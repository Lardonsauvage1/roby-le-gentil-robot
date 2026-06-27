#!/usr/bin/env python3
"""Charge un environnement (obstacles fixes) dans la planning scene de MoveIt.

Chaque environnement = un fichier YAML dans environments/<nom>.yaml décrivant des
boîtes de collision (table, armoire, base, zones humains...). On peut basculer
d'un lieu à l'autre : `scene_loader --env <nom>` efface les obstacles précédents
et publie les nouveaux. Nécessite move_group lancé.

Exemples :
    ros2 run roby_environments scene_loader --env atelier_actuel
    ros2 run roby_environments scene_loader --clear            # tout effacer
    ros2 run roby_environments scene_loader --list             # lister les envs

Convention repère (frame world = base du robot) : +X avant (vers le plan de
travail), +Y gauche, +Z haut. pose=[x,y,z] = CENTRE de la boîte ; size=[x,y,z].
"""
import os
import sys
import math
import argparse
import glob

import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import (PlanningScene, CollisionObject,
                             PlanningSceneComponents, ObjectColor)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA


def quat_from_rpy(r, p, y):
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def envs_dir():
    return os.path.join(get_package_share_directory("roby_environments"),
                        "environments")


def list_envs():
    return sorted(os.path.splitext(os.path.basename(f))[0]
                  for f in glob.glob(os.path.join(envs_dir(), "*.yaml")))


def make_object(obj, frame):
    co = CollisionObject()
    co.header.frame_id = obj.get("frame", frame)
    co.id = obj["name"]
    prim = SolidPrimitive()
    t = obj.get("type", "box").lower()
    if t == "cylinder":
        prim.type = SolidPrimitive.CYLINDER
        prim.dimensions = [float(obj["height"]), float(obj["radius"])]  # [h, r]
    elif t == "sphere":
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [float(obj["radius"])]
    else:  # box
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(v) for v in obj["size"]]
    pose = Pose()
    px, py, pz = (float(v) for v in obj["pose"])
    pose.position.x, pose.position.y, pose.position.z = px, py, pz
    r, p, yaw = (float(v) for v in obj.get("rpy", [0.0, 0.0, 0.0]))
    qx, qy, qz, qw = quat_from_rpy(r, p, yaw)
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw
    co.primitives.append(prim)
    co.primitive_poses.append(pose)
    co.operation = CollisionObject.ADD
    return co


class SceneLoader(Node):
    def __init__(self):
        super().__init__("scene_loader")
        self.apply_cli = self.create_client(ApplyPlanningScene,
                                            "apply_planning_scene")
        self.get_cli = self.create_client(GetPlanningScene,
                                          "get_planning_scene")

    def _wait(self, cli, name):
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"Service '{name}' indisponible — move_group est-il lancé ?")
            return False
        return True

    def _call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result()

    def current_object_ids(self):
        if not self._wait(self.get_cli, "get_planning_scene"):
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        res = self._call(self.get_cli, req)
        if res is None:
            return []
        return [o.id for o in res.scene.world.collision_objects]

    def apply(self, collision_objects, object_colors=None):
        if not self._wait(self.apply_cli, "apply_planning_scene"):
            return False
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects
        if object_colors:
            scene.object_colors = object_colors
        res = self._call(self.apply_cli, ApplyPlanningScene.Request(scene=scene))
        ok = bool(res and res.success)
        if not ok:
            self.get_logger().error("apply_planning_scene a échoué")
        return ok

    def clear(self):
        ids = self.current_object_ids()
        if not ids:
            self.get_logger().info("Aucun obstacle à effacer.")
            return True
        removes = []
        for oid in ids:
            co = CollisionObject()
            co.id = oid
            co.operation = CollisionObject.REMOVE
            removes.append(co)
        ok = self.apply(removes)
        if ok:
            self.get_logger().info(f"Effacé {len(removes)} obstacle(s).")
        return ok

    def load(self, env_name):
        path = os.path.join(envs_dir(), env_name + ".yaml")
        if not os.path.isfile(path):
            self.get_logger().error(
                f"Environnement '{env_name}' introuvable ({path}). "
                f"Dispo: {', '.join(list_envs()) or '(aucun)'}")
            return False
        return self.load_file(path, env_name)

    def load_file(self, path, label=None):
        label = label or os.path.basename(path)
        if not os.path.isfile(path):
            self.get_logger().error(f"Fichier introuvable : {path}")
            return False
        with open(path) as fh:
            data = yaml.safe_load(fh) or {}
        frame = data.get("frame", "world")
        objs = data.get("objects", [])
        # on efface d'abord l'environnement précédent pour un swap propre
        self.clear()
        cos = [make_object(o, frame) for o in objs]
        colors = []
        for o in objs:
            if "color" in o:
                oc = ObjectColor()
                oc.id = o["name"]
                c = o["color"]
                oc.color = ColorRGBA(
                    r=float(c[0]), g=float(c[1]), b=float(c[2]),
                    a=float(c[3]) if len(c) > 3 else 1.0)
                colors.append(oc)
        ok = self.apply(cos, colors)
        if ok:
            self.get_logger().info(
                f"Environnement '{label}' chargé : {len(cos)} obstacle(s) "
                f"[{', '.join(o.id for o in cos)}]")
        return ok


def main():
    parser = argparse.ArgumentParser(description="Chargeur d'environnement MoveIt")
    parser.add_argument("--env", help="nom de l'environnement (sans .yaml)")
    parser.add_argument("--file", help="charger directement un YAML (chemin), "
                        "pratique pour itérer sans rebuild")
    parser.add_argument("--clear", action="store_true",
                        help="effacer tous les obstacles")
    parser.add_argument("--list", action="store_true",
                        help="lister les environnements dispo")
    args, _ = parser.parse_known_args()

    if args.list:
        print("Environnements disponibles :")
        for e in list_envs():
            print("  -", e)
        return

    rclpy.init()
    node = SceneLoader()
    try:
        if args.clear:
            node.clear()
        elif args.file:
            node.load_file(os.path.expanduser(args.file))
        elif args.env:
            node.load(args.env)
        else:
            node.get_logger().error(
                "Préciser --env <nom> | --file <chemin> | --clear | --list")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
