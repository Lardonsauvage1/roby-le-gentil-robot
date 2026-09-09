#!/usr/bin/env python3
"""Duplique un dataset de bags en ajoutant un topic /tcp_pose = pose CARTESIENNE
(FK des /joint_states), representation [x,y,z, rvx,rvy,rvz] (position + rotation-vector).
Tout le reste (images, /gripper, /head_lock, /joint_states) est copie a l'identique.
FK = roby_oracle.fkT (repere link_gripper+6cm) -> coherent avec le DLS de deploiement.

Usage : roby_dataset_to_cartesian.py <batch_src> <batch_dst> [ep_000 ...]
  sans liste d'episodes = tous. Avec = seulement ceux-la (test).
"""
import sys
import os
import shutil

sys.path.insert(0, os.path.expanduser("~"))
import numpy as np
from roby_oracle import fkT
from roby_tool_pickup import rotvec

import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def convert_episode(src_ep, dst_ep):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src_ep, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = reader.get_all_topics_and_types()

    if os.path.exists(dst_ep):
        shutil.rmtree(dst_ep)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst_ep, storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for tm in topics:
        writer.create_topic(tm)
    # nouveau topic cartesien (id unique = apres les existants)
    new_id = max([getattr(tm, "id", i) for i, tm in enumerate(topics)] or [0]) + 1
    tcp_tm = rosbag2_py.TopicMetadata(
        new_id, "/tcp_pose", "std_msgs/msg/Float64MultiArray", "cdr")
    writer.create_topic(tcp_tm)

    n_js, n_other, sample = 0, 0, None
    while reader.has_next():
        topic, data, t = reader.read_next()
        writer.write(topic, data, t)
        if topic == "/joint_states":
            js = deserialize_message(data, JointState)
            d = dict(zip(js.name, js.position))
            if all(j in d for j in J):
                q = [float(d[j]) for j in J]
                T = fkT(q); p = T[:3, 3]; rv = rotvec(T[:3, :3])
                msg = Float64MultiArray()
                msg.data = [float(p[0]), float(p[1]), float(p[2]),
                            float(rv[0]), float(rv[1]), float(rv[2])]
                writer.write("/tcp_pose", serialize_message(msg), t)
                n_js += 1
                if sample is None:
                    sample = msg.data
        else:
            n_other += 1
    del writer
    del reader
    return n_js, n_other, sample


def main():
    args = sys.argv[1:]
    sync = "--sync" in args
    if sync:
        args.remove("--sync")
    src = os.path.expanduser(args[0])
    dst = os.path.expanduser(args[1])
    eps = args[2:]
    os.makedirs(dst, exist_ok=True)
    if not eps:
        eps = sorted(d for d in os.listdir(src)
                     if d.startswith("ep_") and os.path.isdir(os.path.join(src, d)))
    if sync:
        # ne garde que les episodes ABSENTS du cartesien (mirroring des nouveaux)
        before = len(eps)
        eps = [e for e in eps if not os.path.isdir(os.path.join(dst, e))]
        print(f"[sync] {before - len(eps)} deja convertis, {len(eps)} nouveaux a mirrorer")
        if not eps:
            print("=== rien a faire (cartesien deja a jour)"); return
    for ep in eps:
        s = os.path.join(src, ep)
        de = os.path.join(dst, ep)
        n_js, n_other, sample = convert_episode(s, de)
        print(f"  {ep}: /tcp_pose={n_js} msgs, autres copies={n_other}, "
              f"1er tcp=[{', '.join('%.4f' % v for v in sample)}]" if sample else f"  {ep}: aucun joint_states")
        # copie les fichiers annexes (.meta.json, .rec.log)
        for ext in (".meta.json", ".rec.log"):
            f = os.path.join(src, ep + ext)
            if os.path.exists(f):
                shutil.copy2(f, os.path.join(dst, ep + ext))
    print(f"=== {len(eps)} episode(s) -> {dst}")


if __name__ == "__main__":
    main()
