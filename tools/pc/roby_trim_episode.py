#!/usr/bin/env python3
"""roby_trim_episode.py — Rogne le debut STATIQUE d'un episode enregistre.

Detecte le 1er mouvement des joints (>seuil) et reecrit le bag en ne gardant que
[1er_mouvement - LEAD, fin]. Garde un petit debut statique (LEAD s) pour l'etat initial.
Ecrit un nouveau bag <ep>_trim/ (n'ecrase pas l'original sauf --inplace).

Usage : roby_trim_episode.py <chemin_ep> [--lead 0.5] [--inplace]
"""
import argparse
import math
import os
import shutil

import numpy as np
from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                        ConverterOptions, TopicMetadata)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

J = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def first_motion_time(bag, thresh_deg=0.5):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    p0 = None
    while r.has_next():
        tp, d, t = r.read_next()
        if tp == "/joint_states":
            m = deserialize_message(d, JointState)
            idx = {n: i for i, n in enumerate(m.name)}
            try:
                p = np.array([m.position[idx[j]] for j in J])
            except KeyError:
                continue
            if p0 is None:
                p0 = p
            elif max(abs(p - p0)) > math.radians(thresh_deg):
                return t
    return None


def trim(bag, lead, inplace):
    t_move = first_motion_time(bag)
    if t_move is None:
        print(f"  {os.path.basename(bag)} : aucun mouvement detecte, non rogne")
        return
    t_keep = t_move - int(lead * 1e9)

    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("", ""))
    topics = r.get_all_topics_and_types()

    out = bag.rstrip("/") + "_trim"
    if os.path.exists(out):
        shutil.rmtree(out)
    w = SequentialWriter()
    w.open(StorageOptions(uri=out, storage_id="mcap"), ConverterOptions("", ""))
    for i, tm in enumerate(topics):
        w.create_topic(TopicMetadata(id=i, name=tm.name, type=tm.type,
                                     serialization_format="cdr"))
    kept = total = 0
    t0 = None
    while r.has_next():
        tp, d, t = r.read_next()
        total += 1
        if t0 is None:
            t0 = t
        if t >= t_keep:
            w.write(tp, d, t)
            kept += 1
    del w
    dropped_s = (t_keep - t0) / 1e9
    print(f"  {os.path.basename(bag)} : rogne {dropped_s:.1f}s statiques  "
          f"({total}->{kept} msgs)  -> {os.path.basename(out)}")

    if inplace:
        shutil.rmtree(bag)
        os.rename(out, bag)
        print(f"    remplace l'original (inplace)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("ep", help="chemin du dossier episode (ep_XXX)")
    ap.add_argument("--lead", type=float, default=0.5, help="debut statique garde (s)")
    ap.add_argument("--inplace", action="store_true", help="remplace l'original")
    a = ap.parse_args()
    trim(os.path.expanduser(a.ep), a.lead, a.inplace)
