#!/usr/bin/env python3
"""
aruco_calibrate — calibre l'orientation des 6 faces du cube ArUco.

Apprend, pour chaque marqueur, sa rotation vers un repère "cube" commun, en
observant les rotations RELATIVES entre marqueurs co-visibles (2 faces vues en
même temps). Ces rotations relatives sont rigides => INDÉPENDANTES de la caméra
et de son placement. On peut donc calibrer avec un setup caméra provisoire.

Procédure : tourner lentement le cube devant la caméra pour montrer toutes les
faces, en passant par des positions où 2 faces sont visibles à la fois.

Sortie : config/cube_faces.yaml  (reference_marker + quaternion R_marker->cube
par marqueur). aruco_node le charge pour publier une orientation cohérente.

À lancer caméra LIBRE (arrêter aruco_node d'abord) :
    /usr/bin/python3.12 .../aruco_calibrate.py
"""

import sys
import time
import numpy as np
import cv2

VIDEO_DEVICE = 0
WIDTH, HEIGHT = 1280, 720
MARKER_SIZE = 0.10
K = np.array([[1000., 0, 640.], [0, 1000., 360.], [0, 0, 1.]])
DIST = np.zeros((4, 1))
MIN_PAIR_SAMPLES = 20          # paires à accumuler avant de figer une arête
OUT = None                     # chemin de sortie (argv[1] sinon défaut)


def proj_SO3(M):
    """Projette une matrice sur SO(3) (rotation la plus proche, L2)."""
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (R[2, 1]-R[1, 2])/s; y = (R[0, 2]-R[2, 0])/s; z = (R[1, 0]-R[0, 1])/s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0+R[0, 0]-R[1, 1]-R[2, 2])*2
        w = (R[2, 1]-R[1, 2])/s; x = 0.25*s; y = (R[0, 1]+R[1, 0])/s; z = (R[0, 2]+R[2, 0])/s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0+R[1, 1]-R[0, 0]-R[2, 2])*2
        w = (R[0, 2]-R[2, 0])/s; x = (R[0, 1]+R[1, 0])/s; y = 0.25*s; z = (R[1, 2]+R[2, 1])/s
    else:
        s = np.sqrt(1.0+R[2, 2]-R[0, 0]-R[1, 1])*2
        w = (R[1, 0]-R[0, 1])/s; x = (R[0, 2]+R[2, 0])/s; y = (R[1, 2]+R[2, 1])/s; z = 0.25*s
    return [float(x), float(y), float(z), float(w)]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/sam/ros2_ws/src/roby_control/config/cube_faces.yaml'
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0

    cap = cv2.VideoCapture(VIDEO_DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        print("ERREUR: caméra non ouvrable (aruco_node tourne encore ?)"); return
    adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()

    # somme des matrices de rotation relative M_ij = R_i^T R_j, par paire
    pair_sum = {}     # (i,j) -> matrice 3x3 sommée
    pair_cnt = {}     # (i,j) -> nb échantillons
    seen = set()
    print(f"Calibration ~{duration:.0f}s. TOURNE LE CUBE lentement (montre toutes les "
          f"faces, passe par les coins où 2 faces se voient). Ctrl-C pour finir avant.")
    t0 = time.time(); last = 0
    try:
        while time.time() - t0 < duration:
            ok, frame = cap.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, adict, parameters=params)
            if ids is None or len(ids) < 1:
                continue
            ids = ids.flatten()
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, K, DIST)
            Rs = {}
            for k, mid in enumerate(ids):
                if not (0.1 < tvecs[k].reshape(3)[2] < 2.5):
                    continue
                Rs[int(mid)] = cv2.Rodrigues(rvecs[k])[0]
                seen.add(int(mid))
            # paires co-visibles
            mids = sorted(Rs.keys())
            for a in range(len(mids)):
                for b in range(a + 1, len(mids)):
                    i, j = mids[a], mids[b]
                    Mij = Rs[i].T @ Rs[j]
                    pair_sum[(i, j)] = pair_sum.get((i, j), np.zeros((3, 3))) + Mij
                    pair_cnt[(i, j)] = pair_cnt.get((i, j), 0) + 1
            if time.time() - last > 2.0:
                ready = [p for p, c in pair_cnt.items() if c >= MIN_PAIR_SAMPLES]
                print(f"  faces vues: {sorted(seen)} | paires solides: "
                      f"{[f'{i}-{j}' for i, j in ready]}")
                last = time.time()
    except KeyboardInterrupt:
        print("\n(arrêt manuel)")
    cap.release()

    # construire le graphe et propager depuis le marqueur de référence
    edges = {p: proj_SO3(pair_sum[p] / pair_cnt[p])
             for p in pair_sum if pair_cnt[p] >= MIN_PAIR_SAMPLES}
    if not seen:
        print("ERREUR: aucun marqueur vu."); return
    ref = min(seen)
    # adjacence : pour chaque arête (i,j), M_ij = R_i^T R_j et M_ji = M_ij^T
    adj = {}
    for (i, j), M in edges.items():
        adj.setdefault(i, []).append((j, M))         # M = R_i^T R_j
        adj.setdefault(j, []).append((i, M.T))       # R_j^T R_i

    # R_to_cube[m] = R_m^T R_ref  ; BFS depuis ref (cube := repère du marqueur ref)
    R_to_cube = {ref: np.eye(3)}
    queue = [ref]
    while queue:
        k = queue.pop(0)
        for (m, M_km) in adj.get(k, []):
            # M_km = R_k^T R_m  => R_m^T R_ref = (R_m^T R_k)(R_k^T R_ref) = M_km^T @ R_to_cube[k]
            if m not in R_to_cube:
                R_to_cube[m] = proj_SO3(M_km.T @ R_to_cube[k])
                queue.append(m)

    missing = seen - set(R_to_cube.keys())
    print(f"\nRéférence (repère cube) = marqueur {ref}")
    print(f"Faces calibrées : {sorted(R_to_cube.keys())}")
    if missing:
        print(f"⚠ Faces vues mais NON reliées (pas assez de co-visibilité): {sorted(missing)} "
              f"-> recommence en montrant ces faces À CÔTÉ d'une déjà calibrée.")

    # écrire le YAML
    lines = [f"reference_marker: {ref}", "faces:"]
    for m in sorted(R_to_cube.keys()):
        q = R_to_quat(R_to_cube[m])
        lines.append(f"  {m}: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]")
    with open(out, 'w') as f:
        f.write("\n".join(lines) + "\n")
    print(f"\n✅ Écrit {out}")
    print("\n".join(lines))


if __name__ == '__main__':
    main()
