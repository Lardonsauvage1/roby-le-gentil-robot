#!/usr/bin/env python3
"""
roby_oracle_infer.py — Boucle ORACLE de DÉPLOIEMENT : SETUP identique à l'oracle, mais la
partie "ramener la pomme à D" (ligne 411 de roby_oracle.py) est faite par le RÉSEAU.

Par épisode :
  [SETUP oracle, MoveIt/DLS]  prise à D -> pose R aléatoire -> aérien A -> pince OUVERTE
  [RÉSEAU]                    on ARME roby_infer : il prend la main (aérien A -> ramène à D)
                              pendant --return-time s, puis on DÉSARME.
  (reboucle : la pomme est censée être revenue à D)

Le SETUP recrée EXACTEMENT les conditions de départ du dataset (aérien in-distribution,
pince ouverte, pomme posée) -> voir [[project_reseau_pose_depart_indistribution]].

Chaîne : roby_oracle_infer (SETUP direct) + roby_infer (--arm-gate, réseau) -> garde -> moteurs.
Prérequis : le vrai stack (contrôleurs + move_group) + le garde + roby_infer --arm-gate --go
doivent tourner, + les 2 caméras. En "real", MOTEURS peuvent être éteints (dry-run : commandes
envoyées, joint_states = consigne open-loop, aucun mouvement physique).

Modes (comme l'oracle) :
  --mode dry   : logique pure, aucune ROS, log (le handover réseau est juste loggé).
  --mode plan  : IK réelle validée, aucun mouvement.
  --mode real  : bras réel (exige --go). Avec moteurs coupés = dry-run pipeline complet.
"""
import argparse
import os
import random
import sys
import time

sys.path.insert(0, os.path.expanduser("~"))
import roby_oracle as O            # Motion, sample_table, sample_aerien, D_XYZ, J, _in_limits...


def run(mode, n_episodes, seed, vel, cart_speed, return_time, go):
    if seed is None:
        seed = random.SystemRandom().randint(0, 2**31 - 1)
    print(f"[seed] {seed}   (rejouer cette série : --seed {seed})")
    rng = random.Random(seed)
    motion = O.Motion(mode, vel, cart_speed)        # en real : init rclpy + Pickup (roby_tool_pickup)

    # --- client d'armement du réseau (roby_infer) ---
    node = None
    arm_cli = None
    if mode in ("plan", "real"):
        import rclpy
        from rclpy.node import Node
        from std_srvs.srv import SetBool
        if not rclpy.ok():
            rclpy.init()
        node = Node("roby_oracle_infer")
        arm_cli = node.create_client(SetBool, "/roby_infer/arm")
        if not arm_cli.wait_for_service(timeout_sec=10.0):
            print("!! service /roby_infer/arm indisponible — roby_infer est-il lancé avec --arm-gate ?")
            return

    def arm(state):
        """Arme (True) / désarme (False) le réseau. En dry : log seulement."""
        if arm_cli is None:
            print(f"    [réseau] {'ARME (prend la main)' if state else 'DÉSARME (rend la main)'}")
            return
        import rclpy
        from std_srvs.srv import SetBool
        req = SetBool.Request(); req.data = bool(state)
        f = arm_cli.call_async(req); rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)

    def wait(secs):
        """Laisse le réseau piloter pendant secs (en spinnant notre nœud)."""
        if node is None:
            return
        import rclpy
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < secs:
            rclpy.spin_once(node, timeout_sec=0.1)

    print(f"=== ORACLE-INFER mode={mode}  episodes={n_episodes}  return_time={return_time}s ===")
    print(f"D = ({O.D_XYZ[0]:.3f}, {O.D_XYZ[1]:.3f}, {O.D_XYZ[2]:.3f})")
    print("La pomme doit être à D au départ ; le réseau est censé l'y ramener à chaque épisode.\n")

    motion.gripper(close=False)     # pince ouverte au départ

    for i in range(n_episodes):
        print(f"--- Episode {i:03d} ---")
        # ===== SETUP (identique à l'oracle : MoveIt/DLS, in-distribution) =====
        if not motion.pick_at("cone_D", O.D_XYZ):
            print("  !! ECHEC prise D -> ARRET"); break
        R = O.sample_table(motion, rng)
        if R is None:
            print("  !! pas de R atteignable, saut"); continue
        if not motion.place_at("R_aleatoire", R):
            print("  !! ECHEC pose R -> ARRET"); break
        A = O.sample_aerien(motion, rng)
        if A is None:
            print("  !! pas de A atteignable, saut"); continue
        if not motion.move_free("aerien_A", A):
            print("  !! ECHEC aérien -> ARRET"); break
        motion.gripper(close=False)          # pince OUVERTE = état de départ du dataset
        if mode in ("plan", "real"):
            time.sleep(1.0)                  # laisse la pose/pince se stabiliser avant le réseau

        # ===== RÉSEAU (remplace la ligne 411 : ramener la pomme R -> D) =====
        print(f"  >> RÉSEAU prend la main {return_time}s (aérien A -> ramène la pomme à D)")
        arm(True)
        wait(return_time)
        arm(False)
        print(f"  episode {i:03d} : réseau a rendu la main\n")

    if arm_cli is not None:
        arm(False)                            # sécurité : réseau désarmé en fin
    print("=== fin ===")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["dry", "plan", "real"], default="dry")
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--vel", type=float, default=0.30, help="vitesse libre MoveIt du SETUP")
    ap.add_argument("--cart-speed", type=float, default=0.02, help="vitesse ligne droite SETUP (m/s)")
    ap.add_argument("--return-time", type=float, default=25.0,
                    help="durée laissée au réseau pour ramener la pomme (s)")
    ap.add_argument("--go", action="store_true", help="requis pour --mode real (bras réel)")
    args = ap.parse_args()
    if args.mode == "real" and not args.go:
        print("REFUS : --mode real pilote le BRAS RÉEL. Relance avec --go (Sam présent, alim moteurs à jour).")
        return
    run(args.mode, args.episodes, args.seed, args.vel, args.cart_speed, args.return_time, args.go)


if __name__ == "__main__":
    main()
