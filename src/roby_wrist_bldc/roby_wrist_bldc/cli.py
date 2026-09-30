"""Console de banc pour l'axe 5, sans ROS : ``ros2 run roby_wrist_bldc wrist_cli``.

A NE PAS lancer pendant que la stack tourne (le port serie est exclusif).

Modes :
    wrist_cli monitor            affiche l'etat a 5 Hz (lecture seule, n'envoie rien)
    wrist_cli shell              console interactive (tapez 'aide')
Options : --port /dev/ttyACM0 | auto   --simulate   --vmax 0.2
"""

from __future__ import annotations

import argparse
import logging
import shlex
import time
from typing import Callable, Dict, List

from .driver import DriverConfig, WristBldcDriver, WristError, WristStatus
from .sim import SimulatedBoard

HELP = """Commandes :
  etat                    etat complet
  init <nid>              recale sur la position du nid (tete AU NID !) puis enable
  recale <angle>          Z<angle> seul (sans enable)
  va <angle> [vmax]       mouvement en rampe 100 Hz, attend l'arrivee
  p <angle>               consigne brute P (sans rampe)
  stop                    freine et s'arrete (moteur asservi)
  en | dis                enable / disable (disable = roue libre !)
  reset                   efface le defaut
  pos                     demande la position a la carte (?)
  suivi <s>               affiche l'etat pendant <s> secondes
  confiance               considere la carte deja recalee (session precedente)
  q                       quitter (le moteur reste asservi)"""


def format_status(s: WristStatus) -> str:
    pos = "   ---  " if s.position is None else f"{s.position:+.4f}"
    flags = " ".join(
        name
        for name, on in (
            ("LIAISON", s.fresh),
            ("RECALE", s.homed),
            ("ACTIF", s.enabled),
            ("MOUVEMENT", s.streaming),
            ("DEFAUT-CARTE", s.board_fault),
            ("DEFAUT-SUIVI", s.tracking_fault),
        )
        if on
    )
    reason = f"  [bloque : {s.blocking_reason}]" if s.blocking_reason else ""
    return f"pos {pos} rad  I {s.current:5.2f} A  {flags}{reason}"


def build_driver(args: argparse.Namespace) -> WristBldcDriver:
    config = DriverConfig(port=args.port, max_velocity=args.vmax)
    if args.simulate:
        board = SimulatedBoard(start_position=0.0)
        config.port = "sim"
        return WristBldcDriver(config, serial_factory=board.factory)
    return WristBldcDriver(config)


def monitor(driver: WristBldcDriver, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        print(format_status(driver.get_status()))
        time.sleep(0.2)


def shell(driver: WristBldcDriver) -> None:
    def go(words: List[str]) -> None:
        target = float(words[0])
        vmax = float(words[1]) if len(words) > 1 else None
        driver.move_to(target, max_velocity=vmax)
        ok = driver.wait_until_reached(timeout=30.0)
        print(
            ("arrive : " if ok else "PAS arrive : ")
            + format_status(driver.get_status())
        )

    def trust(_: List[str]) -> None:
        driver.config.require_homing = False
        print(
            "ATTENTION : recalage suppose valide "
            "(carte non redemarree depuis le dernier Z)"
        )

    actions: Dict[str, Callable[[List[str]], None]] = {
        "etat": lambda w: print(format_status(driver.get_status())),
        "init": lambda w: print(format_status(driver.initialize_at_nest(float(w[0])))),
        "recale": lambda w: driver.recalibrate(float(w[0])),
        "va": go,
        "p": lambda w: driver.set_position(float(w[0])),
        "stop": lambda w: driver.stop_motion(),
        "en": lambda w: driver.enable(),
        "dis": lambda w: driver.disable(),
        "reset": lambda w: driver.reset_fault(),
        "pos": lambda w: print(f"POS {driver.query_position():+.4f}"),
        "suivi": lambda w: monitor(driver, float(w[0]) if w else 5.0),
        "confiance": trust,
        "aide": lambda w: print(HELP),
    }
    print(HELP)
    while True:
        try:
            line = input("poignet> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return
        words = shlex.split(line)
        if not words:
            continue
        if words[0] in ("q", "quit", "exit"):
            return
        action = actions.get(words[0])
        if action is None:
            print("commande inconnue (aide)")
            continue
        try:
            action(words[1:])
        except (WristError, ValueError, IndexError) as exc:
            print(f"refuse : {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Console de banc de l'axe 5 (poignet BLDC)"
    )
    parser.add_argument(
        "mode", choices=["monitor", "shell"], nargs="?", default="shell"
    )
    parser.add_argument("--port", default="auto")
    parser.add_argument(
        "--simulate", action="store_true", help="carte simulee, aucun materiel"
    )
    parser.add_argument(
        "--vmax", type=float, default=0.3, help="vitesse max rad/s (defaut prudent 0.3)"
    )
    parser.add_argument(
        "--seconds", type=float, default=3600.0, help="duree du monitor"
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    with build_driver(args) as driver:
        try:
            driver.wait_connected(5.0)
        except WristError as exc:
            print(f"carte injoignable : {exc}")
            return
        if args.mode == "monitor":
            monitor(driver, args.seconds)
        else:
            shell(driver)


if __name__ == "__main__":
    main()
