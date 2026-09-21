"""Procedure de reception de l'axe 5 au montage (banc, sans ROS).

    ros2 run roby_wrist_bldc wrist_bench --nest 0.2009
    ros2 run roby_wrist_bldc wrist_bench --simulate --yes      # repetition a blanc

Deroule les essais T1..T11 dans l'ordre, demande confirmation avant chaque
essai qui fait bouger l'axe ou demande une action de l'operateur, et ecrit un
rapport JSON (wrist_bench_<date>.json). La stack ROS doit etre ARRETEE (port
serie exclusif). Tete posee dans le nid au lancement.
"""

from __future__ import annotations

import argparse
import json
import logging
import statistics
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional

from .driver import DriverConfig, FaultActiveError, WristBldcDriver, WristError
from .sim import SimulatedBoard


@dataclass
class Result:
    name: str
    passed: Optional[bool]  # None = saute
    details: Dict[str, Any] = field(default_factory=dict)


class Bench:
    def __init__(
        self,
        driver: WristBldcDriver,
        args: argparse.Namespace,
        board: Optional[SimulatedBoard],
    ):
        self.drv = driver
        self.args = args
        self.board = board
        self.results: List[Result] = []

    # ---------------------------------------------------------------- outils

    def ask(self, question: str) -> bool:
        if self.args.yes:
            print(f"{question} [o/N] o (auto)")
            return True
        return input(f"{question} [o/N] ").strip().lower() in ("o", "oui", "y", "yes")

    def record(self, result: Result) -> None:
        mark = {True: "OK  ", False: "ECHEC", None: "saute"}[result.passed]
        print(f"  -> {mark} {result.name} {json.dumps(result.details, default=str)}")
        self.results.append(result)

    def move_and_track(
        self, target: float, vmax: float, timeout: float = 20.0
    ) -> Dict[str, Any]:
        """Mouvement en rampe ; echantillonne l'ecart consigne/mesure et le courant."""
        self.drv.move_to(target, max_velocity=vmax)
        errors, currents = [], []
        deadline = time.monotonic() + timeout
        reached = False
        while time.monotonic() < deadline:
            s = self.drv.get_status()
            if s.fault:
                raise FaultActiveError(s.blocking_reason or "defaut")
            currents.append(abs(s.current))
            if s.position is not None and s.board_target is not None:
                errors.append(abs(s.board_target - s.position))
                # rampe terminee (derniere consigne = cible) ET axe arrive
                if (
                    abs(s.board_target - target) < 1e-6
                    and abs(s.position - target) < 0.005
                ):
                    reached = True
                    break
            time.sleep(0.02)
        time.sleep(0.3)  # stabilisation
        final = self.drv.get_state().position
        return {
            "reached": reached,
            "final_error": abs((final or 0.0) - target),
            "max_tracking_error": max(errors, default=0.0),
            "max_current": max(currents, default=0.0),
        }

    # ---------------------------------------------------------------- essais

    def t1_link(self) -> None:
        s0 = self.drv.get_status()
        time.sleep(2.0)
        s1 = self.drv.get_status()
        rate = (s1.frames - s0.frames) / 2.0
        self.record(
            Result(
                "T1 liaison 50 Hz",
                40.0 <= rate <= 60.0,
                {"port": s1.port, "trames_par_s": rate},
            )
        )

    def t2_rest(self) -> None:
        samples = []
        for _ in range(50):
            samples.append(abs(self.drv.get_state().current))
            time.sleep(0.02)
        s = self.drv.get_status()
        ok = max(samples) < 0.5 and not s.fault
        self.record(
            Result("T2 repos", ok, {"courant_max": max(samples), "defaut": s.fault})
        )

    def t3_nest(self) -> None:
        status = self.drv.initialize_at_nest(self.args.nest)
        read_back = self.drv.query_position()
        ok = status.homed and abs(read_back - self.args.nest) < 1e-3
        self.record(
            Result("T3 recalage nid", ok, {"nid": self.args.nest, "relu": read_back})
        )

    def t4_small_move(self) -> None:
        if not self.ask("T4 : petit mouvement de +0.1 rad puis retour. Zone libre ?"):
            return self.record(Result("T4 petit mouvement", None))
        out = self.move_and_track(self.args.nest + 0.1, vmax=0.2)
        back = self.move_and_track(self.args.nest, vmax=0.2)
        ok = all(
            r["reached"] and r["final_error"] < 0.01 and r["max_tracking_error"] < 0.1
            for r in (out, back)
        )
        self.record(Result("T4 petit mouvement", ok, {"aller": out, "retour": back}))

    def t5_range(self) -> None:
        low, high = self.args.low, self.args.high
        if not self.ask(
            f"T5 : debattement {low:+.2f} -> {high:+.2f} rad a 0.3 rad/s. "
            "Debattement libre ?"
        ):
            return self.record(Result("T5 debattement", None))
        legs = {
            name: self.move_and_track(t, vmax=0.3)
            for name, t in (("bas", low), ("haut", high))
        }
        legs["nid"] = self.move_and_track(self.args.nest, vmax=0.3)
        ok = all(r["reached"] and r["final_error"] < 0.01 for r in legs.values())
        self.record(Result("T5 debattement", ok, legs))

    def t6_repeatability(self) -> None:
        if not self.ask("T6 : 5 allers-retours nid <-> nid+0.5 rad. OK ?"):
            return self.record(Result("T6 repetabilite (codeur)", None))
        at_nest = []
        for _ in range(5):
            self.move_and_track(self.args.nest + 0.5, vmax=0.4)
            self.move_and_track(self.args.nest, vmax=0.4)
            at_nest.append(self.drv.get_state().position or 0.0)
        spread = max(at_nest) - min(at_nest)
        self.record(
            Result(
                "T6 repetabilite (codeur)",
                spread < 0.005,
                {
                    "ecart_max": spread,
                    "note": "verifier aussi visuellement le repere mecanique",
                },
            )
        )

    def t7_stall(self) -> None:
        if self.board is not None:
            self.board.set_obstacle(-10.0, self.args.nest + 0.1)
        elif not self.ask(
            "T7 : BLOQUEZ l'axe a la main (ou butee douce), puis validez. Pret ?"
        ):
            return self.record(Result("T7 blocage", None))
        start = time.monotonic()
        try:
            self.drv.move_to(
                min(self.args.nest + 0.4, self.args.high), max_velocity=0.2
            )
        except WristError as exc:
            return self.record(Result("T7 blocage", False, {"erreur": str(exc)}))
        detected = False
        while time.monotonic() - start < 5.0:
            if self.drv.get_state().fault:
                detected = True
                break
            time.sleep(0.02)
        delay = time.monotonic() - start
        time.sleep(0.5)
        s = self.drv.get_status()
        freed = abs(s.current) < 1.0
        if self.board is not None:
            self.board.clear_obstacle()
        elif detected:
            self.ask("Relachez l'axe. Fait ?")
        if detected:
            self.drv.reset_fault()
            self.move_and_track(self.args.nest, vmax=0.2)
        details = {
            "detecte": detected,
            "delai_s": round(delay, 2),
            "defaut_suivi": s.tracking_fault,
            "defaut_carte": s.board_fault,
            "courant_apres": s.current,
        }
        self.record(Result("T7 blocage", detected and freed, details))

    def t8_unplug(self) -> None:
        if self.board is not None:
            self.board.unplug()
            time.sleep(0.5)
            self.board.replug()
        elif self.ask(
            "T8 : DEBRANCHEZ puis REBRANCHEZ le cable USB (pas l'alim 24 V). "
            "Pret a le faire ?"
        ):
            print("     ... debranchez, attendez 2 s, rebranchez. (30 s max)")
            deadline = time.monotonic() + 30.0
            while self.drv.get_status().connected and time.monotonic() < deadline:
                time.sleep(0.05)
        else:
            return self.record(Result("T8 debranchement USB", None))
        try:
            self.drv.wait_connected(30.0)
        except WristError as exc:
            return self.record(
                Result("T8 debranchement USB", False, {"erreur": str(exc)})
            )
        s = self.drv.get_status()
        self.record(
            Result("T8 debranchement USB", s.homed and not s.fault, {"recale": s.homed})
        )

    def t9_latency(self) -> None:
        delays = []
        for _ in range(20):
            start = time.monotonic()
            self.drv.query_position()
            delays.append((time.monotonic() - start) * 1000.0)
        details = {
            "mediane_ms": round(statistics.median(delays), 1),
            "max_ms": round(max(delays), 1),
        }
        self.record(Result("T9 latence aller-retour", max(delays) < 50.0, details))

    def t10_back_to_nest(self) -> None:
        if not self.ask("T10 : retour au nid. Zone libre ?"):
            return self.record(Result("T10 retour nid", None))
        r = self.move_and_track(self.args.nest, vmax=0.2)
        seated = self.args.yes or self.ask(
            "La tete rentre-t-elle PROPREMENT dans le nid (sans forcer) ?"
        )
        self.record(
            Result(
                "T10 retour nid",
                r["reached"] and seated,
                {**r, "entre_dans_le_nid": seated},
            )
        )

    def t11_power_cycle_in_nest(self) -> None:
        """Coupure 24 V tete au nid : reboot detecte, puis re-init AU NID.

        Valide aussi l'alignement capteur de initFOC (le moteur bouge d'environ
        1 deg en sortie au boot) alors que la tete est tenue par le nid.
        """
        if self.board is not None:
            self.board.reboot(emit_ready=True)
        elif not self.ask(
            "T11 : tete AU NID. COUPEZ le 24 V ~3 s puis RETABLISSEZ-le "
            "(USB branche). Pret ?"
        ):
            return self.record(Result("T11 coupure 24 V au nid", None))
        lost = False
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            if not self.drv.get_status().homed:
                lost = True
                break
            time.sleep(0.05)
        blocked = False
        try:
            self.drv.move_to(self.args.nest)
        except WristError:
            blocked = True
        time.sleep(3.0)  # la carte attend 2 s + initFOC avant READY
        try:
            status = self.drv.initialize_at_nest(self.args.nest, timeout=20.0)
            reinit = status.homed
        except WristError as exc:
            reinit = False
            print(f"     re-init au nid impossible : {exc}")
        details = {
            "reboot_detecte": lost,
            "mouvement_bloque_avant_recalage": blocked,
            "reinit_au_nid": reinit,
        }
        self.record(
            Result("T11 coupure 24 V au nid", lost and blocked and reinit, details)
        )

    def run(self) -> List[Result]:
        steps: List[Callable[[], None]] = [
            self.t1_link,
            self.t2_rest,
            self.t3_nest,
            self.t4_small_move,
            self.t5_range,
            self.t6_repeatability,
            self.t7_stall,
            self.t8_unplug,
            self.t9_latency,
            self.t10_back_to_nest,
            self.t11_power_cycle_in_nest,
        ]
        for step in steps:
            print(f"\n== {step.__name__}")
            try:
                step()
            except (WristError, ValueError) as exc:
                self.record(Result(step.__name__, False, {"erreur": str(exc)}))
                if step == self.t3_nest:  # methodes liees : == et non `is`
                    print("Recalage impossible : arret de la procedure.")
                    break
        return self.results


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Reception de l'axe 5 (poignet BLDC) au banc"
    )
    parser.add_argument("--port", default="auto")
    parser.add_argument(
        "--nest",
        type=float,
        default=0.04700,   # nid re-etiquete 2026-09-20
        help="position du nid (rad), cf initial_positions.yaml",
    )
    parser.add_argument(
        "--low",
        type=float,
        default=-1.0,
        help="borne basse du debattement a tester (rad)",
    )
    parser.add_argument(
        "--high",
        type=float,
        default=1.0,
        help="borne haute du debattement a tester (rad)",
    )
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument(
        "--yes", action="store_true", help="repond oui a tout (simulation uniquement !)"
    )
    parser.add_argument("--report", default=None, help="chemin du rapport JSON")
    args = parser.parse_args(argv)
    if args.yes and not args.simulate:
        parser.error("--yes n'est autorise qu'avec --simulate")
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")

    board = SimulatedBoard(start_position=0.0) if args.simulate else None
    config = DriverConfig(port="sim" if board else args.port)
    driver = (
        WristBldcDriver(config, serial_factory=board.factory)
        if board
        else WristBldcDriver(config)
    )
    with driver:
        try:
            driver.wait_connected(5.0)
        except WristError as exc:
            print(f"carte injoignable : {exc}")
            return 2
        if board is not None:
            board.stall_time_s = 0.5
        results = Bench(driver, args, board).run()

    report = args.report or f"wrist_bench_{datetime.now():%Y%m%d_%H%M%S}.json"
    with open(report, "w", encoding="utf-8") as fh:
        json.dump(
            {
                "date": datetime.now().isoformat(timespec="seconds"),
                "simulation": args.simulate,
                "nest": args.nest,
                "results": [r.__dict__ for r in results],
            },
            fh,
            indent=2,
            default=str,
        )
    failed = [r.name for r in results if r.passed is False]
    print(f"\nRapport : {report}")
    print("RECEPTION OK" if not failed else f"ECHECS : {', '.join(failed)}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
