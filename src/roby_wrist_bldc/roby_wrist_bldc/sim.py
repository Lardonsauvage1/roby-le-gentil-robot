"""Carte B-G431B-ESC1 simulee : emule le firmware SimpleFOC de l'axe 5.

Reproduit, derriere l'interface d'un port serie (read / write / in_waiting),
le comportement de test_bldc/src/main.cpp :

- trame "S <pos> <courant> <defaut>" toutes les 20 ms ;
- commandes P, Z, S, E, R, ? et leurs reponses, y compris les pieges du
  firmware (atof() : "P" sans nombre vise 0.0 ; buffer de 31 caracteres) ;
- asservissement de position simplifie : v = 8 * erreur, |v| <= 1.67 rad/s
  bras (P_angle.P = 8, velocity_limit = 15 rad/s moteur / 9) ;
- watchdog blocage : |I| > 2.5 A pendant 2 s => "FAULT STALL", moteur coupe.

Sert aux tests (sans materiel) et au mode `simulate` du noeud ROS. Des
methodes d'injection permettent de provoquer obstacle, debranchement, reboot.
"""

from __future__ import annotations

import re
import threading
import time
from typing import List, Optional, Tuple

_ATOF = re.compile(r"\s*[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?")


def c_atof(text: str) -> float:
    """Comme atof() en C : lit le plus long prefixe numerique, 0.0 sinon."""
    match = _ATOF.match(text)
    return float(match.group(0)) if match else 0.0


class SimulatedBoard:
    """Emulation temps reel (horloge murale) de la carte, pas de thread interne."""

    GEAR_RATIO = 9.0

    def __init__(
        self,
        start_position: float = 0.0,
        status_period_s: float = 0.02,
        p_gain: float = 8.0,
        velocity_limit: float = 15.0 / 9.0,
        stall_current: float = 2.5,
        stall_time_s: float = 2.0,
        emit_ready: bool = True,
    ) -> None:
        self._lock = threading.Lock()
        self.status_period_s = status_period_s
        self.p_gain = p_gain
        self.velocity_limit = velocity_limit
        self.stall_current = stall_current
        self.stall_time_s = stall_time_s

        # position_bras = shaft_angle / GEAR_RATIO + offset_bras
        # ici `raw` = shaft_angle / 9
        self.raw = start_position
        self.offset = 0.0
        self.target = start_position
        self.enabled = True
        self.fault = False
        self.current = 0.0
        self._stall_active = False
        self._stall_start = 0.0

        self._obstacle: Optional[Tuple[float, float]] = None
        self._unplugged = False
        self._silent = False
        self.is_open = True
        self.received: List[str] = []

        self._out = bytearray()
        self._cmd = bytearray()
        self._t = time.monotonic()
        self._next_status = self._t
        if emit_ready:
            self._out += b"READY\r\n"

    # ---------------------------------------------------------------- interface "serie"

    def factory(self, port: str, baudrate: int) -> "SimulatedBoard":
        """A passer comme serial_factory au driver : (re)ouvre CETTE carte."""
        with self._lock:
            if self._unplugged:
                raise OSError(f"[sim] {port} : peripherique absent")
            self.is_open = True
        return self

    @property
    def in_waiting(self) -> int:
        with self._lock:
            self._check_plugged()
            self._advance(time.monotonic())
            return len(self._out)

    def read(self, size: int = 1, timeout: float = 0.05) -> bytes:
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                self._check_plugged()
                now = time.monotonic()
                self._advance(now)
                if self._out:
                    data = bytes(self._out[:size])
                    del self._out[:size]
                    return data
                wait = min(deadline, self._next_status) - now
            if time.monotonic() >= deadline:
                return b""
            time.sleep(max(wait, 0.001))

    def write(self, data: bytes) -> int:
        with self._lock:
            self._check_plugged()
            self._advance(time.monotonic())
            for byte in data:
                char = chr(byte)
                if char in "\r\n":
                    if self._cmd:
                        self._execute(self._cmd.decode("ascii", errors="replace"))
                        self._cmd.clear()
                elif len(self._cmd) < 31:
                    self._cmd.append(byte)
        return len(data)

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._out.clear()

    def close(self) -> None:
        with self._lock:
            self.is_open = False

    # ------------------------------------------------------- injection de pannes

    @property
    def position(self) -> float:
        with self._lock:
            return self.raw + self.offset

    def set_obstacle(self, low: float, high: float) -> None:
        """Butee physique : la position (repere carte) reste dans [low, high]."""
        with self._lock:
            self._obstacle = (low, high)

    def clear_obstacle(self) -> None:
        with self._lock:
            self._obstacle = None

    def unplug(self) -> None:
        with self._lock:
            self._unplugged = True

    def replug(self) -> None:
        with self._lock:
            self._unplugged = False

    def set_silent(self, silent: bool) -> None:
        """Carte figee : plus aucune trame (mais le port reste ouvert)."""
        with self._lock:
            self._silent = silent

    def reboot(self, emit_ready: bool = True) -> None:
        """Redemarrage : offset de recalage perdu, consigne = position actuelle."""
        with self._lock:
            self.offset = 0.0
            self.target = self.raw
            self.enabled = True
            self.fault = False
            self._stall_active = False
            self._out.clear()
            if emit_ready:
                self._out += b"READY\r\n"

    # ---------------------------------------------------------------- firmware

    def _check_plugged(self) -> None:
        if self._unplugged or not self.is_open:
            raise OSError("[sim] port ferme ou peripherique debranche")

    def _execute(self, cmd: str) -> None:
        self.received.append(cmd)
        kind, val = cmd[0], c_atof(cmd[1:])
        position = self.raw + self.offset
        if kind == "P":
            self.target = val
        elif kind == "Z":
            self.offset = val - self.raw
            self.target = val
            self._out += f"RECALE {val:.4f}\r\n".encode()
        elif kind == "S":
            self.enabled = False
            self._out += b"DISABLED\r\n"
        elif kind == "E":
            self.enabled = True
            self.target = position
            self._out += b"ENABLED\r\n"
        elif kind == "R":
            self.fault = False
            self._stall_active = False
            self.enabled = True
            self.target = position
            self._out += b"RESET\r\n"
        elif kind == "?":
            self._out += f"POS {position:.4f}\r\n".encode()

    def _advance(self, now: float) -> None:
        dt = 0.001
        if now - self._t > 1.0:  # longue pause (debogueur...) : pas de rattrapage fou
            self._t = now - 1.0
        while self._t + dt <= now:
            self._t += dt
            self._physics(dt)
            if self._t >= self._next_status:
                self._next_status += self.status_period_s
                if not self._silent:
                    position = self.raw + self.offset
                    frame = f"S {position:.4f} {self.current:.2f} {int(self.fault)}\r\n"
                    self._out += frame.encode()

    def _physics(self, dt: float) -> None:
        pushing = False
        if self.enabled and not self.fault:
            error = self.target - (self.raw + self.offset)
            velocity = max(
                -self.velocity_limit, min(self.velocity_limit, self.p_gain * error)
            )
            new_raw = self.raw + velocity * dt
            if self._obstacle is not None:
                low, high = (bound - self.offset for bound in self._obstacle)
                clamped = max(low, min(high, new_raw))
                pushing = clamped != new_raw and abs(error) > 0.01
                new_raw = clamped
            self.raw = new_raw
            self.current = (
                (self.stall_current + 0.5) if pushing else 0.05 + 0.5 * abs(velocity)
            )
        else:
            self.current = 0.0

        if not self.fault:
            if abs(self.current) > self.stall_current:
                if not self._stall_active:
                    self._stall_active = True
                    self._stall_start = self._t
                elif self._t - self._stall_start > self.stall_time_s:
                    self.fault = True
                    self.enabled = False
                    self._out += b"FAULT STALL\r\n"
            else:
                self._stall_active = False
