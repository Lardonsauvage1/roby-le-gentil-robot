"""Driver Python de l'axe 5 (poignet BLDC) : carte B-G431B-ESC1 / SimpleFOC, USB serie.

Utilisation type (sans ROS) ::

    from roby_wrist_bldc.driver import WristBldcDriver, DriverConfig

    with WristBldcDriver(DriverConfig(port="auto")) as wrist:
        wrist.initialize_at_nest(0.2009)      # tete posee dans le nid
        wrist.move_to(0.8)                    # rampe 100 Hz, non bloquant
        wrist.wait_until_reached(timeout=5.0)
        position, courant, defaut = wrist.get_state()

Deux threads :

- E/S : (re)connexion, lecture des trames "S <pos> <courant> <defaut>" et des
  reponses, mise a jour de l'etat sous verrou ;
- streaming : a 100 Hz, fait avancer la rampe (SetpointLimiter) vers la cible
  et envoie les consignes P. Il s'arrete tout seul des qu'un defaut, une perte
  de liaison ou une perte de recalage est detecte : l'appelant decide ensuite
  (reset_fault(), recalibrate(), ...).

Le driver ne coupe jamais le moteur de lui-meme : la carte garde la derniere
consigne (le poignet tient sa position). Sur defaut blocage, c'est la carte qui
coupe.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, List, NamedTuple, Optional, Protocol, Tuple, Type

from . import protocol
from .limiter import SetpointLimiter
from .ports import PortNotFoundError, find_port

LOGGER = logging.getLogger("roby_wrist_bldc")

# La carte affiche 4 decimales : un accuse RECALE/POS est "egal" a la valeur
# envoyee a cette tolerance pres (arrondi d'affichage + float32 de la carte).
ACK_VALUE_TOLERANCE = 1e-3


class WristError(RuntimeError):
    """Erreur generique du driver."""


class AckTimeoutError(WristError):
    """La carte n'a pas accuse reception d'une commande a temps."""


class MotionBlockedError(WristError):
    """Mouvement refuse : l'etat courant ne permet pas d'envoyer de consigne."""


class NotConnectedError(MotionBlockedError):
    """Pas de liaison, ou plus de trame d'etat recente."""


class FaultActiveError(MotionBlockedError):
    """Defaut actif (blocage carte ou ecart de suivi) : reset_fault() requis."""


class NotHomedError(MotionBlockedError):
    """Position non recalee (demarrage, reboot carte) : recalibrate() requis."""


class MotorDisabledError(MotionBlockedError):
    """Moteur desactive (S) : enable() requis, la carte ignorerait la consigne."""


class SerialLike(Protocol):
    """Sous-ensemble de serial.Serial utilise par le driver (pyserial ou simulateur)."""

    @property
    def in_waiting(self) -> int: ...

    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> Optional[int]: ...

    def reset_input_buffer(self) -> None: ...

    def close(self) -> None: ...


SerialFactory = Callable[[str, int], SerialLike]
Listener = Callable[[str, str], None]


def open_pyserial(port: str, baudrate: int) -> SerialLike:
    """Ouvre le port reel. exclusive=True : refuse un port deja ouvert ailleurs."""
    import serial

    port_handle: SerialLike = serial.Serial(
        port, baudrate, timeout=0.05, write_timeout=0.1, exclusive=True
    )
    return port_handle


@dataclass
class DriverConfig:
    """Parametres du driver. Positions et vitesses en unites de l'AXE du bras."""

    port: str = "auto"
    baudrate: int = 115200
    # Butees logicielles (URDF joint_5 : [-1.6, 1.6] rad).
    position_min: float = -1.6
    position_max: float = 1.6
    # Rampe. La carte plafonne a velocity_limit = 15 rad/s moteur = 0.75 rad/s bras ;
    # on reste en dessous pour qu'elle suive la consigne sans trainer.
    max_velocity: float = 0.5
    max_acceleration: float = 1.5
    stream_rate_hz: float = 100.0
    # Consigne renvoyee meme inchangee a cet intervalle pendant un mouvement :
    # detecte au plus tot une ecriture qui echoue.
    keepalive_s: float = 0.2
    # La carte emet a 50 Hz : 10 trames manquees = liaison consideree perdue.
    status_timeout_s: float = 0.2
    # Un port ouvert qui n'emet pas de trame "S" dans ce delai n'est pas la carte.
    handshake_timeout_s: float = 1.0
    reconnect_period_s: float = 1.0
    ack_timeout_s: float = 0.5
    # Refuse tout mouvement tant que la position n'a pas ete recalee (Z).
    require_homing: bool = True
    # Ecart consigne/mesure au-dela duquel le Pi fige l'axe (0 = desactive).
    # Nominal : ~0.06 rad a 0.5 rad/s (gain P_angle = 8). La carte a son propre
    # watchdog (2.5 A pendant 2 s) : ceci arrete de pousser plus tot.
    tracking_error_limit: float = 0.35
    tracking_error_time_s: float = 0.5
    # A la reconnexion, un saut de position plus grand que ceci => recalage perdu.
    reconnect_position_tolerance: float = 0.05


class WristState(NamedTuple):
    """Etat instantane, deballable : ``position, courant, defaut = drv.get_state()``."""

    position: Optional[float]
    current: float
    fault: bool


@dataclass(frozen=True)
class WristStatus:
    """Etat complet du driver, pour l'appelant et le diagnostic."""

    position: Optional[float]
    current: float
    board_fault: bool
    tracking_fault: bool
    connected: bool
    fresh: bool
    homed: bool
    enabled: Optional[bool]
    streaming: bool
    board_target: Optional[float]
    target: Optional[float]
    port: Optional[str]
    age_s: Optional[float]
    board_reboots: int
    frames: int
    blocking_reason: Optional[str]

    @property
    def fault(self) -> bool:
        return self.board_fault or self.tracking_fault

    @property
    def motion_allowed(self) -> bool:
        return self.blocking_reason is None


class _Waiter:
    def __init__(
        self,
        kind: protocol.EventKind,
        predicate: Optional[Callable[[protocol.Event], bool]],
    ):
        self.kind = kind
        self.predicate = predicate
        self.done = threading.Event()
        self.result: Optional[protocol.Event] = None


class WristBldcDriver:
    """Driver de l'axe 5. Thread-safe : toutes les methodes publiques."""

    def __init__(
        self,
        config: Optional[DriverConfig] = None,
        serial_factory: SerialFactory = open_pyserial,
        port_finder: Callable[[str], str] = find_port,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.config = config or DriverConfig()
        self._serial_factory = serial_factory
        self._port_finder = port_finder
        self._log = logger or LOGGER

        # Ordre de verrouillage : _write_lock PUIS _lock, jamais l'inverse.
        # _write_lock serialise les envois : une consigne P calculee avant un
        # recalage Z ne peut pas partir apres lui (elle serait dans l'ancien repere).
        self._write_lock = threading.Lock()
        self._lock = threading.Lock()
        self._frame_cond = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._listeners: List[Listener] = []

        # --- etat protege par _lock ---
        self._serial: Optional[SerialLike] = None
        self._port: Optional[str] = None
        self._connected = False
        self._handshake_deadline = 0.0
        self._disconnect_request: Optional[str] = None
        self._last_frame_t: Optional[float] = None
        self._position: Optional[float] = None
        self._current = 0.0
        self._board_fault = False
        self._tracking_fault = False
        self._homed = False
        self._enabled: Optional[bool] = None
        self._board_reboots = 0
        self._frames = 0
        self._last_position_before_loss: Optional[float] = None
        self._waiters: List[_Waiter] = []
        # Meilleure connaissance de la consigne que la carte poursuit en ce moment.
        self._board_target: Optional[float] = None
        self._streaming = False
        self._limiter = SetpointLimiter(
            self.config.max_velocity, self.config.max_acceleration
        )
        self._last_sent: Optional[float] = None
        self._last_send_t = 0.0
        self._tracking_since: Optional[float] = None
        self._stale_reported = False
        self._last_connect_error: Optional[str] = None
        self._last_connect_log_t = 0.0

    # ------------------------------------------------------------------ cycle de vie

    def start(self) -> "WristBldcDriver":
        """Demarre les threads (connexion en arriere-plan, non bloquant)."""
        if self._threads:
            return self
        self._stop.clear()
        for name, target in (
            ("wrist-bldc-io", self._io_loop),
            ("wrist-bldc-stream", self._stream_loop),
        ):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)
        return self

    def close(self) -> None:
        """Arrete les threads et ferme le port. Le moteur n'est PAS coupe (il tient)."""
        with self._lock:
            self._streaming = False
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads.clear()
        with self._lock:
            self._close_serial_locked()

    def __enter__(self) -> "WristBldcDriver":
        return self.start()

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def add_listener(self, listener: Listener) -> None:
        """Abonne `listener(kind, message)` aux evenements (defaut, deconnexion, ...).

        Appele depuis les threads du driver : doit rendre la main vite.
        Evenements : connected, disconnected, link_lost, link_restored, fault,
        tracking_fault, fault_cleared, board_reboot, homing_lost, homed,
        stream_stopped.
        """
        self._listeners.append(listener)

    # ------------------------------------------------------------------ etat

    def get_state(self) -> WristState:
        """(position rad bras, courant A, defaut).

        defaut = blocage carte OU ecart de suivi.
        """
        with self._lock:
            return WristState(
                self._position, self._current, self._board_fault or self._tracking_fault
            )

    def get_status(self) -> WristStatus:
        now = time.monotonic()
        with self._lock:
            reason = self._blocking_reason_locked(now)
            age = None if self._last_frame_t is None else now - self._last_frame_t
            return WristStatus(
                position=self._position,
                current=self._current,
                board_fault=self._board_fault,
                tracking_fault=self._tracking_fault,
                connected=self._connected,
                fresh=self._is_fresh_locked(now),
                homed=self._homed,
                enabled=self._enabled,
                streaming=self._streaming,
                board_target=self._board_target,
                target=self._limiter.target if self._streaming else None,
                port=self._port,
                age_s=age,
                board_reboots=self._board_reboots,
                frames=self._frames,
                blocking_reason=reason[1] if reason else None,
            )

    def wait_connected(self, timeout: float) -> None:
        """Attend une trame d'etat fraiche. Leve NotConnectedError a l'echeance."""
        deadline = time.monotonic() + timeout
        with self._frame_cond:
            while not self._is_fresh_locked(time.monotonic()):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise NotConnectedError(self._link_description_locked())
                self._frame_cond.wait(min(remaining, 0.05))

    # ------------------------------------------------------------------ commandes

    def set_position(self, angle_rad: float) -> None:
        """Envoie immediatement P<angle> (sans rampe). Interrompt un mouvement en cours.

        La carte limite elle-meme la vitesse (0.75 rad/s bras) mais sans rampe
        d'acceleration cote Pi : preferer move_to() pour les grands deplacements.
        """
        self._check_limits(angle_rad)
        with self._write_lock:
            with self._lock:
                self._raise_if_blocked_locked(time.monotonic())
                self._streaming = False
            self._write_locked(protocol.encode_position(angle_rad))
            with self._lock:
                self._note_sent_locked(angle_rad)

    def move_to(
        self,
        target_rad: float,
        max_velocity: Optional[float] = None,
        max_acceleration: Optional[float] = None,
    ) -> None:
        """Rejoint `target_rad` par une rampe envoyee a 100 Hz. Non bloquant.

        Peut etre rappele a tout moment (y compris a 100 Hz par un flux de
        consignes) : la rampe se re-oriente sans a-coup. Vitesse/acceleration
        optionnelles, plafonnees par la configuration.
        """
        self._check_limits(target_rad)
        cfg = self.config
        vmax = (
            cfg.max_velocity
            if max_velocity is None
            else min(max_velocity, cfg.max_velocity)
        )
        amax = (
            cfg.max_acceleration
            if max_acceleration is None
            else min(max_acceleration, cfg.max_acceleration)
        )
        if vmax <= 0.0 or amax <= 0.0:
            raise ValueError("max_velocity et max_acceleration doivent etre > 0")
        with self._lock:
            self._raise_if_blocked_locked(time.monotonic())
            self._limiter.max_velocity = vmax
            self._limiter.max_acceleration = amax
            if not self._streaming:
                seed = (
                    self._board_target
                    if self._board_target is not None
                    else self._position
                )
                assert (
                    seed is not None
                )  # garanti par _raise_if_blocked_locked (trame recue)
                self._limiter.reset(seed)
                self._tracking_since = None
                self._streaming = True
            self._limiter.set_target(target_rad)

    def stop_motion(self) -> None:
        """Freine selon la rampe et s'arrete (le moteur reste asservi)."""
        with self._lock:
            if not self._streaming:
                return
            v = self._limiter.velocity
            target = self._limiter.target
            stop_at = self._limiter.position + v * abs(v) / (
                2.0 * self._limiter.max_acceleration
            )
            # Si la cible est deja plus proche que le point d'arret, on la garde.
            if (target - stop_at) * v > 0.0 or v == 0.0:
                stop_at = min(
                    max(stop_at, self.config.position_min), self.config.position_max
                )
                self._limiter.set_target(stop_at)

    def wait_until_reached(self, timeout: float, tolerance: float = 0.01) -> bool:
        """Attend la fin de la rampe ET la mesure a `tolerance` de la cible.

        Renvoie False a l'echeance. Leve MotionBlockedError si le mouvement est
        interrompu (defaut, perte de liaison, ...).
        """
        deadline = time.monotonic() + timeout
        with self._frame_cond:
            while True:
                now = time.monotonic()
                self._raise_if_blocked_locked(now)
                if not self._streaming:
                    raise MotionBlockedError("aucun mouvement en cours (stream arrete)")
                target = self._limiter.target
                if (
                    self._limiter.at_target()
                    and self._position is not None
                    and abs(self._position - target) <= tolerance
                ):
                    return True
                if now >= deadline:
                    return False
                self._frame_cond.wait(min(deadline - now, 0.05))

    def recalibrate(self, angle_rad: float) -> None:
        """Envoie Z<angle> : "l'axe est actuellement a angle" (aucun mouvement).

        A n'appeler que si la position mecanique est connue (tete dans le nid).
        """
        self._check_limits(angle_rad)

        def before_send() -> None:
            self._streaming = False

        self._command(
            protocol.encode_recalibrate(angle_rad),
            protocol.EventKind.RECALE,
            predicate=lambda ev: ev.value is not None
            and abs(ev.value - angle_rad) <= ACK_VALUE_TOLERANCE,
            before_send=before_send,
        )

    def enable(self) -> None:
        """E : la carte reprend sa position actuelle comme consigne (pas de saut)."""
        self._command(
            protocol.encode_enable(),
            protocol.EventKind.ENABLED,
            before_send=self._stop_stream_locked,
        )

    def disable(self) -> None:
        """S : moteur coupe (roue libre : le poignet peut tomber sous la charge)."""
        self._command(
            protocol.encode_stop(),
            protocol.EventKind.DISABLED,
            before_send=self._stop_stream_locked,
        )

    def reset_fault(self) -> None:
        """R : efface le defaut, re-active le moteur sur sa position actuelle."""
        self._command(
            protocol.encode_reset(),
            protocol.EventKind.RESET,
            before_send=self._stop_stream_locked,
        )

    def query_position(self) -> float:
        """? : position lue par la carte (rad bras)."""
        event = self._command(protocol.encode_query(), protocol.EventKind.POS)
        assert event.value is not None
        return event.value

    def initialize_at_nest(
        self, nest_position: float, enable: bool = True, timeout: float = 5.0
    ) -> WristStatus:
        """Sequence de demarrage, tete posee dans le nid (position mecanique connue).

        1. attend la liaison ; 2. refuse de continuer si un defaut est actif
        (l'appelant decide du reset) ; 3. Z<nest> ; 4. verifie la position
        relue ; 5. E (re-active le moteur sur place, sans mouvement).
        """
        self.wait_connected(timeout)
        with self._lock:
            if self._board_fault:
                raise FaultActiveError(
                    "defaut blocage actif sur la carte : "
                    "verifier l'axe puis reset_fault()"
                )
        self.recalibrate(nest_position)
        read_back = self.query_position()
        if abs(read_back - nest_position) > ACK_VALUE_TOLERANCE:
            raise WristError(
                f"recalage non confirme : relu {read_back:.4f}, "
                f"attendu {nest_position:.4f}"
            )
        if enable:
            self.enable()
        self._log.info("poignet recale au nid : %.4f rad", nest_position)
        return self.get_status()

    # ------------------------------------------------------------ interne : commandes

    def _stop_stream_locked(self) -> None:
        self._streaming = False

    def _check_limits(self, angle_rad: float) -> None:
        cfg = self.config
        if not cfg.position_min <= angle_rad <= cfg.position_max:
            raise ValueError(
                f"{angle_rad:.4f} rad hors butees logicielles "
                f"[{cfg.position_min}, {cfg.position_max}]"
            )

    def _command(
        self,
        payload: bytes,
        ack: protocol.EventKind,
        predicate: Optional[Callable[[protocol.Event], bool]] = None,
        before_send: Optional[Callable[[], None]] = None,
    ) -> protocol.Event:
        waiter = _Waiter(ack, predicate)
        with self._write_lock:
            with self._lock:
                if not self._is_fresh_locked(time.monotonic()):
                    raise NotConnectedError(self._link_description_locked())
                if before_send is not None:
                    before_send()
                self._waiters.append(waiter)
            try:
                self._write_locked(payload)
            except WristError:
                self._drop_waiter(waiter)
                raise
        if not waiter.done.wait(self.config.ack_timeout_s):
            self._drop_waiter(waiter)
            raise AckTimeoutError(f"pas de reponse {ack.value} a {payload!r}")
        assert waiter.result is not None
        return waiter.result

    def _drop_waiter(self, waiter: _Waiter) -> None:
        with self._lock:
            if waiter in self._waiters:
                self._waiters.remove(waiter)

    def _write_locked(self, payload: bytes) -> None:
        """Ecrit sur le port. Appelant : detient _write_lock (pas _lock)."""
        with self._lock:
            ser = self._serial if self._connected else None
        if ser is None:
            raise NotConnectedError("port serie non connecte")
        try:
            ser.write(payload)
        except (
            Exception
        ) as exc:  # pyserial : SerialException, OSError, timeout d'ecriture...
            with self._lock:
                self._disconnect_request = f"ecriture impossible : {exc}"
            raise NotConnectedError(f"ecriture impossible : {exc}") from exc

    def _note_sent_locked(self, value: float) -> None:
        self._board_target = value
        self._last_sent = value
        self._last_send_t = time.monotonic()

    # ------------------------------------------------------------------ interne : etat

    def _is_fresh_locked(self, now: float) -> bool:
        return (
            self._connected
            and self._last_frame_t is not None
            and now - self._last_frame_t <= self.config.status_timeout_s
        )

    def _link_description_locked(self) -> str:
        if not self._connected:
            reason = self._last_connect_error or "connexion en cours"
            return f"carte non connectee ({reason})"
        age = time.monotonic() - (self._last_frame_t or 0.0)
        return f"plus de trame d'etat depuis {age * 1000:.0f} ms"

    def _blocking_reason_locked(
        self, now: float
    ) -> Optional[Tuple[Type[MotionBlockedError], str]]:
        if not self._is_fresh_locked(now):
            return NotConnectedError, self._link_description_locked()
        if self._board_fault:
            return (
                FaultActiveError,
                "defaut blocage actif (carte) : reset_fault() requis",
            )
        if self._tracking_fault:
            return (
                FaultActiveError,
                "ecart de suivi consigne/mesure : axe fige, reset_fault() requis",
            )
        if self.config.require_homing and not self._homed:
            return NotHomedError, "position non recalee : recalibrate() au nid requis"
        if self._enabled is False:
            return MotorDisabledError, "moteur desactive : enable() requis"
        return None

    def _raise_if_blocked_locked(self, now: float) -> None:
        reason = self._blocking_reason_locked(now)
        if reason is not None:
            exc_type, message = reason
            raise exc_type(message)

    def _emit(self, events: List[Tuple[str, str]]) -> None:
        """Journalise et notifie. Toujours appele HORS verrou."""
        for kind, message in events:
            level = (
                logging.INFO
                if kind in ("connected", "homed", "fault_cleared", "link_restored")
                else logging.WARNING
            )
            self._log.log(level, "[%s] %s", kind, message)
            for listener in list(self._listeners):
                try:
                    listener(kind, message)
                except Exception:  # un abonne fautif ne doit pas tuer le thread
                    self._log.exception("listener %r en erreur", listener)

    # ------------------------------------------------------------------ thread E/S

    def _io_loop(self) -> None:
        buffer = b""
        while not self._stop.is_set():
            with self._lock:
                ser = self._serial
                request = self._disconnect_request
            if request is not None:
                self._emit(self._disconnect(request))
                buffer = b""
                continue
            if ser is None:
                if not self._try_connect():
                    self._stop.wait(self.config.reconnect_period_s)
                buffer = b""
                continue
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except (
                Exception
            ) as exc:  # port arrache, EBADF... le thread ne doit jamais mourir
                with self._lock:
                    self._disconnect_request = f"lecture impossible : {exc}"
                continue
            if chunk:
                buffer += chunk
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    self._handle_line(raw.decode("ascii", errors="replace"))
                if len(buffer) > 256:  # bruit sans fin de ligne
                    buffer = b""
            self._check_link()

    def _try_connect(self) -> bool:
        try:
            port = self._port_finder(self.config.port)
            ser = self._serial_factory(port, self.config.baudrate)
        except (PortNotFoundError, OSError, ValueError) as exc:
            self._note_connect_error(str(exc))
            return False
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        with self._lock:
            self._serial = ser
            self._port = port
            self._connected = False
            self._handshake_deadline = (
                time.monotonic() + self.config.handshake_timeout_s
            )
        self._log.info("port %s ouvert, attente des trames d'etat...", port)
        return True

    def _note_connect_error(self, message: str) -> None:
        now = time.monotonic()
        with self._lock:
            changed = message != self._last_connect_error
            self._last_connect_error = message
            due = changed or now - self._last_connect_log_t > 10.0
            if due:
                self._last_connect_log_t = now
        if due:
            self._log.warning("connexion poignet impossible : %s", message)

    def _close_serial_locked(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self._connected = False

    def _disconnect(self, reason: str) -> List[Tuple[str, str]]:
        with self._lock:
            was_connected = self._connected
            if was_connected and self._position is not None:
                self._last_position_before_loss = self._position
            self._close_serial_locked()
            self._disconnect_request = None
            self._streaming = False
            # Relue sur la 1re trame apres reconnexion : si la carte a redemarre
            # entre-temps, sa consigne est sa position, pas notre derniere valeur.
            self._board_target = None
            self._stale_reported = False
            self._last_connect_error = reason
            self._frame_cond.notify_all()
        return [("disconnected", reason)] if was_connected else []

    def _check_link(self) -> None:
        events: List[Tuple[str, str]] = []
        now = time.monotonic()
        with self._lock:
            if (
                self._serial is not None
                and not self._connected
                and now > self._handshake_deadline
            ):
                self._disconnect_request = (
                    f"aucune trame 'S' sur {self._port} en "
                    f"{self.config.handshake_timeout_s:.1f} s "
                    "(pas la carte du poignet ?)"
                )
            if self._connected and not self._is_fresh_locked(now):
                if not self._stale_reported:
                    self._stale_reported = True
                    if self._streaming:
                        self._streaming = False
                        events.append(("stream_stopped", "trames d'etat interrompues"))
                    events.append(("link_lost", self._link_description_locked()))
                silence = now - (self._last_frame_t or now)
                if silence > 2.0 * self.config.handshake_timeout_s:
                    self._disconnect_request = (
                        f"aucune trame depuis {silence:.1f} s : reouverture du port"
                    )
            elif self._stale_reported and self._connected:
                self._stale_reported = False
                events.append(("link_restored", "trames d'etat de nouveau recues"))
        if events:
            self._emit(events)

    def _handle_line(self, line: str) -> None:
        message = protocol.parse_line(line)
        if message is None:
            if line.strip():
                self._log.debug("ligne ignoree : %r", line)
            return
        events: List[Tuple[str, str]] = []
        now = time.monotonic()
        with self._lock:
            if isinstance(message, protocol.StatusFrame):
                self._on_status_locked(message, now, events)
            else:
                self._on_event_locked(message, events)
                for waiter in list(self._waiters):
                    if waiter.kind is message.kind and (
                        waiter.predicate is None or waiter.predicate(message)
                    ):
                        waiter.result = message
                        waiter.done.set()
                        self._waiters.remove(waiter)
            self._frame_cond.notify_all()
        if events:
            self._emit(events)

    def _on_status_locked(
        self, frame: protocol.StatusFrame, now: float, events: List[Tuple[str, str]]
    ) -> None:
        if not self._connected:
            self._connected = True
            self._last_connect_error = None
            events.append(("connected", f"carte du poignet sur {self._port}"))
            previous = self._last_position_before_loss
            tolerance = self.config.reconnect_position_tolerance
            if (
                self._homed
                and previous is not None
                and abs(frame.position - previous) > tolerance
            ):
                self._homed = False
                events.append(
                    (
                        "homing_lost",
                        f"position {frame.position:.4f} apres reconnexion "
                        f"vs {previous:.4f} avant : "
                        "carte probablement redemarree, recalage au nid requis",
                    )
                )
        self._position = frame.position
        self._current = frame.current
        self._last_frame_t = now
        self._frames += 1
        if self._board_target is None:
            self._board_target = frame.position
        if frame.fault and not self._board_fault:
            self._on_fault_locked("trame d'etat defaut=1", events)
        elif not frame.fault and self._board_fault:
            self._board_fault = False
            events.append(("fault_cleared", "la carte ne signale plus de defaut"))
        self._board_fault = frame.fault

    def _on_fault_locked(self, origin: str, events: List[Tuple[str, str]]) -> None:
        self._board_fault = True
        self._streaming = False
        events.append(
            ("fault", f"defaut blocage carte ({origin}) : consignes suspendues")
        )

    def _on_event_locked(
        self, event: protocol.Event, events: List[Tuple[str, str]]
    ) -> None:
        kind = event.kind
        if kind is protocol.EventKind.FAULT_STALL:
            if not self._board_fault:
                self._on_fault_locked("FAULT STALL", events)
            self._enabled = False
        elif kind is protocol.EventKind.READY:
            # Boot carte : offset de recalage perdu, moteur re-aligne et actif.
            self._board_reboots += 1
            self._streaming = False
            self._board_fault = False
            self._enabled = True
            self._board_target = None
            if self._homed:
                self._homed = False
                events.append(
                    (
                        "homing_lost",
                        "la carte a redemarre (READY) : recalage au nid requis",
                    )
                )
            events.append(
                ("board_reboot", f"READY recu (redemarrage #{self._board_reboots})")
            )
        elif kind is protocol.EventKind.ENABLED:
            self._enabled = True
            self._board_target = self._position
        elif kind is protocol.EventKind.DISABLED:
            self._enabled = False
        elif kind is protocol.EventKind.RESET:
            had_fault = self._board_fault or self._tracking_fault
            self._board_fault = False
            self._tracking_fault = False
            self._enabled = True
            self._board_target = self._position
            if had_fault:
                events.append(
                    ("fault_cleared", "reset effectue, moteur re-active sur place")
                )
        elif kind is protocol.EventKind.RECALE:
            assert event.value is not None
            self._position = event.value
            self._board_target = event.value
            self._last_position_before_loss = None
            if not self._homed:
                events.append(("homed", f"position recalee a {event.value:.4f} rad"))
            self._homed = True

    # ------------------------------------------------------------ thread streaming

    def _stream_loop(self) -> None:
        period = 1.0 / self.config.stream_rate_hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            elif delay < -5 * period:
                next_tick = (
                    time.monotonic()
                )  # gros retard : on ne rattrape pas en rafale
            try:
                self._stream_tick(period)
            except Exception:
                self._log.exception("erreur dans le tick de streaming")

    def _stream_tick(self, dt: float) -> None:
        cfg = self.config
        events: List[Tuple[str, str]] = []
        with self._write_lock:
            with self._lock:
                if not self._streaming:
                    return
                now = time.monotonic()
                reason = self._blocking_reason_locked(now)
                if reason is not None:
                    self._streaming = False
                    events.append(("stream_stopped", reason[1]))
                    payload = None
                elif self._tracking_exceeded_locked(now):
                    # Figer sur place : ne plus pousser contre un obstacle.
                    self._tracking_fault = True
                    self._streaming = False
                    assert self._position is not None
                    freeze_at = self._position
                    events.append(
                        (
                            "tracking_fault",
                            "ecart consigne/mesure > "
                            f"{cfg.tracking_error_limit:.2f} rad pendant "
                            f"{cfg.tracking_error_time_s:.1f} s : "
                            f"axe fige a {freeze_at:.4f}",
                        )
                    )
                    payload = (freeze_at, protocol.encode_position(freeze_at))
                else:
                    setpoint = self._limiter.step(dt)
                    stale = now - self._last_send_t >= self.config.keepalive_s
                    if (
                        self._last_sent is None
                        or abs(setpoint - self._last_sent) > 1e-6
                        or stale
                    ):
                        payload = (setpoint, protocol.encode_position(setpoint))
                    else:
                        payload = None
            if payload is not None:
                value, data = payload
                try:
                    self._write_locked(data)
                except NotConnectedError as exc:
                    with self._lock:
                        self._streaming = False
                    events.append(("stream_stopped", str(exc)))
                else:
                    with self._lock:
                        self._note_sent_locked(value)
        if events:
            self._emit(events)

    def _tracking_exceeded_locked(self, now: float) -> bool:
        limit = self.config.tracking_error_limit
        if limit <= 0.0 or self._board_target is None or self._position is None:
            return False
        if abs(self._board_target - self._position) <= limit:
            self._tracking_since = None
            return False
        if self._tracking_since is None:
            self._tracking_since = now
        return now - self._tracking_since >= self.config.tracking_error_time_s
