"""Noeud ROS 2 du poignet BLDC (axe 5) : pont entre ros2_control et la carte.

Le plugin RobySystem (joint_5_type = bldc) publie la consigne articulaire a
100 Hz sur ~/command ; ce noeud la transmet a la carte via le driver (rampe,
butees, securite) et renvoie l'etat mesure sur ~/state, que le plugin expose
sur /joint_states.

Topics (prefixe /roby/wrist_bldc) :
    command  std_msgs/Float64            consigne rad (plugin, ou manuel au banc)
    state    std_msgs/Float64MultiArray  [position, courant, flags] a 50 Hz
    fault    std_msgs/Bool               defaut actif (latched, sur changement)
    status   std_msgs/String             resume JSON lisible, 2 Hz
Services (std_srvs/Trigger) :
    enable, disable, reset_fault, recalibrate_nest

flags (bits) : 1 liaison OK, 2 recale, 4 defaut, 8 consignes suivies, 16 moteur actif.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from std_srvs.srv import Trigger

from .driver import DriverConfig, MotionBlockedError, WristBldcDriver, WristError
from .sim import SimulatedBoard

FLAG_LINK_OK = 1
FLAG_HOMED = 2
FLAG_FAULT = 4
FLAG_FOLLOWING = 8
FLAG_ENABLED = 16

# Une consigne "egale" au verrou de reprise, a cette tolerance pres, est ignoree.
RESUME_LATCH_TOLERANCE = 1e-3


class _RosLogHandler(logging.Handler):
    """Renvoie les logs Python du driver vers le logger rclpy du noeud."""

    def __init__(self, node: Node) -> None:
        super().__init__()
        self._logger = node.get_logger()

    def emit(self, record: logging.LogRecord) -> None:
        message = self.format(record)
        if record.levelno >= logging.ERROR:
            self._logger.error(message)
        elif record.levelno >= logging.WARNING:
            self._logger.warning(message)
        elif record.levelno >= logging.INFO:
            self._logger.info(message)
        else:
            self._logger.debug(message)


class WristBldcNode(Node):
    def __init__(self, **node_kwargs: object) -> None:
        super().__init__("wrist_bldc", **node_kwargs)  # type: ignore[arg-type]
        p = self._declare_parameters()

        logger = logging.getLogger(
            f"roby_wrist_bldc{self.get_fully_qualified_name().replace('/', '.')}"
        )
        logger.propagate = False
        logger.setLevel(logging.INFO)
        self._log_handler = _RosLogHandler(self)
        logger.addHandler(self._log_handler)
        self._py_logger = logger

        config = DriverConfig(
            port=p["port"],
            baudrate=p["baudrate"],
            position_min=p["position_min"],
            position_max=p["position_max"],
            max_velocity=p["max_velocity"],
            max_acceleration=p["max_acceleration"],
            require_homing=p["require_homing"],
            tracking_error_limit=p["tracking_error_limit"],
            tracking_error_time_s=p["tracking_error_time_s"],
        )
        self._nest = p["nest_position"]
        self._command_timeout = p["command_timeout_s"]

        self._sim: Optional[SimulatedBoard] = None
        if p["simulate"]:
            self._sim = SimulatedBoard(start_position=0.0)
            config.port = "sim"
            self.driver = WristBldcDriver(
                config, serial_factory=self._sim.factory, logger=logger
            )
            self.get_logger().warning(
                "MODE SIMULATION : aucune carte reelle n'est pilotee"
            )
        else:
            self.driver = WristBldcDriver(config, logger=logger)

        # --- etat du pont (acces depuis callbacks ROS et thread d'init) ---
        self._lock = threading.Lock()
        self._last_command: Optional[float] = None
        self._last_command_t = 0.0
        self._resume_latch: Optional[float] = None
        self._was_allowed = False
        self._trigger_actions: dict = {}
        self._stopped_for_timeout = False
        self._last_fault: Optional[bool] = None
        self._last_block_log_t = 0.0

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(Float64MultiArray, "~/state", 10)
        self._fault_pub = self.create_publisher(Bool, "~/fault", latched)
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(Float64, "~/command", self._on_command, 10)

        self._add_trigger("~/enable", self._latch_then(self.driver.enable))
        self._add_trigger("~/disable", self.driver.disable)
        self._add_trigger("~/reset_fault", self._latch_then(self.driver.reset_fault))
        self._add_trigger("~/recalibrate_nest", self._recalibrate_nest)

        self.create_timer(0.02, self._publish_state)
        self.create_timer(0.5, self._publish_status)

        self.driver.start()
        if p["recalibrate_on_start"]:
            threading.Thread(
                target=self._startup_homing,
                args=(p["enable_on_start"], p["startup_homing_timeout_s"]),
                name="wrist-bldc-homing",
                daemon=True,
            ).start()
        else:
            self.get_logger().warning(
                "recalibrate_on_start=false : axe NON recale, "
                "appeler ~/recalibrate_nest tete au nid"
            )

    def _declare_parameters(self) -> dict:
        defaults = {
            "port": "auto",
            "baudrate": 115200,
            "simulate": False,
            "nest_position": 0.2009,
            "recalibrate_on_start": True,
            "enable_on_start": True,
            "startup_homing_timeout_s": 15.0,
            "position_min": -1.6,
            "position_max": 1.6,
            "max_velocity": 0.5,
            "max_acceleration": 1.5,
            "require_homing": True,
            "tracking_error_limit": 0.35,
            "tracking_error_time_s": 0.5,
            "command_timeout_s": 0.5,
        }
        return {
            name: self.declare_parameter(name, value).value
            for name, value in defaults.items()
        }

    # ------------------------------------------------------------------ demarrage

    def _startup_homing(self, enable: bool, timeout: float) -> None:
        """Recalage au nid au lancement de la stack (tete posee dans le nid).

        Uniquement dans la fenetre `timeout` apres le lancement : une carte
        branchee plus tard n'est PAS recalee automatiquement (le bras a pu
        bouger), il faut appeler ~/recalibrate_nest en connaissance de cause.
        """
        try:
            self.driver.initialize_at_nest(self._nest, enable=enable, timeout=timeout)
            self.get_logger().info(
                f"poignet BLDC recale au nid ({self._nest:.4f} rad) : pret"
            )
        except WristError as exc:
            self.get_logger().error(
                f"recalage au nid ECHOUE : {exc}. Axe 5 immobile. "
                "Corriger puis appeler "
                "/roby/wrist_bldc/recalibrate_nest (tete au nid) ou reset_fault."
            )

    def _latch_then(self, action: Callable[[], None]) -> Callable[[], None]:
        """Pose le verrou de reprise AVANT l'action qui re-autorise le mouvement.

        Le poser apres (ou au tick de publication suivant, jusqu'a 20 ms plus tard)
        laissait une fenetre ou la vieille consigne de maintien du controleur etait
        encore suivie : l'axe repartait vers l'obstacle qui avait cause le defaut.
        Trouve le 2026-09-13 en test sous charge (jusqu'a 0,4 rad).
        """

        def run() -> None:
            with self._lock:
                self._resume_latch = self._last_command
            action()

        return run

    def _recalibrate_nest(self) -> None:
        with self._lock:
            self._resume_latch = self._last_command
        self.driver.recalibrate(self._nest)

    def _note_motion_allowed(self, allowed: bool) -> None:
        """A appeler sous self._lock. Reprise apres defaut / reconnexion / recalage :
        la derniere consigne du controleur est peut-etre une vieille valeur de
        maintien. On ne la suit plus tant qu'elle ne change pas (nouvelle
        trajectoire, qui partira de la position mesuree)."""
        if allowed and not self._was_allowed:
            self._resume_latch = self._last_command
        self._was_allowed = allowed

    # ------------------------------------------------------------------ consignes

    def _on_command(self, msg: Float64) -> None:
        now = time.monotonic()
        value = msg.data
        if not math.isfinite(value):
            return
        cfg = self.driver.config
        value = min(max(value, cfg.position_min), cfg.position_max)
        allowed = self.driver.get_status().motion_allowed
        with self._lock:
            # Transition vue ICI aussi, avant de suivre la consigne : si seul le tick
            # de publication la voyait, une consigne arrivee entre-temps passait.
            self._note_motion_allowed(allowed)
            self._last_command = value
            self._last_command_t = now
            self._stopped_for_timeout = False
            latch = self._resume_latch
            if latch is not None:
                if abs(value - latch) <= RESUME_LATCH_TOLERANCE:
                    return
                self._resume_latch = None
        try:
            self.driver.move_to(value)
        except MotionBlockedError as exc:
            if now - self._last_block_log_t > 2.0:
                self._last_block_log_t = now
                self.get_logger().warning(f"consigne axe 5 ignoree : {exc}")

    def _check_command_timeout(self, streaming: bool) -> None:
        with self._lock:
            silent = time.monotonic() - self._last_command_t > self._command_timeout
            if not (streaming and silent and not self._stopped_for_timeout):
                return
            self._stopped_for_timeout = True
        self.get_logger().warning(
            "plus de consigne depuis ros2_control : arret sur rampe"
        )
        self.driver.stop_motion()

    # ------------------------------------------------------------------ publication

    def _publish_state(self) -> None:
        status = self.driver.get_status()
        allowed = status.motion_allowed
        with self._lock:
            self._note_motion_allowed(allowed)
        self._check_command_timeout(status.streaming)

        flags = 0
        if status.fresh:
            flags |= FLAG_LINK_OK
        if status.homed:
            flags |= FLAG_HOMED
        if status.fault:
            flags |= FLAG_FAULT
        if allowed:
            flags |= FLAG_FOLLOWING
        if status.enabled:
            flags |= FLAG_ENABLED
        position = status.position if status.position is not None else math.nan
        self._state_pub.publish(
            Float64MultiArray(data=[position, status.current, float(flags)])
        )

        if status.fault != self._last_fault:
            self._last_fault = status.fault
            self._fault_pub.publish(Bool(data=status.fault))

    def _publish_status(self) -> None:
        s = self.driver.get_status()
        summary = {
            "position": s.position,
            "current": s.current,
            "fault": s.fault,
            "board_fault": s.board_fault,
            "tracking_fault": s.tracking_fault,
            "connected": s.connected,
            "fresh": s.fresh,
            "homed": s.homed,
            "enabled": s.enabled,
            "streaming": s.streaming,
            "target": s.target,
            "port": s.port,
            "board_reboots": s.board_reboots,
            "blocking_reason": s.blocking_reason,
            "simulated": self._sim is not None,
        }
        self._status_pub.publish(String(data=json.dumps(summary)))

    # ------------------------------------------------------------------ services

    def _add_trigger(self, name: str, action: Callable[[], None]) -> None:
        # Garde l'action exacte de chaque service : les tests appellent CE que le
        # service execute, sans la fenetre de temps d'un appel ROS asynchrone.
        self._trigger_actions[name] = action

        def callback(
            _request: Trigger.Request, response: Trigger.Response
        ) -> Trigger.Response:
            try:
                action()
            except (WristError, ValueError) as exc:
                response.success = False
                response.message = str(exc)
            else:
                response.success = True
                response.message = json.dumps(
                    {"blocking_reason": self.driver.get_status().blocking_reason}
                )
            self.get_logger().info(
                f"service {name} : {'OK' if response.success else response.message}"
            )
            return response

        self.create_service(Trigger, name, callback)

    def destroy_node(self) -> None:
        # Le moteur n'est pas coupe : la carte garde la derniere consigne (le
        # poignet tient la tete). Pour couper : service ~/disable avant l'arret.
        self.driver.close()
        self._py_logger.removeHandler(self._log_handler)
        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = WristBldcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
