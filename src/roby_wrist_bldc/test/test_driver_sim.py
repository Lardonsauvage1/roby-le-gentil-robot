"""Tests du driver contre la carte simulee (sim.py), en temps reel. ~15 s au total."""

import threading
import time

import pytest

from roby_wrist_bldc.driver import (
    DriverConfig,
    FaultActiveError,
    MotionBlockedError,
    MotorDisabledError,
    NotConnectedError,
    NotHomedError,
    WristBldcDriver,
)
from roby_wrist_bldc.sim import SimulatedBoard, c_atof

NEST = 0.2009


def make_config(**overrides):
    cfg = DriverConfig(
        port="sim",
        status_timeout_s=0.2,
        handshake_timeout_s=0.5,
        reconnect_period_s=0.1,
        ack_timeout_s=0.5,
    )
    for key, value in overrides.items():
        setattr(cfg, key, value)
    return cfg


class Recorder:
    def __init__(self):
        self.events = []
        self._lock = threading.Lock()

    def __call__(self, kind, message):
        with self._lock:
            self.events.append((kind, time.monotonic()))

    def kinds(self):
        with self._lock:
            return [k for k, _ in self.events]

    def time_of(self, kind):
        with self._lock:
            return next(t for k, t in self.events if k == kind)


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture
def board():
    return SimulatedBoard(start_position=0.0)


@pytest.fixture
def make_driver(board):
    drivers = []

    def _make(**overrides):
        drv = WristBldcDriver(make_config(**overrides), serial_factory=board.factory)
        rec = Recorder()
        drv.add_listener(rec)
        drv.start()
        drivers.append(drv)
        return drv, rec

    yield _make
    for drv in drivers:
        drv.close()


@pytest.fixture
def homed(make_driver):
    drv, rec = make_driver()
    drv.initialize_at_nest(NEST, timeout=2.0)
    return drv, rec


def commands(board, prefix):
    return [c for c in board.received if c.startswith(prefix)]


# -------------------------------------------------- connexion / init


def test_connects_and_blocks_motion_until_homed(make_driver):
    drv, rec = make_driver()
    drv.wait_connected(2.0)
    status = drv.get_status()
    assert status.connected and status.fresh and not status.homed
    assert status.blocking_reason is not None
    with pytest.raises(NotHomedError):
        drv.move_to(0.5)
    assert wait_for(lambda: "connected" in rec.kinds())


def test_initialize_at_nest(homed, board):
    drv, rec = homed
    assert board.received[:3] == ["Z0.20090", "?", "E"]
    assert abs(board.position - NEST) < 1e-6
    assert wait_for(lambda: abs(drv.get_state().position - NEST) < 1e-3)
    status = drv.get_status()
    assert status.homed and status.enabled and status.motion_allowed
    assert wait_for(lambda: "homed" in rec.kinds())


def test_initialize_at_nest_refuses_active_fault(make_driver, board):
    board.fault = True
    drv, _ = make_driver()
    with pytest.raises(FaultActiveError):
        drv.initialize_at_nest(NEST, timeout=2.0)
    assert commands(board, "Z") == []


def test_wait_connected_times_out_when_unplugged(board):
    board.unplug()
    drv = WristBldcDriver(make_config(), serial_factory=board.factory).start()
    try:
        with pytest.raises(NotConnectedError):
            drv.wait_connected(0.5)
    finally:
        drv.close()


def test_port_without_status_frames_is_rejected(board):
    board.set_silent(True)
    closes = []
    original_close = board.close

    def spy_close():
        closes.append(time.monotonic())
        original_close()

    board.close = spy_close
    drv = WristBldcDriver(make_config(), serial_factory=board.factory).start()
    try:
        assert wait_for(lambda: len(closes) >= 1, timeout=2.0)
        assert not drv.get_status().connected
    finally:
        drv.close()


# -------------------------------------------------- mouvement


def test_move_to_is_non_blocking_ramped_and_100hz(homed, board):
    drv, _ = homed
    start = time.monotonic()
    drv.move_to(0.8)
    assert time.monotonic() - start < 0.02
    assert drv.wait_until_reached(timeout=5.0, tolerance=0.005)
    elapsed = time.monotonic() - start

    sent = [c_atof(c[1:]) for c in commands(board, "P")]
    steps = [b - a for a, b in zip(sent, sent[1:])]
    assert all(s >= -1e-9 for s in steps), "rampe monotone"
    assert max(steps) <= 0.5 * 0.01 + 1e-4, "vitesse <= max_velocity"
    assert sent[-1] == pytest.approx(0.8, abs=1e-5)
    # (0.8-0.2)/0.5 + 0.5/1.5 = 1.53 s de rampe a 100 Hz
    ramp_cmds = len([s for s in steps if s > 1e-6])
    assert 130 <= ramp_cmds <= 170
    assert elapsed < 2.5
    assert abs(drv.get_state().position - 0.8) < 0.005


def test_move_to_can_be_retargeted_at_100hz(homed, board):
    drv, _ = homed
    for i in range(100):  # flux type JointTrajectoryController : 0.2 rad/s
        drv.move_to(NEST + 0.002 * (i + 1))
        time.sleep(0.01)
    assert drv.wait_until_reached(timeout=2.0, tolerance=0.01)
    assert drv.get_state().position == pytest.approx(NEST + 0.2, abs=0.01)


def test_limits_are_enforced(homed):
    drv, _ = homed
    with pytest.raises(ValueError):
        drv.move_to(1.7)
    with pytest.raises(ValueError):
        drv.set_position(-2.0)
    with pytest.raises(ValueError):
        drv.recalibrate(3.0)


def test_set_position_sends_raw_command(homed, board):
    drv, _ = homed
    drv.set_position(0.3)
    assert commands(board, "P")[-1] == "P0.30000"
    assert wait_for(lambda: abs(drv.get_state().position - 0.3) < 0.005)


def test_stop_motion_brakes_before_target(homed):
    drv, _ = homed
    drv.move_to(1.5)
    time.sleep(0.6)
    drv.stop_motion()
    assert drv.wait_until_reached(timeout=3.0, tolerance=0.01)
    assert drv.get_state().position < 1.2


def test_disable_blocks_motion_until_enable(homed, board):
    drv, _ = homed
    drv.disable()
    assert board.enabled is False
    with pytest.raises(MotorDisabledError):
        drv.move_to(0.5)
    drv.enable()
    drv.move_to(0.5)
    assert drv.wait_until_reached(timeout=3.0)


def test_no_old_frame_setpoint_after_recalibrate(homed, board):
    drv, _ = homed
    drv.move_to(1.0)
    time.sleep(0.3)
    drv.recalibrate(0.0)
    n_after_z = len(board.received)
    time.sleep(0.2)
    assert board.received[n_after_z - 1] == "Z0.00000"
    assert all(not c.startswith("P") for c in board.received[n_after_z:])
    assert wait_for(lambda: abs(drv.get_state().position) < 1e-3)


# ----------------------------------------------------------------------------- securite


def test_board_stall_fault_stops_streaming(make_driver, board):
    board.stall_time_s = 0.3
    drv, rec = make_driver(tracking_error_limit=0.0)  # on veut le defaut CARTE
    drv.initialize_at_nest(NEST, timeout=2.0)
    board.set_obstacle(-1.0, 0.45)
    drv.move_to(1.2)
    with pytest.raises(FaultActiveError):
        drv.wait_until_reached(timeout=5.0)
    assert drv.get_state().fault
    n = len(commands(board, "P"))
    time.sleep(0.3)
    assert len(commands(board, "P")) == n, "plus aucune consigne P apres le defaut"
    with pytest.raises(FaultActiveError):
        drv.move_to(0.3)
    assert wait_for(lambda: "fault" in rec.kinds())

    board.clear_obstacle()
    drv.reset_fault()
    assert not drv.get_state().fault
    drv.move_to(0.3)
    assert drv.wait_until_reached(timeout=3.0)


def test_tracking_error_freezes_axis_before_board_stall(make_driver, board):
    board.stall_time_s = 10.0
    drv, rec = make_driver(tracking_error_limit=0.2, tracking_error_time_s=0.3)
    drv.initialize_at_nest(NEST, timeout=2.0)
    board.set_obstacle(-1.0, 0.45)
    drv.move_to(1.2)
    assert wait_for(lambda: drv.get_state().fault, timeout=4.0)
    status = drv.get_status()
    assert status.tracking_fault and not status.board_fault
    # la derniere consigne fige l'axe sur sa position mesuree (plus de poussee)
    frozen = c_atof(commands(board, "P")[-1][1:])
    assert frozen == pytest.approx(0.45, abs=0.02)
    assert wait_for(lambda: board.current < 1.0)
    assert wait_for(lambda: "tracking_fault" in rec.kinds())

    board.clear_obstacle()
    drv.reset_fault()
    assert not drv.get_state().fault
    drv.move_to(0.3)
    assert drv.wait_until_reached(timeout=3.0)


def test_unplug_replug_keeps_homing_when_position_is_consistent(homed, board):
    drv, rec = homed
    board.unplug()
    assert wait_for(lambda: not drv.get_status().connected, timeout=2.0)
    with pytest.raises(NotConnectedError):
        drv.move_to(0.5)
    board.replug()
    drv.wait_connected(3.0)
    assert drv.get_status().homed
    drv.move_to(0.5)
    assert drv.wait_until_reached(timeout=3.0)
    assert wait_for(lambda: "disconnected" in rec.kinds())


def test_board_reboot_while_unplugged_loses_homing(homed, board):
    drv, rec = homed
    board.unplug()
    assert wait_for(lambda: not drv.get_status().connected, timeout=2.0)
    board.reboot(emit_ready=False)  # READY perdu : port ferme pendant le boot
    board.replug()
    drv.wait_connected(3.0)
    assert wait_for(lambda: not drv.get_status().homed)
    with pytest.raises(NotHomedError):
        drv.move_to(0.5)
    # les evenements sont notifies juste APRES la mise a jour de l'etat
    assert wait_for(lambda: "homing_lost" in rec.kinds())


def test_board_reboot_with_ready_loses_homing(homed, board):
    drv, rec = homed
    board.reboot(emit_ready=True)
    assert wait_for(lambda: drv.get_status().board_reboots == 1)
    assert not drv.get_status().homed
    assert wait_for(lambda: "board_reboot" in rec.kinds())


def test_silent_board_stops_streaming(homed, board):
    drv, rec = homed
    drv.move_to(1.2)
    time.sleep(0.2)
    board.set_silent(True)
    assert wait_for(lambda: not drv.get_status().fresh, timeout=1.0)
    n = len(commands(board, "P"))
    time.sleep(0.2)
    assert len(commands(board, "P")) == n
    with pytest.raises(MotionBlockedError):
        drv.move_to(0.3)
    assert wait_for(lambda: "link_lost" in rec.kinds())
    board.set_silent(False)
    assert wait_for(lambda: drv.get_status().fresh, timeout=1.0)


# -------------------------------------------------- simulateur


def test_sim_reproduces_firmware_atof_trap():
    # "P" sans nombre => atof("") = 0.0 : la carte viserait 0 rad.
    assert c_atof("") == 0.0
    assert c_atof("abc") == 0.0
    assert c_atof("1.5xyz") == 1.5
    assert c_atof("-0.25") == -0.25
