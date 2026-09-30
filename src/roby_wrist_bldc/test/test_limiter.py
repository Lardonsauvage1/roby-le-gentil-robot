import math

import pytest

from roby_wrist_bldc.limiter import SetpointLimiter

DT = 0.01
VMAX = 0.5
AMAX = 1.5


def run(limiter, targets):
    """targets : liste de cibles (None = inchangee). Renvoie positions, vitesses."""
    positions, velocities = [], [limiter.velocity]
    for target in targets:
        if target is not None:
            limiter.set_target(target)
        positions.append(limiter.step(DT))
        velocities.append(limiter.velocity)
    return positions, velocities


def max_accel(velocities):
    return max(abs(b - a) / DT for a, b in zip(velocities, velocities[1:]))


@pytest.fixture
def limiter():
    lim = SetpointLimiter(VMAX, AMAX)
    lim.reset(0.0)
    return lim


def test_step_reaches_target_without_overshoot(limiter):
    positions, velocities = run(limiter, [1.0] + [None] * 400)
    assert positions[-1] == 1.0
    assert max(positions) <= 1.0
    assert max(abs(v) for v in velocities) <= VMAX + 1e-9
    assert max_accel(velocities) <= AMAX + 1e-6
    # trapeze : 1/0.5 + 0.5/1.5 = 2.33 s
    arrival = next(i for i, p in enumerate(positions) if p == 1.0) * DT
    assert 2.3 <= arrival <= 2.45
    assert limiter.at_target()


def test_reversal_mid_flight_is_smooth(limiter):
    run(limiter, [1.0] + [None] * 99)
    positions, velocities = run(limiter, [-0.5] + [None] * 500)
    assert positions[-1] == -0.5
    assert min(positions) >= -0.5
    assert max_accel(velocities) <= AMAX + 1e-6


def test_feasible_ramp_is_tracked_exactly(limiter):
    targets = [0.3 * (i + 1) * DT for i in range(300)]
    positions, velocities = run(limiter, targets)
    assert max(abs(p - t) for p, t in zip(positions[100:], targets[100:])) < 1e-9
    assert max_accel(velocities) <= AMAX + 1e-6


def test_smooth_stream_is_tracked_closely(limiter):
    targets = [0.2 * math.sin(2.0 * (i + 1) * DT) for i in range(600)]
    positions, _ = run(limiter, targets)
    assert max(abs(p - t) for p, t in zip(positions[100:], targets[100:])) < 1e-3


def test_unfeasible_stream_is_rate_limited(limiter):
    # Cible qui fuit a 2 rad/s : la sortie plafonne a VMAX.
    targets = [2.0 * (i + 1) * DT for i in range(200)]
    _, velocities = run(limiter, targets)
    assert max(abs(v) for v in velocities) <= VMAX + 1e-9


def test_tiny_step(limiter):
    positions, _ = run(limiter, [1e-4] + [None] * 50)
    assert positions[-1] == 1e-4
    assert max(positions) <= 1e-4


def test_reset_stops_and_retargets(limiter):
    run(limiter, [1.0] + [None] * 50)
    limiter.reset(0.3)
    assert limiter.position == 0.3 and limiter.velocity == 0.0 and limiter.target == 0.3


def test_invalid_limits():
    with pytest.raises(ValueError):
        SetpointLimiter(0.0, 1.0)
