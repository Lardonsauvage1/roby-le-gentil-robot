#!/usr/bin/env python3
"""Test stepper motor 1 via GPIO (STEP=GPIO17, DIR=GPIO27).

PREREQUISITES:
  - DM860I driver powered ON
  - Motor 1 DISCONNECTED from the robot (belt removed)
  - test_gpio_motor1.py passed successfully

Usage:
    python3 test_stepper_motor1.py                          # 100 steps CW, slow
    python3 test_stepper_motor1.py --steps 500 --delay-us 200
    python3 test_stepper_motor1.py --round-trip              # aller-retour
    python3 test_stepper_motor1.py --direction ccw --steps 50

Press Ctrl+C to stop immediately (motor stops, GPIO released).
"""

import argparse
import signal
import sys
import time

STEP_PIN = 17
DIR_PIN = 27
GPIO_CHIP = "/dev/gpiochip4"

# Motor 1 specs (for info display only)
STEPS_PER_REV_MOTOR = 12800
GEAR_NUM = 16  # pignon moteur
GEAR_DEN = 85  # poulie robot
STEPS_PER_AXIS_REV = STEPS_PER_REV_MOTOR * GEAR_DEN / GEAR_NUM  # 68000

shutdown = False
request = None
chip = None


def signal_handler(sig, frame):
    global shutdown
    shutdown = True
    print("\n[STOP] Ctrl+C — stopping motor...")


signal.signal(signal.SIGINT, signal_handler)


def cleanup():
    global request, chip
    if request is not None:
        try:
            import gpiod
            request.set_value(STEP_PIN, gpiod.line.Value.INACTIVE)
            request.set_value(DIR_PIN, gpiod.line.Value.INACTIVE)
            request.release()
        except Exception:
            pass
        request = None
    if chip is not None:
        try:
            chip.close()
        except Exception:
            pass
        chip = None


def send_steps(req, gpiod, n_steps, delay_us, direction_cw):
    """Send n_steps pulses. Returns actual steps sent (may be less if interrupted)."""
    # Set direction
    dir_val = gpiod.line.Value.ACTIVE if direction_cw else gpiod.line.Value.INACTIVE
    req.set_value(DIR_PIN, dir_val)

    # Wait for direction stabilization (50ms as per DM860I spec)
    time.sleep(0.05)

    delay_s = delay_us / 1_000_000.0
    pulse_s = 3.0 / 1_000_000.0  # 3us pulse width

    steps_done = 0
    t_start = time.monotonic()

    for i in range(n_steps):
        if shutdown:
            break
        req.set_value(STEP_PIN, gpiod.line.Value.ACTIVE)
        time.sleep(pulse_s)
        req.set_value(STEP_PIN, gpiod.line.Value.INACTIVE)
        time.sleep(delay_s)
        steps_done += 1

    t_elapsed = time.monotonic() - t_start
    return steps_done, t_elapsed


def main():
    global request, chip

    parser = argparse.ArgumentParser(description="Test stepper motor 1 (GPIO)")
    parser.add_argument("--steps", type=int, default=100,
                        help="Number of steps to send (default: 100)")
    parser.add_argument("--delay-us", type=int, default=500,
                        help="Delay between pulses in microseconds (default: 500)")
    parser.add_argument("--direction", choices=["cw", "ccw"], default="cw",
                        help="Direction: cw=clockwise, ccw=counter-clockwise (default: cw)")
    parser.add_argument("--round-trip", action="store_true",
                        help="Do a round trip (go + return)")
    args = parser.parse_args()

    try:
        import gpiod
    except ImportError:
        print("[ERROR] gpiod module not found. Install: pip3 install gpiod")
        sys.exit(1)

    # Info
    motor_degrees = (args.steps / STEPS_PER_REV_MOTOR) * 360
    axis_degrees = (args.steps / STEPS_PER_AXIS_REV) * 360
    freq_hz = 1_000_000 / (args.delay_us + 6) if args.delay_us > 0 else 0  # +6 for pulse width

    print("=" * 50)
    print("  STEPPER MOTOR 1 TEST")
    print("=" * 50)
    print(f"  Steps:       {args.steps}")
    print(f"  Delay:       {args.delay_us} us  (~{freq_hz:.0f} Hz)")
    print(f"  Direction:   {args.direction}")
    print(f"  Round trip:  {args.round_trip}")
    print(f"  Motor angle: {motor_degrees:.2f} deg")
    print(f"  Axis angle:  {axis_degrees:.2f} deg  (with {GEAR_NUM}/{GEAR_DEN} reduction)")
    print()
    print("  SAFETY: Motor must be DISCONNECTED from robot (no belt)!")
    print("  Press Ctrl+C to stop at any time.")
    print("=" * 50)
    print()

    # Countdown
    for i in range(3, 0, -1):
        if shutdown:
            return
        print(f"  Starting in {i}...")
        time.sleep(1)

    # Open GPIO
    try:
        chip = gpiod.Chip(GPIO_CHIP)
        config = gpiod.LineSettings(
            direction=gpiod.line.Direction.OUTPUT,
            output_value=gpiod.line.Value.INACTIVE,
        )
        request = chip.request_lines(
            consumer="test_stepper_motor1",
            config={STEP_PIN: config, DIR_PIN: config},
        )
    except Exception as e:
        print(f"[ERROR] GPIO init failed: {e}")
        cleanup()
        sys.exit(1)

    direction_cw = (args.direction == "cw")

    # --- Forward ---
    print(f"[GO] Sending {args.steps} steps {args.direction}...")
    steps_done, t_elapsed = send_steps(request, gpiod, args.steps, args.delay_us, direction_cw)
    print(f"  Done: {steps_done} steps in {t_elapsed:.3f}s")
    print(f"  Effective rate: {steps_done / t_elapsed:.0f} steps/s" if t_elapsed > 0 else "")

    if shutdown:
        cleanup()
        print("[STOP] Aborted during forward phase.")
        return

    # --- Return (if round trip) ---
    if args.round_trip:
        print()
        print("[PAUSE] Waiting 1s before return...")
        time.sleep(1)
        if shutdown:
            cleanup()
            return

        reverse_dir = "ccw" if direction_cw else "cw"
        print(f"[RETURN] Sending {steps_done} steps {reverse_dir}...")
        steps_back, t_back = send_steps(request, gpiod, steps_done, args.delay_us, not direction_cw)
        print(f"  Done: {steps_back} steps in {t_back:.3f}s")

        if steps_back == steps_done:
            print("  [OK] Round trip complete — motor should be at starting position")
        else:
            print(f"  [WARN] Only {steps_back}/{steps_done} return steps sent (interrupted)")

    # Cleanup
    cleanup()

    print()
    print("[OK] Test complete. GPIO released.")
    if not args.round_trip:
        print(f"  Motor moved {axis_degrees:.2f} deg on axis side.")
        print("  Run with --round-trip to return to start position.")


if __name__ == "__main__":
    main()
