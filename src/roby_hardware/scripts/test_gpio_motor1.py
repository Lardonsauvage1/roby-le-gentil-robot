#!/usr/bin/env python3
"""Test GPIO pins for motor 1 (STEP=GPIO17, DIR=GPIO27).

Run this with the DM860I driver POWERED OFF.
Verifies that gpiod can toggle the STEP and DIR pins.
Check with a multimeter: HIGH ~3.3V, LOW ~0V.

Usage:
    python3 test_gpio_motor1.py

Press Ctrl+C to stop at any time.
"""

import sys
import time
import signal

STEP_PIN = 17
DIR_PIN = 27
GPIO_CHIP = "/dev/gpiochip4"

# Graceful shutdown on Ctrl+C
shutdown = False

def signal_handler(sig, frame):
    global shutdown
    print("\n[STOP] Ctrl+C received, releasing GPIO...")
    shutdown = True

signal.signal(signal.SIGINT, signal_handler)


def main():
    try:
        import gpiod
    except ImportError:
        print("[ERROR] gpiod module not found. Install: pip3 install gpiod")
        sys.exit(1)

    print(f"gpiod version: {gpiod.__version__}")
    print(f"GPIO chip: {GPIO_CHIP}")
    print(f"STEP pin: GPIO {STEP_PIN}")
    print(f"DIR pin:  GPIO {DIR_PIN}")
    print()

    # Open chip and request lines
    try:
        chip = gpiod.Chip(GPIO_CHIP)
    except Exception as e:
        print(f"[ERROR] Cannot open {GPIO_CHIP}: {e}")
        sys.exit(1)

    config = gpiod.LineSettings(
        direction=gpiod.line.Direction.OUTPUT,
        output_value=gpiod.line.Value.INACTIVE,
    )

    try:
        request = chip.request_lines(
            consumer="test_gpio_motor1",
            config={STEP_PIN: config, DIR_PIN: config},
        )
    except Exception as e:
        print(f"[ERROR] Cannot request GPIO lines: {e}")
        print("  Check: is another process using these pins?")
        print("  Check: is user in 'gpio' group? (groups)")
        chip.close()
        sys.exit(1)

    print("[OK] GPIO lines acquired")
    print()

    # --- Test 1: DIR pin toggle ---
    print("=== Test 1: DIR pin (GPIO 27) toggle ===")
    print("  Measure GPIO 27 with multimeter...")

    request.set_value(DIR_PIN, gpiod.line.Value.ACTIVE)
    print("  DIR = HIGH (expect ~3.3V) — waiting 2s...")
    time.sleep(2)
    if shutdown:
        request.release()
        chip.close()
        return

    request.set_value(DIR_PIN, gpiod.line.Value.INACTIVE)
    print("  DIR = LOW  (expect ~0V)   — waiting 2s...")
    time.sleep(2)
    if shutdown:
        request.release()
        chip.close()
        return

    print("  [OK] DIR toggle done")
    print()

    # --- Test 2: STEP pin toggle ---
    print("=== Test 2: STEP pin (GPIO 17) toggle ===")
    print("  Measure GPIO 17 with multimeter...")

    request.set_value(STEP_PIN, gpiod.line.Value.ACTIVE)
    print("  STEP = HIGH (expect ~3.3V) — waiting 2s...")
    time.sleep(2)
    if shutdown:
        request.release()
        chip.close()
        return

    request.set_value(STEP_PIN, gpiod.line.Value.INACTIVE)
    print("  STEP = LOW  (expect ~0V)   — waiting 2s...")
    time.sleep(2)
    if shutdown:
        request.release()
        chip.close()
        return

    print("  [OK] STEP toggle done")
    print()

    # --- Test 3: 10 step pulses ---
    print("=== Test 3: 10 STEP pulses (slow, ~100ms each) ===")
    print("  Oscilloscope or LED on GPIO 17 to verify...")

    for i in range(10):
        if shutdown:
            break
        request.set_value(STEP_PIN, gpiod.line.Value.ACTIVE)
        time.sleep(0.05)  # 50ms high (slow, visible on LED)
        request.set_value(STEP_PIN, gpiod.line.Value.INACTIVE)
        time.sleep(0.05)  # 50ms low
        print(f"  Pulse {i+1}/10")

    if not shutdown:
        print("  [OK] 10 pulses sent")
        print()

    # --- Cleanup ---
    request.set_value(STEP_PIN, gpiod.line.Value.INACTIVE)
    request.set_value(DIR_PIN, gpiod.line.Value.INACTIVE)
    request.release()
    chip.close()

    print("=" * 40)
    print("[OK] GPIO test complete. All pins released.")
    print()
    print("Next step: if these tests passed, proceed to")
    print("test_stepper_motor1.py with the DM860I powered ON")
    print("and the motor DISCONNECTED from the robot (no belt).")


if __name__ == "__main__":
    main()
