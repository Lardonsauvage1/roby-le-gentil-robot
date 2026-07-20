/**
 * Manual hardware test for StepperDriver on motor 1.
 *
 * PREREQUISITES:
 *   - DM860I driver powered ON
 *   - Motor 1 DISCONNECTED from robot (belt removed)
 *   - Build with HAS_GPIOD on Pi5
 *
 * Usage:
 *   ros2 run roby_hardware test_stepper_hw
 *
 * This is NOT a gtest — it drives real hardware.
 * Press Ctrl+C to stop.
 */

#include <csignal>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>

#include "roby_hardware/stepper_driver.hpp"

static volatile bool g_shutdown = false;

void signal_handler(int /*sig*/)
{
  g_shutdown = true;
}

int main()
{
  std::signal(SIGINT, signal_handler);

  printf("=== StepperDriver Hardware Test — Motor 1 ===\n");
  printf("  GPIO chip: /dev/gpiochip0\n");
  printf("  STEP pin:  17\n");
  printf("  DIR pin:   27\n");
  printf("  Gear:      16/85 (belt)\n");
  printf("  Steps/rev: 12800 (motor) -> 68000 (axis)\n");
  printf("\n");
  printf("  SAFETY: Motor must be DISCONNECTED from robot!\n");
  printf("  Press Ctrl+C to stop.\n");
  printf("\n");

  // Configure for motor 1
  roby_hardware::StepperConfig cfg;
  cfg.gpio_chip = "/dev/gpiochip4";
  cfg.step_pin = 17;
  cfg.dir_pin = 27;
  cfg.steps_per_rev = 12800;
  cfg.gear_ratio_num = 16;
  cfg.gear_ratio_den = 85;
  cfg.inverted = false;
  cfg.mock = false;

  roby_hardware::StepperDriver driver;

  printf("[1/6] Initializing StepperDriver...\n");
  if (!driver.init(cfg)) {
    printf("[ERROR] Failed to initialize StepperDriver.\n");
    printf("  Check: GPIO permissions, gpiochip0 exists, pins not in use.\n");
    return 1;
  }
  printf("  [OK] StepperDriver initialized\n");
  printf("  Steps per axis rev: %.0f\n", driver.steps_per_axis_rev());
  printf("  Position: %.6f rad (%ld steps)\n",
         driver.get_position_rad(), driver.get_position_steps());
  printf("\n");

  // Countdown
  for (int i = 3; i > 0; --i) {
    if (g_shutdown) { driver.shutdown(); return 0; }
    printf("  Starting in %d...\n", i);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Test: move to 0.1 rad (~5.7 deg, ~1082 steps)
  double target_rad = 0.1;
  int max_steps_per_cycle = 500;
  int64_t expected_steps = driver.rad_to_steps(target_rad);

  printf("\n[2/6] Moving to %.4f rad (~%.2f deg, ~%ld steps)...\n",
         target_rad, target_rad * 180.0 / M_PI, expected_steps);

  int total_steps = 0;
  while (!g_shutdown) {
    int steps = driver.move_toward(target_rad, max_steps_per_cycle);
    total_steps += steps;
    if (steps == 0) break;  // arrived
    // Small delay between cycles to simulate ros2_control loop
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  printf("  Total steps sent: %d\n", total_steps);
  printf("  Position: %.6f rad (%ld steps)\n",
         driver.get_position_rad(), driver.get_position_steps());
  printf("  Expected: %.6f rad (%ld steps)\n",
         target_rad, expected_steps);

  if (g_shutdown) { driver.shutdown(); return 0; }

  // Pause
  printf("\n[3/6] Waiting 2s at target position...\n");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  if (g_shutdown) { driver.shutdown(); return 0; }

  // Return to 0
  printf("\n[4/6] Returning to 0.0 rad...\n");
  total_steps = 0;
  while (!g_shutdown) {
    int steps = driver.move_toward(0.0, max_steps_per_cycle);
    total_steps += steps;
    if (steps == 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  printf("  Total steps sent: %d\n", total_steps);
  printf("  Position: %.6f rad (%ld steps)\n",
         driver.get_position_rad(), driver.get_position_steps());

  if (g_shutdown) { driver.shutdown(); return 0; }

  // Verify
  printf("\n[5/6] Verification...\n");
  if (driver.get_position_steps() == 0) {
    printf("  [OK] Position is exactly 0 steps — no step loss!\n");
  } else {
    printf("  [WARN] Position is %ld steps (expected 0) — possible step loss\n",
           driver.get_position_steps());
  }

  // Shutdown
  printf("\n[6/6] Shutting down...\n");
  driver.shutdown();
  printf("  [OK] GPIO released.\n");
  printf("\n=== Test complete ===\n");

  return 0;
}
