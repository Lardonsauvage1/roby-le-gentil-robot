#include <gtest/gtest.h>
#include <cmath>
#include "roby_hardware/safety_monitor.hpp"

using roby_hardware::SafetyMonitor;
using roby_hardware::JointSafetyConfig;

static constexpr double DEG = M_PI / 180.0;
static constexpr double TOLERANCE = 1e-6;

class SafetyMonitorTest : public ::testing::Test
{
protected:
  SafetyMonitor monitor;
  std::vector<JointSafetyConfig> configs;

  void SetUp() override
  {
    // Stepper-like joint (axis 1: ±180°)
    JointSafetyConfig stepper;
    stepper.name = "joint_1";
    stepper.position_min_rad = -M_PI;
    stepper.position_max_rad = M_PI;
    stepper.max_velocity_rad_per_tick = 3.0 * DEG;
    stepper.warning_deviation_rad = 5.0 * DEG;
    stepper.critical_deviation_rad = 15.0 * DEG;
    stepper.decel_zone_fraction = 0.10;
    stepper.decel_min_factor = 0.25;
    configs.push_back(stepper);

    // Servo-like joint (axis 4: 0-270°)
    JointSafetyConfig servo;
    servo.name = "joint_4";
    servo.position_min_rad = 0.0;
    servo.position_max_rad = 270.0 * DEG;
    servo.max_velocity_rad_per_tick = 8.0 * DEG;
    servo.warning_deviation_rad = 8.0 * DEG;
    servo.critical_deviation_rad = 20.0 * DEG;
    servo.decel_zone_fraction = 0.10;
    servo.decel_min_factor = 0.25;
    configs.push_back(servo);

    monitor.init(configs);
  }
};

// ---- Velocity clamping ----

TEST_F(SafetyMonitorTest, ClampCommand_WithinVelocityLimit)
{
  // Small movement within 3°/tick limit
  double cmd = monitor.clamp_command(0, 0.0, 1.0 * DEG);
  EXPECT_NEAR(cmd, 1.0 * DEG, TOLERANCE);
}

TEST_F(SafetyMonitorTest, ClampCommand_ExceedsVelocityLimit)
{
  // Large jump: 90° in one tick, should be clamped to ~3°
  double cmd = monitor.clamp_command(0, 0.0, 90.0 * DEG);
  EXPECT_NEAR(cmd, 3.0 * DEG, 0.01 * DEG);
}

TEST_F(SafetyMonitorTest, ClampCommand_NegativeDirection)
{
  double cmd = monitor.clamp_command(0, 0.0, -90.0 * DEG);
  EXPECT_NEAR(cmd, -3.0 * DEG, 0.01 * DEG);
}

TEST_F(SafetyMonitorTest, ClampCommand_ServoVelocityLimit)
{
  // Servo: 8°/tick
  double cmd = monitor.clamp_command(1, 135.0 * DEG, 200.0 * DEG);
  EXPECT_NEAR(cmd, 143.0 * DEG, 0.01 * DEG);
}

// ---- Hard limits ----

TEST_F(SafetyMonitorTest, ClampCommand_BeyondUpperLimit)
{
  // Command beyond +180° should be clamped
  double cmd = monitor.clamp_command(0, 179.0 * DEG, 200.0 * DEG);
  // Clamped to PI, then velocity limited from 179°
  EXPECT_LE(cmd, M_PI + TOLERANCE);
}

TEST_F(SafetyMonitorTest, ClampCommand_BeyondLowerLimit)
{
  double cmd = monitor.clamp_command(0, -179.0 * DEG, -200.0 * DEG);
  EXPECT_GE(cmd, -M_PI - TOLERANCE);
}

// ---- Deceleration zone ----

TEST_F(SafetyMonitorTest, DecelZone_Center_FullSpeed)
{
  // At center of range (0 rad), no deceleration
  double cmd = monitor.clamp_command(0, 0.0, 10.0 * DEG);
  EXPECT_NEAR(cmd, 3.0 * DEG, 0.01 * DEG);  // full 3°/tick
}

TEST_F(SafetyMonitorTest, DecelZone_NearEdge_Reduced)
{
  // Near the edge: ±180°, decel zone = 10% of 360° = 36°
  // At 170° from center = 10° from +180° edge → inside 36° zone
  double near_edge = 175.0 * DEG;  // 5° from edge
  double cmd = monitor.clamp_command(0, near_edge, near_edge + 10.0 * DEG);
  double delta = cmd - near_edge;
  // Should be less than full 3°/tick due to deceleration
  EXPECT_LT(delta, 3.0 * DEG);
  EXPECT_GT(delta, 0.0);
}

// ---- Deviation watchdog ----

TEST_F(SafetyMonitorTest, Deviation_OK)
{
  EXPECT_EQ(monitor.check_deviation(0, 0.0, 1.0 * DEG), 0);
}

TEST_F(SafetyMonitorTest, Deviation_Warning)
{
  // 5°+ deviation → warning for stepper
  EXPECT_EQ(monitor.check_deviation(0, 0.0, 6.0 * DEG), 1);
}

TEST_F(SafetyMonitorTest, Deviation_Critical)
{
  // 15°+ deviation → critical for stepper
  EXPECT_EQ(monitor.check_deviation(0, 0.0, 16.0 * DEG), 2);
}

TEST_F(SafetyMonitorTest, Deviation_ServoCritical)
{
  // 20°+ for servo
  EXPECT_EQ(monitor.check_deviation(1, 0.0, 21.0 * DEG), 2);
}

TEST_F(SafetyMonitorTest, CheckAllDeviations_NoCritical)
{
  std::vector<double> actual = {0.0, 0.0};
  std::vector<double> commanded = {1.0 * DEG, 1.0 * DEG};
  EXPECT_FALSE(monitor.check_all_deviations(actual, commanded));
}

TEST_F(SafetyMonitorTest, CheckAllDeviations_OneCritical)
{
  std::vector<double> actual = {0.0, 0.0};
  std::vector<double> commanded = {20.0 * DEG, 0.0};  // stepper: 20° > 15° critical
  EXPECT_TRUE(monitor.check_all_deviations(actual, commanded));
}

// ---- Communication watchdog ----

TEST_F(SafetyMonitorTest, CommWatchdog_Normal)
{
  EXPECT_NEAR(SafetyMonitor::comm_watchdog_factor(0), 1.0, TOLERANCE);
  EXPECT_NEAR(SafetyMonitor::comm_watchdog_factor(49), 1.0, TOLERANCE);
}

TEST_F(SafetyMonitorTest, CommWatchdog_Rampdown)
{
  double f = SafetyMonitor::comm_watchdog_factor(75);  // midway 50-100
  EXPECT_GT(f, 0.0);
  EXPECT_LT(f, 1.0);
  EXPECT_NEAR(f, 0.5, 0.01);
}

TEST_F(SafetyMonitorTest, CommWatchdog_Stopped)
{
  EXPECT_NEAR(SafetyMonitor::comm_watchdog_factor(100), 0.0, TOLERANCE);
  EXPECT_NEAR(SafetyMonitor::comm_watchdog_factor(200), 0.0, TOLERANCE);
}

// ---- Acceleration limiting ----

class AccelLimitTest : public ::testing::Test
{
protected:
  SafetyMonitor monitor;

  void SetUp() override
  {
    JointSafetyConfig j;
    j.name = "joint_1";
    j.position_min_rad = -M_PI;
    j.position_max_rad = M_PI;
    j.max_velocity_rad_per_tick = 3.0 * DEG;
    j.max_accel_rad_per_tick2 = 0.5 * DEG;   // ramp velocity by 0.5°/tick each cycle
    j.decel_zone_fraction = 0.10;
    j.decel_min_factor = 0.25;
    std::vector<JointSafetyConfig> cfgs{j};
    monitor.init(cfgs);
  }
};

TEST_F(AccelLimitTest, FirstStepLimitedByAccel)
{
  // From rest (prev_delta = 0), a far target would be velocity-clamped to 3°,
  // but the acceleration limit caps the first delta to 0.5°.
  double cmd = monitor.clamp_command(0, 0.0, 90.0 * DEG);
  EXPECT_NEAR(cmd, 0.5 * DEG, TOLERANCE);
}

TEST_F(AccelLimitTest, RampsUpOverCycles)
{
  double pos = 0.0;
  double d1 = monitor.clamp_command(0, pos, 90.0 * DEG) - pos; pos += d1;
  double d2 = monitor.clamp_command(0, pos, 90.0 * DEG) - pos; pos += d2;
  double d3 = monitor.clamp_command(0, pos, 90.0 * DEG) - pos;
  EXPECT_NEAR(d1, 0.5 * DEG, TOLERANCE);
  EXPECT_NEAR(d2, 1.0 * DEG, TOLERANCE);
  EXPECT_NEAR(d3, 1.5 * DEG, TOLERANCE);
}

TEST_F(AccelLimitTest, SaturatesAtVelocityLimit)
{
  double pos = 0.0;
  double d = 0.0;
  for (int i = 0; i < 20; ++i) {
    d = monitor.clamp_command(0, pos, 90.0 * DEG) - pos;
    pos += d;
  }
  // After ramping up, the per-cycle delta saturates at the velocity limit.
  EXPECT_NEAR(d, 3.0 * DEG, TOLERANCE);
}

TEST_F(AccelLimitTest, DisabledWhenAccelZero)
{
  // With max_accel = 0 the acceleration limit is bypassed: behaviour is the
  // original pure velocity clamp (jumps straight to 3°).
  SafetyMonitor m;
  JointSafetyConfig j;
  j.name = "j";
  j.position_min_rad = -M_PI;
  j.position_max_rad = M_PI;
  j.max_velocity_rad_per_tick = 3.0 * DEG;
  j.max_accel_rad_per_tick2 = 0.0;  // disabled
  std::vector<JointSafetyConfig> cfgs{j};
  m.init(cfgs);
  double cmd = m.clamp_command(0, 0.0, 90.0 * DEG);
  EXPECT_NEAR(cmd, 3.0 * DEG, TOLERANCE);
}
