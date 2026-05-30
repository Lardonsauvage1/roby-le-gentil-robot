#include <gtest/gtest.h>
#include <cmath>
#include "roby_hardware/stepper_driver.hpp"

using roby_hardware::StepperDriver;
using roby_hardware::StepperConfig;

static constexpr double TWO_PI = 2.0 * M_PI;
static constexpr double TOLERANCE = 1e-6;

class StepperDriverTest : public ::testing::Test
{
protected:
  StepperDriver driver;
  StepperConfig config;

  void SetUp() override
  {
    config.mock = true;
    config.steps_per_rev = 12800;
  }
};

// ---- Axis 1: belt 16/85 ----

TEST_F(StepperDriverTest, Axis1_StepsPerAxisRev)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  // 12800 * (85/16) = 68000
  EXPECT_NEAR(driver.steps_per_axis_rev(), 68000.0, 0.1);
}

TEST_F(StepperDriverTest, Axis1_RadToSteps_FullRev)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  // One full axis revolution = 2*PI rad = 68000 steps
  EXPECT_EQ(driver.rad_to_steps(TWO_PI), 68000);
  EXPECT_EQ(driver.rad_to_steps(-TWO_PI), -68000);
}

TEST_F(StepperDriverTest, Axis1_StepsToRad_FullRev)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  EXPECT_NEAR(driver.steps_to_rad(68000), TWO_PI, TOLERANCE);
  EXPECT_NEAR(driver.steps_to_rad(-68000), -TWO_PI, TOLERANCE);
}

TEST_F(StepperDriverTest, Axis1_RadToSteps_90deg)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  // 90° = PI/2 rad = 68000/4 = 17000 steps
  EXPECT_EQ(driver.rad_to_steps(M_PI / 2.0), 17000);
}

// ---- Axis 2: gear 15/44 ----

TEST_F(StepperDriverTest, Axis2_StepsPerAxisRev)
{
  config.gear_ratio_num = 15;
  config.gear_ratio_den = 44;
  ASSERT_TRUE(driver.init(config));

  // 12800 * (44/15) ≈ 37546.67
  EXPECT_NEAR(driver.steps_per_axis_rev(), 37546.667, 0.1);
}

TEST_F(StepperDriverTest, Axis2_RadToSteps_FullRev)
{
  config.gear_ratio_num = 15;
  config.gear_ratio_den = 44;
  ASSERT_TRUE(driver.init(config));

  int64_t steps = driver.rad_to_steps(TWO_PI);
  EXPECT_EQ(steps, 37547);  // rounded
}

// ---- Axis 3: 2-stage (15*20)/(44*32) = 300/1408 ----

TEST_F(StepperDriverTest, Axis3_StepsPerAxisRev)
{
  config.gear_ratio_num = 300;
  config.gear_ratio_den = 1408;
  ASSERT_TRUE(driver.init(config));

  // 12800 * (1408/300) ≈ 60074.67
  EXPECT_NEAR(driver.steps_per_axis_rev(), 60074.667, 0.1);
}

TEST_F(StepperDriverTest, Axis3_RadToSteps_FullRev)
{
  config.gear_ratio_num = 300;
  config.gear_ratio_den = 1408;
  ASSERT_TRUE(driver.init(config));

  int64_t steps = driver.rad_to_steps(TWO_PI);
  EXPECT_EQ(steps, 60075);  // rounded
}

// ---- Roundtrip conversion ----

TEST_F(StepperDriverTest, RoundtripConversion)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  double original = 1.234;
  int64_t steps = driver.rad_to_steps(original);
  double recovered = driver.steps_to_rad(steps);

  // Resolution: 2*PI / 68000 ≈ 0.0000924 rad
  EXPECT_NEAR(recovered, original, 0.0001);
}

// ---- Position tracking ----

TEST_F(StepperDriverTest, SetPosition)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  driver.set_position_rad(1.0);
  EXPECT_NEAR(driver.get_position_rad(), 1.0, 0.0001);
}

TEST_F(StepperDriverTest, MoveToward_MockMode)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  // Start at 0, move toward 0.1 rad
  int steps = driver.move_toward(0.1, 10000);
  EXPECT_GT(steps, 0);
  EXPECT_NEAR(driver.get_position_rad(), 0.1, 0.0001);
}

TEST_F(StepperDriverTest, MoveToward_MaxStepsLimit)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  // Try to move a full revolution but limit to 100 steps per cycle
  int steps = driver.move_toward(TWO_PI, 100);
  EXPECT_EQ(steps, 100);
  EXPECT_LT(driver.get_position_rad(), TWO_PI);
}

TEST_F(StepperDriverTest, MoveToward_AlreadyAtTarget)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  int steps = driver.move_toward(0.0, 1000);
  EXPECT_EQ(steps, 0);
}

TEST_F(StepperDriverTest, ZeroPosition)
{
  config.gear_ratio_num = 16;
  config.gear_ratio_den = 85;
  ASSERT_TRUE(driver.init(config));

  EXPECT_EQ(driver.get_position_steps(), 0);
  EXPECT_NEAR(driver.get_position_rad(), 0.0, TOLERANCE);
}
