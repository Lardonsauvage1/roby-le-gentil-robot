#include <gtest/gtest.h>
#include <cmath>
#include "roby_hardware/servo_driver.hpp"

using roby_hardware::ServoDriver;
using roby_hardware::ServoConfig;

static constexpr double TOLERANCE = 0.01;

class ServoDriverTest : public ::testing::Test
{
protected:
  ServoDriver driver;
  ServoConfig config;

  void SetUp() override
  {
    config.mock = true;
    config.channel = 0;
    config.angle_min_deg = 0.0;
    config.angle_max_deg = 270.0;
    config.angle_init_deg = 135.0;
    config.inverted = false;
  }
};

// ---- PWM conversion ----

TEST_F(ServoDriverTest, PulseAt0Deg)
{
  // pulse_µs = 500 + (0/180) * 2000 = 500
  EXPECT_NEAR(ServoDriver::angle_to_pulse_us(0.0), 500.0, TOLERANCE);
}

TEST_F(ServoDriverTest, PulseAt90Deg)
{
  // pulse_µs = 500 + (90/180) * 2000 = 1500
  EXPECT_NEAR(ServoDriver::angle_to_pulse_us(90.0), 1500.0, TOLERANCE);
}

TEST_F(ServoDriverTest, PulseAt180Deg)
{
  // pulse_µs = 500 + (180/180) * 2000 = 2500
  EXPECT_NEAR(ServoDriver::angle_to_pulse_us(180.0), 2500.0, TOLERANCE);
}

TEST_F(ServoDriverTest, PulseAt270Deg)
{
  // pulse_µs = 500 + (270/180) * 2000 = 3500
  EXPECT_NEAR(ServoDriver::angle_to_pulse_us(270.0), 3500.0, TOLERANCE);
}

TEST_F(ServoDriverTest, DutyAt0Deg)
{
  // duty = (500/20000) * 65535 = 1638
  EXPECT_EQ(ServoDriver::angle_to_duty(0.0), 1638);
}

TEST_F(ServoDriverTest, DutyAt90Deg)
{
  // duty = (1500/20000) * 65535 = 4915
  EXPECT_EQ(ServoDriver::angle_to_duty(90.0), 4915);
}

TEST_F(ServoDriverTest, DutyAt180Deg)
{
  // duty = (2500/20000) * 65535 = 8191
  EXPECT_EQ(ServoDriver::angle_to_duty(180.0), 8191);
}

// ---- Angle tracking ----

TEST_F(ServoDriverTest, InitialAngle)
{
  ASSERT_TRUE(driver.init(config));
  EXPECT_NEAR(driver.get_angle_deg(), 135.0, TOLERANCE);
}

TEST_F(ServoDriverTest, SetAngle)
{
  ASSERT_TRUE(driver.init(config));
  driver.set_angle_deg(90.0);
  EXPECT_NEAR(driver.get_angle_deg(), 90.0, TOLERANCE);
}

TEST_F(ServoDriverTest, SetAngle_Clamped)
{
  ASSERT_TRUE(driver.init(config));
  // Angle > max should be clamped internally but tracked as requested
  driver.set_angle_deg(300.0);
  EXPECT_NEAR(driver.get_angle_deg(), 300.0, TOLERANCE);
}

// ---- Degree/Radian conversion ----

TEST_F(ServoDriverTest, DegToRad)
{
  EXPECT_NEAR(ServoDriver::deg_to_rad(180.0), M_PI, 1e-6);
  EXPECT_NEAR(ServoDriver::deg_to_rad(90.0), M_PI / 2.0, 1e-6);
  EXPECT_NEAR(ServoDriver::deg_to_rad(0.0), 0.0, 1e-6);
}

TEST_F(ServoDriverTest, RadToDeg)
{
  EXPECT_NEAR(ServoDriver::rad_to_deg(M_PI), 180.0, 1e-6);
  EXPECT_NEAR(ServoDriver::rad_to_deg(M_PI / 2.0), 90.0, 1e-6);
  EXPECT_NEAR(ServoDriver::rad_to_deg(0.0), 0.0, 1e-6);
}

// ---- Axis 5 config (inverted) ----

TEST_F(ServoDriverTest, InvertedServo)
{
  config.angle_min_deg = 0.0;
  config.angle_max_deg = 180.0;
  config.angle_init_deg = 90.0;
  config.inverted = true;
  ASSERT_TRUE(driver.init(config));

  // Inverted: effective_angle = (180 + 0) - 90 = 90 → same initial
  EXPECT_NEAR(driver.get_angle_deg(), 90.0, TOLERANCE);

  driver.set_angle_deg(0.0);
  // User commands 0° but servo goes to 180° (inverted)
  EXPECT_NEAR(driver.get_angle_deg(), 0.0, TOLERANCE);
}

// ---- Mock mode ----

TEST_F(ServoDriverTest, MockInit)
{
  config.mock = true;
  EXPECT_TRUE(driver.init(config));
}
