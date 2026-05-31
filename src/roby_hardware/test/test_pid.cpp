// Tests unitaires du PID closed-loop (pid.hpp). Pur, sans hardware.
#include <gtest/gtest.h>

#include "roby_hardware/pid.hpp"

using roby_hardware::PidState;
using roby_hardware::pid_step;

// Gains a 0 => correction nulle quelle que soit l'erreur (= open-loop pur).
TEST(PidTest, ZeroGainsNoCorrection)
{
  PidState s;  // kp=ki=kd=0 par defaut
  EXPECT_FALSE(s.enabled());
  EXPECT_DOUBLE_EQ(pid_step(s, 1.0, 0.01), 0.0);
  EXPECT_DOUBLE_EQ(pid_step(s, -0.5, 0.01), 0.0);
}

// Terme proportionnel seul.
TEST(PidTest, ProportionalOnly)
{
  PidState s;
  s.kp = 0.5;
  EXPECT_TRUE(s.enabled());
  EXPECT_DOUBLE_EQ(pid_step(s, 2.0, 0.01), 1.0);   // 0.5 * 2.0
  EXPECT_DOUBLE_EQ(pid_step(s, -1.0, 0.01), -0.5);
}

// L'integrale accumule l'erreur dans le temps.
TEST(PidTest, IntegralAccumulates)
{
  PidState s;
  s.ki = 1.0;          // i_clamp=0 => pas de borne ici... attention :
  s.i_clamp = 100.0;   // borne large pour ne pas clamper ce test
  // erreur constante 1.0, dt=0.1 => integral += 0.1 a chaque pas
  EXPECT_NEAR(pid_step(s, 1.0, 0.1), 0.1, 1e-9);
  EXPECT_NEAR(pid_step(s, 1.0, 0.1), 0.2, 1e-9);
  EXPECT_NEAR(pid_step(s, 1.0, 0.1), 0.3, 1e-9);
}

// Anti-windup : |ki * integral| borne par i_clamp.
TEST(PidTest, IntegralAntiWindup)
{
  PidState s;
  s.ki = 2.0;
  s.i_clamp = 0.5;  // terme integral plafonne a 0.5
  // On pousse une grosse erreur longtemps : le terme i doit saturer a 0.5.
  double out = 0.0;
  for (int k = 0; k < 100; ++k) {
    out = pid_step(s, 10.0, 0.1);
  }
  EXPECT_NEAR(out, 0.5, 1e-9);          // sature, ne depasse pas i_clamp
  // integral physique = i_clamp / ki = 0.25
  EXPECT_NEAR(s.integral, 0.25, 1e-9);
}

// Terme derive : reagit a la variation d'erreur.
TEST(PidTest, DerivativeOnChange)
{
  PidState s;
  s.kd = 0.5;
  // 1er pas : prev_error=0 -> derivee = (1.0-0)/0.1 = 10 -> 0.5*10 = 5
  EXPECT_NEAR(pid_step(s, 1.0, 0.1), 5.0, 1e-9);
  // 2e pas : erreur stable 1.0 -> derivee 0 -> sortie 0
  EXPECT_NEAR(pid_step(s, 1.0, 0.1), 0.0, 1e-9);
}

// dt <= 0 : garde le terme P, ignore integral/derivee (et n'altere pas l'etat I).
TEST(PidTest, NonPositiveDtKeepsProportionalOnly)
{
  PidState s;
  s.kp = 1.0;
  s.ki = 1.0;
  s.kd = 1.0;
  s.i_clamp = 100.0;
  double before = s.integral;
  EXPECT_DOUBLE_EQ(pid_step(s, 3.0, 0.0), 3.0);  // P seul
  EXPECT_DOUBLE_EQ(s.integral, before);          // integrale inchangee
}

// Deadband : sous le seuil, correction nulle et integrale gelee ; au-dela,
// le PID reprend normalement.
TEST(PidTest, Deadband)
{
  PidState s;
  s.kp = 0.5;
  s.ki = 1.0;
  s.i_clamp = 100.0;
  s.deadband = 0.02;  // rad
  // erreur sous le seuil => 0, integrale inchangee
  EXPECT_DOUBLE_EQ(pid_step(s, 0.01, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(pid_step(s, -0.02, 0.1), 0.0);  // bord inclus
  EXPECT_DOUBLE_EQ(s.integral, 0.0);               // gelee, pas d'accumulation
  // erreur au-dela du seuil => correction normale (P au moins)
  double out = pid_step(s, 0.10, 0.1);
  EXPECT_GT(out, 0.0);
}

// reset() remet l'etat a zero (pas de windup herite entre activations).
TEST(PidTest, ResetClearsState)
{
  PidState s;
  s.ki = 1.0;
  s.i_clamp = 100.0;
  pid_step(s, 5.0, 0.1);
  pid_step(s, 5.0, 0.1);
  EXPECT_GT(s.integral, 0.0);
  s.reset();
  EXPECT_DOUBLE_EQ(s.integral, 0.0);
  EXPECT_DOUBLE_EQ(s.prev_error, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
