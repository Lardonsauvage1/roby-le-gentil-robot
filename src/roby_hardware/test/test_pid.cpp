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
  s.ki = 1.0;
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
  double out = 0.0;
  for (int k = 0; k < 100; ++k) {
    out = pid_step(s, 10.0, 0.1);
  }
  EXPECT_NEAR(out, 0.5, 1e-9);          // sature, ne depasse pas i_clamp
  EXPECT_NEAR(s.integral, 0.25, 1e-9);  // integral physique = i_clamp / ki
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

// Rampe deadband a 0 => comportement "mur" d'origine : des qu'on sort de la
// deadband, la correction est pleine (P au moins), sans mise a l'echelle.
TEST(PidTest, DeadbandRampZeroIsHardWall)
{
  PidState s;
  s.kp = 0.5;
  s.deadband = 0.02;
  s.deadband_ramp = 0.0;  // mur
  // juste au-dessus du seuil : correction = pleine valeur P (0.5 * 0.03)
  EXPECT_NEAR(pid_step(s, 0.03, 0.1), 0.015, 1e-9);
}

// Rampe deadband > 0 => la correction monte lineairement de 0 (au bord) a la
// pleine valeur (a deadband + deadband_ramp). Supprime la discontinuite.
TEST(PidTest, DeadbandRampScalesLinearly)
{
  PidState s;
  s.kp = 1.0;
  s.deadband = 0.02;
  s.deadband_ramp = 0.02;  // bande de transition [0.02, 0.04]

  // Au bord exact (|error| = deadband) on est encore dans la deadband => 0.
  EXPECT_DOUBLE_EQ(pid_step(s, 0.02, 0.1), 0.0);

  // Au milieu de la bande (error = 0.03) : scale = (0.03-0.02)/0.02 = 0.5
  // correction pleine = kp*error = 0.03 ; mise a l'echelle => 0.015
  EXPECT_NEAR(pid_step(s, 0.03, 0.1), 0.015, 1e-9);

  // A la fin de la bande (error = 0.04) : scale = 1.0 => pleine correction 0.04
  EXPECT_NEAR(pid_step(s, 0.04, 0.1), 0.04, 1e-9);

  // Au-dela de la bande (error = 0.10) : scale plafonne a 1 => pleine 0.10
  EXPECT_NEAR(pid_step(s, 0.10, 0.1), 0.10, 1e-9);

  // Symetrie en negatif : milieu de bande => -0.015
  PidState n;
  n.kp = 1.0;
  n.deadband = 0.02;
  n.deadband_ramp = 0.02;
  EXPECT_NEAR(pid_step(n, -0.03, 0.1), -0.015, 1e-9);
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

// Two-phase desactive (settle_cycles=0) : deadband toujours = settled.
TEST(PidTest, TwoPhaseDisabledAlwaysSettled)
{
  PidState s;
  s.deadband_settled = 0.04;
  s.deadband_moving = 0.005;
  s.settle_cycles = 0;
  s.update_deadband(1.0);
  EXPECT_DOUBLE_EQ(s.deadband, 0.04);
  s.update_deadband(2.0);
  EXPECT_DOUBLE_EQ(s.deadband, 0.04);
}

// Two-phase actif : deadband serree tant que la commande bouge / pas encore
// stabilisee, puis large apres settle_cycles cycles de commande stable.
TEST(PidTest, TwoPhaseTightWhileMovingThenWideWhenSettled)
{
  PidState s;
  s.deadband_settled = 0.04;
  s.deadband_moving = 0.005;
  s.settle_cycles = 3;
  s.update_deadband(1.0);            // 1er cycle, commande "change" => moving
  EXPECT_DOUBLE_EQ(s.deadband, 0.005);
  s.update_deadband(1.1);            // change encore => moving, stable_count=0
  EXPECT_DOUBLE_EQ(s.deadband, 0.005);
  s.update_deadband(1.1);            // stable_count=1 => moving
  EXPECT_DOUBLE_EQ(s.deadband, 0.005);
  s.update_deadband(1.1);            // stable_count=2 => moving
  EXPECT_DOUBLE_EQ(s.deadband, 0.005);
  s.update_deadband(1.1);            // stable_count=3 >= 3 => settled
  EXPECT_DOUBLE_EQ(s.deadband, 0.04);
  s.update_deadband(1.1);            // reste stable => settled
  EXPECT_DOUBLE_EQ(s.deadband, 0.04);
  s.update_deadband(2.0);            // nouvelle commande => repasse moving
  EXPECT_DOUBLE_EQ(s.deadband, 0.005);
}

// reset() remet la deadband active a settled et efface le compteur de stabilite.
TEST(PidTest, TwoPhaseResetRestoresSettled)
{
  PidState s;
  s.deadband_settled = 0.04;
  s.deadband_moving = 0.005;
  s.settle_cycles = 2;
  s.update_deadband(1.0);            // moving
  EXPECT_DOUBLE_EQ(s.deadband, 0.005);
  s.reset();
  EXPECT_DOUBLE_EQ(s.deadband, 0.04);
  EXPECT_EQ(s.stable_count, 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
