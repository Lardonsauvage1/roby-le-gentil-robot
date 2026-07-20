// Tests du filtre rejet d'outliers + porte de sortie (outlier_filter.hpp).
// Pur, sans hardware.
#include <gtest/gtest.h>

#include "roby_hardware/outlier_filter.hpp"

using roby_hardware::accept_with_escape;

static constexpr double MAX_STEP = 20.0;
static constexpr int MAX_CONSEC = 5;

// Lecture normale (saut <= seuil) : acceptee, compteur a zero.
TEST(OutlierFilter, NormalAccepted)
{
  int consec = 0;
  EXPECT_TRUE(accept_with_escape(5.0, MAX_STEP, consec, MAX_CONSEC));
  EXPECT_EQ(consec, 0);
  EXPECT_TRUE(accept_with_escape(-19.9, MAX_STEP, consec, MAX_CONSEC));  // bord
  EXPECT_EQ(consec, 0);
}

// Outlier isole : rejete, compteur incremente.
TEST(OutlierFilter, OutlierRejected)
{
  int consec = 0;
  EXPECT_FALSE(accept_with_escape(50.0, MAX_STEP, consec, MAX_CONSEC));
  EXPECT_EQ(consec, 1);
}

// Un outlier suivi d'une lecture normale : le compteur est remis a zero.
TEST(OutlierFilter, RejectThenNormalResets)
{
  int consec = 0;
  EXPECT_FALSE(accept_with_escape(100.0, MAX_STEP, consec, MAX_CONSEC));
  EXPECT_EQ(consec, 1);
  EXPECT_TRUE(accept_with_escape(2.0, MAX_STEP, consec, MAX_CONSEC));
  EXPECT_EQ(consec, 0);
}

// Porte de sortie : apres MAX_CONSEC rejets consecutifs, la lecture suivante
// (toujours un gros saut) est ACCEPTEE et le compteur remis a zero.
TEST(OutlierFilter, EscapeHatchAfterMaxRejects)
{
  int consec = 0;
  for (int i = 0; i < MAX_CONSEC; ++i) {
    EXPECT_FALSE(accept_with_escape(80.0, MAX_STEP, consec, MAX_CONSEC))
      << "rejet attendu au tour " << i;
  }
  EXPECT_EQ(consec, MAX_CONSEC);
  // Le (MAX_CONSEC+1)e gros saut consecutif : porte de sortie => accepte.
  EXPECT_TRUE(accept_with_escape(80.0, MAX_STEP, consec, MAX_CONSEC));
  EXPECT_EQ(consec, 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
