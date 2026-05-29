/*
 * Copyright 2020 Electrooptical Innovations
 */
#include <PIDTemplates/p_filter.hpp>

#include <gtest/gtest.h>
#include <cassert>
#include <cstdlib>

TEST(PFilter, Startup) {
  const double setpoint = 100;
  pid_templates::PFilter<double> filter{1.0, setpoint};
  EXPECT_EQ(setpoint, filter.get_setpoint());
  const double noise_max = (5.0 / 100) * filter.get_setpoint();
  for (std::size_t i = 0; i < 1024; ++i) {
    const auto noise =
        (static_cast<double>(rand()) / RAND_MAX) * noise_max;  // NOLINT
    assert(noise <= noise_max);
    filter.update(filter.get_setpoint() + noise);
    EXPECT_NEAR(0, filter.get_control(), noise_max);
  }
}
