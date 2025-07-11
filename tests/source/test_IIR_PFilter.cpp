/*
 * Copyright 2020 Electrooptical Innovations
 * */
#include <gtest/gtest.h>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <vector>

#include <cnl/fixed_point.h>
#include <cnl/scaled_integer.h>
#include <PIDTemplates/PFilter.h>

/*
 * Check that the filter jumps to 0 error and stays their
 */
TEST(PFilter, Startup) {
  const double setpoint = 100;
  IIR_P_Filter<double> filter{1.0, setpoint};
  EXPECT_EQ(setpoint, filter.get_set());
  const double noise_max = (double{5.}/100)*(filter.get_set());
  for (size_t i=0; i<1024; i++) {
    const auto noise = (static_cast<double>(rand())/RAND_MAX)*noise_max;
    assert(noise <= noise_max);
    filter.update(filter.get_set() + noise);  //  5% noise about the setpoint
    EXPECT_NEAR(0, filter.get_control(), noise_max);  //  Expect a control equal to the noise
  }
}

