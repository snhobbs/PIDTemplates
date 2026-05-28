/*
 * Copyright 2020 Electrooptical Innovations
 */
#include <PIDTemplates/pi_filter.hpp>

#include <gtest/gtest.h>
#include <cnl/fixed_point.h>
#include <cnl/scaled_integer.h>
#include <cstdint>

using s16_16_t = cnl::scaled_integer<int32_t, cnl::power<-16>>;
using s22_10_t = cnl::scaled_integer<int32_t, cnl::power<-10>>;
using type_t = double;

TEST(PID, get_set_limits_filter) {
  const pid_templates::FilterLimits<type_t> limits{-1000, 1000};
  pid_templates::PiFilterLimited<type_t> filter{1, 1, 1, limits, 100};
  const pid_templates::FilterLimits<type_t> limits2{-2000, 2000};

  EXPECT_NE(filter.get_limits(), limits2);
  filter.set_limits(limits2);
  EXPECT_EQ(filter.get_limits(), limits2);
}

TEST(PID, Startup) {
  // Filter runs as proportional-only until the 2-sample priming period ends.
  const type_t set = 100;
  pid_templates::PiFilter<type_t> filter{1, 1, 1, set};

  int32_t procv = 100000;
  auto err = procv - filter.get_setpoint();
  auto cntrl = filter.update(procv);

  int32_t expected = -err * filter.get_ki();
  EXPECT_NEAR(cntrl, expected, 1);
}

TEST(PID, Startup_fixed_point) {
  using fp_t = s16_16_t;
  const fp_t set = 100;
  pid_templates::PiFilter<fp_t> filter{1, 1, 1, set};

  fp_t procv = 100000;
  auto err = procv - filter.get_setpoint();
  auto cntrl = filter.update(procv);

  fp_t expected = -err * filter.get_ki();
  EXPECT_EQ(cntrl, expected);
}

TEST(PID, ZeroError) {
  const type_t set = 100;
  pid_templates::PiFilter<type_t> filter{1, 1, 1, set};

  for (std::size_t i = 0; i < 1000; ++i) {
    filter.update(set);
  }
  EXPECT_EQ(filter.get_control(), 0);
}

TEST(PID, UpdateReturnsControl) {
  const type_t set = 100;
  pid_templates::PiFilter<type_t> filter{1, 1, 1, set};

  filter.update(set + 1);
  filter.update(set + 1);

  for (std::size_t i = 0; i < 1000; ++i) {
    const auto control = filter.update(set + 1);
    EXPECT_EQ(filter.get_control(), control);
  }
}

TEST(PID, ControlDecreases) {
  const type_t set = 100;
  pid_templates::PiFilter<type_t> filter{1, 1, 1, set};

  filter.update(set + 1);
  filter.update(set + 1);
  s22_10_t last_control = filter.get_control();

  for (std::size_t i = 0; i < 10; ++i) {
    filter.update(set + 1);
    EXPECT_GT(last_control, filter.get_control());
    EXPECT_GT(0, filter.get_control());
  }
  EXPECT_LT(filter.get_control(), 0);
}

TEST(PID, ConstantError) {
  const type_t set = 100;
  pid_templates::PiFilter<type_t> filter{1, 1, 1, set};

  const s22_10_t error = 20;
  filter.update(set + error);
  EXPECT_EQ(filter.get_control(), -error * filter.get_kp());
  filter.update(set + error);
  EXPECT_EQ(filter.get_control(), -error * filter.get_kp());
  filter.update(set + error);
  EXPECT_EQ(filter.get_control(),
            -(error * filter.get_ki() + error * filter.get_kp()));
}

TEST(PID, ConstantIntegral) {
  const type_t set = 100;
  pid_templates::PiFilter<type_t> filter{1, 1, 1, set};

  const std::size_t loops = 100;
  const s22_10_t error = 0.05;
  for (std::size_t i = 0; i < loops; ++i) {
    filter.update(set + error);
  }
  EXPECT_EQ(filter.get_control(),
            -(static_cast<s22_10_t>(loops - 2) * error * filter.get_ki() +
              error * filter.get_kp()));
}
