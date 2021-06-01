/*
 * Copyright 2020 Electrooptical Innovations
 * main.cpp

 *
 *      Author: simon
 */

#include <gtest/gtest.h>
#include <stdint.h>
#include <array>
#include <iostream>
#include <vector>
#include <utility>
#include <cstdint>
#include "cnl/fixed_point.h"
#include "cnl/all.h"
#include "linear_fit.h"
#include "PIDTemplates/PIFilter.h"
#include "PIDTemplates/PFilter.h"
#include "PIDTemplates/models/DelayIntegratorPlantModel.h"

using type_t = cnl::fixed_point<signed long long, -20>;

namespace BumpTestFixedPoint {
class BumpTestFixedPointFixture: public testing::Test {
public:
  static const constexpr double delay_ = 1;  // s
  static const constexpr double gain_ = 10;  // c/control units/s
  static const constexpr double update_rate_ = 2000;  // Hz
  static const constexpr double ambient_ = 20;  // c
  static const constexpr double heat_leak_ = 0;//0.05;//0.1;  // 1/c diff^2 temp lost to ambient factor
  static const constexpr double drive_current_ = gain_*0.1; // c/amp/s -> want a slope of 10c/s
  static const constexpr auto tuning_values_ =
                            DelayIntegratorSmicTuning(
                            		(delay_),
									(gain_),
									(update_rate_));

  DelayIntegratorPlantModel plant{delay_, gain_, update_rate_, ambient_, heat_leak_};
  IIR_PI_Filter<type_t> temp_filter {tuning_values_.first, tuning_values_.second, 1/update_rate_, ambient_};
  std::array<type_t, static_cast<size_t>(2*delay_*update_rate_)> bump_test_data{};

};

TEST_F(BumpTestFixedPointFixture, CheckDelay) {
  temp_filter.set_control(drive_current_);
  for (size_t i=0; i<bump_test_data.size(); i++) {
    const double reading = plant.update(drive_current_);
    const type_t reading_t = reading;
    bump_test_data[i] = reading_t;
  }
  //  Check Delay
  for (size_t i=0; i<static_cast<size_t>(delay_*update_rate_); i++) {
    EXPECT_NEAR(static_cast<double>(bump_test_data[i]), ambient_, 1e-2);
  }
  EXPECT_GT(bump_test_data[static_cast<size_t>(delay_*update_rate_)+1], ambient_);
}
#if 1
#if 0
TEST_F(BumpTestFixedPointFixture, CheckFit) {
  temp_filter.set_control(drive_current_);
  for (size_t i=0; i<bump_test_data.size(); i++) {
    bump_test_data[i] = plant.update(drive_current_);
  }
  auto fit = fit_delay_integrator<type_t>(bump_test_data.data(), bump_test_data.size(), 2, size_t{1});

  //  Slope and offset
  assert(std::get<1>(fit) != 0);
  const double delay = static_cast<double>(std::get<0>(fit)/std::get<1>(fit));
  EXPECT_NEAR(delay, static_cast<double>(delay_*update_rate_), double{1e-8});
  EXPECT_NEAR(static_cast<double>(std::get<2>(fit)), 0, 1e-8);  // Expect a good fit
}
#endif

TEST(BumpTest, Translation) {
  static const constexpr type_t delay = 1;  // s
  static const constexpr type_t gain = 10;  // c/control units/s
  static const constexpr type_t update_rate = 2000;  // Hz
  static const constexpr type_t drive_current = gain*0.1; // c/amp/s -> want a slope of 10c/s

  std::tuple<type_t, type_t, type_t> fit{update_rate*delay, gain/drive_current, 0};
  const type_t measured_delay = std::get<0>(fit)/update_rate;  //  reading/box -> box/s
  EXPECT_NEAR(static_cast<double>(measured_delay), static_cast<double>(delay), delay/100.);
  const type_t measured_gain = std::get<1>(fit)/drive_current; // temp/amp/second from temp/unit/box -> a / drive_current (*units/amps) * update_rate (boxes/s)
  EXPECT_NEAR(static_cast<double>(measured_gain), static_cast<double>(gain), gain/100.);
}

#if 0
TEST_F(BumpTestFixedPointFixture, translate_parameters) {
  temp_filter.set_control(drive_current_);
  for (size_t i=0; i<bump_test_data.size(); i++) {
	const double reading = plant.update(drive_current_);
	const type_t reading_t = reading;
	bump_test_data[i] = reading_t;
  }
  std::vector<type_t> vect(bump_test_data.begin(), bump_test_data.end());
  auto fit = fit_delay_integrator<type_t>(bump_test_data.data(), bump_test_data.size(), 2, 1);
  const auto measured_params = translate_parameters<type_t>(fit, update_rate_);
  EXPECT_GT(static_cast<type_t>(measured_params.second), type_t{0});  //  positive response coefficient
  const auto slope = static_cast<double>(measured_params.second);
  EXPECT_NEAR(slope, gain_*drive_current_, slope/100.);
}
#endif

TEST(find_center_limits, DelayedLine) {
  std::array<type_t, 1000> data{};
  const size_t delay = 100;
  const size_t end = data.size()-delay;
  const type_t slope = 10;
  for (size_t i=0; i<data.size(); i++) {
    if (i < delay) {
      data[i] = 0;
    } else if (i > end) {
      data[i] = slope*(end-delay);
    } else {
      data[i] = slope*(i-delay);
    }
  }
  const type_t expected_max = delay + ((end-delay)*3)/4;
  const type_t expected_min = delay + ((end-delay)*1)/4;
  const auto limits = find_center_limits<type_t, 25, 75>(data.data(), data.size(), 1, 1);
  EXPECT_EQ(static_cast<type_t>(std::get<0>(limits)), expected_min);
  EXPECT_EQ(static_cast<type_t>(std::get<1>(limits)), expected_max);
}

TEST(find_center_limits, DelayedLineDecreasing) {
  std::array<type_t, 1000> data{};
  const size_t delay = 100;
  const size_t end = data.size()-delay;
  const type_t slope = -10;
  for (size_t i=0; i<data.size(); i++) {
    if (i < delay) {
      data[i] = 0;
    } else if (i > end) {
      data[i] = slope*(end-delay);
    } else {
      data[i] = slope*(i-delay);
    }
  }
  const type_t expected_max = delay + ((end-delay)*3)/4;
  const type_t expected_min = delay + ((end-delay)*1)/4;
  const auto limits = find_center_limits<type_t, 25, 75>(data.data(), data.size(), 1, 1);
  EXPECT_EQ(static_cast<type_t>(std::get<0>(limits)), expected_min);
  EXPECT_EQ(static_cast<type_t>(std::get<1>(limits)), expected_max);
}
#endif
}

