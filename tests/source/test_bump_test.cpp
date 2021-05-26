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
#include "linear_fit.h"
#include "PIDTemplates/PIFilter.h"
#include "PIDTemplates/PFilter.h"
#include "PIDTemplates/models/DelayIntegratorPlantModel.h"

#if 0
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

class BumpTestFixture: public testing::Test {
public:
  static const constexpr double delay = 1;  // s
  static const constexpr double gain = 10;  // c/control units/s
  static const constexpr double update_rate = 2000;  // Hz
  static const constexpr double ambient = 20;  // c
  static const constexpr double heat_leak = 0;//0.05;//0.1;  // 1/c diff^2 temp lost to ambient factor
  static const constexpr double drive_current = gain*0.1; // c/amp/s -> want a slope of 10c/s
  static const constexpr auto tuning_values =
                            DelayIntegratorSmicTuning(delay, gain, update_rate);

  DelayIntegratorPlantModel plant{delay, gain, update_rate, ambient, heat_leak};
  IIR_PI_Filter<double> temp_filter {tuning_values.first, tuning_values.second, 1/update_rate, ambient};
  std::array<double, static_cast<size_t>(2*delay*update_rate)> bump_test_data{};

};

TEST_F(BumpTestFixture, CheckDelay) {
  temp_filter.set_control(drive_current);
  for (size_t i=0; i<bump_test_data.size(); i++) {
    bump_test_data[i] = plant.update(drive_current);
  }
  //  Check Delay
  for (size_t i=0; i<static_cast<size_t>(delay*update_rate); i++) {
    EXPECT_EQ(bump_test_data[i], ambient);
  }
  EXPECT_GT(bump_test_data[static_cast<size_t>(delay*update_rate)+1], ambient);
}

TEST_F(BumpTestFixture, CheckFit) {
  temp_filter.set_control(drive_current);
  for (size_t i=0; i<bump_test_data.size(); i++) {
    bump_test_data[i] = plant.update(drive_current);
  }
  auto fit = fit_delay_integrator<double>(bump_test_data.data(), bump_test_data.size(), 1, 1);

  //  Slope and offset
  EXPECT_NEAR(static_cast<double>(std::get<0>(fit))/std::get<1>(fit), static_cast<double>(delay*update_rate), double{1});
  EXPECT_NEAR(static_cast<double>(std::get<2>(fit)), 0, double{1});  // Expect a good fit
}

TEST(BumpTest, Translation) {
  static const constexpr double delay = 1;  // s
  static const constexpr double gain = 10;  // c/control units/s
  static const constexpr double update_rate = 2000;  // Hz
  static const constexpr double drive_current = gain*0.1; // c/amp/s -> want a slope of 10c/s

  std::tuple<double, double, double> fit{update_rate*delay, gain/drive_current, 0};
  const double measured_delay = std::get<0>(fit)/update_rate;  //  reading/box -> box/s
  EXPECT_NEAR(measured_delay, delay, delay/100);
  const double measured_gain = std::get<1>(fit)/drive_current; // temp/amp/second from temp/unit/box -> a / drive_current (*units/amps) * update_rate (boxes/s)
  EXPECT_NEAR(measured_gain, gain, gain/100);
}

#if 0
TEST_F(BumpTestFixture, PlotResponse) {
  temp_filter.set_control(drive_current);
  for (size_t i=0; i<bump_test_data.size(); i++) {
	bump_test_data[i] = plant.update(drive_current);
  }
  std::vector<double> vect(bump_test_data.begin(), bump_test_data.end());
  auto fit = fit_delay_integrator<double>(bump_test_data.data(), bump_test_data.size(), 1, 1);
  const double measured_slope = std::get<1>(fit)*update_rate;  //  reading/box -> box/s
  const double g = measured_slope/drive_current; // temp/amp/second from temp/unit/box -> a / drive_current (*units/amps) * update_rate (boxes/s)
  const double x_intercept = std::get<0>(fit);  //  temp at x = 0
  const auto expected_x_intercept = -delay*(gain*drive_current);
  const double y_intercept = -x_intercept/measured_slope;  //  time at t = 0
  const double delay_samples = x_intercept/std::get<1>(fit);
  std::vector<double> gain_fit;
  for (size_t i=0; i<vect.size(); i++) {
	  gain_fit.push_back(static_cast<double>(i)*std::get<1>(fit) + std::get<0>(fit));
  }
  plt::plot(vect);
  plt::plot(gain_fit);
  plt::plot(std::vector<double>{0, bump_test_data.size()}, std::vector<double>{x_intercept, x_intercept});
  plt::plot(std::vector<double>{-y_intercept*update_rate, -y_intercept*update_rate}, std::vector<double>{0, 50});
  plt::show();
}
#endif

TEST_F(BumpTestFixture, translate_parameters) {
  temp_filter.set_control(drive_current);
  for (size_t i=0; i<bump_test_data.size(); i++) {
    bump_test_data[i] = plant.update(drive_current);
  }
  std::vector<double> vect(bump_test_data.begin(), bump_test_data.end());
  auto fit = fit_delay_integrator<double>(bump_test_data.data(), bump_test_data.size(), 1, 1);
  const auto measured_params = translate_parameters<double>(fit, update_rate);
  EXPECT_GT(static_cast<double>(measured_params.first), double{0});  //  positive response coefficient
  EXPECT_NEAR(static_cast<double>(measured_params.second), gain*drive_current, static_cast<double>(measured_params.second/100));
}

#if 0
TEST(find_center_limits, FlatLine) {
  std::array<double, 1000> data{};
  const auto limits = find_center_limits<double, 4>(data.data(), data.size(), 1, 1);
  EXPECT_EQ(std::get<0>(limits), std::get<1>(limits));
  EXPECT_EQ(std::get<0>(limits), 0);
}
#endif

TEST(find_center_limits, DelayedLine) {
  std::array<double, 1000> data{};
  const size_t delay = 100;
  const size_t end = data.size()-delay;
  const double slope = 10;
  for (size_t i=0; i<data.size(); i++) {
    if (i < delay) {
      data[i] = 0;
    } else if (i > end) {
      data[i] = slope*(end-delay);
    } else {
      data[i] = slope*(i-delay);
    }
  }
  const double expected_max = delay + ((end-delay)*3)/4;
  const double expected_min = delay + ((end-delay)*1)/4;
  const auto limits = find_center_limits<double, 25, 75>(data.data(), data.size(), 1, 1);
  EXPECT_EQ(static_cast<double>(std::get<0>(limits)), expected_min);
  EXPECT_EQ(static_cast<double>(std::get<1>(limits)), expected_max);
}

TEST(find_center_limits, DelayedLineDecreasing) {
  std::array<double, 1000> data{};
  const size_t delay = 100;
  const size_t end = data.size()-delay;
  const double slope = -10;
  for (size_t i=0; i<data.size(); i++) {
    if (i < delay) {
      data[i] = 0;
    } else if (i > end) {
      data[i] = slope*(end-delay);
    } else {
      data[i] = slope*(i-delay);
    }
  }
  const double expected_max = delay + ((end-delay)*3)/4;
  const double expected_min = delay + ((end-delay)*1)/4;
  const auto limits = find_center_limits<double, 25, 75>(data.data(), data.size(), 1, 1);
  EXPECT_EQ(static_cast<double>(std::get<0>(limits)), expected_min);
  EXPECT_EQ(static_cast<double>(std::get<1>(limits)), expected_max);
}
