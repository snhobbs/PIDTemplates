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
#include "linear_fit.h"
#include "PIDTemplates/PIFilter.h"
#include "PIDTemplates/PFilter.h"
#include "PIDTemplates/models/DelayIntegratorPlantModel.h"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class BumpTestFixture: public testing::Test {
public:
  static const constexpr double delay = 1;  // s
  static const constexpr double gain = 10;  // c/control units/s
  static const constexpr double update_rate = 2000;  // Hz
  static const constexpr double ambient = 20;  // c
  static const constexpr double heat_leak = 0;//0.05;//0.1;  // 1/c diff^2 temp lost to ambient factor
  static const constexpr double drive_current = gain*0.1; // c/amp/s -> want a slope of 10c/s
  static const constexpr std::pair<double, double> tuning_values =
                            DelayIntegratorSmicTuning(delay, gain, update_rate);

  DelayIntegratorPlantModel plant{delay, gain, update_rate, ambient, heat_leak};
  IIR_PI_Filter<double> temp_filter {tuning_values.first, tuning_values.second, 1/update_rate, ambient};
  std::array<double, static_cast<size_t>(5*delay*update_rate)> bump_test_data{};

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
#if 0
  std::vector<double> vect(bump_test_data.begin(), bump_test_data.end());
  plt::plot(vect);
#endif

  auto fit = fit_delay_integrator<double>(bump_test_data.data(), bump_test_data.size(), 1, 1);

  //  Slope and offset
  EXPECT_NEAR(std::get<0>(fit)/std::get<1>(fit), delay*update_rate, 1e-8);
  EXPECT_NEAR(std::get<2>(fit), 0, 1e-8);  // Expect a good fit
#if 0
  plt::plot(std::vector<double>{0, bump_test_data.size()}, std::vector<double>{std::get<0>(fit), std::get<0>(fit)});
  plt::plot(std::vector<double>{std::get<0>(fit)/std::get<1>(fit), std::get<0>(fit)/std::get<1>(fit)}, std::vector<double>{0, 50});
  plt::show();
#endif
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
#if 0
  const double x_intercept = std::get<0>(fit);  //  temp at x = 0
  const double expected_x_intercept = -delay*(gain*drive_current);
  EXPECT_NEAR(x_intercept, expected_x_intercept, expected_x_intercept/100);
  //  mx + b = 0
  //  -b/m = x
  const double y_intercept = -x_intercept/measured_slope;  //  time at t = 0
  EXPECT_NEAR(x_intercept/std::get<1>(fit), delay*update_rate, 1e-8);
  EXPECT_NEAR(y_intercept, delay, delay/100);
#endif
}

TEST_F(BumpTestFixture, Translation) {
  temp_filter.set_control(drive_current);
  for (size_t i=0; i<bump_test_data.size(); i++) {
    bump_test_data[i] = plant.update(drive_current);
  }
  std::vector<double> vect(bump_test_data.begin(), bump_test_data.end());
  //plt::plot(vect);

  auto fit = fit_delay_integrator<double>(bump_test_data.data(), bump_test_data.size(), 1, 1);

  //  Slope and offset
  EXPECT_NEAR(std::get<0>(fit)/std::get<1>(fit), delay*update_rate, 1e-8);
  EXPECT_NEAR(std::get<2>(fit), 0, 1e08);  // Expect a good fit

  const double measured_slope = std::get<1>(fit)*update_rate;  //  reading/box -> box/s
  const double g = measured_slope/drive_current; // temp/amp/second from temp/unit/box -> a / drive_current (*units/amps) * update_rate (boxes/s)
  EXPECT_GT(g, 0);
  const double x_intercept = std::get<0>(fit);  //  temp at x = 0
  printf("xintercept %E\n", x_intercept);
  EXPECT_LT(x_intercept, 0);
  const auto expected_x_intercept = -delay*(gain*drive_current);
  EXPECT_NEAR(x_intercept, expected_x_intercept, expected_x_intercept/100);
  //  mx + b = 0
  //  -b/m = x
  const double y_intercept = -x_intercept/measured_slope;  //  time at t = 0
  EXPECT_GT(y_intercept, 0);
  EXPECT_NEAR(x_intercept/std::get<1>(fit), delay*update_rate, 1e-8);
  printf("%.4E, %.4E, %.4E\n", y_intercept, g, std::get<2>(fit));
  printf("%.4E, %.4E\n", delay, gain);
  EXPECT_NEAR(g, gain, gain/100);
  EXPECT_NEAR(y_intercept, delay, delay/100);
  EXPECT_NEAR(std::get<2>(fit), 0, 1);  // Expect a good fit
  //plt::plot(std::vector<double>{0, bump_test_data.size()}, std::vector<double>{x_intercept, x_intercept});
  //plt::plot(std::vector<double>{-y_intercept*update_rate, -y_intercept*update_rate}, std::vector<double>{0, 50});
  //plt::show();
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
  const auto limits = find_center_limits<double, 25>(data.data(), data.size(), 1, 1);
  EXPECT_EQ(std::get<0>(limits), expected_min);
  EXPECT_EQ(std::get<1>(limits), expected_max);
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
  const auto limits = find_center_limits<double, 25>(data.data(), data.size(), 1, 1);
  EXPECT_EQ(std::get<0>(limits), expected_min);
  EXPECT_EQ(std::get<1>(limits), expected_max);
}
