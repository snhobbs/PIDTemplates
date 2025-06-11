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
#include "PIDTemplates/models/FitDelayIntegratorPlantModel.h"

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
  auto fit = fit_delay_integrator<double>(bump_test_data.data(), bump_test_data.size(), 2, 4);

  //  Slope and offset
  EXPECT_NEAR(static_cast<double>(std::get<0>(fit))/std::get<1>(fit), static_cast<double>(delay*update_rate), update_rate/100);
  EXPECT_NEAR(static_cast<double>(std::get<2>(fit)), 0, double{1});  // Expect a good fit
}

/*
 * y intercept -> x = 0
 * line ignores delay
 * ambient + (delay)*slope
 * */
TEST(fit_delay_integrator, correct_y_intercept) {
  const size_t update_rate = 1000;
  const double delay = 3;
  const double slope = 10;
  const double ambient = 35;
  std::vector<double> data{};
  size_t i = 0;
  while(true) {
    double pt = ambient;
    if (i > delay*update_rate) {
      pt = (static_cast<double>(i)/update_rate-delay)*slope;
    }
    //printf("%E\n", pt);
    data.push_back(pt);
    if (pt > ambient + 5) {
      break;
    }
    i++;
  }
  auto fit = fit_delay_integrator<double>(data.data(), data.size(), 2, 1);
  EXPECT_NEAR(-std::get<0>(fit)/(slope), delay, 0.001);
  EXPECT_NEAR(std::get<1>(fit)*update_rate, slope, slope/100);
  EXPECT_NEAR(static_cast<double>(std::get<2>(fit)), 0, 1e-4);  // Expect a good fit
}

#if 0
class translate_parameters_Fixture : public testing::Test {
 public:
  static const constexpr double ambient = 20;
  static const constexpr double delay = 1.2;  // s
  static const constexpr double gain = 13;  // c/control units/s
  static const constexpr double update_rate = 2000;  // Hz
  static const constexpr double drive_current = gain*0.1; // c/amp/s -> want a slope of 10c/s
  //const double x_intercept = ambient - (delay)*gain*drive_current;
  const double slope = gain*drive_current * update_rate;
  const double y_intercept = (ambient - delay*gain*drive_current);
  std::tuple<double, double, double> fit{y_intercept, slope, 0};
};

TEST_F(translate_parameters_Fixture, delay) {
  const auto params = translate_parameters<double>(fit, update_rate, drive_current, ambient);
  EXPECT_NEAR(params.first, delay, delay/1000);
}
TEST_F(translate_parameters_Fixture, gain) {
  const auto params = translate_parameters<double>(fit, update_rate, drive_current, ambient);
  EXPECT_NEAR(params.second, gain, gain/1000);
}
#endif


#if 0
TEST_F(BumpTestFixture, PlotResponse) {
  temp_filter.set_control(drive_current);
  for (size_t i=0; i<bump_test_data.size(); i++) {
	bump_test_data[i] = plant.update(drive_current);
  }
  std::vector<double> vect(bump_test_data.begin(), bump_test_data.end());
  auto fit = fit_delay_integrator<double>(bump_test_data.data(), bump_test_data.size(), 2, 1);
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

/*
 * Returns the delay and gain given the fit to the line
 * */
TEST_F(BumpTestFixture, translate_parameters) {
  const std::pair measured_params {delay, gain*drive_current};
  //EXPECT_GT(static_cast<double>(measured_params.first), double{0});  //  positive response coefficient
  EXPECT_NEAR(static_cast<double>(measured_params.first), delay, static_cast<double>(delay/100));
  EXPECT_NEAR(static_cast<double>(measured_params.second), gain, static_cast<double>(measured_params.second/100));
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
