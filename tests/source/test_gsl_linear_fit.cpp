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
#include "cnl/scaled_integer.h"
#include "cnl/all.h"


#if 0
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

template<typename type_t>
void check_fit() {
  const double slope_ = 2.3;
  const double intercept_ = 493;
  std::array<type_t, 1024> y{};
  std::array<type_t, 1024> x{};
  static_assert(sizeof(y) == sizeof(x));
  for (size_t i=0; i<y.size(); i++) {
	x[i] = i;
	y[i] = x[i]*slope_ + intercept_;
  }
  type_t intercept = 0;
  type_t slope = 0;
  type_t residual = 0;
  type_t cov_00 = 0;
  type_t cov_01 = 0;
  type_t cov_11 = 0;
  fit_linear<type_t, float> (x.data(),
				y.data(),
				x.size(),
				&intercept, &slope,
				&cov_00, &cov_01, &cov_11, &residual);

  std::array<type_t, 1024> fit_line{};
  for (size_t i=0; i<fit_line.size(); i++) {
    fit_line[i] = x[i]*type_t{slope} + type_t{intercept};
  }
#if 0
  plt::clf();
  plt::plot(std::vector<double>{x.begin(),x.end()}, std::vector<double>{y.begin(),y.end()});
  plt::plot(std::vector<double>{x.begin(),x.end()}, std::vector<double>{fit_line.begin(),fit_line.end()});
  plt::show();
#endif

  //  Slope and offset
  EXPECT_NEAR(static_cast<double>(slope_), double{slope}, double{slope_/100});
  EXPECT_NEAR(static_cast<double>(intercept_), double{intercept}, double{intercept_/100});
  EXPECT_NEAR(static_cast<double>(residual), double{0}, double{1});
}

TEST(linear_fit, CheckFit) {
	check_fit<double>();
}

TEST(linear_fit, CheckFit_32_8) {
  using fixed_32_n8 = cnl::scaled_integer<int32_t, cnl::power<-8>>;
	check_fit<fixed_32_n8>();
}

TEST(gsl_fit_linear, line) {
  std::array<double, 1024> x {};
  std::array<double, 1024> y {};
  double slope = 2.5;
  double intercept = 7.18;
  for (size_t i=0; i<x.size(); i++) {
    x[i] = i;
    y[i] = x[i] *slope + intercept;
  }
  double calc_intercept = 0;
  double calc_slope = 0;
  double cov_00 = 0;
  double cov_01 = 0;
  double cov_11 = 0;
  double residual = 0;
  fit_linear<double, double> (x.data(),
	                y.data(),
	                x.size(),
	                &calc_intercept, &calc_slope,
	                &cov_00, &cov_01, &cov_11, &residual);
  EXPECT_NEAR(slope, calc_slope, slope/1000);
  EXPECT_NEAR(intercept, calc_intercept, intercept/1000);
  EXPECT_NEAR(residual, 0, 1e-5);
}
