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
  EXPECT_NEAR(slope, calc_slope, slope/100);
  EXPECT_NEAR(intercept, calc_intercept, intercept/100);
  EXPECT_NEAR(residual, 0, 1e-8);
}
