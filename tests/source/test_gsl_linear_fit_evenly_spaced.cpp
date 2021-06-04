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
#include "cnl/fixed_point.h"
#include "cnl/all.h"


#if 0
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

namespace CheckFitEvenlySpaced {
template<typename type_t>
void check_fit() {
  const double slope_ = 2.3;
  const double intercept_ = 493;
  std::array<type_t, 1024> y{};
  for (size_t i=0; i<y.size(); i++) {
    y[i] = i*slope_ + intercept_;
  }
  type_t intercept = 0;
  type_t slope = 0;
  type_t residual = 0;
  type_t cov_00 = 0;
  type_t cov_01 = 0;
  type_t cov_11 = 0;
  fit_linear_evenly_spaced<type_t, float> (0,
        y.data(),
        y.size(),
        &intercept, &slope,
        &cov_00, &cov_01, &cov_11, &residual);

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

TEST(linear_fit_even_spacing, CheckFit) {
  check_fit<double>();
}

TEST(linear_fit_even_spacing, CheckFit_32_8) {
  check_fit<cnl::fixed_point<int32_t, -8>>();
}
}
