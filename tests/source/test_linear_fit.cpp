/*
 * Copyright 2020 Electrooptical Innovations
 */

#include "linear_fit.hpp"

#include <gtest/gtest.h>
#include <cnl/all.h>
#include <cnl/scaled_integer.h>
#include <array>
#include <cstdint>

namespace {

template<typename type_t>
void check_fit() {
  constexpr double kSlope     = 2.3;
  constexpr double kIntercept = 493;
  std::array<type_t, 1024> x{};
  std::array<type_t, 1024> y{};
  static_assert(sizeof(y) == sizeof(x));
  for (size_t i = 0; i < y.size(); i++) {
    x[i] = i;
    y[i] = x[i] * kSlope + kIntercept;
  }
  type_t intercept = 0;
  type_t slope     = 0;
  type_t residual  = 0;
  type_t cov_00    = 0;
  type_t cov_01    = 0;
  type_t cov_11    = 0;
  pid_templates::fit_linear<type_t, float>(
      x.data(), y.data(), x.size(),
      &intercept, &slope,
      &cov_00, &cov_01, &cov_11, &residual);

  EXPECT_NEAR(static_cast<double>(kSlope),     double{slope},     double{kSlope / 100});
  EXPECT_NEAR(static_cast<double>(kIntercept), double{intercept}, double{kIntercept / 100});
  EXPECT_NEAR(static_cast<double>(residual),   double{0},         double{1});
}

}  // namespace

TEST(LinearFit, check_fit) {
  check_fit<double>();
}

TEST(LinearFit, check_fit_32_8) {
  using Fixed32n8 = cnl::scaled_integer<int32_t, cnl::power<-8>>;
  check_fit<Fixed32n8>();
}

TEST(FitLinear, fit_linear_double) {
  constexpr double kSlope     = 2.5;
  constexpr double kIntercept = 7.18;
  std::array<double, 1024> x{};
  std::array<double, 1024> y{};
  for (size_t i = 0; i < x.size(); i++) {
    x[i] = i;
    y[i] = x[i] * kSlope + kIntercept;
  }
  double calc_intercept = 0;
  double calc_slope     = 0;
  double cov_00         = 0;
  double cov_01         = 0;
  double cov_11         = 0;
  double residual       = 0;
  pid_templates::fit_linear<double, double>(
      x.data(), y.data(), x.size(),
      &calc_intercept, &calc_slope,
      &cov_00, &cov_01, &cov_11, &residual);

  EXPECT_NEAR(kSlope,     calc_slope,     kSlope / 1000);
  EXPECT_NEAR(kIntercept, calc_intercept, kIntercept / 1000);
  EXPECT_NEAR(residual,   0,              1e-5);
}
