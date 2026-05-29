/*
 * Copyright 2020 Electrooptical Innovations
 */

#include "linear_fit.hpp"

#include <gtest/gtest.h>
#include <cnl/all.h>
#include <cnl/fixed_point.h>
#include <array>
#include <cstdint>

namespace {

template<typename type_t>
void check_fit() {
  constexpr double kSlope     = 2.3;
  constexpr double kIntercept = 493;
  std::array<type_t, 1024> y{};
  for (size_t i = 0; i < y.size(); i++) {
    y[i] = i * kSlope + kIntercept;
  }
  type_t intercept = 0;
  type_t slope     = 0;
  type_t residual  = 0;
  type_t cov_00    = 0;
  type_t cov_01    = 0;
  type_t cov_11    = 0;
  pid_templates::fit_linear_evenly_spaced<type_t, float>(
      0, y.data(), y.size(),
      &intercept, &slope,
      &cov_00, &cov_01, &cov_11, &residual);

  EXPECT_NEAR(static_cast<double>(kSlope),     double{slope},     double{kSlope / 100});
  EXPECT_NEAR(static_cast<double>(kIntercept), double{intercept}, double{kIntercept / 100});
  EXPECT_NEAR(static_cast<double>(residual),   double{0},         double{1});
}

}  // namespace

TEST(LinearFitEvenSpacing, check_fit) {
  check_fit<double>();
}

TEST(LinearFitEvenSpacing, check_fit_32_8) {
  using Fixed32n8 = cnl::scaled_integer<int32_t, cnl::power<-8>>;
  check_fit<Fixed32n8>();
}
