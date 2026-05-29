#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace pid_templates {

// Online Welford mean over all n elements of v.
template<typename type_t, typename float_type = float>
[[nodiscard]] inline float_type compute_mean(
    const type_t* v, const size_t n) noexcept {
  float_type mean_v = 0;
  for (size_t i = 0; i < n; i++) {
    const float_type i_f = static_cast<float_type>(i);
    mean_v += (v[i] - mean_v) / (i_f + 1);
  }
  return mean_v;
}

// Online Welford sample covariance of two arrays x and y, length n.
// Returns sum((xi - mean_x)(yi - mean_y)) / (n-1).
template<typename type_t, typename float_type = float>
[[nodiscard]] inline float_type compute_covariance(
    const type_t* x, const type_t* y, const size_t n) noexcept {
  float_type mean_x = 0;
  float_type mean_y = 0;
  float_type c = 0;
  for (size_t i = 0; i < n; i++) {
    const float_type dx = x[i] - mean_x;
    const float_type dy = y[i] - mean_y;
    mean_x += dx / (i + 1);
    c += (x[i] - mean_x) * dy;  // Welford: (x[i] - new_mean_x) * old_dy
    mean_y += dy / (i + 1);
  }
  return c / (n - 1);
}

// Online Welford sample covariance where x = x_start, x_start+1, ..., x_start+n-1.
// Returns sum((xi - mean_x)(yi - mean_y)) / (n-1).
template<typename type_t, typename float_type = float>
[[nodiscard]] inline float_type compute_covariance_evenly_spaced(
    const size_t x_start, const type_t* y, const size_t n) noexcept {
  float_type mean_x = 0;
  float_type mean_y = 0;
  float_type c = 0;
  for (size_t i = 0; i < n; i++) {
    const float_type xi = static_cast<float_type>(x_start + i);
    const float_type dx = xi - mean_x;
    const float_type dy = y[i] - mean_y;
    mean_x += dx / (i + 1);
    c += (xi - mean_x) * dy;  // Welford: (xi - new_mean_x) * old_dy
    mean_y += dy / (i + 1);
  }
  return c / (n - 1);
}

// Fit y = intercept + slope * x using ordinary least squares.
// Output parameters receive the fitted coefficients, covariance matrix entries,
// and total sum of squared residuals.
// Returns 0 on success.
template<typename type_t, typename float_type = float>
[[nodiscard]] inline int fit_linear(
    const type_t* x, const type_t* y, const size_t n,
    type_t* intercept, type_t* slope,
    type_t* cov_00, type_t* cov_01, type_t* cov_11,
    type_t* sum_sq) noexcept {
  const float_type mean_x = compute_mean<type_t, float_type>(x, n);
  const float_type mean_y = compute_mean<type_t, float_type>(y, n);
  const float_type var_x  = compute_covariance<type_t, float_type>(x, x, n);
  const float_type cov_xy = compute_covariance<type_t, float_type>(x, y, n);

  assert(var_x > 0);
  const float_type b = cov_xy / var_x;
  const float_type a = mean_y - mean_x * b;

  *intercept = a;
  *slope     = b;

  float_type d2 = 0;
  for (size_t i = 0; i < n; i++) {
    const float_type dx = x[i] - mean_x;
    const float_type dy = y[i] - mean_y;
    const float_type d  = dy - b * dx;
    d2 += d * d;
  }

  const float_type s2 = d2 / static_cast<float_type>(n - 2);
  *cov_00 = s2 * (float_type{1} / static_cast<float_type>(n))
                * (float_type{1} + mean_x * mean_x / var_x);
  *cov_11 = s2 / (static_cast<float_type>(n) * var_x);
  *cov_01 = s2 * (-mean_x) / (static_cast<float_type>(n) * var_x);
  *sum_sq = d2;

  return 0;
}

// Fit y = intercept + slope * x where x is evenly spaced integers starting at x_start.
// Analytical mean and variance avoid iterating over x, reducing computation.
// Returns 0 on success.
template<typename type_t, typename float_type = float>
[[nodiscard]] inline int fit_linear_evenly_spaced(
    const size_t x_start, const type_t* y, const size_t n,
    type_t* intercept, type_t* slope,
    type_t* cov_00, type_t* cov_01, type_t* cov_11,
    type_t* sum_sq) noexcept {
  // Analytical mean and sample variance of x = x_start .. x_start+n-1.
  const float_type mean_x = static_cast<float_type>(x_start)
                           + (static_cast<float_type>(n) - 1) / 2;
  const float_type mean_y = compute_mean<type_t, float_type>(y, n);
  const float_type var_x  = static_cast<float_type>(n)
                           * static_cast<float_type>(n + 1) / 12;
  const float_type cov_xy =
      compute_covariance_evenly_spaced<type_t, float_type>(x_start, y, n);

  assert(var_x > 0);
  const float_type b = cov_xy / var_x;
  const float_type a = mean_y - mean_x * b;

  *intercept = a;
  *slope     = b;

  float_type d2 = 0;
  for (size_t i = 0; i < n; i++) {
    const float_type dx = static_cast<float_type>(x_start + i) - mean_x;
    const float_type dy = y[i] - mean_y;
    const float_type d  = dy - b * dx;
    d2 += d * d;
  }

  const float_type s2 = d2 / static_cast<float_type>(n - 2);
  *cov_00 = s2 * (float_type{1} / static_cast<float_type>(n))
                * (float_type{1} + mean_x * mean_x / var_x);
  *cov_11 = s2 / (static_cast<float_type>(n) * var_x);
  *cov_01 = s2 * (-mean_x) / (static_cast<float_type>(n) * var_x);
  *sum_sq = d2;

  return 0;
}

}  // namespace pid_templates
