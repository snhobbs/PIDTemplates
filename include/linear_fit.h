#pragma once

#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cassert>

/**
 * Linear regression: Fit y = a + b * x
 *
 * Computes coefficients a and b along with the covariance matrix and residual sum of squares.
 *
 * @tparam type_t       Data type of input/output (e.g. double, float)
 * @tparam float_type   Internal accumulation type (e.g. double for precision)
 * @return 0 on success
 */

template<typename type_t, typename float_type=float_t>
inline float_type calculate_mean(const type_t* v, const size_t n) {
  /*
   * Use an online algorithm to calculate mean. Numerically stable.
   * */
  float_type m_v = 0;
  for (size_t i = 0; i < (n-1); i++) {
      const float_type i_f = static_cast<float_type>(i);
      m_v += (v[i+1] - m_v) / (i_f + 1);
  }
  return m_v;
}

template<typename type_t, typename float_type=float_t>
inline float_type calculate_covariance(const type_t* x, const type_t* y, const size_t n) {
  /*
   * Calculate the running variance (m_dx2) and covariance (m_dxdy)
   * m_dx2: the variance of x (actually E[(x - mean_x)^2])
   * m_dxdy: the covariance between x and y (E[(x - mean_x)(y - mean_y)])
   * */
  float_type meanx = 0;
  float_type meany = 0;
  float_type c = 0;

  for (size_t i = 0; i < n; i++) {
    float_type dx = x[i] - meanx;
    float_type dy = y[i] - meany;

    meanx += dx / (i+1);
    meany += dy / (i+1);
    c += dx * dy;
  };

  float_type m_dxdy = c / (n-1);
  return m_dxdy;
}

template<typename type_t, typename float_type=float_t>
inline float_type calculate_covariance_constant_x(const size_t xstart, const type_t* y, const size_t n) {
  /*
   * Calculate the running variance (m_dx2) and covariance (m_dxdy)
   * m_dx2: the variance of x (actually E[(x - mean_x)^2])
   * m_dxdy: the covariance between x and y (E[(x - mean_x)(y - mean_y)])
   * */
  float_type meanx = 0;
  float_type meany = 0;
  float_type c = 0;

  for (size_t i = 0; i < n; i++) {
    float_type dx = (xstart+i) - meanx;
    float_type dy = y[i] - meany;

    meanx += dx / (i+1);
    meany += dy / (i+1);
    c += dx * dy;
  };

  float_type m_dxdy = c / (n-1);
  return m_dxdy;
}

template<typename type_t, typename float_type=float_t>
inline int fit_linear (const type_t *x,
    const type_t *y,
    const size_t n,
    type_t *c0, type_t *c1,
    type_t *cov_00, type_t *cov_01,
    type_t *cov_11, type_t *sumsq) {


  /*
   * Calculate the mean spacing in x and y using a stable online algorithm
   * */
  const float_type m_x = calculate_mean(x, n);
  const float_type m_y = calculate_mean(y, n);
  const float_type m_dx2 = calculate_covariance(x, x, n);
  const float_type m_dxdy = calculate_covariance(x, y, n);

  /*
   * Now we have the variance an covariance we can fit the line
   * slope = cov/var
   * intercept = mean(y) - slope*mean(x)
   * */
  {
    assert(m_dx2 > 0);
    const float_type b = m_dxdy / m_dx2;
    const float_type a = m_y - m_x * b;

    *c0 = a;
    *c1 = b;

    /* Compute chi^2 = \sum (y_i - (a + b * x_i))^2 */
    float_type d2 = 0;
    for (size_t i = 0; i < n; i++)
      {
        const float_type dx = x[i] - m_x;
        const float_type dy = y[i] - m_y;
        const float_type d = dy - b * dx;
        d2 += d * d;
      }

    const size_t degrees_of_freedom = n-2;
    const float_type s2 = d2 / (static_cast<float_type>(degrees_of_freedom));        /* chisq per degree of freedom */

    *cov_00 = s2 * (float_type{1.0} / static_cast<float_type>(n)) * (float_type{1.0} + m_x * m_x / m_dx2);
    *cov_11 = s2 * float_type{1.0} / (static_cast<float_type>(n) * m_dx2);
    *cov_01 = s2 * (-m_x) / (static_cast<float_type>(n) * m_dx2);
    *sumsq = d2;
  }

  return 0;
}


template<typename type_t, typename float_type=float_t>
inline int fit_linear_evenly_spaced (const size_t xstart,
                const type_t *y,
                const size_t n,
                type_t *c0, type_t *c1,
                type_t *cov_00, type_t *cov_01, type_t *cov_11, type_t *sumsq) {

  const float_type m_x = xstart + n/2;
  const float_type m_y = calculate_mean(y, n);
  const float_type m_dx2 = static_cast<float_type>(n*n-1)/12;
  const float_type m_dxdy = calculate_covariance_constant_x(xstart, y, n);

  /*
   * Now we have the variance an covariance we can fit the line
   * slope = cov/var
   * intercept = mean(y) - slope*mean(x)
   * */
  {
    assert(m_dx2 > 0);
    const float_type b = m_dxdy / m_dx2;
    const float_type a = m_y - m_x * b;

    *c0 = a;
    *c1 = b;

    /* Compute chi^2 = \sum (y_i - (a + b * x_i))^2 */
    float_type d2 = 0;
    for (size_t i = 0; i < n; i++)
      {
        const float_type dx = xstart + i - m_x;
        const float_type dy = y[i] - m_y;
        const float_type d = dy - b * dx;
        d2 += d * d;
      }

    const size_t degrees_of_freedom = n-2;
    const float_type s2 = d2 / (static_cast<float_type>(degrees_of_freedom));        /* chisq per degree of freedom */

    *cov_00 = s2 * (float_type{1.0} / static_cast<float_type>(n)) * (float_type{1.0} + m_x * m_x / m_dx2);
    *cov_11 = s2 * float_type{1.0} / (static_cast<float_type>(n) * m_dx2);
    *cov_01 = s2 * (-m_x) / (static_cast<float_type>(n) * m_dx2);
    *sumsq = d2;
  }

  return 0;
}

