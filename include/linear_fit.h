#include <stdlib.h>
#include <cstdint>
#include <cmath>


#pragma once

/*
 * Linear regression algorithm
 *
 * */

template<typename type_t, typename float_type=float_t>
inline int fit_linear (const type_t *x,
                const type_t *y,
                const size_t n,
                type_t *c0, type_t *c1,
                type_t *cov_00, type_t *cov_01, type_t *cov_11, type_t *sumsq) {

  float_type m_x = 0, m_y = 0, m_dx2 = 0, m_dxdy = 0;

  /*
   * m_x_i = (x_i - m_x_0_i-1)/(i+1)
   * i/(i+1) + (i+1 - i/i+1)/(i+2)
   * */
  for (size_t i = 0; i < n; i++) {
      m_x += (x[i] - m_x) / (static_cast<float_type>(i) + float_type{1.0});
      m_y += (y[i] - m_y) / (static_cast<float_type>(i) + float_type{1.0});
  }

  for (size_t i = 0; i < n; i++) {
      const float_type dx = x[i] - m_x;
      const float_type dy = y[i] - m_y;

      m_dx2 += (dx * dx - m_dx2) / (static_cast<float_type>(i) + float_type{1.0});
      m_dxdy += (dx * dy - m_dxdy) / (static_cast<float_type>(i) + float_type{1.0});
    }

  /* In terms of y = a + b x */

  {
    float_type d2 = 0;
    assert(m_dx2 > 0);
    const float_type b = m_dxdy / m_dx2;
    const float_type a = m_y - m_x * b;

    *c0 = a;
    *c1 = b;

    /* Compute chi^2 = \sum (y_i - (a + b * x_i))^2 */

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
