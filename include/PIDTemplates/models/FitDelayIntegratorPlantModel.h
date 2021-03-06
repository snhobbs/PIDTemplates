#pragma once
#include <cstdint>
#include <tuple>
#include <algorithm>
#include <numeric>
#include <cassert>
#include <cmath>
#include <deque>
#include <cnl/fixed_point.h>


#include "linear_fit.h"

template<typename type_t>
type_t accumulate(const type_t* start, const type_t* end, type_t value) {
  for (const type_t* p=start; p!=end; p++) {
    value += *p;
  }
  return value;
}

template<typename type_t, size_t percent, size_t percent_stop>
std::tuple<size_t, size_t> find_center_limits(type_t* data, size_t data_length, size_t start_average=8, size_t end_average=8) {
  const constexpr size_t percent_max = 100;
  static_assert(percent <= percent_max, "");
  static_assert(percent_stop <= percent_max, "");
  static_assert(percent_stop > percent, "");

  const type_t divisor_start = type_t{0} >= start_average? 1 : start_average;
  const type_t divisor_end = type_t{0} >= end_average? 1 : end_average;

  const type_t start_value = accumulate<type_t>(&data[0], &data[start_average], type_t{0})/divisor_start;
  const type_t end_value = accumulate<type_t>(&data[data_length-end_average], &data[data_length], 0)/divisor_end;

  size_t start_position = 0;
  size_t end_position = data_length;

  const type_t difference = end_value < start_value ? start_value - end_value : end_value - start_value;
  const type_t plow_value = std::min(start_value, end_value) + (difference*(percent))/100;
  const type_t phigh_value = std::min(start_value, end_value) + (difference*(percent_stop))/100;

  assert(phigh_value <= std::max(start_value, end_value));
  assert(phigh_value > plow_value);
  assert(plow_value > std::min(start_value, end_value));

  for (size_t i=0; i<data_length; i++) {
  if (end_value < start_value) {  //  decreasing
    if (data[i] <= phigh_value && (start_position == 0)) {  //  take first lower than threshold if decreasing
    start_position = i;
    } else if (data[i] <= plow_value && (end_position == data_length)) {
    end_position = i;
    break;
    }
  } else {  // increasing
    if (data[i] >= plow_value && (start_position == 0)) {  //  take first above threshold if increasing
    start_position = i;
    } else if (data[i] >= phigh_value && (end_position == data_length)) {
    end_position = i;
    break;
    }
  }
  }
  assert(end_position > start_position);
  return {static_cast<size_t>(std::round(start_position)), static_cast<size_t>(std::round(end_position))};
}

/*
 * Fits the line from 5% to the end of the input data. This data needs to be truncated such
 * that the end of the intergration line is still in the linear region.
 * */
template<typename type_t>
std::tuple<type_t, type_t, type_t> fit_delay_integrator(const type_t* data, size_t data_length, const type_t temp_rise=2, size_t start_average=8) {
  if (data_length <= (start_average)) {
    assert(0);
    return {0,0,0};
  }

  const type_t divisor_start = type_t{0} >= start_average? 1 : start_average;
  const type_t averaged_start_temp = accumulate<type_t>(&data[0], &data[start_average], type_t{0})/divisor_start;
  const type_t temp_start = averaged_start_temp + temp_rise;
  assert(temp_rise >= 0 || data[0] > data[data_length-1]);  //  either positive change or decreasing signal
  assert(temp_rise <= 0 || data[0] < data[data_length-1]);  //  either negative change or increaing signal
  size_t start_position = 0;
  for(; start_position<data_length; start_position++) {
    if (data[0] < data[data_length-1]) {  //  increasing
      if (data[start_position] >= temp_start) break;
    } else {
        if (data[start_position] <= temp_start) break;
    }
  }
  const std::pair<size_t, size_t> limits{start_position, data_length-1};
  //  const auto limits = find_levels<type_t, 5, 100>(data, data_length, start_average, end_average);
  type_t intercept = 0;
  type_t slope = 0;
  type_t residual = 0;
  type_t cov_00 = 0;
  type_t cov_01 = 0;
  type_t cov_11 = 0;
  const size_t span = std::get<1>(limits) - std::get<0>(limits) + 1;
  assert(span > 12);
  const type_t* y_data = &data[std::get<0>(limits)];
  fit_linear_evenly_spaced<type_t, double> (std::get<0>(limits),
                y_data,
                span,
                &intercept, &slope,
                &cov_00, &cov_01, &cov_11, &residual);


  //  we have the line with intercept, now extend get the x intercept to give the delay
  //  intercept -= static_cast<type_t>(std::get<0>(limits))*slope;
  return {intercept, slope, residual};
}
//
//  Factor in the update rate and return the scaled values
//  Takes the {intercept, slope, residual} in update rate units of time
//  and return {delay, gain}
template<typename type_t>
std::pair<type_t, type_t> translate_parameters(const std::tuple<type_t, type_t, type_t>& fit, size_t update_rate, type_t drive, type_t start_temp) {
  //  yintercept, slope
  //  y = mx+b
  //  y = (x-delay)*slope
  //  y(0) = b == -delay*slope
  //  m = slope
  //  b = -delay/slope
  //  delay = -b*slope
  //  std::get<1>(fit) -> units/time
  //  std::get<1>(fit) * Hz = units/s
  const type_t slope = (std::get<1>(fit)*update_rate);
  const type_t gain = slope/drive;
  const type_t delay = std::get<0>(fit)*slope;
#if 0
  const type_t measured_slope = std::get<1>(fit)/update_rate;  //  reading/box -> box/s
  const type_t gain = measured_slope/drive;
  const type_t y_intercept = std::get<0>(fit);  //  temp at x = 0
  //  Delay is when the line crosses the initial temperature
  //  ambient = slope*delay + yintercept
  //  delay = (ambient-intercept)/slope
  const type_t delay = (start_temp - y_intercept)/measured_slope;
#endif
  assert(delay>0);
  return {delay, gain};
}


