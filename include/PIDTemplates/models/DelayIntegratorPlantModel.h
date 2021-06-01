/*
 * Copyright 2020 ElectroOptical Innovations, LLC
 * */
#pragma once
#include <cstdint>
#include <tuple>
#include <algorithm>
#include <numeric>
#include <cassert>
#include <cmath>
#include <cnl/all.h>
#include "linear_fit.h"

template<typename type_t>
type_t accumulate(type_t* start, type_t* end, type_t value) {
  for (type_t* p=start; p!=end; p++) {
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
std::tuple<type_t, type_t, type_t> fit_delay_integrator(type_t* data, size_t data_length, const type_t temp_rise=2, size_t start_average=8) {
  // Use least squares fit to the 25% to 75% slope. Extend this line down in time, delay is the number
  // of samples from the start to where this delay starts, the slope is the 25 to 75% section
  if (data_length <= (start_average)) {
    assert(0);
    return {0,0,0};
  }

  const type_t divisor_start = type_t{0} >= start_average? 1 : start_average;
  const type_t averaged_start_temp = accumulate<type_t>(&data[0], &data[start_average], type_t{0})/divisor_start;
  const type_t temp_start = averaged_start_temp + temp_rise;
  size_t start_position = 0;

  assert(temp_rise > 0 || data[0] > data[data_length-1]);  //  either positive change or decreasing signal
  assert(temp_rise < 0 || data[0] < data[data_length-1]);  //  either negative change or increaing signal

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
  type_t* x_data = new type_t[span];
  type_t* y_data = &data[std::get<0>(limits)];
  assert(x_data != nullptr);
  for (size_t i=0; i<span; i++) {
    x_data[i] = static_cast<double>(std::get<0>(limits)+i);
  }

  fit_linear<type_t, double> (x_data,
                y_data,
                span,
                &intercept, &slope,
                &cov_00, &cov_01, &cov_11, &residual);


  delete[] x_data;
  //  we have the line with intercept, now extend get the x intercept to give the delay
  //  intercept -= static_cast<type_t>(std::get<0>(limits))*slope;
  return {intercept, slope, residual};
}

//  Factor in the update rate and return the scaled values
//  Takes the {intercept, slope, residual} in update rate units of time
//  and return {delay, gain}
template<typename type_t>
std::pair<type_t, type_t> translate_parameters(const std::tuple<type_t, type_t, type_t>& fit, size_t update_rate, type_t drive, type_t start_temp) {
  const type_t measured_slope = std::get<1>(fit)/update_rate;  //  reading/box -> box/s
  const type_t gain = measured_slope/drive;
  const type_t y_intercept = std::get<0>(fit);  //  temp at x = 0
  //  Delay is when the line crosses the initial temperature
  //  ambient = slope*delay + yintercept
  //  delay = (ambient-intercept)/slope
  const type_t delay = (start_temp - y_intercept)/measured_slope;
  assert(delay>0);
  return {delay, gain};
}

class DelayIntegratorPlantModel {
  const double delay_;
  const double gain_;
  const double update_rate_;
  double temperature_;
  double ambient_;
  const double heat_leak_;
  size_t time_step_count_ = 0;
  double control_ = 0;

  double QuadraticCalculateHeatLeak(double temperature, double ambient, double factor) {
    //  acts like a control value in the direction of ambient temp
    const double temp_diff = (ambient - temperature);
    const double heat_leak = temp_diff * temp_diff * factor;
    return heat_leak * (temperature > ambient ? -1 : 1);
  }

  double LinearCalculateHeatLeak(double temperature, double ambient, double factor) {
    //  acts like a control value in the direction of ambient temp
    return (ambient - temperature) * factor;  //  temp > ambient -> negative leak
  }

 public:
  DelayIntegratorPlantModel(double delay, double gain,
      double update_rate, double temp_start=0, double heat_leak=0)
      : delay_(delay), gain_(gain), update_rate_(update_rate),
      temperature_{temp_start}, ambient_{temp_start}, heat_leak_{heat_leak} {}
[[deprecated]]
  double update(double control) {
    return update(control, 1);
  }

  double update(const double control, const size_t nsteps) {
    const double time_step = 1./update_rate_;
    time_step_count_+= nsteps;
    const double time = time_step * time_step_count_;  //  seconds
    if (time >= delay_) {
      const double temperature_forcing = LinearCalculateHeatLeak(get_temperature(), ambient_, heat_leak_);
      temperature_ += (control_ + temperature_forcing) * time_step * nsteps * gain_;
      //  amps * dt * dT/dt/amps
      //  temp/s/control, use last control setting as there is a delay
      control_ = control;
    }
    return get_temperature();
  }

  double get_temperature(void) const { return temperature_; }
  const void set_temperature(double temp) { temperature_ = temp; }
};
