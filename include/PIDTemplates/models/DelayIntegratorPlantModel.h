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
#include "linear_fit.h"


template<typename type_t, size_t percent>
std::tuple<size_t, size_t> find_center_limits(type_t* data, size_t data_length, size_t start_average=8, size_t end_average=8) {
  static_assert(percent < 50);
  type_t start_value = data[0];
  type_t end_value = data[data_length-1];
  if (start_average > 1) {
    start_value = std::accumulate(&data[0], &data[start_average], 0)/start_average;
  }
  if (end_average > 1) {
    end_value = std::accumulate(&data[data_length-end_average], &data[data_length], 0)/end_average;
  }
  size_t start_position = 0;
  size_t end_position = data_length;

  const type_t difference = std::abs(end_value - start_value);
  type_t plow_value = std::min(start_value, end_value) + difference*(type_t{percent})/100;
  type_t phigh_value = std::min(start_value, end_value) + difference*(type_t{100}-type_t{percent})/100;

  assert(phigh_value < std::max(start_value, end_value));
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

template<typename type_t>
std::tuple<type_t, type_t, type_t> fit_delay_integrator(type_t* data, size_t data_length, size_t start_average=8, size_t end_average=8) {
  // Use least squares fit to the 25% to 75% slope. Extend this line down in time, delay is the number
  // of samples from the start to where this delay starts, the slope is the 25 to 75% section
  if (data_length < (start_average + end_average)) {
    assert(0);
    return {0,0,0};
  }

  const auto limits = find_center_limits<double, 45>(data, data_length, start_average, end_average);
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

  gsl_fit_linear (x_data, 1,
                y_data, 1,
                span,
                &intercept, &slope,
                &cov_00, &cov_01, &cov_11, &residual);


  delete[] x_data;
  //  we have the line with intercept, now extend get the x intercept to give the delay
  return {intercept, slope, residual};
}

class DelayIntegratorPlantModel {
  const double delay_;
  const double slope_;
  const double update_rate_;
  double temperature_;
  double ambient_;
  const double heat_leak_;

  const double time_step_ = 1./update_rate_;
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
  double update(double control) {
    const double time = time_step_ * time_step_count_;  //  seconds
    if (time >= delay_) {
      const double temperature_forcing = LinearCalculateHeatLeak(get_temperature(), ambient_, heat_leak_);
      temperature_ += (slope_ * (control_ + temperature_forcing) * time_step_);
      //  temp/s/control, use last control setting as there is a delay
      control_ = control;
    }
    time_step_count_++;
    return get_temperature();
  }
  double get_temperature(void) const { return temperature_; }
  const void set_temperature(double temp) { temperature_ = temp; }

  DelayIntegratorPlantModel(double delay, double slope,
      double update_rate, double temp_start=0, double heat_leak=0)
      : delay_(delay), slope_(slope), update_rate_(update_rate),
      temperature_{temp_start}, ambient_{temp_start}, heat_leak_{heat_leak} {}
};
