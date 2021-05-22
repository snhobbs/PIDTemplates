/*
 * Copyright 2020 ElectroOptical Innovations, LLC
 * PIDFilter.h
 *
 *  Created on: Sep 21, 2018
 *      Author: simon
 */

#pragma once

#include <cassert>
#include <cstdint>
#include <limits>
#include <numeric>
#include <array>

static constexpr std::pair<double, double> DelayIntegratorSmicTuning(double delay_seconds,
										   double slope, // aka gain
										   double update_frequency) {
  const double alpha = 16;
  const double beta = 0.4;
  const double kp = beta / (2 * slope * delay_seconds);
  const double ti = alpha * delay_seconds;
  const double ki = (kp / ti) / update_frequency;
  return std::pair<double, double>{kp, ki};
}

/*
 * Trapasoidal integral (p1+p2)/2 * h
 * */
template<typename type_t>
inline constexpr type_t trapasoid_integral(const type_t p1, const type_t p2, const type_t h=1) {
  const type_t area = ((p1+p2)*h)/2;
  return area;
}


template<typename type_t>
class IIR_PI_Filter{
 private:
  const type_t kp_ = 1;
  const type_t ki_ = 1;
  const type_t sample_time_ = 1;
  type_t set_ = 0;

  std::array<type_t, 2> errors_{};
  type_t integral_ = 0;
  type_t control_ = 0;
  size_t sample_ = 0;

 public:
  IIR_PI_Filter(type_t kp, type_t ki, type_t sample_time, type_t set) : kp_{kp}, ki_{ki}, sample_time_{sample_time}, set_{set} {}

  type_t update(const type_t reading) {
    const type_t error = reading - set_;
    //assert(error + set_ == reading);

    errors_[1] = errors_[0];
    errors_[0] = error;

    //  Give time to prime filter
    if (sample_ >= errors_.size()) {
      integral_ += trapasoid_integral<type_t>(errors_[0], errors_[1], sample_time_);
    }
    control_ = -(error*kp_ + integral_*ki_);
    sample_++;
    return control_;
  }

  type_t get_ki(void) const {
    return ki_;
  }
  type_t get_kp(void) const {
    return kp_;
  }
  type_t get_set(void) const {
    return set_;
  }
  void set_set(type_t set) {
    set_ = set;
  }
  type_t get_control(void) const {
      return control_;
  }
  void set_control(type_t control) {
    control_ = control;
  }
};

