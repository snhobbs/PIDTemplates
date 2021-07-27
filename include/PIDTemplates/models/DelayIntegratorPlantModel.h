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
#include <deque>
//  #include <cnl/fixed_point.h>

class DelayIntegratorPlantModel {
  const double delay_;
  const double gain_;
  const double update_rate_;
  double temperature_;
  double ambient_;
  const double heat_leak_;
  std::deque<double> history{};

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
      temperature_{temp_start}, ambient_{temp_start}, heat_leak_{heat_leak} {
        //history.resize(delay*update_rate_ + 1);
      }
  void reset(void) {
    history.clear();
    temperature_ = ambient_;
  }

  double update(double control) {
    const double time_step = 1./update_rate_;
    history.push_back(control);
    while (history.size() > delay_*update_rate_) {
      const auto setting = history.front();
      history.pop_front();
      const double temperature_forcing = LinearCalculateHeatLeak(get_temperature(), ambient_, heat_leak_);
      temperature_ += (setting + temperature_forcing) * time_step * gain_;
      //  amps * dt * dT/dt/amps
      //  temp/s/control, use last control setting as there is a delay
    }
    return get_temperature();
  }

  double get_temperature(void) const { return temperature_; }
  void set_temperature(double temp) { temperature_ = temp; }
};
