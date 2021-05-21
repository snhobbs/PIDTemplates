/*
 * Copyright 2020 ElectroOptical Innovations, LLC
 * */
#pragma once
#ifndef PIDFILTERS_DELAYINTEGRATORPLANTMODEL_H_
#define PIDFILTERS_DELAYINTEGRATORPLANTMODEL_H_

#include "PIDTemplates/PIFilter.h"
#include <cstdint>

class DelayIntegratorPlantModel {
  const double delay_;
  const double slope_;
  const double update_rate_;
  double temperature_;
  double ambient_;
  const double heat_leak_;

  const double time_step_ = 1./update_rate_;
  uint32_t time_step_count_ = 0;
  double control_ = 0;

  double CalculateHeatLeak(double temperature, double ambient, double factor) {
    //  acts like a control value in the direction of ambient temp
    double temp_diff = (ambient - temperature);
    double heat_leak = temp_diff * temp_diff * factor;
    return heat_leak * (temperature > ambient ? -1 : 1);
  }

 public:
  double update(double control) {
    time_step_count_++;
    double time = time_step_ * time_step_count_;  //  seconds
    if (time > delay_) {
      const double temperature_forcing = CalculateHeatLeak(get_temperature(), ambient_, heat_leak_);
      temperature_ += (slope_ * (control_ + temperature_forcing) * time_step_);
      //  temp/s/control, use last control setting as there is a delay
      control_ = control;
    }
    return get_temperature();
  }
  double get_temperature(void) const { return temperature_; }
  const void set_temperature(double temp) { temperature_ = temp; }

  DelayIntegratorPlantModel(double delay, double slope,
      double update_rate, double temp_start=0, double heat_leak=0)
      : delay_(delay), slope_(slope), update_rate_(update_rate),
      temperature_{temp_start}, ambient_{temp_start}, heat_leak_{heat_leak} {}
};
#endif  //  PIDFILTERS_DELAYINTEGRATORPLANTMODEL_H_
