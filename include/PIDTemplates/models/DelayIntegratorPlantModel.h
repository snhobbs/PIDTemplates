/*
 * Copyright 2020 ElectroOptical Innovations, LLC
 *
 * Host-only simulation model: uses std::deque (heap allocation).
 * Do not include in firmware builds.
 */

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <numeric>
#include <tuple>

class DelayIntegratorPlantModel {
 public:
  DelayIntegratorPlantModel(double delay, double gain, double update_rate,
                            double temp_start = 0, double heat_leak = 0)
      : delay_{delay},
        gain_{gain},
        update_rate_{update_rate},
        temperature_{temp_start},
        ambient_{temp_start},
        heat_leak_{heat_leak} {}

  void reset() {
    history_.clear();
    temperature_ = ambient_;
  }

  double update(double control) {
    const double time_step = 1.0 / update_rate_;
    history_.push_back(control);
    while (history_.size() > static_cast<std::size_t>(delay_ * update_rate_)) {
      const auto setting = history_.front();
      history_.pop_front();
      const double forcing =
          calculate_linear_heat_leak(temperature_, ambient_, heat_leak_);
      temperature_ += (setting + forcing) * time_step * gain_;
    }
    return get_temperature();
  }

  [[nodiscard]] double get_temperature() const { return temperature_; }
  void set_temperature(double temp) { temperature_ = temp; }

 private:
  [[nodiscard]] static double calculate_linear_heat_leak(
      double temperature, double ambient, double factor) {
    return (ambient - temperature) * factor;
  }

  const double delay_;
  const double gain_;
  const double update_rate_;
  double temperature_;
  double ambient_;
  const double heat_leak_;
  std::deque<double> history_{};
};
