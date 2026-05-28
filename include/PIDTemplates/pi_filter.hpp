/*
 * Copyright 2020 ElectroOptical Innovations, LLC
 */

#ifndef PID_TEMPLATES_PI_FILTER_HPP_
#define PID_TEMPLATES_PI_FILTER_HPP_

#include <array>
#include <cstddef>
#include <utility>

namespace pid_templates {

template <typename T>
struct FilterLimits {
    T low{};
    T high{};
    constexpr FilterLimits(T low_in, T high_in) noexcept
        : low{low_in}, high{high_in} {}
};

template <typename T>
[[nodiscard]] constexpr bool operator==(
    const FilterLimits<T>& a, const FilterLimits<T>& b) noexcept {
    return a.low == b.low && a.high == b.high;
}

template <typename T>
[[nodiscard]] constexpr bool operator!=(
    const FilterLimits<T>& a, const FilterLimits<T>& b) noexcept {
    return !(a == b);
}

template <typename T>
struct PiFilterStatus {
    T kp{};
    T ki{};
    T setpoint{};
    T integral{};
    T error{};
};

// Trapezoidal integral: (p1 + p2) / 2 * h
template <typename T>
[[nodiscard]] constexpr T trapezoid_integral(T p1, T p2, T h = T{1}) noexcept {
    return ((p1 + p2) * h) / T{2};
}

// SMIC (Simplified Model-based Internal Control) tuning for a delay-integrator plant.
// Returns {kp, ki} for a PI controller.
template <typename T>
[[nodiscard]] constexpr std::pair<T, T> calculate_smic_tuning(
    T delay, T slope, T update_frequency, T tc = T{2}) noexcept {
    // SMIC formula: ti = 4 * (delay * (1 + tc)), kp = 1 / (slope * mod_tc)
    constexpr T kTiMultiplier{4};
    const T mod_tc = delay * (T{1} + tc);
    const T ti = kTiMultiplier * mod_tc;
    const T kp = T{1} / (slope * mod_tc);
    const T ki = (kp / ti) / update_frequency;
    return {kp, ki};
}


// Discrete IIR PI filter with trapezoidal integration.
//
// Three update modes:
//   update()          — normal; commits integral after 2-sample priming period
//   update_frozen()   — anti-windup; freezes integral (proportional-only)
//   update_limited()  — commits integral only when output stays within [low, high]
template <typename T>
class PiFilter {
 public:
  // Two error samples are required before the integral begins accumulating.
  static constexpr std::size_t kErrorHistory = 2u;

  PiFilter(T kp, T ki, T sample_time, T setpoint = T{}) noexcept
      : kp_{kp}, ki_{ki}, sample_time_{sample_time}, setpoint_{setpoint} {}

  [[nodiscard]] T update(T reading) noexcept {
    const T error = reading - setpoint_;
    shift_errors(error);

    const T new_integral =
        integral_ + trapezoid_integral<T>(errors_[0], errors_[1], sample_time_);
    const T new_control = -(error * kp_ + new_integral * ki_);

    if (sample_ >= kErrorHistory) {
      integral_ = new_integral;
      control_ = new_control;
    } else {
      control_ = -(error * kp_ + integral_ * ki_);
    }
    if (sample_ < kErrorHistory) { ++sample_; }
    return control_;
  }

  [[nodiscard]] T update_frozen(T reading) noexcept {
    const T error = reading - setpoint_;
    shift_errors(error);
    control_ = -(error * kp_ + integral_ * ki_);
    if (sample_ < kErrorHistory) { ++sample_; }
    return control_;
  }

  [[nodiscard]] T update_limited(T reading, T low, T high) noexcept {
    const T error = reading - setpoint_;
    shift_errors(error);

    const T new_integral =
        integral_ + trapezoid_integral<T>(errors_[0], errors_[1], sample_time_);
    const T new_control = -(error * kp_ + new_integral * ki_);

    if (sample_ >= kErrorHistory &&
        new_control >= low && new_control <= high) {
      integral_ = new_integral;
      control_ = new_control;
    } else {
      control_ = -(error * kp_ + integral_ * ki_);
    }
    if (sample_ < kErrorHistory) { ++sample_; }
    return control_;
  }

  void reset() noexcept {
    integral_ = T{};
    control_ = T{};
    sample_ = 0u;
    errors_ = {};
  }

  [[nodiscard]] T get_kp()       const noexcept { return kp_; }
  [[nodiscard]] T get_ki()       const noexcept { return ki_; }
  [[nodiscard]] T get_setpoint() const noexcept { return setpoint_; }
  [[nodiscard]] T get_error()    const noexcept { return errors_[0]; }
  [[nodiscard]] T get_integral() const noexcept { return integral_; }
  [[nodiscard]] T get_control()  const noexcept { return control_; }
  [[nodiscard]] std::size_t get_sample() const noexcept { return sample_; }

  [[nodiscard]] PiFilterStatus<T> get_status() const noexcept {
    return {kp_, ki_, setpoint_, integral_, errors_[0]};
  }

  void set_kp(T kp)           noexcept { kp_ = kp; reset(); }
  void set_ki(T ki)           noexcept { ki_ = ki; reset(); }
  // Sets both gains atomically with a single reset.
  void set_coeffs(T kp, T ki) noexcept { kp_ = kp; ki_ = ki; reset(); }
  void set_setpoint(T s)      noexcept { setpoint_ = s; reset(); }
  void set_control(T control) noexcept { control_ = control; }

 private:
  void shift_errors(T error) noexcept {
    errors_[1] = errors_[0];
    errors_[0] = error;
  }

  T kp_;
  T ki_;
  const T sample_time_;
  T setpoint_;
  std::array<T, kErrorHistory> errors_{};
  T integral_{};
  T control_{};
  std::size_t sample_{0u};
};


// PiFilter with stored output limits. Integral only advances when the
// candidate output stays within [limits.low, limits.high].
template <typename T>
class PiFilterLimited {
 public:
  PiFilterLimited(T kp, T ki, T sample_time, FilterLimits<T> limits,
                  T setpoint = T{}) noexcept
      : filter_{kp, ki, sample_time, setpoint}, limits_{limits} {}

  [[nodiscard]] T update(T reading) noexcept {
    return filter_.update_limited(reading, limits_.low, limits_.high);
  }

  void reset() noexcept { filter_.reset(); }

  [[nodiscard]] FilterLimits<T> get_limits()              const noexcept { return limits_; }
  void set_limits(const FilterLimits<T>& limits)                noexcept { limits_ = limits; }

  [[nodiscard]] T get_kp()           const noexcept { return filter_.get_kp(); }
  [[nodiscard]] T get_ki()           const noexcept { return filter_.get_ki(); }
  [[nodiscard]] T get_setpoint()     const noexcept { return filter_.get_setpoint(); }
  [[nodiscard]] T get_error()        const noexcept { return filter_.get_error(); }
  [[nodiscard]] T get_integral()     const noexcept { return filter_.get_integral(); }
  [[nodiscard]] T get_control()      const noexcept { return filter_.get_control(); }
  [[nodiscard]] std::size_t get_sample() const noexcept { return filter_.get_sample(); }
  [[nodiscard]] PiFilterStatus<T> get_status() const noexcept { return filter_.get_status(); }

  void set_kp(T kp)           noexcept { filter_.set_kp(kp); }
  void set_ki(T ki)           noexcept { filter_.set_ki(ki); }
  void set_setpoint(T s)      noexcept { filter_.set_setpoint(s); }
  void set_control(T control) noexcept { filter_.set_control(control); }

 private:
  PiFilter<T> filter_;
  FilterLimits<T> limits_;
};

}  // namespace pid_templates

#endif  // PID_TEMPLATES_PI_FILTER_HPP_
