/*
 * Copyright 2021 ElectroOptical Innovations, LLC
 */

#ifndef PID_TEMPLATES_P_FILTER_HPP_
#define PID_TEMPLATES_P_FILTER_HPP_

namespace pid_templates {

template <typename T>
class PFilter {
 public:
  PFilter(T kp, T setpoint) noexcept : kp_{kp}, setpoint_{setpoint} {}

  [[nodiscard]] T update(T reading) noexcept {
    const T error = reading - setpoint_;
    control_ = -(error * kp_);
    return control_;
  }

  [[nodiscard]] T get_kp()       const noexcept { return kp_; }
  [[nodiscard]] T get_setpoint() const noexcept { return setpoint_; }
  [[nodiscard]] T get_control()  const noexcept { return control_; }

  void set_setpoint(T setpoint) noexcept {
    setpoint_ = setpoint;
    control_ = T{};
  }

 private:
  const T kp_;
  T setpoint_;
  T control_{};
};

}  // namespace pid_templates

#endif  // PID_TEMPLATES_P_FILTER_HPP_
