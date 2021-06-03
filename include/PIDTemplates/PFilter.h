/*
 * Copyright 2021 ElectroOptical Innovations, LLC
 * PFilter.h
 */

#pragma once

template<typename type_t>
class IIR_P_Filter {
 private:
  const type_t kp_ = 1;
  type_t set_ = 0;
  type_t control_ = 0;

 public:
  IIR_P_Filter(type_t kp, type_t set) : kp_{kp}, set_{set} {}

  type_t update(const type_t reading) {
    const type_t error = reading - set_;
    control_ -= (error*kp_);
    return control_;
  }

  type_t get_kp(void) const {
    return kp_;
  }
  void set_set(const type_t set) {
    set_ = set;
  }
  type_t get_set(void) const {
    return set_;
  }
  type_t get_control(void) const {
      return control_;
  }
};
