/*
 * Copyright 2020 ElectroOptical Innovations, LLC
 * PIDFilter.h
 */

#pragma once

#include <array>
#include <cassert>
#include <utility>

template <typename type_t>
static constexpr std::pair<type_t, type_t> calculate_smic_tuning_parameters(
    type_t delay, type_t slope, type_t update_frequency, type_t tc=2) {
  const type_t mod_tc = delay * (1 + tc);
  const type_t ti = 4 * mod_tc;
  const type_t kp = 1 / (slope*mod_tc);
  const type_t ki = (kp / ti) / update_frequency;
  return {kp, ki};
}


#if 0
template <typename type_t>
static constexpr std::pair<type_t, type_t> DelayIntegratorFromSmicTuning(
    type_t kp, type_t ki, type_t update_frequency) {
  const type_t alpha = 16;
  const type_t beta = 0.4;
  const type_t ti = (kp / ki) / update_frequency;
  const type_t delay_seconds = ti / alpha;
  const type_t gain = beta / (2 * kp * delay_seconds);
  return {delay_seconds, gain};
}
#endif

/*
 * Trapasoidal integral (p1+p2)/2 * h
 * */
template <typename type_t>
inline constexpr type_t trapezoid_integral(const type_t p1, const type_t p2,
                                           const type_t h = 1) {
  const type_t area = ((p1 + p2) * h) / 2;
  return area;
}


template <typename type_t>
struct PI_Filter_Status{
  type_t kp{};
  type_t ki{};
  type_t set{};
  type_t integral{};
  type_t error{};
};



template <typename type_t>
class IIR_PI_Filter {
 protected:
  type_t kp_ = 1;
  type_t ki_ = 1;
  const type_t sample_time_ = 1;
  type_t set_ = 0;

  std::array<type_t, 2> errors_{};
  type_t integral_ = 0;
  type_t control_ = 0;
  std::size_t sample_ = 0;

 public:
  IIR_PI_Filter(type_t kp, type_t ki, type_t sample_time,
      type_t set=0)
      : kp_{kp}, ki_{ki}, sample_time_{sample_time}, set_{set} {}


  void reset() {
	  integral_ = 0;
	  control_ = 0;
	  sample_ = 0;
  }

  type_t update(const type_t reading) {
    const type_t error = reading - set_;
    // assert(error + set_ == reading);

    errors_[1] = errors_[0];
    errors_[0] = error;

    const type_t new_integral = integral_ +
      trapezoid_integral<type_t>(errors_[0], errors_[1], sample_time_);
    const type_t new_control = -(error * kp_ + new_integral * ki_);

    //  Give time to prime filter and check for overflowing the
    //  filter limits. Only update the integral if we're within the limits.
    if (sample_ >= errors_.size()) {
      integral_ = new_integral;
      control_ = new_control;
    } else {
      control_ = -(error * kp_ + integral_ * ki_);
    }
    sample_++;
    return control_;
  }

  type_t update_in_limit(const type_t reading) {
	const type_t error = reading - set_;

	errors_[1] = errors_[0];
	errors_[0] = error;
    control_ = -(error * kp_ + integral_ * ki_);
	sample_++;
	return control_;
  }

  type_t get_error(void) const { return errors_[0]; }

  type_t get_integral(void) const { return integral_; }

  type_t get_ki(void) const { return ki_; }
  void set_ki(type_t ki) { ki_ = ki; }

  type_t get_kp(void) const { return kp_; }
  void set_kp(type_t kp) { kp_ = kp; }

  type_t get_set(void) const { return set_; }
  void set_set(type_t set) {
	reset();
    set_ = set;
  }

  type_t get_control(void) const { return control_; }
  void set_control(type_t control) { control_ = control; }

  size_t get_sample(void) const { return sample_; }
  PI_Filter_Status<type_t> get_status(void) const {
    const PI_Filter_Status<type_t> status {
      get_kp(),
      get_ki(),
      get_set(),
      get_integral(),
      get_error()
    };
    return status;
  }
};

template <typename type_t>
class IIR_PI_Filter_Limited: public IIR_PI_Filter<type_t> {
public:
	std::pair<type_t, type_t> limits_{};

	 public:
	  IIR_PI_Filter_Limited(type_t kp, type_t ki, type_t sample_time,
		  std::pair<type_t, type_t> limits,
	      type_t set=0)
	      : IIR_PI_Filter<type_t>{kp, ki, sample_time, set}, limits_{limits} {}
	  std::pair<type_t, type_t> get_limits() const {
		  return limits_;
	  }

	  type_t update_with_limits(const type_t reading) {
	    const type_t error = reading - this->set_;
	    // assert(error + set_ == reading);

	    this->errors_[1] = this->errors_[0];
	    this->errors_[0] = error;

	    const type_t new_integral = this->integral_ +
	      trapezoid_integral<type_t>(this->errors_[0], this->errors_[1], this->sample_time_);
	    const type_t new_control = -(error * this->kp_ + new_integral * this->ki_);

	    //  Give time to prime filter and check for overflowing the
	    //  filter limits. Only update the integral if we're within the limits.
	    if (this->sample_ >= this->errors_.size() &&
	        new_control >= limits_.first &&
	        new_control <= limits_.second) {
	    	set_integral(new_integral);
	    	set_control(new_control);
	    } else {
	      set_control( -(error * this->kp_ + this->integral_ * this->ki_));
	    }
	    this->sample_++;
	    return this->get_control();
	  }

	  void set_limits(const std::pair<type_t, type_t>& limits) {
		  limits_ = limits;
	  }
};
