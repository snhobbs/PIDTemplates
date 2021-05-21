/*
 * Copyright 2020 Electrooptical Innovations
 * */
#include <gtest/gtest.h>
#include <cstdint>
#include <iostream>
#include <vector>
#include "PIDTemplates/models/DelayIntegratorPlantModel.h"
#include "PIDTemplates/PIFilter.h"

#include <cnl/fixed_point.h>
#include <cnl/scaled_integer.h>

typedef cnl::scaled_integer<int32_t, cnl::power<-16>> s16_16_t;
typedef cnl::scaled_integer<int32_t, cnl::power<-10>> s22_10_t;

static const double delay = 1;  // s
static const double slope = 0.1;  // c/control units
static const double update_rate = 10;  // Hz
static const double ambient = 20;  // c
static const double heat_leak = 0.1;  // 1/c diff^2 temp lost to ambient factor

TEST(PIDelayIntegratorControl, MaintainsTemp) {
  const std::pair<double, double> tuning_values = DelayIntegratorSmicTuning(delay,
											   slope, // aka gain
											   update_rate);

  const constexpr double set = 10;
  IIR_PI_Filter<double> filter{tuning_values.first, tuning_values.second, 1/update_rate, set};
  DelayIntegratorPlantModel plant{delay, slope,
      update_rate, ambient, heat_leak};

  for(size_t i=0; i<5000; i++) {
    plant.update(filter.update(plant.get_temperature()));
  }
  EXPECT_NEAR(set, plant.get_temperature(), 0.5);
}

TEST(PIDelayIntegratorControl, MaintainsPositiveTemp) {
  const std::pair<double, double> tuning_values = DelayIntegratorSmicTuning(delay,
											   slope, // aka gain
											   update_rate);

  const constexpr double set = 100;
  IIR_PI_Filter<double> filter{tuning_values.first, tuning_values.second, 1/update_rate, set};
  DelayIntegratorPlantModel plant{delay, slope,
      update_rate, ambient, heat_leak};

  for(size_t i=0; i<100000; i++) {
    plant.update(filter.update(plant.get_temperature()));
  }
  EXPECT_NEAR(set, plant.get_temperature(), 0.5);
}
