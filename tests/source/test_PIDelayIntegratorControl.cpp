/*
 * Copyright 2020 Electrooptical Innovations
 */
#include <PIDTemplates/pi_filter.hpp>
#include <PIDTemplates/models/DelayIntegratorPlantModel.h>

#include <gtest/gtest.h>
#include <cnl/fixed_point.h>
#include <cnl/scaled_integer.h>
#include <cstdint>

using type_t = double;

static const double kDelay      = 1.0;   // s
static const double kSlope      = 0.1;   // °C / control unit
static const double kUpdateRate = 10.0;  // Hz
static const double kAmbient    = 20.0;  // °C
static const double kHeatLeak   = 0.1;   // heat leak factor

TEST(PIDelayIntegratorControl, MaintainsTemp) {
  const auto [kp, ki] = pid_templates::calculate_smic_tuning<type_t>(
      kDelay, kSlope, kUpdateRate);

  constexpr type_t set = 10;
  pid_templates::PiFilter<type_t> filter{kp, ki, 1.0 / kUpdateRate, set};
  DelayIntegratorPlantModel plant{kDelay, kSlope, kUpdateRate, kAmbient, kHeatLeak};

  for (std::size_t i = 0; i < 5000; ++i) {
    plant.update(filter.update(plant.get_temperature()));
  }
  EXPECT_NEAR(set, plant.get_temperature(), 0.5);
}

TEST(PIDelayIntegratorControl, MaintainsPositiveTemp) {
  const auto [kp, ki] = pid_templates::calculate_smic_tuning<type_t>(
      kDelay, kSlope, kUpdateRate);

  constexpr type_t set = 100;
  pid_templates::PiFilter<type_t> filter{kp, ki, 1.0 / kUpdateRate, set};
  DelayIntegratorPlantModel plant{kDelay, kSlope, kUpdateRate, kAmbient, kHeatLeak};

  for (std::size_t i = 0; i < 100000; ++i) {
    plant.update(filter.update(plant.get_temperature()));
  }
  EXPECT_NEAR(set, plant.get_temperature(), 0.5);
}
