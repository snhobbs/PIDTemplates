#include "PIDTemplates/models/DelayIntegratorPlantModel.h"
#include <gtest/gtest.h>

static const double delay = 1;  // s
static const double slope = 0.1;  // c/control units
static const double update_rate = 10;  // Hz
static const double ambient = 20;  // c
static const double heat_leak = 0.1;  // 1/c diff^2 temp lost to ambient factor

TEST(DelayIntegratorModel, Instantiates) {
  DelayIntegratorPlantModel plant{delay, slope,
      update_rate, ambient, heat_leak};
}

TEST(DelayIntegratorModel, FallsToAmbient) {
  DelayIntegratorPlantModel plant{delay, slope,
      update_rate, ambient, heat_leak};
  plant.set_temperature(ambient + 100);
  for(size_t i=0; i<50000; i++) {
    plant.update(0);
  }
  EXPECT_NEAR(plant.get_temperature(), ambient, 0.5);
}

TEST(DelayIntegratorModel, RisesToAmbient) {
  DelayIntegratorPlantModel plant{delay, slope,
      update_rate, ambient, heat_leak};
  plant.set_temperature(ambient - 100);
  for(size_t i=0; i<50000; i++) {
    plant.update(0);
  }
  EXPECT_NEAR(plant.get_temperature(), ambient, 0.5);
}
