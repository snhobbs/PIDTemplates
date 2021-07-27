/*
 * Copyright 2020 Electrooptical Innovations
 * main.cpp

 *
 *      Author: simon
 */

#include <gtest/gtest.h>
#include <stdint.h>
#include <array>
#include <iostream>
#include <vector>

#include "PIDTemplates/PIFilter.h"
#include "PIDTemplates/PFilter.h"
#include "TemperatureController.h"
#include "PIDTemplates/models/DelayIntegratorPlantModel.h"
#if 0
static const constexpr double delay = 5;  // s
static const double gain = 2;  // c/control units/s
static const constexpr double update_rate = 1000;  // Hz
static const double ambient = 20;  // c
static const double heat_leak = 0.075;  // 1/c diff^2 temp lost to ambient factor
static const double drive_current = gain*2;  //  c/s = gain**2*10 -> slope
static const double fit_degree_rise = 5;  // c
const auto params = DelayIntegratorSmicTuning<double>(delay, gain, update_rate);
const size_t buffer_size = (delay + fit_degree_rise/(gain*drive_current))*update_rate;
static_assert(buffer_size > 100, "");
//gain * drive = c/s -> degrees/(gain*drive)
std::array<double, buffer_size> bump_test_data{};
TemperatureController temp{delay, update_rate, ambient, bump_test_data.data(), bump_test_data.size()};
static DelayIntegratorPlantModel plant{delay, gain, update_rate, ambient, heat_leak};

size_t cycles = 0;

static inline void read_adc(double* data) {
  // Read data from peripheral
  data[0] = temp.iprog_filter.get_set();
  data[1] = temp.vcc_filter.get_set();
  data[2] = plant.get_temperature();
  temp.update(cycles++, {data[0], data[1], data[2]});
}

TEST(BumpTest, full) {
	temp.setup_bump_test(drive_current);
	temp.temp_filter.set_set(5);
	  while (true) {
	    std::array<double, 7> data{};
	    read_adc(data.data());
	    plant.update(temp.iprog_filter.get_set());
	    printf("%.4f\n", plant.get_temperature());
	  }
}
#endif
