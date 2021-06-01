#include <gtest/gtest.h>
#include <cstdint>
#include <iostream>
#include <vector>
#include "PIDTemplates/models/DelayIntegratorPlantModel.h"
#include "PIDTemplates/PIFilter.h"

TEST(fit_delay_integrator, full_line) {
  std::vector<double> data{};
  for(size_t i=0; i< 1024; i++) {
    data.push_back(-42 + i*.1);
  }
  const auto fit = fit_delay_integrator<double>(data.data(), data.size(), 10, 1);
  EXPECT_NEAR(std::get<0>(fit), -42, 1e-4);
  EXPECT_NEAR(std::get<1>(fit), .1, 1e-4);
}

TEST(fit_delay_integrator, offset) {
  std::vector<double> data{};
  double slope = .1;
  double yintercept = -42;
  size_t offset = 100;
  for(size_t i=0; i< offset; i++) {
    data.push_back(yintercept+offset*slope);
  }
  for(size_t i=offset; i< 1024; i++) {
    data.push_back(yintercept + i*slope);
  }
  const auto fit = fit_delay_integrator<double>(data.data(), data.size(), 10, 1);
  EXPECT_NEAR(std::get<0>(fit), yintercept, 1e-4);
  EXPECT_NEAR(std::get<1>(fit), slope, 1e-4);
}
