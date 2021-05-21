/*
 * Copyright 2020 Electrooptical Innovations
 * */
#include <gtest/gtest.h>
#include <cstdint>
#include <iostream>
#include <vector>

#include <cnl/fixed_point.h>
#include <cnl/scaled_integer.h>
#include <PIDTemplates/PIFilter.h>

typedef cnl::scaled_integer<int32_t, cnl::power<-16>> s16_16_t;
typedef cnl::scaled_integer<int32_t, cnl::power<-10>> s22_10_t;

TEST(PID, Startup) {
  /*
   Check that the start of the filter is running as a proportional controller
   until the filter is primed
   */
  IIR_PI_Filter<int32_t> filter{1, 1, 1, 100};

  int32_t procv = 100000;
  auto err = procv - filter.get_set();
  auto cntrl = filter.update(procv);

  int32_t expected = -err*filter.get_ki();
  EXPECT_NEAR(cntrl, expected, 1);
}

TEST(PID, Startup_fixed_point) {
  /*
   Check that the start of the filter is running as a proportional controller
   until the filter is primed
   */
  IIR_PI_Filter<s16_16_t> filter{1, 1, 1, 100};

  s16_16_t procv = 100000;
  auto err = procv - filter.get_set();
  auto cntrl = filter.update(procv);

  s16_16_t expected = -err*filter.get_ki();
  EXPECT_EQ(cntrl, expected);
}

/*
 * Input error of zero, check that the output stays at 0
 * */
TEST(PID, ZeroError) {
  const s16_16_t set = 100;
  IIR_PI_Filter<s16_16_t> filter{1, 1, 1, set};

  for (size_t i=0; i<1000; i++) {
    filter.update(set);
  }
  EXPECT_EQ(filter.get_control(), 0);
}

TEST(PID, UpdateReturnsControl) {
  const s22_10_t set = 100;
  IIR_PI_Filter<s22_10_t> filter{1, 1, 1, set};


  // Prime filter
  filter.update(set+1);
  filter.update(set+1);
  s16_16_t last_control = filter.get_control();

  for (size_t i=0; i<1000; i++) {
    const auto control = filter.update(set+1);
    EXPECT_EQ(filter.get_control(), control);
  }

}
/*
 * With none zero error, integral increases so output decreases 
 * */
TEST(PID, ControlDecreases) {
  const s22_10_t set = 100;
  IIR_PI_Filter<s22_10_t> filter{1, 1, 1, set};


  // Prime filter
  filter.update(set+1);
  filter.update(set+1);
  s22_10_t last_control = filter.get_control();

  for (size_t i=0; i<10; i++) {
    filter.update(set+1);
    EXPECT_GT(last_control, filter.get_control());
    EXPECT_GT(0, filter.get_control());
  }
  EXPECT_LT(filter.get_control(), 0);
}

/*
 * Input a constant line and check that the output the proper calculated value
 * */
TEST(PID, ConstantError) {
  const s22_10_t set = 100;
  IIR_PI_Filter<s22_10_t> filter{1, 1, 1, set};

  const s22_10_t error = 20;
  filter.update(set+error);
  EXPECT_EQ(filter.get_control(), -error*filter.get_kp());
  filter.update(set+error);
  EXPECT_EQ(filter.get_control(), -error*filter.get_kp());
  filter.update(set+error);
  EXPECT_EQ(filter.get_control(), -(error*filter.get_ki() + error*filter.get_kp()));
}

/*
 * Input a constant error and check that the output the proper calculated value
 * */
TEST(PID, ConstantIntegral) {
  const s22_10_t set = 100;
  IIR_PI_Filter<s22_10_t> filter{1, 1, 1, set};

  const size_t loops = 100;
  const s22_10_t error = 0.05;
  for (size_t i=0; i<loops; i++) {
    filter.update(set+error);
  }
  EXPECT_EQ(filter.get_control(), -(static_cast<s22_10_t>(loops-2)*error*filter.get_ki() + error*filter.get_kp()));
}
