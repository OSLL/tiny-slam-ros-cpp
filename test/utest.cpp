#include <gtest/gtest.h>
#include <iostream>

#include "area_estimator_test.h"

Rectangle cell = read_cell(1, -1, -1, 1);

TEST(AreaEstimator, NO1) {
  Ray ray = read_ray(-50, 0, 50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, NO2) {
  Ray ray = read_ray(50, 0, -50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, NO3) {
  Ray ray = read_ray(0, -50, 0, 50);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, NO4) {
  Ray ray = read_ray(0, 50, 0, -50);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}


TEST(AreaEstimator, ND1) {
  Ray ray = read_ray(-2, -2, 2, 2);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, NT1) {
  Ray ray = read_ray(-0.5, 1.5, 1.5, -0.5);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.125});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, OO1) {
  Ray ray = read_ray(-50, 0, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, OO2) {
  Ray ray = read_ray(-50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell,ray, {0.25, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, OO3) {
  Ray ray = read_ray(50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell,ray, {0.75, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, OD1) {
  Ray ray = read_ray(-50, -50, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.125, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, OD2) {
  Ray ray = read_ray(50, 50, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.875, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator, OT1) {
  Ray ray = read_ray(-2, 4, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorBounds, NO1) {
  Ray ray = read_ray(0, -4, 0, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorBounds, NO2) {
  Ray ray = read_ray(4, 0, -1, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorBounds, NO3) {
  Ray ray = read_ray(-1, -2, -1, 2);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.0});
  EXPECT_TRUE(test_flag);
}


TEST(AreaEstimatorBounds, OO1) {
  Ray ray = read_ray(1.5, 0, 1, 0);
  bool test_flag = test_estimator(cell,ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorBounds, OO2) {
  Ray ray = read_ray(0, 1.5, 0, 1);
  bool test_flag = test_estimator(cell,ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, NI1) {
  Ray ray = read_ray(-0.5, -0.5, 50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, NI2) {
  Ray ray = read_ray(-0.5, -0.5, 4, 4);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, OI1) {
  Ray ray = read_ray(-0.5, -0.5, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, OI2) {
  Ray ray = read_ray(0.5, 0.5, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.001, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, OI3) {
  Ray ray = read_ray(0.5, 0.5, -0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.25, 1});
  EXPECT_TRUE(test_flag);
}

int main (int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

