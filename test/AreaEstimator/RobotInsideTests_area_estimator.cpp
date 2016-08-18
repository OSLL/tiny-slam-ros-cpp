#include <gtest/gtest.h>

#include "area_estimator_test_utils.h"

Rectangle cell = read_cell(1, -1, -1, 1);
double nan_occ = std::numeric_limits<double>::quiet_NaN();

TEST(AreaEstimator_RobotInside, Empty_FromInside_Out) {
  Ray ray = read_ray(-0.5, -0.5, 50, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.252475});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotInside, Occ_FromInside_Out) {
  Ray ray = read_ray(-0.5, -0.5, 50, 0);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotInside, Empty_FormInside_ToBorder) { 
  Ray ray = read_ray(-0.5, -0.5, 1, 1);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotInside, Occ_FormInside_ToBorder) { 
  Ray ray = read_ray(-0.5, -0.5, 1, 1);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotInside, Occ_FromInside_StopsCenter) {
  Ray ray = read_ray(-0.5, -0.5, 0, 0);
  bool test_flag = test_estimator(cell, true, ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotInside, Empty_FromInside_StopsCenter) {
  Ray ray = read_ray(-0.5, -0.5, 0, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotInside, SamePlace) {
  Ray ray = read_ray(0.5, 0.5, 0.5, 0.5);
  bool test_flag = test_estimator(cell, false, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotOnBorder, Empty_ToOutSide) {
  Ray ray = read_ray(0, 1, 1, -2);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.333333});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotOnBorder, Occ_ToOutSide) {
  Ray ray = read_ray(0, 1, 1, -2);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotOnBorder, Occ_ToOutSide_NotPassesCell) {
  Ray ray = read_ray(0, 1, 2, 2);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_RobotOnBorder, Empty_ToOutSide_NotPassesCell) {
  Ray ray = read_ray(0, 1, 2, 2);
  bool test_flag = test_estimator(cell, false, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
