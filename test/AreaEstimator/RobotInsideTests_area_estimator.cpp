#include <gtest/gtest.h>

#include "area_estimator_test_utils.h"

Rectangle cell = create_cell(1, -1, -1, 1);

TEST(AreaEstimator_RobotInside, Empty_FromInside_Out) {
  TestRay ray = create_ray(-0.5, -0.5, 0, 50);
  Occupancy expected(0.01, 102.0/404.0);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotInside, Occ_FromInside_Out) {
  TestRay ray = create_ray(-0.5, -0.5, 50, 0);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotInside, Empty_FormInside_ToBorder) { 
  TestRay ray = create_ray(-0.5, -0.5, 1, 1);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotInside, Occ_FormInside_ToBorder) { 
  TestRay ray = create_ray(-0.5, -0.5, 1, 1);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotInside, Occ_FromInside_StopsCenter) {
  TestRay ray = create_ray(-0.5, -0.5, 0, 0);
  Occupancy expected(0.5, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotInside, Empty_FromInside_StopsCenter) {
  TestRay ray = create_ray(-0.5, -0.5, 0, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotInside, SamePlace) {
  TestRay ray = create_ray(0.5, 0.5, 0.5, 0.5);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotOnBorder, Empty_ToOutSide) {
  TestRay ray = create_ray(0, 1, 1, -2);
  Occupancy expected(0.01, 1.0/3.0);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotOnBorder, Occ_ToOutSide) {
  TestRay ray = create_ray(0, 1, 1, -2);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotOnBorder, Occ_ToOutSide_NotPassesCell) {
  TestRay ray = create_ray(0, 1, 2, 2);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_RobotOnBorder, Empty_ToOutSide_NotPassesCell) {
  TestRay ray = create_ray(0, 1, 2, 2);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
