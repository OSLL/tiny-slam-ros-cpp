#include <gtest/gtest.h>
#include <iostream>

#include "area_estimator_test_utils.h"

Rectangle cell = read_cell(1, -1, -1, 1);

TEST(AreaEstimator_SmokeTesting, Left_Right_ParallelToSide_ThroughCenter) {
  Ray ray = read_ray(-50, 0, 50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Right_Left_ParallelToSide_ThroughCenter) {
  Ray ray = read_ray(50, 0, -50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Down_Up_ParallelToSide_ThroughCenter) {
  Ray ray = read_ray(0, -50, 0, 50);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Up_Down_ParallelToSide_ThroughCenter) {
  Ray ray = read_ray(0, 50, 0, -50);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}


TEST(AreaEstimator_SmokeTesting, CellDiagonal_Down_Up) {
  Ray ray = read_ray(-2, -2, 2, 2);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Near_Cell_Corner_AtAngle) {
  Ray ray = read_ray(-0.5, 1.5, 1.5, -0.5);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.125});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Left_Right_ParallelToSide_InCenter) {
  Ray ray = read_ray(-50, 0, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Left_Right_ParallelToSide_ShortCenter) {
  Ray ray = read_ray(-50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell,ray, {0.25, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Right_Left_ParallelToSide_PassCenter) {
  Ray ray = read_ray(50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell,ray, {0.75, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, CellDiagonal_Down_Up_PassCenter) {
  Ray ray = read_ray(-50, -50, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.125, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, CellDiagonal_Up_Down_ShortCenter) {
  Ray ray = read_ray(50, 50, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.875, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Up_Down_AtAngle_InCenter) {
  Ray ray = read_ray(-2, 4, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Down_Up_ParallelToSide_ThroughCentre) {
  Ray ray = read_ray(0, -4, 0, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Down_Up_ParallelToSide_NotTroughCenter) {
  Ray ray = read_ray(4, 0, -1, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Down_Up_ThroughSide) {
  Ray ray = read_ray(-1, -2, -1, 2);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.0});
  EXPECT_TRUE(test_flag);
}


TEST(AreaEstimator_GettingIntoBorder, Right_Left_ParallelToSide_NotThroughCenter) {
  Ray ray = read_ray(1.5, 0, 1, 0);
  bool test_flag = test_estimator(cell,ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Down_Up_Diagonal_NotThroughCenter) {
  Ray ray = read_ray(2, -2, 1, -1);
  bool test_flag = test_estimator(cell,ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Down_Up_Diagonal_ThroughCenter) {
  Ray ray = read_ray(2, -2, -1, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, FromInside_Out) {
  Ray ray = read_ray(-0.5, -0.5, 50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, FormInside_ToBorder) { 
  Ray ray = read_ray(-0.5, -0.5, 1, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, FromInside_InCenter) {
  Ray ray = read_ray(-0.5, -0.5, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, SamePlace) {
  Ray ray = read_ray(0.5, 0.5, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.001, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, FromInside_ToInside) {
  Ray ray = read_ray(0.5, 0.5, -0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.25, 1});
  EXPECT_TRUE(test_flag);
}

int main (int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

