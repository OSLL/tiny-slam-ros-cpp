#include <gtest/gtest.h>

#include "area_estimator_test_utils.h"

Rectangle cell = read_cell(1, -1, -1, 1);

TEST(AreaEstimator_SmokeTesting, LR_SideAligned_ThroughCenter_Empty) {
  Ray ray = read_ray(-50, 0, 50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, RL_SideAligned_ThroughCenter_Empty) {
  Ray ray = read_ray(50, 0, -50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, DU_SideAligned_ThroughCenter_Empty) {
  Ray ray = read_ray(0, -50, 0, 50);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, UD_SideAligned_ThroughCenter_Empty) {
  Ray ray = read_ray(0, 50, 0, -50);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}


TEST(AreaEstimator_SmokeTesting, DU_CellD_Empty) {
  Ray ray = read_ray(-2, -2, 2, 2);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, NearCellCorner_AtAngle_Empty) {
  Ray ray = read_ray(-0.5, 1.5, 1.5, -0.5);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.125});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, LR_SideAligned_InCenter_Occ) {
  Ray ray = read_ray(-50, 0, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, LR_SideAligned_ShortCenter_Occ) {
  Ray ray = read_ray(-50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell,ray, {0.25, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, RL_SideAligned_PassCenter_Empty) {
  Ray ray = read_ray(50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell,ray, {0.75, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, DU_CellDiagonal_PassCenter_Occ) {
  Ray ray = read_ray(-50, -50, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.125, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, UD_CellDiagonal_ShortCenter_Occ) {
  Ray ray = read_ray(50, 50, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.875, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, UD_AtAngle_InCenter_Occ) {
  Ray ray = read_ray(-2, 4, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, DU_SideAligned_ThroughCentre_Occ) {
  Ray ray = read_ray(0, -4, 0, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, DU_SideAligned_NotTroughCenter_Occ) {
  Ray ray = read_ray(4, 0, -1, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, DU_ThroughSide_Empty) {
  Ray ray = read_ray(-1, -2, -1, 2);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.0});
  EXPECT_TRUE(test_flag);
}


TEST(AreaEstimator_GettingIntoBorder, RL_SideAligned_NotThroughCenter_Occ) {
  Ray ray = read_ray(1.5, 0, 1, 0);
  bool test_flag = test_estimator(cell,ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, DU_Diagonal_NotThroughCenter_Occ) {
  Ray ray = read_ray(2, -2, 1, -1);
  bool test_flag = test_estimator(cell,ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, DU_Diagonal_ThroughCenter_Empty) {
  Ray ray = read_ray(2, -2, -1, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, FromInside_Out_Empty) {
  Ray ray = read_ray(-0.5, -0.5, 50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, FormInside_ToBorder_Empty) { 
  Ray ray = read_ray(-0.5, -0.5, 1, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, FromInside_InCenter_Occ) {
  Ray ray = read_ray(-0.5, -0.5, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, SamePlace) {
  Ray ray = read_ray(0.5, 0.5, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.001, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, FromInside_ToInside_Occ) {
  Ray ray = read_ray(0.5, 0.5, -0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.25, 1});
  EXPECT_TRUE(test_flag);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  RUN_ALL_TESTS();
}

