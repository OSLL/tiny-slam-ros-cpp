#include <gtest/gtest.h>

#include "area_estimator_test_utils.h"

Rectangle cell = read_cell(1, -1, -1, 1);

TEST(AreaEstimator_SmokeTesting, Empty_LR_SideAligned_PassesCenter) {
  Ray ray = read_ray(-50, 0, 50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Empty_RL_SideAligned_PassesCenter) {
  Ray ray = read_ray(50, 0, -50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Empty_DU_SideAligned_PassesCenter) {
  Ray ray = read_ray(0, -50, 0, 50);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Empty_UD_SideAligned_PassesCenter) {
  Ray ray = read_ray(0, 50, 0, -50);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Empty_DL_UR_DiagAligned_PassesCenter) {
  Ray ray = read_ray(-2, -2, 2, 2);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Empty_UD_DiagAligned_NearCellAngle) {
  Ray ray = read_ray(-0.5, 1.5, 1.5, -0.5);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.125});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Occ_LR_SideAligned_StopsCenter) {
  Ray ray = read_ray(-50, 0, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Occ_LR_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(-50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell,ray, {0.25, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Occ_RL_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell,ray, {0.75, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Occ_DU_DiagAligned_NotReachingCenter) {
  Ray ray = read_ray(-50, -50, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.125, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Occ_UD_DiagAligned_NotReachingCenter) {
  Ray ray = read_ray(50, 50, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.875, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_SmokeTesting, Occ_UD_AtAngle_StopsCenter) {
  Ray ray = read_ray(-2, 4, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_DU_SideAligned_PassesCentre) {
  Ray ray = read_ray(0, -4, 0, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_DU_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(4, 0, -1, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_DU_PassesSide) {
  Ray ray = read_ray(-1, -2, -1, 2);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.0});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_DU_PassesSide_StopsSide) {
  Ray ray = read_ray(-1, -2, -1, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 0.01});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_RL_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(1.5, 0, 1, 0);
  bool test_flag = test_estimator(cell,ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_DU_DiagAligned_NotReachingCenter) {
  Ray ray = read_ray(2, -2, 1, -1);
  bool test_flag = test_estimator(cell,ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_DU_DiagAligned_PassesCenter) {
  Ray ray = read_ray(2, -2, -1, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, Empty_FromInside_Out) {
  Ray ray = read_ray(-0.5, -0.5, 50, 0);
  bool test_flag = test_estimator(cell,ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, Empty_FormInside_ToBorder) { 
  Ray ray = read_ray(-0.5, -0.5, 1, 1);
  bool test_flag = test_estimator(cell,ray, {0.01, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, Occ_FromInside_StopsCenter) {
  Ray ray = read_ray(-0.5, -0.5, 0, 0);
  bool test_flag = test_estimator(cell,ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, SamePlace) {
  Ray ray = read_ray(0.5, 0.5, 0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.001, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimatorRobotInside, Occ_FromInside_ToInside) {
  Ray ray = read_ray(0.5, 0.5, -0.5, 0.5);
  bool test_flag = test_estimator(cell,ray, {0.25, 1});
  EXPECT_TRUE(test_flag);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

