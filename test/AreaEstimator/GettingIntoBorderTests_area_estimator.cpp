#include <gtest/gtest.h>

#include "area_estimator_test_utils.h"

Rectangle cell = read_cell(1, -1, -1, 1);
double nan_occ = std::numeric_limits<double>::quiet_NaN();

TEST(AreaEstimator_GettingIntoBorder, Empty_DU_SideAligned_PassesCentre) {
  Ray ray = read_ray(0, -4, 0, 1);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_DU_SideAligned_PassesCentre) {
  Ray ray = read_ray(0, -4, 0, 1);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_DU_SideAligned_NotReachingCentre) {
  Ray ray = read_ray(0, -4, 0, -1);
  bool test_flag = test_estimator(cell, true, ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_DU_SideAligned_NotReachingCentre) {
  Ray ray = read_ray(0, -4, 0, -1);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_UD_SideAligned_NotReachingCentre) {
  Ray ray = read_ray(0, 4, 0, 1);
  bool test_flag = test_estimator(cell, true, ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_UD_SideAligned_NotReachingCentre) {
  Ray ray = read_ray(0, 4, 0, 1);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_UD_SideAligned_PassesCentre) {
  Ray ray = read_ray(0, 4, 0, -1);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_UD_SideAligned_PassesCentre) {
  Ray ray = read_ray(0, 4, 0, -1);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_RL_SideAligned_PassesCenter) {
  Ray ray = read_ray(4, 0, -1, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_RL_SideAligned_PassesCenter) {
  Ray ray = read_ray(4, 0, -1, 0);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, 0.01});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_RL_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(4, 0, 1, 0);
  bool test_flag = test_estimator(cell, true, ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_RL_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(4, 0, 1, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_LR_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(-4, 0, -1, 0);
  bool test_flag = test_estimator(cell, true, ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_LR_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(-4, 0, -1, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_LR_SideAligned_PassesCenter) {
  Ray ray = read_ray(-4, 0, 1, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_LR_SideAligned_PassesCenter) {
  Ray ray = read_ray(-4, 0, 1, 0);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_DU_PassesSide) {
  Ray ray = read_ray(-1, -2, -1, 2);
  bool test_flag = test_estimator(cell, false, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_DU_PassesSide) {
  Ray ray = read_ray(-1, -2, -1, 2);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_DU_PassesSide_StopsSide) {
  Ray ray = read_ray(-1, -2, -1, 0);
  bool test_flag = test_estimator(cell, true, ray, {0.5, 0.01});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_DU_PassesSide_StopsSide) {
  Ray ray = read_ray(-1, -2, -1, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.01});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_DU_DiagAligned_NotReachingCenter) {
  Ray ray = read_ray(2, -2, 1, -1);
  bool test_flag = test_estimator(cell, true, ray, {1, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_DU_DiagAligned_NotReachingCenter) {
  Ray ray = read_ray(2, -2, 1, -1);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Empty_DU_DiagAligned_PassesCenter) {
  Ray ray = read_ray(2, -2, -1, 1);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_GettingIntoBorder, Occ_DU_DiagAligned_PassesCenter) {
  Ray ray = read_ray(2, -2, -1, 1);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

