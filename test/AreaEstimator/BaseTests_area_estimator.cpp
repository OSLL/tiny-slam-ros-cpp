#include <gtest/gtest.h>

#include "area_estimator_test_utils.h"

Rectangle cell = read_cell(1, -1, -1, 1);
double nan_occ = std::numeric_limits<double>::quiet_NaN();

TEST(AreaEstimator_BaseTests, Empty_LR_SideAligned_PassesCenter) {
  Ray ray = read_ray(-50, 0, 50, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_RL_SideAligned_PassesCenter) {
  Ray ray = read_ray(50, 0, -50, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_DU_SideAligned_PassesCenter) {
  Ray ray = read_ray(0, -50, 0, 50);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_UD_SideAligned_PassesCenter) {
  Ray ray = read_ray(0, 50, 0, -50);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_DL_UR_DiagAligned_PassesCenter) {
  Ray ray = read_ray(-2, -2, 2, 2);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_UD_DiagAligned_NearCellAngle) {
  Ray ray = read_ray(-0.5, 1.5, 1.5, -0.5);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.125});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Occ_LR_SideAligned_StopsCenter) {
  Ray ray = read_ray(-50, 0, 0, 0);
  bool test_flag = test_estimator(cell, true, ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_LR_SideAligned_StopsCenter) {
  Ray ray = read_ray(-50, 0, 0, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Occ_LR_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(-50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell, true, ray, {0.25, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_LR_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(-50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Occ_RL_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell, true, ray, {0.75, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_RL_SideAligned_NotReachingCenter) {
  Ray ray = read_ray(50, 0, 0.5, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Occ_DU_DiagAligned_NotReachingCenter) {
  Ray ray = read_ray(-50, -50, 0.5, 0.5);
  bool test_flag = test_estimator(cell, true, ray, {0.125, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_DU_DiagAligned_NotReachingCenter) {
  Ray ray = read_ray(-50, -50, 0.5, 0.5);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Occ_UD_DiagAligned_NotReachingCenter) {
  Ray ray = read_ray(50, 50, 0.5, 0.5);
  bool test_flag = test_estimator(cell, true, ray, {0.875, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Occ_UD_AtAngle_StopsCenter) {
  Ray ray = read_ray(-2, 4, 0, 0);
  bool test_flag = test_estimator(cell, true, ray, {0.5, 1});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Empty_UD_AtAngle_StopsCenter) {
  Ray ray = read_ray(-2, 4, 0, 0);
  bool test_flag = test_estimator(cell, false, ray, {0.01, 0.5});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Occ_NotCrossCell) {
  Ray ray = read_ray(2, 2, 3, 3);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

TEST(AreaEstimator_BaseTests, Occ_TangentToCell) {
  Ray ray = read_ray(-2, 0, 0, 2);
  bool test_flag = test_estimator(cell, true, ray, {nan_occ, nan_occ});
  EXPECT_TRUE(test_flag);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
