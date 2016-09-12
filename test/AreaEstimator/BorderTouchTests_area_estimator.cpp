#include <gtest/gtest.h>

#include "area_estimator_test_utils.h"

Rectangle cell = create_cell(1, -1, -1, 1);

TEST(AreaEstimator_BorderTouch, Empty_DU_SideAligned_PassesCenter) {
  TestRay ray(0, -4, 0, 1);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_DU_SideAligned_PassesCenter) {
  TestRay ray(0, -4, 0, 1);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_DU_SideAligned_NotReachingCenter) {
  TestRay ray(0, -4, 0, -1);
  Occupancy expected(1, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_DU_SideAligned_NotReachingCenter) {
  TestRay ray(0, -4, 0, -1);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_UD_SideAligned_NotReachingCenter) {
  TestRay ray(0, 4, 0, 1);
  Occupancy expected(1, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_UD_SideAligned_NotReachingCenter) {
  TestRay ray(0, 4, 0, 1);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_UD_SideAligned_PassesCenter) {
  TestRay ray(0, 4, 0, -1);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_UD_SideAligned_PassesCenter) {
  TestRay ray(0, 4, 0, -1);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_RL_SideAligned_PassesCenter) {
  TestRay ray(4, 0, -1, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_RL_SideAligned_PassesCenter) {
  TestRay ray(4, 0, -1, 0);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_RL_SideAligned_NotReachingCenter) {
  TestRay ray(4, 0, 1, 0);
  Occupancy expected(1, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_RL_SideAligned_NotReachingCenter) {
  TestRay ray(4, 0, 1, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_LR_SideAligned_NotReachingCenter) {
  TestRay ray(-4, 0, -1, 0);
  Occupancy expected(1, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_LR_SideAligned_NotReachingCenter) {
  TestRay ray(-4, 0, -1, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_LR_SideAligned_PassesCenter) {
  TestRay ray(-4, 0, 1, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_LR_SideAligned_PassesCenter) {
  TestRay ray(-4, 0, 1, 0);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_DU_PassesSide) {
  TestRay ray(-1, -2, -1, 2);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_DU_PassesSide) {
  TestRay ray(-1, -2, -1, 2);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_DU_PassesSide_StopsSide) {
  TestRay ray(-1, -2, -1, 0);
  Occupancy expected(0.5, 0.01);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_DU_PassesSide_StopsSide) {
  TestRay ray(-1, -2, -1, 0);
  Occupancy expected(0.01, 0.01);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_DU_DiagAligned_NotReachingCenter) {
  TestRay ray(2, -2, 1, -1);
  Occupancy expected(1, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_DU_DiagAligned_NotReachingCenter) {
  TestRay ray(2, -2, 1, -1);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Empty_DU_DiagAligned_PassesCenter) {
  TestRay ray(2, -2, -1, 1);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BorderTouch, Occ_DU_DiagAligned_PassesCenter) {
  TestRay ray(2, -2, -1, 1);
  Occupancy expected(0.01, 0.01);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

