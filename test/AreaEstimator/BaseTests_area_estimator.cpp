#include <gtest/gtest.h>

#include "area_estimator_test_utils.h"

Rectangle cell = create_cell(1, -1, -1, 1);

TEST(AreaEstimator_BaseTests, Empty_LR_SideAligned_PassesCenter) {
  TestRay ray(-50, 0, 50, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_RL_SideAligned_PassesCenter) {
  TestRay ray(50, 0, -50, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_DU_SideAligned_PassesCenter) {
  TestRay ray(0, -50, 0, 50);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_UD_SideAligned_PassesCenter) {
  TestRay ray(0, 50, 0, -50);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_DL_UR_DiagAligned_PassesCenter) {
  TestRay ray(-2, -2, 2, 2);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_UD_DiagAligned_NearCellAngle) {
  TestRay ray(-0.5, 1.5, 1.5, -0.5);
  Occupancy expected(0.01, 0.125);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_LR_SideAligned_StopsCenter) {
  TestRay ray(-50, 0, 0, 0);
  Occupancy expected(0.5, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_LR_SideAligned_StopsCenter) {
  TestRay ray(-50, 0, 0, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_LR_SideAligned_NotReachingCenter) {
  TestRay ray(-50, 0, 0.5, 0);
  Occupancy expected(0.25, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_LR_SideAligned_NotReachingCenter) {
  TestRay ray(-50, 0, 0.5, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_RL_SideAligned_NotReachingCenter) {
  TestRay ray(50, 0, 0.5, 0);
  Occupancy expected(0.75, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_RL_SideAligned_NotReachingCenter) {
  TestRay ray(50, 0, 0.5, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_DU_DiagAligned_NotReachingCenter) {
  TestRay ray(-50, -50, 0.5, 0.5);
  Occupancy expected(0.125, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_DU_DiagAligned_NotReachingCenter) {
  TestRay ray(-50, -50, 0.5, 0.5);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_UD_DiagAligned_NotReachingCenter) {
  TestRay ray(50, 50, 0.5, 0.5);
  Occupancy expected(0.875, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_UD_AtAngle_StopsCenter) {
  TestRay ray(-2, 4, 0, 0);
  Occupancy expected(0.5, 1);
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_UD_AtAngle_StopsCenter) {
  TestRay ray(-2, 4, 0, 0);
  Occupancy expected(0.01, 0.5);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_NotCrossCell) {
  TestRay ray(2, 2, 3, 3);
  Occupancy expected(0.01, 1);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_CoinsideWithLeftToptSide) {
  TestRay ray(-2, 0, 0, 2);
  Occupancy expected(0.01, 1);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_CoinsideWithLeftTopSide) {
  TestRay ray(-2, 0, 0, 2);
  Occupancy expected(0.01, 1);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_CoinsideWithLeftBotSide) {
  TestRay ray(-2, 0, 0, -2);
  Occupancy expected(0.01, 1);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_CoinsideWithLeftBotSide) {
  TestRay ray(-2, 0, 0, -2);
  Occupancy expected(0.01, 1);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_CoinsideWithRightBotSide) {
  TestRay ray(2, 0, 0, -2);
  Occupancy expected(0.01, 1);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_CoinsideWithRightBotSide) {
  TestRay ray(2, 0, 0, -2);
  Occupancy expected(0.01, 1);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Occ_CoinsideWithRightTopSide) {
  TestRay ray(2, 0, 0, 2);
  Occupancy expected(0.01, 1);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, true, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_CoinsideWithRightTopSide) {
  TestRay ray(2, 0, 0, 2);
  Occupancy expected(0.01, 1);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_UD_AtAngle_NotCrossCell) {
  TestRay ray(0, 50, -0.5, -0.5);
  Occupancy expected(0.01, 102.0/404.0);
  expected.invalidate();
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

TEST(AreaEstimator_BaseTests, Empty_DU_SideAligned_NearCellBorder) {
  TestRay ray(0.9, -2, 0.9, 2);
  Occupancy expected(0.01, 0.05);
  bool is_succeed = test_estimator(cell, false, ray, expected);
  EXPECT_TRUE(is_succeed);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
