#include <gtest/gtest.h>

#include "Map_test.h"

TEST(AutoExpandbleMapBaseTests, Expand_Right) {
  MapParam param = test_auto_expand({2, 0});
  bool is_succeed = (param.height == 1) && (param.width == 4);
  EXPECT_TRUE(is_succeed);
  is_succeed = (param.x == 0) && (param.y == 0);
  EXPECT_TRUE(is_succeed);
}

TEST(AutoExpandbleMapBaseTests, Expand_Top) {
  MapParam param = test_auto_expand({0, 3});
  bool is_succeed = (param.height == 8) && (param.width == 4);
  EXPECT_TRUE(is_succeed);
  is_succeed = (param.x == 0) && (param.y == 0);
  EXPECT_TRUE(is_succeed);
}

TEST(AutoExpandbleMapBaseTests, Expand_Left) {
  MapParam param = test_auto_expand({-3, 0});
  bool is_succeed = (param.height == 8) && (param.width == 8);
  EXPECT_TRUE(is_succeed);
  is_succeed = (param.x == 4) && (param.y == 0);
  EXPECT_TRUE(is_succeed);
}

TEST(AutoExpandbleMapBaseTests, Expand_Down) {
  MapParam param = test_auto_expand({0, -3});
  bool is_succeed = (param.height == 16) && (param.width == 8);
  EXPECT_TRUE(is_succeed);
  is_succeed = (param.x == 4) && (param.y == 8);
  EXPECT_TRUE(is_succeed);
}

TEST(AutoExpandbleMapBaseTests, Expand_Right_Top) {
  MapParam param = test_auto_expand({3, 2});
  bool is_succeed = (param.height == 16) && (param.width == 8);
  EXPECT_TRUE(is_succeed);
  is_succeed = (param.x == 4) && (param.y == 8);
  EXPECT_TRUE(is_succeed);
}

TEST(AutoExpandbleMapBaseTests, Expand_Right_Down) {
  MapParam param = test_auto_expand({3, -2});
  bool is_succeed = (param.height == 16) && (param.width == 8);
  EXPECT_TRUE(is_succeed);
  is_succeed = (param.x == 4) && (param.y == 8);
  EXPECT_TRUE(is_succeed);
}

TEST(AutoExpandbleMapBaseTests, Expand_Left_Top) {
  MapParam param = test_auto_expand({-3, 2});
  bool is_succeed = (param.height == 16) && (param.width == 8);
  EXPECT_TRUE(is_succeed);
  is_succeed = (param.x == 4) && (param.y == 8);
  EXPECT_TRUE(is_succeed);
}

TEST(AutoExpandbleMapBaseTests, Expand_Left_Down) {
  MapParam param = test_auto_expand({-3, -2});
  bool is_succeed = (param.height == 16) && (param.width == 8);
  EXPECT_TRUE(is_succeed);
  is_succeed = (param.x == 4) && (param.y == 8);
  EXPECT_TRUE(is_succeed);
}

TEST(AutoExpandbleMapBaseTests, NoExpand) {
  MapParam param = test_auto_expand({0, 0});
  bool is_succeed = (param.height == 16) && (param.width == 8);
  EXPECT_TRUE(is_succeed);
  is_succeed = (param.x == 4) && (param.y == 8);
  EXPECT_TRUE(is_succeed);
}

TEST(AutoExpandbleMapBaseTests, CheckCellsValue) {
  bool is_succeed = test_CellValueNotChange();
  EXPECT_TRUE(is_succeed);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
