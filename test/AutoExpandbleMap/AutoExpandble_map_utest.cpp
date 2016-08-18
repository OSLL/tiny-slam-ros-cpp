#include <gtest/gtest.h>

#include "Map_test.h"

TEST(AutoExpandbleMapBaseTests, Right) {
  HeWi param = AutoExpandbleMap_Test({2, 0});
  bool flag = (param.height == 1)&&(param.width == 4);
  EXPECT_TRUE(flag);
  flag = (param.x == 0)&&(param.y == 0);
  EXPECT_TRUE(flag);
}

TEST(AutoExpandbleMapBaseTests, Top) {
  HeWi param = AutoExpandbleMap_Test({0, 3});
  bool flag = (param.height == 8)&&(param.width == 4);
  EXPECT_TRUE(flag);
  flag = (param.x == 0)&&(param.y == 0);
  EXPECT_TRUE(flag);
}

TEST(AutoExpandbleMapBaseTests, Left) {
  HeWi param = AutoExpandbleMap_Test({-3, 0});
  bool flag = (param.height == 8)&&(param.width == 8);
  EXPECT_TRUE(flag);
  flag = (param.x == 4)&&(param.y == 0);
  EXPECT_TRUE(flag);
}

TEST(AutoExpandbleMapBaseTests, Down) {
  HeWi param = AutoExpandbleMap_Test({0, -3});
  bool flag = (param.height == 16)&&(param.width == 8);
  EXPECT_TRUE(flag);
  flag = (param.x == 4)&&(param.y == 8);
  EXPECT_TRUE(flag);
}

TEST(AutoExpandbleMapBaseTests, Right_Top) {
  HeWi param = AutoExpandbleMap_Test({3, 2});
  bool flag = (param.height == 16)&&(param.width == 8);
  EXPECT_TRUE(flag);
  flag = (param.x == 4)&&(param.y == 8);
  EXPECT_TRUE(flag);
}

TEST(AutoExpandbleMapBaseTests, Right_Down) {
  HeWi param = AutoExpandbleMap_Test({3, -2});
  bool flag = (param.height == 16)&&(param.width == 8);
  EXPECT_TRUE(flag);
  flag = (param.x == 4)&&(param.y == 8);
  EXPECT_TRUE(flag);
}

TEST(AutoExpandbleMapBaseTests, Left_Top) {
  HeWi param = AutoExpandbleMap_Test({-3, 2});
  bool flag = (param.height == 16)&&(param.width == 8);
  EXPECT_TRUE(flag);
  flag = (param.x == 4)&&(param.y == 8);
  EXPECT_TRUE(flag);
}

TEST(AutoExpandbleMapBaseTests, Left_Down) {
  HeWi param = AutoExpandbleMap_Test({-3, -2});
  bool flag = (param.height == 16)&&(param.width == 8);
  EXPECT_TRUE(flag);
  flag = (param.x == 4)&&(param.y == 8);
  EXPECT_TRUE(flag);
}

TEST(AutoExpandbleMapBaseTests, NoExpand) {
  HeWi param = AutoExpandbleMap_Test({0, 0});
  bool flag = (param.height == 16)&&(param.width == 8);
  EXPECT_TRUE(flag);
  flag = (param.x == 4)&&(param.y == 8);
  EXPECT_TRUE(flag);
}

TEST(AutoExpandbleMapBaseTests, CheckCellsValue) {
  bool flag = CellValue_test();
  EXPECT_TRUE(flag);
}

int main (int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
