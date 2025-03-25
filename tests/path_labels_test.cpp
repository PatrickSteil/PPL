#include "../datastructures/path_labels.h"

#include <gtest/gtest.h>

TEST(PathLabelTest, DefaultConstructorEmpty) {
  PathLabel pl;
  EXPECT_TRUE(pl.empty());
  EXPECT_EQ(pl.size(), 0);
}

TEST(PathLabelTest, PushBackAndSize) {
  PathLabel pl;
  PathHub hub1(1, 100);
  PathHub hub2(2, 200);
  pl.push_back(hub1);
  EXPECT_FALSE(pl.empty());
  EXPECT_EQ(pl.size(), 1);
  pl.push_back(hub2);
  EXPECT_EQ(pl.size(), 2);
}

TEST(PathLabelTest, RandomAccessOperator) {
  std::vector<PathHub> hubs = {PathHub(1, 10), PathHub(2, 20), PathHub(3, 30)};
  PathLabel pl(hubs);
  EXPECT_EQ(pl[0].getPath(), 1);
  EXPECT_EQ(pl[1].getPathPos(), 20);
  EXPECT_EQ(pl[2].getPath(), 3);
}

TEST(PathLabelTest, SortFunctionality) {
  std::vector<PathHub> hubs = {PathHub(2, 20), PathHub(1, 30), PathHub(1, 10),
                               PathHub(2, 10)};
  PathLabel pl(hubs);
  pl.sort();

  EXPECT_EQ(pl[0].getPath(), 1);
  EXPECT_EQ(pl[0].getPathPos(), 10);
  EXPECT_EQ(pl[1].getPath(), 1);
  EXPECT_EQ(pl[1].getPathPos(), 30);
  EXPECT_EQ(pl[2].getPath(), 2);
  EXPECT_EQ(pl[2].getPathPos(), 10);
  EXPECT_EQ(pl[3].getPath(), 2);
  EXPECT_EQ(pl[3].getPathPos(), 20);
}

TEST(PathHubTest, IsOnSamePath) {
  PathHub hub1(1, 10);
  PathHub hub2(1, 20);
  PathHub hub3(2, 30);
  EXPECT_TRUE(hub1.isOnSamePath(hub2));
  EXPECT_FALSE(hub1.isOnSamePath(hub3));
}

TEST(PathHubTest, IsReachable) {
  PathHub hub1(1, 10);
  PathHub hub2(1, 20);
  PathHub hub3(1, 5);
  PathHub hub4(2, 15);
  EXPECT_TRUE(hub1.isReachable(hub2));
  EXPECT_FALSE(hub1.isReachable(hub3));
  EXPECT_FALSE(hub1.isReachable(hub4));
}

TEST(QueryTest, BothEmpty) {
  PathLabel from;
  PathLabel to;
  EXPECT_FALSE(query(from, to));
}

TEST(QueryTest, OneEmpty) {
  PathLabel from;
  PathLabel to;
  from.emplace_back(1, 10);
  from.sort();
  EXPECT_FALSE(query(from, to));

  from.clear();
  to.emplace_back(1, 20);
  to.sort();
  EXPECT_FALSE(query(from, to));
}

TEST(QueryTest, Reachable) {
  PathLabel from;
  PathLabel to;
  from.emplace_back(1, 10);
  from.emplace_back(1, 20);
  to.emplace_back(1, 15);
  from.sort();
  to.sort();
  EXPECT_TRUE(query(from, to));
}

TEST(QueryTest, NotReachable) {
  PathLabel from;
  PathLabel to;
  from.emplace_back(1, 20);
  to.emplace_back(1, 10);
  from.sort();
  to.sort();
  EXPECT_FALSE(query(from, to));
}

TEST(QueryTest, MultipleHubs) {
  PathLabel from;
  PathLabel to;
  from.emplace_back(1, 10);
  from.emplace_back(2, 20);
  to.emplace_back(1, 5);
  to.emplace_back(2, 30);
  from.sort();
  to.sort();
  EXPECT_TRUE(query(from, to));
}
