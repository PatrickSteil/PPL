#include "../datastructures/chain.h"

#include "gtest/gtest.h"

TEST(ChainTest, EmptyChain) {
  Chain chain;
  EXPECT_TRUE(chain.empty());
  EXPECT_EQ(chain.size(), 0);
}

TEST(ChainTest, SinglePush) {
  Chain chain;
  Vertex v = 42;
  chain.push(v);
  EXPECT_FALSE(chain.empty());
  EXPECT_EQ(chain.size(), 1);
  EXPECT_EQ(chain.getLastVertex(), v);

  const std::list<Vertex>& vertices = chain.getVertices();
  auto it = vertices.begin();
  ASSERT_NE(it, vertices.end());
  EXPECT_EQ(*it, v);
  ++it;
  EXPECT_EQ(it, vertices.end());
}

TEST(ChainTest, MultiplePushes) {
  Chain chain;
  std::vector<Vertex> expected = {1, 2, 3, 4, 5};
  for (Vertex v : expected) {
    chain.push(v);
  }
  EXPECT_EQ(chain.size(), expected.size());
  EXPECT_EQ(chain.getLastVertex(), expected.back());

  const std::list<Vertex>& vertices = chain.getVertices();
  auto it = vertices.begin();
  for (Vertex exp : expected) {
    ASSERT_NE(it, vertices.end());
    EXPECT_EQ(*it, exp);
    ++it;
  }
  EXPECT_EQ(it, vertices.end());
}