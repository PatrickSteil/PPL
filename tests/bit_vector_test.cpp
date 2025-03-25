#include "../datastructures/bit_vector.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>

TEST(BitVectorTest, PushBackAndAccess) {
  BitVector<> bv;
  bv.push_back(1);
  bv.push_back(0);
  bv.push_back(1);

  EXPECT_EQ(bv.size(), 3);
  EXPECT_EQ(bv[0], true);
  EXPECT_EQ(bv[1], false);
  EXPECT_EQ(bv[2], true);
}

TEST(BitVectorTest, PushBackAndAccessDifferentType) {
  {
    BitVector<uint8_t> bv;

    for (int i = 0; i < 33; ++i) {
      bv.push_back(i & 1);
    }

    EXPECT_EQ(bv.size(), 33);
    for (int i = 0; i < 33; ++i) {
      EXPECT_EQ(bv[i], (i & 1));
    }
  }
  {
    BitVector<uint16_t> bv;

    for (int i = 0; i < 33; ++i) {
      bv.push_back(i & 1);
    }

    EXPECT_EQ(bv.size(), 33);
    for (int i = 0; i < 33; ++i) {
      EXPECT_EQ(bv[i], (i & 1));
    }
  }
  {
    BitVector<uint32_t> bv;

    for (int i = 0; i < 33; ++i) {
      bv.push_back(i & 1);
    }

    EXPECT_EQ(bv.size(), 33);
    for (int i = 0; i < 33; ++i) {
      EXPECT_EQ(bv[i], (i & 1));
    }
  }
}

TEST(BitVectorTest, Clear) {
  BitVector<> bv;
  bv.push_back(1);
  bv.push_back(0);
  bv.clear();

  EXPECT_EQ(bv.size(), 0);
}

TEST(BitVectorTest, Reserve) {
  BitVector<> bv;
  bv.reserve(128);

  EXPECT_GE(bv.capacity(), 128);
}

TEST(BitVectorTest, Iterators) {
  BitVector<> bv;
  bv.push_back(1);
  bv.push_back(0);
  bv.push_back(1);

  std::vector<bool> expected = {true, false, true};
  size_t i = 0;
  for (bool bit : bv) {
    EXPECT_EQ(bit, expected[i++]);
  }
}

TEST(BitVectorTest, LargeInsertion) {
  BitVector<> bv;
  constexpr size_t N = 1000;
  for (size_t i = 0; i < N; ++i) {
    bv.push_back(i % 2);
  }

  EXPECT_EQ(bv.size(), N);
  for (size_t i = 0; i < N; ++i) {
    EXPECT_EQ(bv[i], i % 2);
  }
}

TEST(BitVectorTest, SerializeDeserialize) {
  BitVector<> bv;
  constexpr size_t N = 100;

  for (size_t i = 0; i < N; ++i) {
    bv.push_back(i % 2);
  }

  const std::string filename = "test_bitvector.bin";

  ASSERT_TRUE(bv.serialize(filename));

  BitVector<> bv2;
  ASSERT_TRUE(bv2.deserialize(filename));

  EXPECT_EQ(bv.size(), bv2.size());
  for (size_t i = 0; i < N; ++i) {
    EXPECT_EQ(bv[i], bv2[i]);
  }

  std::filesystem::remove(filename);
}

TEST(BitVectorTest, BitwiseANDAssignment) {
  BitVector<> bv1;
  BitVector<> bv2;

  bv1.push_back(1);
  bv1.push_back(0);
  bv1.push_back(1);

  bv2.push_back(1);
  bv2.push_back(1);
  bv2.push_back(0);

  bv1 &= bv2;

  EXPECT_EQ(bv1.size(), 3);
  EXPECT_EQ(bv1[0], true);   // 1 & 1 = 1
  EXPECT_EQ(bv1[1], false);  // 0 & 1 = 0
  EXPECT_EQ(bv1[2], false);  // 1 & 0 = 0
}

TEST(BitVectorTest, BitwiseORAssignment) {
  BitVector<> bv1;
  BitVector<> bv2;

  bv1.push_back(1);
  bv1.push_back(0);
  bv1.push_back(1);

  bv2.push_back(1);
  bv2.push_back(1);
  bv2.push_back(0);

  bv1 |= bv2;

  EXPECT_EQ(bv1.size(), 3);
  EXPECT_EQ(bv1[0], true);  // 1 | 1 = 1
  EXPECT_EQ(bv1[1], true);  // 0 | 1 = 1
  EXPECT_EQ(bv1[2], true);  // 1 | 0 = 1
}

TEST(BitVectorTest, BitwiseXORAssignment) {
  BitVector<> bv1;
  BitVector<> bv2;

  bv1.push_back(1);
  bv1.push_back(0);
  bv1.push_back(1);

  bv2.push_back(1);
  bv2.push_back(1);
  bv2.push_back(0);

  bv1 ^= bv2;

  EXPECT_EQ(bv1.size(), 3);
  EXPECT_EQ(bv1[0], false);  // 1 ^ 1 = 0
  EXPECT_EQ(bv1[1], true);   // 0 ^ 1 = 1
  EXPECT_EQ(bv1[2], true);   // 1 ^ 0 = 1
}

TEST(BitVectorTest, BitwiseNOTAssignment) {
  BitVector<> bv;

  bv.push_back(1);
  bv.push_back(0);
  bv.push_back(1);

  ~bv;

  EXPECT_EQ(bv.size(), 3);
  EXPECT_EQ(bv[0], false);  // ~1 = 0
  EXPECT_EQ(bv[1], true);   // ~0 = 1
  EXPECT_EQ(bv[2], false);  // ~1 = 0
}
