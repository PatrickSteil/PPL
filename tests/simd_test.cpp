/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#include "../datastructures/simd.h"

#include <gtest/gtest.h>

// ----------------------------
// Tests for simd<128>
// ----------------------------
TEST(Simd128Test, ClearTest) {
  simd<128> vec;
  vec.clear(42);
  for (int i = 0; i < simd<128>::lanes; ++i) {
    EXPECT_EQ(vec[i], 42) << "Lane " << i << " should be 42";
  }
}

TEST(Simd128Test, AdditionTest) {
  simd<128> a(10);
  simd<128> b(20);
  simd<128> c = a + b;
  for (int i = 0; i < simd<128>::lanes; ++i) {
    EXPECT_EQ(c[i], 30) << "Lane " << i << " should be 30";
  }
}

TEST(Simd128Test, SetMinTest) {
  simd<128> a(100);
  simd<128> b(50);
  a.setMin(b);
  for (int i = 0; i < simd<128>::lanes; ++i) {
    EXPECT_EQ(a[i], 50) << "Lane " << i << " should be 50 after setMin";
  }
}

TEST(Simd128Test, SetMaxTest) {
  simd<128> a(100);
  simd<128> b(150);
  a.setMax(b);
  for (int i = 0; i < simd<128>::lanes; ++i) {
    EXPECT_EQ(a[i], 150) << "Lane " << i << " should be 150 after setMax";
  }
}

// ----------------------------
// Tests for simd<256>
// ----------------------------
TEST(Simd256Test, ClearTest) {
  simd<256> vec;
  vec.clear(55);
  for (int i = 0; i < simd<256>::lanes; ++i) {
    EXPECT_EQ(vec[i], 55) << "Lane " << i << " should be 55";
  }
}

TEST(Simd256Test, AdditionTest) {
  simd<256> a(10);
  simd<256> b(15);
  simd<256> c = a + b;
  for (int i = 0; i < simd<256>::lanes; ++i) {
    EXPECT_EQ(c[i], 25) << "Lane " << i << " should be 25";
  }
}

TEST(Simd256Test, SetMinTest) {
  simd<256> a(200);
  simd<256> b(180);
  a.setMin(b);
  for (int i = 0; i < simd<256>::lanes; ++i) {
    EXPECT_EQ(a[i], 180) << "Lane " << i << " should be 180 after setMin";
  }
}

TEST(Simd256Test, SetMaxTest) {
  simd<256> a(100);
  simd<256> b(120);
  a.setMax(b);
  for (int i = 0; i < simd<256>::lanes; ++i) {
    EXPECT_EQ(a[i], 120) << "Lane " << i << " should be 120 after setMax";
  }
}

// ----------------------------
// Tests for simd<512>
// ----------------------------
TEST(Simd512Test, ClearTest) {
  simd<512> vec;
  vec.clear(77);
  for (int i = 0; i < simd<512>::lanes; ++i) {
    EXPECT_EQ(vec[i], 77) << "Lane " << i << " should be 77";
  }
}

TEST(Simd512Test, AdditionTest) {
  simd<512> a(33);
  simd<512> b(44);
  simd<512> c = a + b;
  for (int i = 0; i < simd<512>::lanes; ++i) {
    EXPECT_EQ(c[i], 77) << "Lane " << i << " should be 77";
  }
}

TEST(Simd512Test, SetMinTest) {
  simd<512> a(90);
  simd<512> b(80);
  a.setMin(b);
  for (int i = 0; i < simd<512>::lanes; ++i) {
    EXPECT_EQ(a[i], 80) << "Lane " << i << " should be 80 after setMin";
  }
}

TEST(Simd512Test, SetMaxTest) {
  simd<512> a(10);
  simd<512> b(20);
  a.setMax(b);
  for (int i = 0; i < simd<512>::lanes; ++i) {
    EXPECT_EQ(a[i], 20) << "Lane " << i << " should be 20 after setMax";
  }
}