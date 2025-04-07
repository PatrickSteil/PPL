/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#include "../datastructures/simd_tools.h"

#include <gtest/gtest.h>

// Test default constructor: should fill with 0xFFFF (-1)
TEST(SimdU16x8Test, DefaultFill) {
  simd_u16x8 vec;
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(vec[i], 0xFFFF);
  }
}

// Test constructor with specific values and operator[] access
TEST(SimdU16x8Test, ConstructAndAccess) {
  simd_u16x8 vec(1, 2, 3, 4, 5, 6, 7, 8);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(vec[i], static_cast<std::uint16_t>(i + 1));
  }
}

// Test addition operator overloading
TEST(SimdU16x8Test, AdditionOperator) {
  simd_u16x8 vec1(1, 2, 3, 4, 5, 6, 7, 8);
  simd_u16x8 vec2(8, 7, 6, 5, 4, 3, 2, 1);
  simd_u16x8 result = vec1 + vec2;
  simd_u16x8 expected(9, 9, 9, 9, 9, 9, 9, 9);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(result[i], expected[i]);
  }
}

// Test subtraction operator overloading
TEST(SimdU16x8Test, SubtractionOperator) {
  simd_u16x8 vec1(10, 10, 10, 10, 10, 10, 10, 10);
  simd_u16x8 vec2(1, 2, 3, 4, 5, 6, 7, 8);
  simd_u16x8 result = vec1 - vec2;
  simd_u16x8 expected(9, 8, 7, 6, 5, 4, 3, 2);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(result[i], expected[i]);
  }
}

// Test bitwise operations: AND, OR, XOR and NOT
TEST(SimdU16x8Test, BitwiseOperators) {
  simd_u16x8 vec1(0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA,
                  0xAAAA);
  simd_u16x8 vec2(0x5555, 0x5555, 0x5555, 0x5555, 0x5555, 0x5555, 0x5555,
                  0x5555);

  simd_u16x8 andResult = vec1 & vec2;  // Expected: 0x0000
  simd_u16x8 orResult = vec1 | vec2;   // Expected: 0xFFFF
  simd_u16x8 xorResult = vec1 ^ vec2;  // Expected: 0xFFFF

  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(andResult[i], 0);
    EXPECT_EQ(orResult[i], 0xFFFF);
    EXPECT_EQ(xorResult[i], 0xFFFF);
  }

  // Test bitwise NOT: ~vec1 should invert 0xAAAA to 0x5555.
  simd_u16x8 notResult = ~vec1;
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(notResult[i], 0x5555);
  }
}

// Test element-wise min function using SSE4.1 intrinsic
TEST(SimdU16x8Test, MinFunction) {
  simd_u16x8 vec1(5, 6, 7, 8, 9, 10, 11, 12);
  simd_u16x8 vec2(10, 9, 8, 7, 6, 5, 4, 3);
  simd_u16x8 result = vec1.min(vec2);
  simd_u16x8 expected(5, 6, 7, 7, 6, 5, 4, 3);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(result[i], expected[i]);
  }
}