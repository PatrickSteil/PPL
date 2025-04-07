/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <emmintrin.h>
#include <smmintrin.h>

#include <cstdint>
#include <iostream>

// Holds a 128-bit register and performs operations on the 8x16 unsigned ints
struct simd_u16x8 {
  union {
    __m128i data;
    std::uint16_t arr[8];
  };

  // Default constructor: fills with default_value (defaults to -1, i.e. 0xFFFF)
  simd_u16x8(std::uint16_t default_value = -1)
      : data(_mm_set1_epi16(default_value)) {}

  // Constructor from __m128i.
  explicit simd_u16x8(__m128i d) : data(d) {}

  // Constructor from 8 unsigned 16-bit integers.
  simd_u16x8(std::uint16_t a0, std::uint16_t a1, std::uint16_t a2,
             std::uint16_t a3, std::uint16_t a4, std::uint16_t a5,
             std::uint16_t a6, std::uint16_t a7) {
    data = _mm_setr_epi16(a0, a1, a2, a3, a4, a5, a6, a7);
  }

  // Clear the register by filling it with -1 (0xFFFF for 16-bit unsigned ints)
  void clear(const std::uint16_t value = -1) { data = _mm_set1_epi16(value); }

  // Overload operator[] for random access (non-const version).
  std::uint16_t& operator[](int index) { return arr[index]; }

  // Overload operator[] for random access (const version).
  const std::uint16_t& operator[](int index) const { return arr[index]; }

  // Overload addition operator.
  simd_u16x8 operator+(const simd_u16x8& other) const {
    return simd_u16x8(_mm_add_epi16(data, other.data));
  }

  // Overload subtraction operator.
  simd_u16x8 operator-(const simd_u16x8& other) const {
    return simd_u16x8(_mm_sub_epi16(data, other.data));
  }

  // Overload bitwise AND operator.
  simd_u16x8 operator&(const simd_u16x8& other) const {
    return simd_u16x8(_mm_and_si128(data, other.data));
  }

  // Overload bitwise OR operator.
  simd_u16x8 operator|(const simd_u16x8& other) const {
    return simd_u16x8(_mm_or_si128(data, other.data));
  }

  // Overload bitwise XOR operator.
  simd_u16x8 operator^(const simd_u16x8& other) const {
    return simd_u16x8(_mm_xor_si128(data, other.data));
  }

  // Overload bitwise NOT operator.
  simd_u16x8 operator~() const {
    __m128i ones = _mm_set1_epi16(-1);
    return simd_u16x8(_mm_xor_si128(data, ones));
  }

  // Element-wise minimum using SSE4.1 intrinsic.
  simd_u16x8 min(const simd_u16x8& other) const {
    return simd_u16x8(_mm_min_epu16(data, other.data));
  }

  // Element-wise maximum using SSE4.1 intrinsic.
  simd_u16x8 max(const simd_u16x8& other) const {
    return simd_u16x8(_mm_max_epu16(data, other.data));
  }

  // Sets element wise min using SSE4.1 intrinsic.
  void setMin(const simd_u16x8& other) {
    data = _mm_min_epu16(data, other.data);
  }

  // Similarly, fix setMax if needed:
  void setMax(const simd_u16x8& other) {
    data = _mm_max_epu16(data, other.data);
  }

  // Element-wise addition using SSE2 intrinsic.
  simd_u16x8 add(const simd_u16x8& other) const {
    return simd_u16x8(_mm_add_epi16(data, other.data));
  }

  // Random access: get the element at the given index.
  std::uint16_t get(int index) const { return arr[index]; }

  // Random access: set the element at the given index.
  void set(int index, std::uint16_t value) { arr[index] = value; }

  // Returns the sum of the 8 unsigned 16-bit integers.
  std::uint32_t sum() const {
    __m128i zero = _mm_setzero_si128();

    __m128i low = _mm_unpacklo_epi16(data, zero);
    __m128i high = _mm_unpackhi_epi16(data, zero);

    __m128i sum32 = _mm_add_epi32(low, high);

    sum32 = _mm_hadd_epi32(sum32, sum32);
    sum32 = _mm_hadd_epi32(sum32, sum32);

    return static_cast<std::uint32_t>(_mm_cvtsi128_si32(sum32));
  }

  // Print the vector elements for debugging.
  void print() const {
    for (int i = 0; i < 8; ++i) {
      std::cout << static_cast<int>(arr[i]) << " ";
    }
    std::cout << std::endl;
  }
};

bool prune(const simd_u16x8& left, const simd_u16x8& right,
           const std::size_t offset = 0) {
  bool result = false;
  for (std::size_t i = 0; i < offset; ++i) {
    result |= (left[i] <= right[i]);
  }
  return result;
}

/*
bool prune(const simd_u16x8& left, const simd_u16x8& right, const std::size_t
offset = 0) {
    // Adjust the unsigned values to signed space by subtracting 0x8000.
    __m128i shift = _mm_set1_epi16(0x8000);
    __m128i left_shift = _mm_sub_epi16(left.data, shift);
    __m128i right_shift = _mm_sub_epi16(right.data, shift);

    // left[i] <= right[i] is equivalent to !(right[i] < left[i])
    __m128i cmp = _mm_cmplt_epi16(right_shift, left_shift);
    // Invert the mask: if left[i] <= right[i], the lane becomes 0xFFFF.
    __m128i le_mask = _mm_andnot_si128(cmp, _mm_set1_epi16(0xFFFF));

    // Create an index vector [0, 1, 2, 3, 4, 5, 6, 7]
    __m128i indices = _mm_setr_epi16(0, 1, 2, 3, 4, 5, 6, 7);
    __m128i offset_vec = _mm_set1_epi16(static_cast<std::uint16_t>(offset));
    // Mask lanes where the index is less than offset.
    __m128i idx_mask = _mm_cmplt_epi16(indices, offset_vec);

    // Combine both conditions.
    __m128i final_mask = _mm_and_si128(le_mask, idx_mask);

    // Pack the most significant bits of each byte into an integer mask.
    int mask = _mm_movemask_epi8(final_mask);
    return mask != 0;
}
*/