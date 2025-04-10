/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <emmintrin.h>
#include <immintrin.h>
#include <smmintrin.h>

#include <cassert>
#include <cstdint>
#include <iostream>
#include <type_traits>

#include <cassert>
#include <cstdint>
#include <emmintrin.h>
#include <immintrin.h>
#include <iostream>
#include <smmintrin.h>
#include <type_traits>

template <int Bits> struct simd;

// Specialization for 128 bits.
// We choose __m128i as the storage. In our example we assume a lane type of
// std::uint16_t so that the vector contains 8 elements.
template <> struct simd<128> {
  static constexpr int lanes = 8; // 128 / 16 = 8 elements
  union {
    __m128i vec;
    std::uint16_t arr[lanes];
  };

  // Default constructor fills with the given value (default 0xFFFF).
  simd(std::uint16_t default_value = 0xFFFF) {
    vec = _mm_set1_epi16(default_value);
  }

  // Constructor from __m128i.
  explicit simd(__m128i v) : vec(v) {}

  // clear() fills the register with the given value.
  void clear(const std::uint16_t value = -1) { vec = _mm_set1_epi16(value); }

  // setMin(): in-place element-wise minimum.
  void setMin(const simd<128> &other) { vec = _mm_min_epu16(vec, other.vec); }

  // setMax(): in-place element-wise maximum.
  void setMax(const simd<128> &other) { vec = _mm_max_epu16(vec, other.vec); }

  // Subscript operators.
  std::uint16_t &operator[](int i) {
    assert(i >= 0 && i < lanes);
    return arr[i];
  }
  const std::uint16_t &operator[](int i) const {
    assert(i >= 0 && i < lanes);
    return arr[i];
  }

  // Addition operator.
  simd operator+(const simd &other) const {
    return simd(_mm_add_epi16(vec, other.vec));
  }

  // Print routine.
  void print() const {
    for (int i = 0; i < lanes; ++i)
      std::cout << arr[i] << " ";
    std::cout << std::endl;
  }
};

// Specialization for 256 bits.
// We use __m256i as the underlying type and assume a lane type of
// std::uint16_t, giving 16 elements (256 / 16).
template <> struct simd<256> {
  static constexpr int lanes = 16;
  union {
    __m256i vec;
    std::uint16_t arr[lanes];
  };

  simd(std::uint16_t default_value = 0xFFFF) {
    vec = _mm256_set1_epi16(default_value);
  }

  explicit simd(__m256i v) : vec(v) {}

  // clear() fills the 256-bit register with the given value.
  void clear(const std::uint16_t value = -1) { vec = _mm256_set1_epi16(value); }

  // setMin(): in-place element-wise minimum.
  void setMin(const simd<256> &other) {
    vec = _mm256_min_epu16(vec, other.vec);
  }

  // setMax(): in-place element-wise maximum.
  void setMax(const simd<256> &other) {
    vec = _mm256_max_epu16(vec, other.vec);
  }

  std::uint16_t &operator[](int i) {
    assert(i >= 0 && i < lanes);
    return arr[i];
  }
  const std::uint16_t &operator[](int i) const {
    assert(i >= 0 && i < lanes);
    return arr[i];
  }

  simd operator+(const simd &other) const {
    return simd(_mm256_add_epi16(vec, other.vec));
  }

  void print() const {
    for (int i = 0; i < lanes; ++i)
      std::cout << arr[i] << " ";
    std::cout << std::endl;
  }
};

// Specialization for 512 bits.
// We represent 512 bits as two __m256i registers. Using 16-bit lanes each,
// we have 32 elements.
template <> struct simd<512> {
  static constexpr int lanes = 32;
  union {
    struct {
      __m256i lo, hi;
    };
    std::uint16_t arr[lanes];
  };

  simd(std::uint16_t default_value = 0xFFFF) {
    lo = _mm256_set1_epi16(default_value);
    hi = _mm256_set1_epi16(default_value);
  }

  simd(__m256i l, __m256i h) : lo(l), hi(h) {}

  // clear() fills both halves with the given value.
  void clear(const std::uint16_t value = -1) {
    lo = _mm256_set1_epi16(value);
    hi = _mm256_set1_epi16(value);
  }

  // setMin(): in-place element-wise minimum.
  void setMin(const simd<512> &other) {
    lo = _mm256_min_epu16(lo, other.lo);
    hi = _mm256_min_epu16(hi, other.hi);
  }

  // setMax(): in-place element-wise maximum.
  void setMax(const simd<512> &other) {
    lo = _mm256_max_epu16(lo, other.lo);
    hi = _mm256_max_epu16(hi, other.hi);
  }

  std::uint16_t &operator[](int i) {
    assert(i >= 0 && i < lanes);
    return arr[i];
  }
  const std::uint16_t &operator[](int i) const {
    assert(i >= 0 && i < lanes);
    return arr[i];
  }

  // Addition operator.
  simd operator+(const simd &other) const {
    return simd(_mm256_add_epi16(lo, other.lo), _mm256_add_epi16(hi, other.hi));
  }

  void print() const {
    for (int i = 0; i < lanes; ++i)
      std::cout << arr[i] << " ";
    std::cout << std::endl;
  }
};

template <typename SIMD_HOLDER>
bool prune(const SIMD_HOLDER &left, const SIMD_HOLDER &right,
           const std::size_t offset = 0) {
  assert(offset < SIMD_HOLDER::lanes);
  bool result = false;
  for (std::size_t i = 0; i < offset; ++i) {
    result |= (left[i] <= right[i]);
  }
  return result;
}

template <>
bool prune<simd<512>>(const simd<512> &left, const simd<512> &right,
                      const std::size_t offset) {
  assert(offset <= simd<512>::lanes);
  __m256i min_lo = _mm256_min_epu16(left.lo, right.lo);
  __m256i cmp_lo = _mm256_cmpeq_epi16(left.lo, min_lo);
  int mask_lo = _mm256_movemask_epi8(cmp_lo);
  __m256i min_hi = _mm256_min_epu16(left.hi, right.hi);
  __m256i cmp_hi = _mm256_cmpeq_epi16(left.hi, min_hi);
  int mask_hi = _mm256_movemask_epi8(cmp_hi);
  uint64_t combined_mask = ((uint64_t)mask_hi << 32) | (uint32_t)mask_lo;
  uint64_t offset_mask;
  if (offset * 2 < 64)
    offset_mask = ((uint64_t)1 << (offset * 2)) - 1;
  else
    offset_mask = 0xFFFFFFFFFFFFFFFFULL;
  return (combined_mask & offset_mask) != 0;
}
