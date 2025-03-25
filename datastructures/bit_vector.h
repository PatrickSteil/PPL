/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <cassert>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

template <typename T>
concept UnsignedIntegral = std::is_unsigned_v<T>;

/**
 * @class BitVector
 * @brief A compact container for storing and manipulating bits efficiently.
 *
 * This class uses a vector of unsigned integers (e.g., uint64_t) to store bits.
 * It provides methods to push bits, access bits, and serialize/deserialize the
 * bit vector.
 *
 * @tparam TYPE The unsigned integer type used to store bits (default:
 * uint64_t).
 */
template <UnsignedIntegral TYPE = uint64_t>
class BitVector {
 private:
  std::vector<TYPE> data;
  size_t size_ = 0;

  static constexpr size_t BITS_PER_WORD = sizeof(TYPE) * 8;

  /**
   * @brief Calculates the index of the word (in `data`) that contains the
   * specified bit.
   * @param bit The index of the bit.
   * @return The index of the word in `data`.
   */
  static size_t word_index(size_t bit) { return bit / BITS_PER_WORD; }

  /**
   * @brief Calculates the position of the bit within its word.
   * @param bit The index of the bit.
   * @return The bit's position within its word (0 to BITS_PER_WORD - 1).
   */
  static size_t bit_index(size_t bit) { return bit % BITS_PER_WORD; }

 public:
  /**
   * @brief Default constructor.
   */
  BitVector() = default;

  /**
   * @brief Constructs a BitVector from a vector of bools.
   * @param bits A vector of bools to initialize the BitVector.
   */
  explicit BitVector(const std::vector<bool>& bits) {
    reserve(bits.size());
    for (bool bit : bits) {
      push_back(bit);
    }
  }

  /**
   * @brief Constructs a BitVector from a vector of bools.
   * @param bits A vector of bools to initialize the BitVector.
   */
  BitVector(const std::size_t numberOfElements) {
    resize(numberOfElements);

    std::fill(data.begin(), data.end(), 0);
  }

  BitVector(const BitVector& other) = default;
  BitVector& operator=(const BitVector& other) = default;
  BitVector(BitVector&& other) noexcept = default;
  BitVector& operator=(BitVector&& other) noexcept = default;

  /**
   * @brief Adds a bit to the end of the BitVector.
   * @param value The bit value to add (true = 1, false = 0).
   */
  void push_back(bool value) {
    if (size_ % BITS_PER_WORD == 0) {
      data.emplace_back(0);
    }
    data[word_index(size_)] |= (TYPE(value) << bit_index(size_));
    ++size_;
  }

  /**
   * @brief Accesses a bit at a specific index.
   * @param index The index of the bit to access.
   * @return The value of the bit at the specified index.
   * @throws std::out_of_range if the index is out of bounds.
   */
  bool operator[](size_t index) const {
    assert(index < size_);
    return (data[word_index(index)] >> bit_index(index)) & 1;
  }

  /**
   * @brief Clears the BitVector, removing all bits.
   */
  void clear() {
    data.clear();
    size_ = 0;
  }

  /**
   * @brief Bitwise And
   */
  BitVector& operator&=(const BitVector& other) {
    assert(size_ == other.size_);

    for (size_t i = 0; i < data.size(); ++i) {
      data[i] &= other.data[i];
    }

    return *this;
  }

  /**
   * @brief Bitwise Or
   */
  BitVector& operator|=(const BitVector& other) {
    assert(size_ == other.size_);

    for (size_t i = 0; i < data.size(); ++i) {
      data[i] |= other.data[i];
    }

    return *this;
  }

  /**
   * @brief Bitwise XOr
   */
  BitVector& operator^=(const BitVector& other) {
    assert(size_ == other.size_);

    for (size_t i = 0; i < data.size(); ++i) {
      data[i] ^= other.data[i];
    }

    return *this;
  }

  /**
   * @brief Bitwise flip
   */
  BitVector& operator~() {
    for (size_t i = 0; i < data.size(); ++i) {
      data[i] = ~data[i];
    }

    return *this;
  }

  /**
   * @brief Reserves space for a specified number of bits.
   * @param bits The number of bits to reserve space for.
   */
  void reserve(size_t bits) {
    data.reserve((bits + BITS_PER_WORD - 1) / BITS_PER_WORD);
  }

  /**
   * @brief Resizes space for a specified number of bits.
   * @param bits The number of bits to reserve space for.
   */
  void resize(size_t bits) {
    data.resize((bits + BITS_PER_WORD - 1) / BITS_PER_WORD);
    size_ = bits;
  }

  /**
   * @brief Returns the number of bits stored in the BitVector.
   * @return The size of the BitVector in bits.
   */
  size_t size() const { return size_; }

  /**
   * @brief Returns the total capacity of the BitVector in bits.
   * @return The capacity of the BitVector in bits.
   */
  size_t capacity() const { return data.capacity() * BITS_PER_WORD; }

  /**
   * @brief Calculates the total memory usage of the BitVector in bytes.
   * @return The size of the BitVector in bytes, including overhead.
   */
  size_t byteSize() const {
    return sizeof(BitVector) + (data.size() * sizeof(TYPE));
  }

  /**
   * @brief Serializes the BitVector to a binary file.
   * @param filename The name of the file to write to.
   * @return True if the serialization was successful, false otherwise.
   */
  bool serialize(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file) return false;

    file.write(reinterpret_cast<const char*>(&size_), sizeof(size_));
    size_t data_size = data.size();
    file.write(reinterpret_cast<const char*>(&data_size), sizeof(data_size));
    file.write(reinterpret_cast<const char*>(data.data()),
               data.size() * sizeof(TYPE));

    return file.good();
  }

  /**
   * @brief Deserializes the BitVector from a binary file.
   * @param filename The name of the file to read from.
   * @return True if the deserialization was successful, false otherwise.
   */
  bool deserialize(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) return false;

    file.read(reinterpret_cast<char*>(&size_), sizeof(size_));
    size_t data_size;
    file.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));

    data.resize(data_size);
    file.read(reinterpret_cast<char*>(data.data()), data_size * sizeof(TYPE));

    return file.good();
  }

  /**
   * @class Iterator
   * @brief A forward iterator for traversing the bits in the BitVector.
   */
  class Iterator {
   private:
    const BitVector* bv;
    size_t index;

   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = bool;
    using difference_type = std::ptrdiff_t;
    using pointer = void;
    using reference = bool;

    /**
     * @brief Constructs an Iterator for a BitVector.
     * @param bv The BitVector to iterate over.
     * @param index The starting bit index.
     */
    Iterator(const BitVector* bv, size_t index) : bv(bv), index(index) {}

    /**
     * @brief Dereferences the iterator to access the current bit.
     * @return The value of the current bit.
     */
    bool operator*() const { return (*bv)[index]; }

    /**
     * @brief Advances the iterator to the next bit.
     * @return A reference to the updated iterator.
     */
    Iterator& operator++() {
      ++index;
      return *this;
    }

    /**
     * @brief Advances the iterator to the next bit (post-increment).
     * @return A copy of the iterator before advancement.
     */
    Iterator operator++(int) {
      Iterator temp = *this;
      ++(*this);
      return temp;
    }

    /**
     * @brief Compares two iterators for equality.
     * @param other The iterator to compare with.
     * @return True if the iterators point to the same bit, false otherwise.
     */
    bool operator==(const Iterator& other) const {
      return index == other.index;
    }

    /**
     * @brief Compares two iterators for inequality.
     * @param other The iterator to compare with.
     * @return True if the iterators point to different bits, false otherwise.
     */
    bool operator!=(const Iterator& other) const { return !(*this == other); }
  };

  /**
   * @brief Returns an iterator to the first bit in the BitVector.
   * @return An iterator pointing to the first bit.
   */
  Iterator begin() const { return Iterator(this, 0); }

  /**
   * @brief Returns an iterator to one past the last bit in the BitVector.
   * @return An iterator pointing to one past the last bit.
   */
  Iterator end() const { return Iterator(this, size_); }
};

// Some benchmark results compared to vector<bool>
/*
---------------------------------------------------------------------
Benchmark                           Time             CPU   Iterations
---------------------------------------------------------------------
BM_VectorBool_RandomAccess       5.82 ns         5.82 ns    119532178
BM_BitVector_RandomAccess        5.33 ns         5.33 ns    131385561
BM_VectorBool_Iteration       1013484 ns      1013472 ns          691
BM_BitVector_Iteration         605940 ns       605924 ns         1153
BM_VectorBool_PushBack        1733511 ns      1733499 ns          405
BM_BitVector_PushBack         1534437 ns      1534386 ns          455
*/
