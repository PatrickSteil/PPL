#pragma once

#include <cassert>
#include <cstdint>
#include <type_traits>
#include <vector>

template <typename T>
class LookupStorage {
  static_assert(std::is_integral_v<T>, "T must be an integral type");

 public:
  explicit LookupStorage(std::size_t size)
      : timestamps_(size, 0), storage_(), size_(size), current_generation_(1) {}

  void mark(T index) {
    assert(static_cast<std::size_t>(index) < size_);
    timestamps_[index] = current_generation_;
  }

  bool isMarked(T index) const {
    assert(static_cast<std::size_t>(index) < size_);
    return timestamps_[index] == current_generation_;
  }

  void add(T value) {
    if (!isMarked(value)) {
      mark(value);
      storage_.push_back(value);
    }
  }

  const std::vector<T>& getStorage() const { return storage_; }

  void clear() {
    storage_.clear();
    ++current_generation_;
    if (current_generation_ == 0) {
      std::fill(timestamps_.begin(), timestamps_.end(), 0);
      current_generation_ = 1;
    }
  }

  std::size_t size() const { return storage_.size(); }

 private:
  std::vector<uint32_t> timestamps_;
  std::vector<T> storage_;
  std::size_t size_;
  uint32_t current_generation_;
};