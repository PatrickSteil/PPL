/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <atomic>
#include <cassert>
#include <concepts>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "spinlock.h"
#include "utils.h"

namespace bfs {
// Class representing a fixed-size queue for storing vertices.
// It allows pushing and popping elements in FIFO order.
template <typename VertexType = std::uint64_t>
class FixedSizedQueue {
 public:
  // Constructor initializes the queue with the given size.
  explicit FixedSizedQueue(std::size_t size = 0)
      : data(size), read(0), write(0) {}

  // Resizes the queue and resets all values.
  void resize(std::size_t size) {
    data.assign(size, VertexType(0));
    read = 0;
    write = 0;
  }

  // Pushes a vertex into the queue.
  // Ensures there is enough space before inserting.
  void push(VertexType v) {
    assert(write < data.size());
    data[write++] = v;
  }

  // Pops a vertex from the queue.
  // Ensures there is at least one element before removing.
  VertexType pop() {
    assert(read < write);
    return data[read++];
  }

  // Checks if the queue is empty.
  bool isEmpty() const { return read == write; }

  // Resets the queue by setting read and write pointers to zero.
  void reset() { read = write = 0; }

  std::vector<VertexType> data;
  std::size_t read;
  std::size_t write;
};

template <typename VertexType = std::uint64_t>
class FixedSizedQueueThreadSafe {
 public:
  explicit FixedSizedQueueThreadSafe(std::size_t size = 0)
      : data(size), read(0), write(0) {}

  // Resize the queue safely (not thread-safe in this version)
  void resize(std::size_t size) {
    data.resize(size);
    read.store(0, std::memory_order_relaxed);
    write.store(0, std::memory_order_relaxed);
  }

  void push(VertexType v) {
    auto index = write.fetch_add(1, std::memory_order_relaxed);
    assert(index < data.size());  // Ensure no overflows
    data[index] = v;
  }

  VertexType pop() {
    auto current_read = read.load(std::memory_order_relaxed);
    if (current_read >= write.load(std::memory_order_acquire)) {
      return static_cast<VertexType>(-1);  // Queue empty
    }
    auto index = read.fetch_add(1, std::memory_order_relaxed);
    assert(index < data.size());  // Ensure valid access
    return data[index];
  }

  bool isEmpty() const {
    return read.load(std::memory_order_acquire) ==
           write.load(std::memory_order_acquire);
  }

  void reset() {
    read.store(0, std::memory_order_relaxed);
    write.store(0, std::memory_order_relaxed);
  }

  std::vector<VertexType> data;
  std::atomic_size_t read;
  std::atomic_size_t write;
};

template <typename VertexType = std::uint64_t>
class FixedSizedQueueThreadSafeSpinlock {
 public:
  explicit FixedSizedQueueThreadSafeSpinlock(std::size_t size = 0)
      : data(size), read(0), write(0) {}

  void resize(std::size_t size) {
    mutex_read.lock();
    mutex_write.lock();

    data.resize(size);
    read = 0;
    write = 0;

    mutex_write.unlock();
    mutex_read.unlock();
  }

  void push(VertexType v) {
    std::lock_guard<Spinlock> lock(mutex_write);
    assert(write < data.size());
    data[write++] = v;
  }

  VertexType pop() {
    mutex_read.lock();
    mutex_write.lock();

    if (read == write) [[unlikely]] {
      mutex_write.unlock();
      mutex_read.unlock();
      return static_cast<VertexType>(-1);
    }

    VertexType value = data[read++];

    mutex_write.unlock();
    mutex_read.unlock();

    return value;
  }

  bool isEmpty() const {
    mutex_read.lock();
    mutex_write.lock();

    bool result = (read == write);

    mutex_write.unlock();
    mutex_read.unlock();

    return result;
  }

  void reset() {
    mutex_read.lock();
    mutex_write.lock();

    read = 0;
    write = 0;

    mutex_write.unlock();
    mutex_read.unlock();
  }

 private:
  mutable Spinlock mutex_read;
  mutable Spinlock mutex_write;
  std::vector<VertexType> data;
  std::size_t read;
  std::size_t write;
};

// From exersice 4
template <class T>
class ConcurrentQueue {
  static_assert(std::is_default_constructible<T>::value,
                "T must be default constructible.");

 public:
  explicit ConcurrentQueue(std::size_t capacity)
      : capacity_(capacity),
        data_(std::make_unique<AlignedData<T>[]>(capacity)),
        read_(0),
        write_(0) {}

  bool try_push(const T &value) {
    assert(value != T{});

    std::size_t write_index = write_.load(std::memory_order_relaxed);
    std::size_t next_write_index = (write_index + 1) % capacity_;

    if (next_write_index == read_.load(std::memory_order_acquire)) {
      return false;
    }

    data_[write_index]().store(value, std::memory_order_release);
    write_.store(next_write_index, std::memory_order_release);
    return true;
  }

  bool try_pop(T &value) {
    std::size_t read_index = read_.load(std::memory_order_relaxed);

    if (read_index == write_.load(std::memory_order_acquire)) {
      return false;
    }

    value = data_[read_index]().load(std::memory_order_acquire);
    data_[read_index]().store(T{}, std::memory_order_release);
    read_.store((read_index + 1) % capacity_, std::memory_order_release);
    return true;
  }

 private:
  template <typename L>
  struct alignas(64) AlignedData {
    std::atomic<L> data;

    std::atomic<L> &operator()() { return data; }

    const std::atomic<L> &operator()() const { return data; }
  };

  const std::size_t capacity_;
  std::unique_ptr<AlignedData<T>[]> data_;
  std::atomic<std::size_t> read_;
  std::atomic<std::size_t> write_;
};

// Class that tracks whether a vertex has been seen using a generational
// counter. This avoids having to clear the entire seen array by incrementing
// the generation counter.
template <std::integral GenerationType = std::uint16_t>
class GenerationChecker {
 public:
  // Constructor initializes the seen array with the given size and sets the
  // generation to 1.
  explicit GenerationChecker(std::size_t size = 0)
      : seen(size, 0), generation(1) {}

  // Resizes the seen array and resets all values to zero.
  void resize(std::size_t size) {
    seen.assign(size, static_cast<GenerationType>(0));
    generation = 1;
  }

  // Increments the generation counter, effectively resetting all marks.
  // If the counter overflows, resets the entire seen array.
  void reset() {
    ++generation;
    if (generation == 0) {
      std::fill(seen.begin(), seen.end(), 0);
      ++generation;
    }
  }

  // Checks whether the given index is within bounds.
  inline bool isValid(std::size_t i) const { return i < seen.size(); }

  // Checks if a vertex at index `i` is marked as seen.
  inline bool isMarked(std::size_t i) const {
    assert(isValid(i));
    return seen[i] == generation;
  }

  // Marks a vertex at index `i` as seen by setting its value to the current
  // generation.
  inline void mark(std::size_t i) {
    assert(isValid(i));
    seen[i] = generation;
  }

  std::vector<GenerationType> seen;
  GenerationType generation;
};

// Thread-safe variant of GenerationChecker using atomic operations.
template <std::integral GenerationType = std::uint16_t>
class GenerationCheckerThreadSafe {
 public:
  // Constructor initializes the seen array and sets the generation counter
  // to 1.
  explicit GenerationCheckerThreadSafe(std::size_t size = 0)
      : seen(size), generation(1) {
    for (auto &val : seen) {
      val.store(0, std::memory_order_release);
    }
  }

  // Resizes the seen array, preserving existing values where possible.
  void resize(std::size_t size) {
    std::vector<std::atomic<GenerationType>> new_seen(size);
    for (std::size_t i = 0; i < size && i < seen.size(); ++i) {
      new_seen[i].store(seen[i].load(std::memory_order_acquire),
                        std::memory_order_release);
    }
    for (std::size_t i = seen.size(); i < size; ++i) {
      new_seen[i].store(0, std::memory_order_release);
    }
    seen = std::move(new_seen);
    generation.store(1, std::memory_order_relaxed);
  }

  // Increments the generation counter safely.
  // If it overflows, resets the entire seen array.
  void reset() {
    auto current_gen = generation.fetch_add(1, std::memory_order_acq_rel) + 1;
    if (current_gen == 0) {
      for (auto &val : seen) {
        val.store(0, std::memory_order_release);
      }
      generation.store(1, std::memory_order_release);
    }
  }

  // Checks whether the given index is within bounds.
  bool isValid(std::size_t i) const { return i < seen.size(); }

  // Checks if a vertex at index `i` is marked as seen.
  bool isMarked(std::size_t i) const {
    assert(isValid(i));
    return seen[i].load(std::memory_order_acquire) ==
           generation.load(std::memory_order_acquire);
  }

  // Marks a vertex at index `i` as seen by setting its value to the current
  // generation.
  void mark(std::size_t i) {
    assert(isValid(i));
    seen[i].store(generation.load(std::memory_order_acquire),
                  std::memory_order_release);
  }

  // Returns true if this vertex has not been seen yet, and marks it
  bool firstOccur(std::size_t i) {
    bool seenBefore = (seen[i].load(std::memory_order_acquire) == generation);
    seen[i].store(generation, std::memory_order_release);
    return !seenBefore;
  }

 private:
  std::vector<std::atomic<GenerationType>> seen;
  std::atomic<GenerationType> generation;
};

}  // namespace bfs
