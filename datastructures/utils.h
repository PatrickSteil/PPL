#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <bitset>
#include <concepts>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <ranges>
#include <thread>
#include <utility>
#include <vector>

inline int branchlessConditional(const bool predicate, const int ifTrue,
                                 const int ifFalse) noexcept {
  int result = 0;
  __asm__ __volatile__(
      "mov    %[ifTrue], %[result]\n\t"
      "test   %[predicate], %[predicate]\n\t"
      "cmovz  %[ifFalse], %[result]\n\t"
      : [result] "=&r"(result)
      : [predicate] "r"(predicate), [ifTrue] "r"(ifTrue), [ifFalse] "r"(ifFalse)
      : "cc");
  return result;
}

template <typename FUNC>
void parallelFor(std::size_t start, std::size_t end, FUNC &&func,
                 std::size_t numThreads = std::thread::hardware_concurrency()) {
  std::vector<std::thread> threads;
  std::size_t chunkSize = (end - start + numThreads - 1) / numThreads;

  for (std::size_t t = 0; t < numThreads; ++t) {
    std::size_t chunkStart = start + t * (chunkSize);
    std::size_t chunkEnd = std::min(chunkStart + chunkSize, end);

    if (chunkStart >= end) break;

    threads.emplace_back([=, &func]() {
      for (std::size_t i = chunkStart; i < chunkEnd; ++i) {
        func(t, i);
      }
    });
  }

  for (auto &thread : threads) {
    thread.join();
  }
}

template <typename T>
void sortAndRemoveDuplicates(std::vector<T> &vec) {
  std::sort(vec.begin(), vec.end());
  auto newEnd = std::unique(vec.begin(), vec.end());
  vec.erase(newEnd, vec.end());
}

template <typename T, typename Compare>
std::vector<std::size_t> sort_permutation(const std::vector<T> &vec,
                                          const Compare &&compare) {
  std::vector<std::size_t> p(vec.size());
  std::iota(p.begin(), p.end(), 0);
  std::sort(p.begin(), p.end(), [&](std::size_t i, std::size_t j) {
    return compare(vec[i], vec[j]);
  });
  return p;
}

template <typename T>
std::vector<T> apply_permutation(const std::vector<T> &vec,
                                 const std::vector<std::size_t> &p) {
  std::vector<T> sorted_vec(vec.size());
  std::transform(p.begin(), p.end(), sorted_vec.begin(),
                 [&](std::size_t i) { return vec[i]; });
  return sorted_vec;
}

template <typename T>
void apply_permutation_in_place(std::vector<T> &vec,
                                const std::vector<std::size_t> &p) {
  std::vector<bool> done(vec.size());
  for (std::size_t i = 0; i < vec.size(); ++i) {
    if (done[i]) {
      continue;
    }
    done[i] = true;
    std::size_t prev_j = i;
    std::size_t j = p[i];
    while (i != j) {
      std::swap(vec[prev_j], vec[j]);
      done[j] = true;
      prev_j = j;
      j = p[j];
    }
  }
}

template <std::size_t N>
void *getUnderlyingPointer(std::bitset<N> &bs) {
  return reinterpret_cast<void *>(&bs);
}

template <std::size_t N>
std::size_t findFirstOne(std::bitset<N> &left, std::bitset<N> &right) {
  static_assert(N % 64 == 0, "Bitset size must be a multiple of 64");
  constexpr std::size_t chunkSize = 64;
  constexpr std::size_t numChunks = N / chunkSize;

  void *ptrLeft = getUnderlyingPointer(left);
  void *ptrRight = getUnderlyingPointer(right);

  uint64_t *chunksLeft = reinterpret_cast<uint64_t *>(ptrLeft);
  uint64_t *chunksRight = reinterpret_cast<uint64_t *>(ptrRight);

#pragma GCC unroll(4)
  for (std::size_t i = 0; i < numChunks; ++i) {
    if (chunksLeft[i] & chunksRight[i]) {
      return (i * chunkSize + static_cast<std::size_t>(std::countr_zero(
                                  chunksLeft[i] & chunksRight[i])));
    }
  }
  return N + 1;
}

// template <typename T>
// void parallel_fill(std::vector<T> &v, const T &value) {
//   if (v.empty()) return;

// #pragma omp parallel
//   {
//     auto tid = omp_get_thread_num();
//     auto num_threads = omp_get_num_threads();

//     auto chunksize = v.size() / num_threads;

//     auto begin = v.begin() + chunksize * tid;
//     auto end = (tid == num_threads - 1) ? v.end() : begin + chunksize;

//     std::fill(begin, end, value);
//   }
// }

// template <typename T>
// void parallel_assign(std::vector<T> &v, std::size_t n, const T &value) {
//   v.clear();
//   v.reserve(n);

//   auto data_ptr = v.data();
// #pragma omp parallel
//   {
//     auto tid = omp_get_thread_num();
//     auto num_threads = omp_get_num_threads();
//     auto chunksize = n / num_threads;
//     auto remainder = n % num_threads;

//     auto start =
//         chunksize * tid + std::min(static_cast<std::size_t>(tid),
//                                    static_cast<std::size_t>(remainder));
//     auto end =
//         start + chunksize +
//         (static_cast<std::size_t>(tid) < static_cast<std::size_t>(remainder)
//              ? 1
//              : 0);

//     std::uninitialized_fill(data_ptr + start, data_ptr + end, value);
//   }
//   v.resize(n);  // Update size to match the number of elements
// }

// template <typename T>
// void parallel_iota(std::vector<T> &v, T start_value) {
//   if (v.empty()) return;
// #pragma omp parallel
//   {
//     auto tid = omp_get_thread_num();
//     auto num_threads = omp_get_num_threads();

//     auto chunksize = v.size() / num_threads;

//     auto begin = v.begin() + chunksize * tid;
//     auto end = (tid == num_threads - 1) ? v.end() : begin + chunksize;

//     T value = start_value + (begin - v.begin());
//     for (auto it = begin; it != end; ++it) {
//       *it = value++;
//     }
//   }
// }

// template <typename T>
// void parallel_assign_iota(std::vector<T> &v, std::size_t n, T start_value) {
//   v.resize(n);
//   parallel_iota(v, start_value);
// }

template <std::integral VALUE>
std::vector<std::pair<VALUE, VALUE>> generateRandomQueries(int numQueries,
                                                           int minVertex,
                                                           int maxVertex) {
  std::vector<std::pair<VALUE, VALUE>> queries;
  queries.reserve(numQueries);
  std::srand(42);

  for (int i = 0; i < numQueries; ++i) {
    VALUE source = minVertex + std::rand() % (maxVertex - minVertex);
    VALUE target = minVertex + std::rand() % (maxVertex - minVertex);

    queries.emplace_back(source, target);
  }

  return queries;
}

template <typename T>
bool fetch_max(std::atomic<T> &atomicValue, T newValue) {
  T oldValue = atomicValue.load();
  while (newValue > oldValue) {
    if (atomicValue.compare_exchange_weak(oldValue, newValue)) {
      return true;
    }
  }
  return false;
}

template <typename T>
bool intersect(const std::vector<T> &A, const std::vector<T> &B) {
  assert(std::is_sorted(A.begin(), A.end()));
  assert(std::is_sorted(B.begin(), B.end()));

  std::size_t i = 0;
  std::size_t j = 0;

  const std::size_t ASize = A.size();
  const std::size_t BSize = B.size();

  while (i < ASize && j < BSize) {
    if (A[i] == B[j]) return true;

    if (A[i] < B[j])
      ++i;
    else
      ++j;
  }

  return false;
}

template <typename Iterator>
bool intersect(Iterator A_begin, Iterator A_end, Iterator B_begin,
               Iterator B_end) {
  Iterator it_A = A_begin;
  Iterator it_B = B_begin;

  while (it_A != A_end && it_B != B_end) {
    if (*it_A == *it_B) return true;

    if (*it_A < *it_B)
      ++it_A;
    else
      ++it_B;
  }

  return false;
}

template <typename Iterator>
bool intersect_delta(Iterator A_begin, Iterator A_end, Iterator B_begin,
                     Iterator B_end) {
  if (A_begin == A_end || B_begin == B_end) return false;

  // Decode first elements
  auto val_A = *A_begin;
  auto val_B = *B_begin;

  ++A_begin;
  ++B_begin;

  while (A_begin != A_end && B_begin != B_end) {
    if (val_A == val_B) return true;

    if (val_A < val_B) {
      val_A += *A_begin + 1;
      ++A_begin;
    } else {
      val_B += *B_begin + 1;
      ++B_begin;
    }
  }

  // Process remaining elements in A
  while (A_begin != A_end) {
    val_A += *A_begin + 1;
    if (val_A == val_B) return true;
    ++A_begin;
  }

  // Process remaining elements in B
  while (B_begin != B_end) {
    val_B += *B_begin + 1;
    if (val_A == val_B) return true;
    ++B_begin;
  }

  return val_A == val_B;
}