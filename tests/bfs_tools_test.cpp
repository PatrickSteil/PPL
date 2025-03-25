#include "../datastructures/bfs_tools.h"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

TEST(GenerationCheckerThreadSafeTest, SequentialMarking) {
  const static std::size_t SIZE = 1000;
  bfs::GenerationCheckerThreadSafe<> checker(SIZE);

  std::vector<std::size_t> q(SIZE, 0);

  for (std::size_t i = 0; i < SIZE; ++i) {
    if (checker.isMarked(i)) continue;
    checker.mark(i);
    q[i] = i;
  }

  std::size_t totalSum = std::accumulate(q.begin(), q.end(), 0);
  EXPECT_EQ(totalSum, (SIZE * (SIZE - 1)) / 2);
}

TEST(GenerationCheckerThreadSafeTest, ConcurrentMarking) {
  const static std::size_t SIZE = 1000;
  bfs::GenerationCheckerThreadSafe<> checker(SIZE);

  std::vector<std::size_t> q(SIZE, 0);
  auto worker = [&](std::size_t thread_id) {
    for (std::size_t i = thread_id; i < SIZE; ++i) {
      if (checker.isMarked(i)) continue;
      checker.mark(i);
      q[i] = i;
    }
  };

  const static std::size_t NUM_THREADS = 16;
  std::vector<std::thread> threads;
  for (std::size_t t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back(worker, t);
  }

  for (auto &t : threads) {
    t.join();
  }

  std::size_t totalSum = std::accumulate(q.begin(), q.end(), 0);
  EXPECT_EQ(totalSum, (SIZE * (SIZE - 1)) / 2);
}
