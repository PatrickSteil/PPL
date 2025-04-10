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

TEST(BinaryHeapTest, SingleAndMultiplePushPop) {
  bfs::BinaryHeap<int> heap(10);

  heap.push(5);
  heap.push(3);
  heap.push(8);
  heap.push(1);

  EXPECT_EQ(heap.top(), 1);

  EXPECT_EQ(heap.pop(), 1);
  EXPECT_EQ(heap.pop(), 3);

  EXPECT_EQ(heap.top(), 5);

  std::vector<int> poppedElements;
  heap.doForAllPoppedElements([&](int v) { poppedElements.push_back(v); });
  std::vector<int> expectedPopped = {1, 3};
  EXPECT_EQ(poppedElements, expectedPopped);
}

TEST(BinaryHeapTest, PushVectorHeapify) {
  bfs::BinaryHeap<int> heap(10);
  std::vector<int> elems = {10, 2, 15, 4, 7};
  heap.push(elems);

  EXPECT_EQ(heap.top(), 2);

  std::vector<int> popped;
  while (!heap.empty()) {
    popped.push_back(heap.pop());
  }
  std::vector<int> expected = {2, 4, 7, 10, 15};
  EXPECT_EQ(popped, expected);
}

TEST(BinaryHeapTest, PopEmptyHeapThrows) {
  bfs::BinaryHeap<int> heap(5);
  EXPECT_THROW(heap.pop(), std::runtime_error);
}

TEST(BinaryHeapTest, ActiveHeapFullThrows) {
  bfs::BinaryHeap<int> heap(3);
  heap.push(1);
  heap.push(2);
  heap.push(3);
  EXPECT_THROW(heap.push(4), std::runtime_error);
}

TEST(BinaryHeapTest, ClearOnPushVector) {
  bfs::BinaryHeap<int> heap(10);
  heap.push(5);
  heap.push(3);
  heap.pop();

  std::vector<int> elems = {20, 10, 30};
  heap.push(elems);
  EXPECT_EQ(heap.top(), 10);

  std::vector<int> poppedElements;
  heap.doForAllPoppedElements([&](int v) { poppedElements.push_back(v); });
  EXPECT_TRUE(poppedElements.empty());
}