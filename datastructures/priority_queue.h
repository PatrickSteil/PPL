/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <algorithm>
#include <cassert>
#include <concepts>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

#include "types.h"

template <typename Comparator = std::less<uint32_t>,
          std::integral IndexInt = std::uint32_t>
class PriorityQueue {
 private:
  using HeapElement = std::pair<uint32_t, Vertex>;
  const IndexInt noIndexInt = static_cast<IndexInt>(-1);

  std::vector<HeapElement> heap;
  std::vector<IndexInt> mapper;
  Comparator comp;

 public:
  PriorityQueue(size_t size = 0) : heap(), mapper(size, noIndexInt), comp() {}

  size_t size() const { return heap.size(); }
  bool empty() const { return heap.empty(); }
  size_t capacity() const { return mapper.size(); }

  bool isValid(IndexInt i) const { return i < mapper.size(); }

  bool heapPropertyFullfilled() const {
    for (IndexInt i = 0; i < size(); ++i) {
      IndexInt left = getLeftChild(i);
      IndexInt right = getRightChild(i);

      if (left < size() && comp(heap[left].first, heap[i].first)) {
        std::cerr << "[FAIL] Heap Property violated from " << int(i) << " to "
                  << int(left) << " (vals: " << heap[i].first << " <-> "
                  << heap[left].first << " )" << std::endl;
        return false;
      }
      if (right < size() && comp(heap[right].first, heap[i].first)) {
        std::cerr << "[FAIL] Heap Property violated from " << int(i) << " to "
                  << int(right) << " (vals: " << heap[i].first << " <-> "
                  << heap[right].first << " )" << std::endl;
        return false;
      }
    }
    return true;
  }

  uint32_t get(Vertex v) const {
    if (mapper[v] == noIndexInt) return uint32_t(-1);
    return heap[mapper[v]].first;
  }

  IndexInt getLeftChild(IndexInt i) const { return (i * 2) + 1; }

  IndexInt getRightChild(IndexInt i) const { return (i * 2) + 2; }

  IndexInt getParentIndex(IndexInt i) const { return (i - 1) / 2; }

  void swap(IndexInt i, IndexInt j) {
    assert(isValid(i) && isValid(j));
    mapper[heap[i].second] = j;
    mapper[heap[j].second] = i;
    std::swap(heap[i], heap[j]);
  }

  void siftDown(IndexInt i) {
    assert(isValid(i));
    while (true) {
      auto left = getLeftChild(i);
      auto right = getRightChild(i);
      auto smallest = i;

      if (left < size() && comp(heap[left].first, heap[smallest].first)) {
        smallest = left;
      }
      if (right < size() && comp(heap[right].first, heap[smallest].first)) {
        smallest = right;
      }
      if (smallest == i) return;

      swap(i, smallest);
      i = smallest;
    }
  }

  void siftUp(IndexInt i) {
    assert(isValid(i));
    while (i > 0) {
      IndexInt parent = getParentIndex(i);
      if (comp(heap[i].first, heap[parent].first)) {
        swap(i, parent);
        i = parent;
      } else {
        return;
      }
    }
  }

  void push(Vertex v, uint32_t value) {
    assert(isValid(v));

    if (mapper[v] == noIndexInt) {
      heap.emplace_back(value, v);
      mapper[v] = size() - 1;
      siftUp(size() - 1);
    } else {
      IndexInt i = mapper[v];
      if (heap[i].first == value) {
        return;
      }

      if (comp(heap[i].first, value)) {
        heap[i].first = value;
        siftDown(i);
      } else {
        heap[i].first = value;
        siftUp(i);
      }
    }
  }

  std::pair<uint32_t, Vertex> pop() {
    if (empty()) {
      return {0, noVertex};
    }

    auto result = heap[0];
    swap(0, size() - 1);
    heap.pop_back();
    mapper[result.second] = noIndexInt;

    if (!empty()) {
      siftDown(0);
    }

    assert(heapPropertyFullfilled());
    return result;
  }

  const HeapElement& front() const {
    assert(!empty());
    return heap.front();
  }

  void buildFrom(std::vector<uint32_t>& values) {
    mapper.clear();
    heap.clear();

    if (values.size() == 0) return;

    mapper.resize(values.size());
    heap.resize(values.size());

    for (Vertex v = 0; v < values.size(); ++v) {
      heap[v] = std::make_pair(values[v], v);
      mapper[v] = v;
    }
    buildHeapRecursive(0);
    assert(heapPropertyFullfilled());
  }

  void buildHeapRecursive(IndexInt i) {
    if (2 * i + 1 < size()) {
      buildHeapRecursive(getLeftChild(i));
    }
    if (2 * i + 2 < size()) {
      buildHeapRecursive(getRightChild(i));
    }

    siftDown(i);
  }

  void reset() {
    std::fill(mapper.begin(), mapper.end(), noIndexInt);
    heap.clear();
    heap.reserve(capacity());
  }
};

struct MaxHeapComparator {
  bool operator()(uint32_t a, uint32_t b) const { return a < b; }
};

template <typename T = Vertex, typename Comparator = std::less<T>,
          typename IndexInt = std::uint32_t>
class PriorityQueueExternal {
 private:
  const IndexInt noIndexInt = static_cast<IndexInt>(-1);

  std::vector<IndexInt> heap;
  std::vector<IndexInt> mapper;
  const std::vector<T>* values;
  Comparator comp;

 public:
  explicit PriorityQueueExternal(size_t size = 0,
                                 Comparator comp = Comparator())
      : heap(), mapper(size, noIndexInt), values(nullptr), comp(comp) {}

  void setValues(const std::vector<T>& external_values) {
    values = &external_values;
  }

  size_t size() const { return heap.size(); }
  bool empty() const { return heap.empty(); }
  size_t capacity() const { return mapper.size(); }

  bool isValid(IndexInt i) const { return i < mapper.size(); }

  bool heapPropertyFulfilled() const {
    assert(values);
    for (IndexInt i = 0; i < size(); ++i) {
      IndexInt left = getLeftChild(i);
      IndexInt right = getRightChild(i);

      if (left < size() && comp((*values)[heap[left]], (*values)[heap[i]])) {
        return false;
      }
      if (right < size() && comp((*values)[heap[right]], (*values)[heap[i]])) {
        return false;
      }
    }
    return true;
  }

  T get(IndexInt i) const {
    assert(isValid(i));
    if (mapper[i] == noIndexInt) return T{};
    return (*values)[heap[mapper[i]]];
  }

  IndexInt getLeftChild(IndexInt i) const { return (i * 2) + 1; }
  IndexInt getRightChild(IndexInt i) const { return (i * 2) + 2; }
  IndexInt getParentIndex(IndexInt i) const { return (i - 1) / 2; }

  void swap(IndexInt i, IndexInt j) {
    assert(isValid(i) && isValid(j));
    mapper[heap[i]] = j;
    mapper[heap[j]] = i;
    std::swap(heap[i], heap[j]);
  }

  void siftDown(IndexInt i) {
    assert(values);
    assert(isValid(i));
    while (true) {
      auto left = getLeftChild(i);
      auto right = getRightChild(i);
      auto smallest = i;

      if (left < size() &&
          comp((*values)[heap[left]], (*values)[heap[smallest]])) {
        smallest = left;
      }
      if (right < size() &&
          comp((*values)[heap[right]], (*values)[heap[smallest]])) {
        smallest = right;
      }
      if (smallest == i) return;

      swap(i, smallest);
      i = smallest;
    }
  }

  void siftUp(IndexInt i) {
    assert(values);
    assert(isValid(i));
    while (i > 0) {
      IndexInt parent = getParentIndex(i);
      if (comp((*values)[heap[i]], (*values)[heap[parent]])) {
        swap(i, parent);
        i = parent;
      } else {
        return;
      }
    }
  }

  void update(IndexInt i) {
    assert(values);
    assert(isValid(i));
    assert(mapper[i] != noIndexInt);
    IndexInt pos = mapper[i];
    siftDown(pos);
    siftUp(pos);
  }

  IndexInt pop() {
    assert(values);
    if (empty()) return noIndexInt;

    IndexInt result = heap[0];
    swap(0, size() - 1);
    heap.pop_back();
    mapper[result] = noIndexInt;

    if (!empty()) siftDown(0);

    return result;
  }

  IndexInt front() const {
    assert(!empty());
    return heap.front();
  }

  void buildFrom(std::vector<T>& external_values) {
    setValues(external_values);
    mapper.clear();
    heap.clear();

    if (values->empty()) return;

    mapper.resize(values->size(), noIndexInt);
    heap.resize(values->size());

    for (IndexInt i = 0; i < values->size(); ++i) {
      heap[i] = i;
      mapper[i] = i;
    }
    buildHeapRecursive(0);
  }

  void buildHeapRecursive(IndexInt i) {
    if (2 * i + 1 < size()) buildHeapRecursive(getLeftChild(i));
    if (2 * i + 2 < size()) buildHeapRecursive(getRightChild(i));
    siftDown(i);
  }

  void reset() {
    std::fill(mapper.begin(), mapper.end(), noIndexInt);
    heap.clear();
    heap.reserve(capacity());
  }
};
