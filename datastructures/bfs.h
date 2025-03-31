/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#ifdef __GNUC__
#define PREFETCH(addr) __builtin_prefetch(addr)
#else
#define PREFETCH(addr)
#endif

#include <optional>

#include "../external/statistics_collecter.h"
#include "bfs_tools.h"
#include "graph.h"
#include "utils.h"

namespace bfs {

auto noOp = [](const Vertex /* v */) { return false; };

// Breadth-First Search (BFS) structure for traversing a graph.
struct BFS {
  const Graph &graph;
  FixedSizedQueue<Vertex> q;
  GenerationChecker<> seen;

  // Constructor initializes BFS with the given graph.
  BFS(const Graph &graph)
      : graph(graph), q(graph.numVertices()), seen(graph.numVertices()) {}

  // Resets the BFS state and resizes the queue and seen marker.
  void reset(const std::size_t numVertices) {
    q.reset();
    q.resize(numVertices);
    seen.reset();
    seen.resize(numVertices);
  }

  // Resets seen datastructure.
  void resetSeen() { seen.reset(); }

  // Applies a function to all vertices currently in the queue.
  template <typename FUNC>
  void doForAllVerticesInQ(FUNC &&func) {
    for (std::size_t i = 0; i < q.read; ++i) {
      const Vertex u = q.data[i];
      func(u);
    }
  }

  // Runs BFS from a given root vertex.
  // Uses optional callbacks for processing vertices on pop and edge
  // relaxations.
  template <bool RESET_MARKER = true,
            typename ON_POP = decltype([](const Vertex) { return false; }),
            typename ON_RELAX = decltype([](const Vertex, const Vertex) {
              return false;
            })>
  void run(const Vertex root, ON_POP &&onPop, ON_RELAX &&onRelax) {
    q.reset();

    if (RESET_MARKER) {
      seen.reset();
    }

    if (seen.isMarked(root)) [[unlikely]] {
      return;
    }

    q.push(root);
    seen.mark(root);

    while (!q.isEmpty()) {
      const Vertex u = q.pop();

      if (onPop(u)) continue;

      for (std::size_t i = graph.beginEdge(u), end = graph.endEdge(u); i < end;
           ++i) {
        if (i + 4 < end) {
          PREFETCH(&graph.toVertex[i + 4]);
        }
        const Vertex w = graph.toVertex[i];

        if (seen.isMarked(w)) continue;
        seen.mark(w);

        if (onRelax(u, w)) {
          continue;
        }

        q.push(w);
      }
    }
  }
};

struct BFSParallelFrontier {
  explicit BFSParallelFrontier(const Graph &graph, const int numThreads)
      : graph(graph),
        seen(graph.numVertices()),
        q(graph.numVertices()),
        local_cache(numThreads),
        read(0),
        write(0),
        numThreads(numThreads) {}

  void reset(const std::size_t numVertices) {
    q.resize(numVertices);
    read.store(0);
    write.store(0);
    seen.reset();
    seen.resize(numVertices);
    local_cache.assign(local_cache.size(), {});
  }

  void resetSeen() { seen.reset(); }

  template <typename FUNC>
  void doForAllVerticesInQ(FUNC &&func) {
    const auto n = read.load(std::memory_order_acquire);
    for (std::size_t i = 0; i < n; ++i) {
      func(q[i]);
    }
  }

  // Runs BFS from the given root vertex.
  // The template parameter RESET_MARKER defaults to true.
  template <bool RESET_MARKER = true,
            typename ON_POP = decltype([](const Vertex) { return false; }),
            typename ON_RELAX = decltype([](const Vertex, const Vertex) {
              return false;
            })>
  void run(const Vertex root, ON_POP &&onPop, ON_RELAX &&onRelax) {
    read.store(0);
    write.store(0);
    if constexpr (RESET_MARKER) {
      seen.reset();
    }
    if (seen.isMarked(root)) [[unlikely]]
      return;

    std::size_t pos = write.fetch_add(1, std::memory_order_release);
    assert(pos < q.size());
    q[pos] = root;
    seen.mark(root);

    for (auto &cache : local_cache) {
      cache.clear();
    }

    while (read.load(std::memory_order_acquire) <
           write.load(std::memory_order_acquire)) {
      processLevel(std::forward<ON_POP>(onPop),
                   std::forward<ON_RELAX>(onRelax));
    }
    assert(read.load(std::memory_order_acquire) ==
           write.load(std::memory_order_acquire));
  }

  // Processes one level of the BFS in parallel.
  template <typename ON_POP, typename ON_RELAX>
  void processLevel(ON_POP &&onPop, ON_RELAX &&onRelax) {
    const std::size_t left = read.load(std::memory_order_acquire);
    const std::size_t right = write.load(std::memory_order_acquire);

    assert(left <= right);
    assert(right <= q.size());

    parallelFor(
        left, right,
        [&](const std::size_t t, const std::size_t i) {
          const Vertex u = q[i];
          if (onPop(u)) return;

          for (std::size_t k = graph.beginEdge(u), endEdge = graph.endEdge(u);
               k < endEdge; ++k) {
            if (k + 4 < endEdge) {
              PREFETCH(&graph.toVertex[k + 4]);
            }
            const Vertex w = graph.toVertex[k];
            if (!seen.firstOccur(w)) continue;

            if (onRelax(u, w)) continue;
            local_cache[t].push_back(w);
          }
        },
        numThreads);

    for (auto &cache : local_cache) {
      if (!cache.empty()) {
        const std::size_t pos =
            write.fetch_add(cache.size(), std::memory_order_release);
        assert(pos + cache.size() <= q.size());
        std::copy(cache.begin(), cache.end(), q.begin() + pos);
        cache.clear();
      }
    }
    read.store(right, std::memory_order_release);
  }

  const Graph &graph;
  GenerationCheckerThreadSafe<> seen;
  std::vector<Vertex> q;
  std::vector<std::vector<Vertex>> local_cache;
  std::atomic_size_t read;
  std::atomic_size_t write;
  int numThreads;
};

}  // namespace bfs
