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
};  // namespace bfs
