/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include "bfs.h"
#include "bit_vector.h"
#include "graph.h"
#include "status_log.h"

#include <algorithm>
#include <cassert>
#include <numeric>
#include <random>
#include <stack>
#include <vector>

struct IterativeCentrality {
  const Graph &graph;

  bfs::FixedSizedQueue<Vertex> q;
  std::vector<std::size_t> distance;
  std::vector<std::size_t> sigma;
  std::vector<double> delta;
  std::vector<double> value;
  std::vector<std::vector<Vertex>> predecessors;
  std::vector<Vertex> stack;
  std::vector<Vertex> order;
  BitVector<uint64_t> removed;

  std::mt19937 rng{std::random_device{}()};

  explicit IterativeCentrality(Graph &graph)
      : graph(graph), q(graph.numVertices()), distance(graph.numVertices()),
        sigma(graph.numVertices()), delta(graph.numVertices()),
        value(graph.numVertices(), 0.0), predecessors(graph.numVertices()),
        stack(), order(), removed(graph.numVertices()) {}

  std::vector<Vertex> &getOrder() { return order; }

  void resetStructures() {
    std::fill(value.begin(), value.end(), 0.0);
    for (auto &pred : predecessors)
      pred.clear();
  }

  void run(const int sampleSize) {
    StatusLog log("Computing Iterative Centrality");
    order.clear();
    order.reserve(graph.numVertices());
    removed.clear();
    removed.resize(graph.numVertices());

    const std::size_t n = graph.numVertices();
    std::uniform_int_distribution<std::size_t> dist(0, n - 1);

    while (order.size() != n) {
      resetStructures();

      // --- 1) Approximate betweenness using sampleSize sources ---
      for (int sample = 0; sample < sampleSize; ++sample) {
        Vertex s = dist(rng);
        while (removed[s])
          s = dist(rng);

        bfsPhase(s);
      }

      // --- 2) Pick the vertex with the highest value ---
      double maxValue = -1.0;
      std::vector<Vertex> candidates;
      for (Vertex v = 0; v < n; ++v) {
        if (removed[v])
          continue;
        if (value[v] > maxValue) {
          maxValue = value[v];
          candidates.clear();
          candidates.push_back(v);
        } else if (value[v] == maxValue) {
          candidates.push_back(v);
        }
      }

      assert(!candidates.empty());
      Vertex chosen = candidates[std::uniform_int_distribution<std::size_t>(
          0, candidates.size() - 1)(rng)];

      // --- 3) Append vertex to order and mark removed ---
      order.push_back(chosen);
      removed[chosen] = true;
    }
  }

private:
  void bfsPhase(Vertex s) {
    q.reset();
    stack.clear();

    std::fill(distance.begin(), distance.end(), static_cast<std::size_t>(-1));
    std::fill(sigma.begin(), sigma.end(), 0);
    std::fill(delta.begin(), delta.end(), 0.0);
    for (auto &pred : predecessors)
      pred.clear();

    distance[s] = 0;
    sigma[s] = 1;
    q.push(s);

    // --- BFS phase ---
    while (!q.isEmpty()) {
      Vertex v = q.pop();
      stack.push_back(v);

      graph.relaxAllEdges(v, [&](const Vertex from, const Vertex to) {
        if (removed[to])
          return;
        if (distance[to] == static_cast<std::size_t>(-1)) {
          distance[to] = distance[from] + 1;
          q.push(to);
        }
        if (distance[to] == distance[from] + 1) {
          sigma[to] += sigma[from];
          predecessors[to].push_back(from);
        }
      });
    }

    // --- Accumulation phase ---
    while (!stack.empty()) {
      Vertex w = stack.back();
      stack.pop_back();
      for (Vertex v : predecessors[w]) {
        delta[v] +=
            (static_cast<double>(sigma[v]) / sigma[w]) * (1.0 + delta[w]);
      }
      if (w != s)
        value[w] += delta[w];
    }
  }
};
