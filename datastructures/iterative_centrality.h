/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include "graph.h"
#include "status_log.h"

#include <algorithm>
#include <cassert>
#include <numeric>
#include <omp.h>
#include <queue>
#include <random>
#include <stack>
#include <vector>

struct IterativeCentrality {
  const Graph &graph;

  std::vector<bool> removed;
  std::vector<double> value;
  std::vector<Vertex> order;
  std::vector<Vertex> activeVertices; // active vertex pool

  std::mt19937 rng{std::random_device{}()};

  explicit IterativeCentrality(Graph &graph)
      : graph(graph), removed(graph.numVertices(), false),
        value(graph.numVertices(), 0.0), order(),
        activeVertices(graph.numVertices()) {
    std::iota(activeVertices.begin(), activeVertices.end(), 0);
  }

  std::vector<Vertex> &getOrder() { return order; }

  void run(const int sampleSize, const int numThreads = 1,
           const double dampingFactor = 0.8) {
    StatusLog log("Computing Iterative Centrality");
    order.clear();
    order.reserve(graph.numVertices());
    std::fill(removed.begin(), removed.end(), false);

    const std::size_t n = graph.numVertices();

    while (!activeVertices.empty()) {
      std::fill(value.begin(), value.end(), 0.0);

      const std::size_t activeCount = activeVertices.size();
      std::vector<std::vector<double>> localValues(numThreads,
                                                   std::vector<double>(n, 0.0));

#pragma omp parallel
      {
        int tid = omp_get_thread_num();
        std::mt19937 rng_local(std::random_device{}() ^ (tid * 7919));
        std::uniform_int_distribution<std::size_t> dist(0, activeCount - 1);

        std::vector<std::size_t> distance(n);
        std::vector<std::size_t> sigma(n);
        std::vector<double> delta(n);
        std::vector<std::vector<Vertex>> predecessors(n);
        std::vector<Vertex> stack;
        std::queue<Vertex> q;

#pragma omp for schedule(dynamic)
        for (int sample = 0; sample < sampleSize; ++sample) {
          Vertex s = activeVertices[dist(rng_local)];
          bfsPhaseThread(s, q, distance, sigma, delta, predecessors, stack,
                         localValues[tid]);
        }
      }

#pragma omp parallel for schedule(static)
      for (std::size_t v = 0; v < n; ++v) {
        double sum = 0.0;
        for (int t = 0; t < numThreads; ++t)
          sum += localValues[t][v];
        value[v] = sum;
      }

      double maxValue = -1.0;
      std::vector<Vertex> candidates;
      for (Vertex v : activeVertices) {
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

      order.push_back(chosen);
      removed[chosen] = true;

      graph.relaxAllEdges(chosen, [&](const Vertex /* from */, const Vertex to) {
        if (!removed[to]) {
          value[to] *= dampingFactor;
        }
      });

      auto it = std::find(activeVertices.begin(), activeVertices.end(), chosen);
      assert(it != activeVertices.end());
      std::swap(*it, activeVertices.back());
      activeVertices.pop_back();
    }
  }

private:
  void bfsPhaseThread(Vertex s, std::queue<Vertex> &q,
                      std::vector<std::size_t> &distance,
                      std::vector<std::size_t> &sigma,
                      std::vector<double> &delta,
                      std::vector<std::vector<Vertex>> &predecessors,
                      std::vector<Vertex> &stack,
                      std::vector<double> &value) const {
    while (!q.empty())
      q.pop();
    stack.clear();

    std::fill(distance.begin(), distance.end(), static_cast<std::size_t>(-1));
    std::fill(sigma.begin(), sigma.end(), 0);
    std::fill(delta.begin(), delta.end(), 0.0);
    for (auto &p : predecessors)
      p.clear();

    distance[s] = 0;
    sigma[s] = 1;
    q.push(s);

    // --- BFS phase ---
    while (!q.empty()) {
      Vertex v = q.front();
      q.pop();
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
