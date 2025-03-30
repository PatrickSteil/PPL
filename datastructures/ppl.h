/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <numeric>
#include <ranges>
#include <stack>
#include <thread>
#include <vector>

#include "bfs_tools.h"
#include "chain.h"
#include "graph.h"
#include "path_labels.h"
#include "topological_sort.h"

template <typename PATHLABEL_TYPE = PathLabel>
class PPL {
 public:
  std::array<Graph*, 2> graphs;
  std::array<std::vector<PATHLABEL_TYPE>, 2> labels;

  TopologicalSort topoSorter;
  std::vector<std::size_t> rank;

  std::array<bfs::FixedSizedQueue<Vertex>, 2> queue;
  std::array<bfs::GenerationChecker<>, 2> seen;

  std::vector<std::vector<Vertex>> paths;

  const int numThreads;

  PPL(Graph* fwdGraph, Graph* bwdGraph, const int numberOfThreads = 1)
      : graphs{fwdGraph, bwdGraph},
        labels{std::vector<PATHLABEL_TYPE>(fwdGraph->numVertices()),
               std::vector<PATHLABEL_TYPE>(fwdGraph->numVertices())},
        topoSorter(*fwdGraph),
        rank(graphs[FWD]->numVertices()),
        queue{bfs::FixedSizedQueue<Vertex>(fwdGraph->numVertices()),
              bfs::FixedSizedQueue<Vertex>(fwdGraph->numVertices())},
        seen{bfs::GenerationChecker<>(fwdGraph->numVertices()),
             bfs::GenerationChecker<>(fwdGraph->numVertices())},
        paths(),
        numThreads(numberOfThreads) {
    parallelFor(
        0, rank.size(),
        [&](const std::size_t, const std::size_t r) {
          rank[topoSorter[r]] = r;
        },
        numThreads);
  };

  void showStats() { showLabelStats(labels); }

  void clear() {
    StatusLog log("Clear Datastructures");

    assert(labels[FWD].size() == labels[BWD].size());
    assert(labels[FWD].size() == graphs[FWD]->numVertices());
    assert(labels[FWD].size() == graphs[BWD]->numVertices());

    parallelFor(
        0, labels[FWD].size(),
        [&](const std::size_t, const std::size_t v) {
          labels[FWD][v].clear();
          labels[BWD][v].clear();
        },
        numThreads);
  }

  void run() {
    StatusLog log("Computing Path Labels");

    auto processVertexFromPath = [&](const DIRECTION dir, const Vertex root,
                                     const std::uint32_t path_id,
                                     const std::uint32_t position) -> void {
      assert(!seen[dir].isMarked(root));
      std::size_t oldRead = queue[dir].read;
      seen[dir].mark(root);
      queue[dir].push(root);

      while (!queue[dir].isEmpty()) {
        const Vertex u = queue[dir].pop();

        for (std::size_t i = graphs[dir]->beginEdge(u),
                         end = graphs[dir]->endEdge(u);
             i < end; ++i) {
          if (i + 4 < end) {
            PREFETCH(&graphs[dir]->toVertex[i + 4]);
          }
          const Vertex w = graphs[dir]->toVertex[i];
          if (seen[dir].isMarked(w)) continue;
          seen[dir].mark(w);

          if (dir == FWD && query(labels[FWD][root], labels[BWD][w])) continue;
          if (dir == BWD && query(labels[FWD][w], labels[BWD][root])) continue;
          queue[dir].push(w);
        }
      }

      std::size_t newRead = queue[dir].read;
      assert(oldRead <= newRead);
      assert(oldRead < queue[dir].data.size());
      assert(newRead <= queue[dir].data.size());

      for (; oldRead < newRead; ++oldRead) {
        if (oldRead + 4 < newRead) {
          PREFETCH(&labels[!dir][queue[dir].data[oldRead + 4]]);
        }
        Vertex other = queue[dir].data[oldRead];
        assert(other < labels[!dir].size());
        labels[!dir][other].emplace_back(path_id, position);
        labels[!dir][other].sort();
      }
    };

    auto processChain = [&](const DIRECTION dir, const std::size_t p,
                            auto range) -> void {
      seen[dir].reset();
      queue[dir].reset();

      for (std::size_t i : range) {
        Vertex root = paths[p][i];
        processVertexFromPath(dir, root, p, i);
      }
    };

    for (std::uint32_t p = 0; p < paths.size(); ++p) {
      const std::size_t start = 0;
      const std::size_t end = paths[p].size();

      processChain(FWD, p,
                   std::views::iota(start, end) | std::ranges::views::reverse);
      processChain(BWD, p, std::views::iota(start, end));
    }
  }

  void chainDecomposition() {
    StatusLog log("Computing Chain Decomposition");
    const std::size_t n = graphs[FWD]->numVertices();

    std::vector<Chain*> chainOfVertex(n, nullptr);
    Chains chains;
    std::vector<bool> visited(n, false);

    auto reversedDFSlookup = [&](Vertex v) -> Vertex {
      std::vector<Vertex> stack;
      stack.push_back(v);

      while (!stack.empty()) {
        Vertex cur = stack.back();
        stack.pop_back();

        for (std::size_t i = graphs[BWD]->beginEdge(cur);
             i < graphs[BWD]->endEdge(cur); ++i) {
          Vertex w = graphs[BWD]->toVertex[i];

          if (!visited[w]) {
            auto chainPtr = chainOfVertex[w];

            if (chainPtr != nullptr && chainPtr->getLastVertex() == w) {
              for (Vertex other : stack) {
                visited[other] = false;
              }
              return w;
            }

            stack.push_back(w);
            visited[w] = true;
          }
        }
      }

      return noVertex;
    };

    for (Vertex v : topoSorter.getOrdering()) {
      bool belongToChain = (chainOfVertex[v] != nullptr);
      Vertex toAdd = noVertex;
      std::size_t min_outdegree = n;

      if (!belongToChain) {
        graphs[BWD]->relaxAllEdges(
            v, [&](const Vertex, const Vertex adjSource) {
              int adjRank = rank[adjSource];
              Chain* C = chainOfVertex[adjRank];
              if (C != nullptr && !C->empty() &&
                  C->getVertices().back() == adjSource) {
                std::size_t adjOutdegree = graphs[FWD]->degree(adjSource);
                if (adjOutdegree < min_outdegree) {
                  min_outdegree = adjOutdegree;
                  toAdd = adjSource;
                }
              }
            });

        if (toAdd == noVertex) {
          toAdd = reversedDFSlookup(v);
        }
      }

      if (toAdd != noVertex) {
        Chain* C = chainOfVertex[toAdd];
        C->push(v);
        chainOfVertex[v] = C;
      } else if (!belongToChain) {
        chains.push_back(Chain());
        Chain* C = &chains.back();
        C->push(v);
        chainOfVertex[v] = C;
      }

      for (std::size_t i = graphs[FWD]->beginEdge(v);
           i < graphs[FWD]->endEdge(v); ++i) {
        Vertex w = graphs[FWD]->toVertex[i];

        if (graphs[BWD]->degree(w) == 1) {
          Chain* C = chainOfVertex[v];
          C->push(w);
          chainOfVertex[w] = C;
          break;
        }
      };
    }

    paths.clear();

    paths.reserve(chains.size());

    for (const auto& chain : chains) {
      paths.emplace_back();
      paths.back().reserve(chain.size());

      for (Vertex v : chain.getVertices()) {
        paths.back().push_back(v);
      }
    }
  }

  void sortPaths() {
    assert(verifyPathDecomposition(paths));

    const std::size_t n = graphs[FWD]->numVertices();

    std::vector<std::size_t> value(n);
    parallelFor(
        0, n,
        [&](const std::size_t, const std::size_t v) {
          value[v] =
              (graphs[FWD]->degree(v) + 1) * (graphs[BWD]->degree(v) + 1);
        },
        numThreads);

    std::sort(paths.begin(), paths.end(),
              [&](const std::vector<Vertex>& pathA,
                  const std::vector<Vertex>& pathB) {
                const std::size_t sumA = std::accumulate(
                    pathA.begin(), pathA.end(), 0ULL,
                    [&](std::size_t acc, Vertex v) { return acc + value[v]; });
                const std::size_t sumB = std::accumulate(
                    pathB.begin(), pathB.end(), 0ULL,
                    [&](std::size_t acc, Vertex v) { return acc + value[v]; });
                return sumA > sumB;
              });
  }

  bool verifyPathDecomposition(
      const std::vector<std::vector<Vertex>>& toVerify) {
    std::vector<bool> hit(graphs[FWD]->numVertices(), false);

    for (auto& p : toVerify) {
      for (auto v : p) {
        if (hit[v]) {
          std::cout << "Wrong: Vertex " << v << " has been in another path!"
                    << std::endl;
          return false;
        }

        hit[v] = true;
      }
    }

    return std::all_of(hit.begin(), hit.end(), [](bool v) { return v; });
  }

  void showPathStats() {
    if (paths.empty()) {
      std::cout << "No paths available.\n";
      return;
    }

    std::size_t totalPaths = paths.size();
    std::atomic<std::size_t> minLength{std::numeric_limits<std::size_t>::max()};
    std::atomic<std::size_t> maxLength{0};
    std::atomic<std::size_t> totalLength{0};

    parallelFor(
        0, paths.size(),
        [&](const std::size_t, const std::size_t i) {
          const auto& path = paths[i];
          std::size_t length = path.size();

          std::size_t currentMin = minLength.load();
          while (length < currentMin &&
                 !minLength.compare_exchange_weak(currentMin, length)) {
          }

          std::size_t currentMax = maxLength.load();
          while (length > currentMax &&
                 !maxLength.compare_exchange_weak(currentMax, length)) {
          }

          totalLength.fetch_add(length, std::memory_order_relaxed);
        },
        numThreads);

    double avgLength = static_cast<double>(totalLength.load()) / totalPaths;

    std::cout << "Path Statistics:\n";
    std::cout << "  Total Paths:    " << totalPaths << "\n";
    std::cout << "  Min Length:     " << minLength.load() << "\n";
    std::cout << "  Max Length:     " << maxLength.load() << "\n";
    std::cout << "  Average Length: " << avgLength << "\n";
  }

  void computePaths() {
    StatusLog log("Computing DPInOut");

    const std::size_t n = graphs[FWD]->numVertices();

    std::vector<std::size_t> value(n, 0);
    for (std::size_t v = 0; v < n; ++v) {
      value[v] = (graphs[FWD]->degree(v) + 1) * (graphs[BWD]->degree(v) + 1);
    }

    graphs[FWD]->sortByRank(rank);
    graphs[BWD]->sortByRank(rank);

    std::vector<bool> already_chosen(n, false);
    std::vector<std::size_t> sum(n, 0);
    std::vector<Vertex> parent(n, noVertex);

    paths.clear();

    bool foundAnyPath = true;

    while (foundAnyPath) {
      foundAnyPath = false;

      std::fill(parent.begin(), parent.end(), noVertex);
      std::fill(sum.begin(), sum.end(), 0);

      Vertex best_v = noVertex;
      std::size_t best_val = 0;

      for (Vertex u : topoSorter.getOrdering()) {
        if (already_chosen[u]) continue;

        graphs[FWD]->relaxAllEdges(u, [&](const Vertex, const Vertex v) {
          if (already_chosen[v]) return;

          const std::size_t new_sum = sum[u] + value[v];
          bool update = (new_sum > sum[v]);
          sum[v] = update ? new_sum : sum[v];
          parent[v] = update ? u : parent[v];

          bool new_best = (sum[v] > best_val);
          best_val = new_best ? sum[v] : best_val;
          best_v = new_best ? v : best_v;
        });
      }

      if (best_v != noVertex && best_val > 0) {
        foundAnyPath = true;
        std::vector<Vertex> path;
        path.reserve(sum.size());
        for (Vertex cur = best_v; cur != noVertex; cur = parent[cur]) {
          path.push_back(cur);
        }
        // std::reverse(path.begin(), path.end());
        for (auto node : path) {
          already_chosen[node] = true;
        }
        paths.push_back(path);
      }
    }

    // std::cout << "\n[Info] Nr. Paths: " << paths.size() << std::endl;

    // // Output all the paths found.
    // for (auto& path : paths) {
    //   for (auto v : path) std::cout << v << " ";
    //   std::cout << std::endl;
    // }
  }
};