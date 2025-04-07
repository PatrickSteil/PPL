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

#include "bfs.h"
#include "chain.h"
#include "graph.h"
#include "path_labels.h"
#include "simd_tools.h"
#include "topological_sort.h"

class PPLSimd {
 public:
  std::array<Graph*, 2> graphs;
  std::array<std::vector<ThreadSafePathLabel>, 2> labels;

  TopologicalSort topoSorter;
  std::vector<std::size_t> rank;

  std::vector<std::array<bfs::BFS, 2>> bfs;

  std::vector<std::vector<Vertex>> paths;

  std::array<std::vector<simd_u16x8>, 2> pathReachability;
  std::vector<std::pair<Vertex, Vertex>> edges;

  const int numThreads;

  PPLSimd(Graph* fwdGraph, Graph* bwdGraph, const int numberOfThreads = 1)
      : graphs{fwdGraph, bwdGraph},
        labels{std::vector<ThreadSafePathLabel>(fwdGraph->numVertices()),
               std::vector<ThreadSafePathLabel>(fwdGraph->numVertices())},
        topoSorter(*fwdGraph),
        rank(graphs[FWD]->numVertices()),
        bfs(numberOfThreads,
            std::array<bfs::BFS, 2>{bfs::BFS(*fwdGraph), bfs::BFS(*bwdGraph)}),
        paths(),
        pathReachability{std::vector<simd_u16x8>(fwdGraph->numVertices()),
                         std::vector<simd_u16x8>(fwdGraph->numVertices())},
        edges(),
        numThreads(numberOfThreads) {
    parallelFor(
        0, rank.size(),
        [&](const std::size_t, const std::size_t r) {
          rank[topoSorter[r]] = r;
        },
        numThreads);
    buildEdgesSortedTopo();
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

  void clearReachability() {
    assert(pathReachability[FWD].size() == pathReachability[BWD].size());
    parallelFor(
        0, pathReachability[FWD].size(),
        [&](const std::size_t, const std::size_t v) {
          pathReachability[FWD][v].clear();
          pathReachability[BWD][v].clear(0);
        },
        numThreads);
  }

  void run() {
    StatusLog log("Computing Path Labels");

    auto processChain = [&](const std::size_t threadId, const DIRECTION dir,
                            const std::size_t p, auto range,
                            const bool PRUNE_WITH_SIMD = false) -> void {
      assert(threadId < bfs.size());
      bfs[threadId][dir].resetSeen();

      for (std::size_t i : range) {
        Vertex root = paths[p][i];

        bfs[threadId][dir].template run<false>(
            root, bfs::noOp, [&](const Vertex, const Vertex to) {
              Vertex fromVertex = (dir == FWD ? root : to);
              Vertex toVertex = (dir == FWD ? to : root);

              if (PRUNE_WITH_SIMD) {
                if (prune(pathReachability[FWD][fromVertex],
                          pathReachability[BWD][toVertex], threadId)) {
                  return true;
                }
              }

              return query(labels[FWD][fromVertex], labels[BWD][toVertex]);
            });

        bfs[threadId][dir].doForAllVerticesInQ([&](const Vertex v) {
          assert(v < labels[!dir].size());
          labels[!dir][v].emplace_back(p, i);
          labels[!dir][v].sort();
        });
      }
    };

    std::size_t p = 0;

    for (const std::size_t end = paths.size(); p + 8 < end; p += 8) {
      topoSweep(p);

      parallelFor(
          0, 8,
          [&](const std::size_t threadId, const std::size_t i) {
            const std::size_t start = 0;
            const std::size_t end = paths[i].size();

            processChain(
                threadId, FWD, i,
                std::views::iota(start, end) | std::ranges::views::reverse,
                true);
            processChain(threadId, BWD, i, std::views::iota(start, end), true);
          },
          numThreads);
    }

    // const std::size_t startOfPaths = p;
    // const std::size_t endOfPaths = paths.size();

    // parallelFor(startOfPaths, endOfPaths, [&](const std::size_t threadId,
    // const std::size_t i) {
    //     const std::size_t start = 0;
    //     const std::size_t end = paths[i].size();

    //     processChain(threadId, FWD, i, std::views::iota(start, end) |
    //     std::ranges::views::reverse); processChain(threadId, BWD, i,
    //     std::views::iota(start, end));
    //   }, numThreads);
    for (; p < paths.size(); ++p) {
      const std::size_t start = 0;
      const std::size_t end = paths[p].size();

      processChain(0, FWD, p,
                   std::views::iota(start, end) | std::ranges::views::reverse);
      processChain(0, BWD, p, std::views::iota(start, end));
    }
  }

  // Modified sortPaths() function that sorts chains using the selected ranking
  // method.
  void sortPaths(RankingMethod method = RankingMethod::SUM) {
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

    // Use a lambda for sorting the paths based on the selected ranking.
    switch (method) {
      case RankingMethod::LENGTH:
        std::sort(paths.begin(), paths.end(),
                  [&](const std::vector<Vertex>& pathA,
                      const std::vector<Vertex>& pathB) {
                    return pathA.size() > pathB.size();
                  });
        break;

      case RankingMethod::SUM:
        std::sort(paths.begin(), paths.end(),
                  [&](const std::vector<Vertex>& pathA,
                      const std::vector<Vertex>& pathB) {
                    return computeSum(pathA, value) > computeSum(pathB, value);
                  });
        break;

      case RankingMethod::AVERAGE:
        std::sort(paths.begin(), paths.end(),
                  [&](const std::vector<Vertex>& pathA,
                      const std::vector<Vertex>& pathB) {
                    return computeAverage(pathA, value) >
                           computeAverage(pathB, value);
                  });
        break;

      case RankingMethod::MIN:
        std::sort(paths.begin(), paths.end(),
                  [&](const std::vector<Vertex>& pathA,
                      const std::vector<Vertex>& pathB) {
                    return computeMin(pathA, value) > computeMin(pathB, value);
                  });
        break;

      case RankingMethod::MAX:
        std::sort(paths.begin(), paths.end(),
                  [&](const std::vector<Vertex>& pathA,
                      const std::vector<Vertex>& pathB) {
                    return computeMax(pathA, value) > computeMax(pathB, value);
                  });
        break;

      case RankingMethod::HYBRID:
        std::sort(paths.begin(), paths.end(),
                  [&](const std::vector<Vertex>& pathA,
                      const std::vector<Vertex>& pathB) {
                    return computeHybrid(pathA, value) >
                           computeHybrid(pathB, value);
                  });
        break;

      default:
        // Fallback to SUM ranking if method is unknown.
        std::sort(paths.begin(), paths.end(),
                  [&](const std::vector<Vertex>& pathA,
                      const std::vector<Vertex>& pathB) {
                    return computeSum(pathA, value) > computeSum(pathB, value);
                  });
        break;
    }
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
    std::vector<std::size_t> pathLengths(totalPaths);

    parallelFor(
        0, totalPaths,
        [&](const std::size_t, const std::size_t i) {
          const auto& path = paths[i];
          std::size_t length = path.size();
          pathLengths[i] = length;

          fetch_min(minLength, length);
          fetch_max(maxLength, length);
          totalLength.fetch_add(length, std::memory_order_relaxed);
        },
        numThreads);

    double avgLength = static_cast<double>(totalLength.load()) / totalPaths;

    std::cout << "Path Statistics:\n";
    std::cout << "  Total Paths:    " << totalPaths << "\n";
    std::cout << "  Min Length:     " << minLength.load() << "\n";
    std::cout << "  Max Length:     " << maxLength.load() << "\n";
    std::cout << "  Average Length: " << avgLength << "\n";

    // std::sort(pathLengths.begin(), pathLengths.end());
    // std::cout << "  Percentiles:\n";
    // for (int p = 10; p <= 90; p += 10) {
    //     std::size_t index = (p * totalPaths) / 100;
    //     if (index >= totalPaths) index = totalPaths - 1;
    //     std::cout << "    " << p << "%: " << pathLengths[index] << "\n";
    // }
  }

  void buildEdgesSortedTopo() {
    edges.clear();
    edges.reserve(graphs[FWD]->numEdges());

    graphs[FWD]->doForAllEdges([&](const Vertex from, const Vertex to) {
      edges.emplace_back(from, to);
    });

    std::sort(edges.begin(), edges.end(),
              [&](const auto& left, const auto& right) {
                return std::tie(rank[left.first], rank[left.second]) <
                       std::tie(rank[right.first], rank[right.second]);
              });
  }

  void topoSweep(std::size_t nextPathId) {
    clearReachability();

    parallelFor(0, 8, [&](const std::size_t, const std::size_t i = 0) {
      assert(nextPathId + i < paths.size());
      const auto& path = paths[nextPathId + i];

      for (std::size_t pos = 0; pos < path.size(); ++pos) {
        Vertex v = path[pos];
        pathReachability[FWD][v][i] = static_cast<std::uint16_t>(pos + 1);
        pathReachability[BWD][v][i] = static_cast<std::uint16_t>(pos + 1);
      }
    });

    assert(std::is_sorted(
        edges.begin(), edges.end(), [&](const auto& left, const auto& right) {
          return std::tie(rank[left.first], rank[left.second]) <
                 std::tie(rank[right.first], rank[right.second]);
        }));

    std::thread reverseThread([&]() {
      for (auto [from, to] : edges) {
        pathReachability[BWD][to].setMax(pathReachability[BWD][from]);
      }
    });

    std::thread forwardThread([&]() {
      for (auto [from, to] : edges | std::views::reverse) {
        pathReachability[FWD][from].setMin(pathReachability[FWD][to]);
      }
    });

    reverseThread.join();
    forwardThread.join();
  }
};