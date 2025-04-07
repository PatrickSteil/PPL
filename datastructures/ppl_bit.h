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
#include <variant>
#include <vector>

#include "bfs.h"
#include "chain.h"
#include "graph.h"
#include "path_labels.h"
#include "topological_sort.h"

class PPLBit {
 public:
  std::array<Graph*, 2> graphs;
  std::array<std::vector<PathLabel>, 2> labels;

  TopologicalSort topoSorter;
  std::vector<std::size_t> rank;

  using BFSVariant = std::variant<bfs::TBFS<FWD>, bfs::TBFS<BWD>>;
  std::array<BFSVariant, 2> bfs;
  std::array<std::vector<std::uint16_t>, 2> bestPosToPath;

  std::vector<std::vector<Vertex>> paths;

  const int numThreads;

  PPLBit(Graph* fwdGraph, Graph* bwdGraph, const int numberOfThreads = 1)
      : graphs{fwdGraph, bwdGraph},
        labels{std::vector<PathLabel>(fwdGraph->numVertices()),
               std::vector<PathLabel>(fwdGraph->numVertices())},
        topoSorter(*fwdGraph),
        rank(graphs[FWD]->numVertices()),
        bfs{bfs::TBFS<FWD>(*fwdGraph), bfs::TBFS<BWD>(*bwdGraph)},
        bestPosToPath{
            std::vector<std::uint16_t>(fwdGraph->numVertices(), 0),
            std::vector<std::uint16_t>(bwdGraph->numVertices(), noPathPos)},
        paths(),
        numThreads(numberOfThreads) {
    parallelFor(
        0, rank.size(),
        [&](const std::size_t, const std::size_t r) {
          rank[topoSorter[r]] = r;
        },
        numThreads);
  };

  void reorderByRank() {
    graphs[FWD]->reorderByRank(rank);
    graphs[BWD]->reorderByRank(rank);

    parallelFor(
        0, paths.size(),
        [&](const std::size_t, const std::size_t p) {
          auto& path = paths[p];
          for (auto& v : path) {
            v = rank[v];
          }

          std::sort(path.begin(), path.end());
        },
        numThreads);
  }

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

    auto processChain = [&](const DIRECTION dir, const std::size_t p) -> void {
      const std::vector<Vertex>& path = paths[p];

      std::visit(
          [&](auto& thisBFS) {
            thisBFS.resetSeen();
            thisBFS.addRoots(path);

            std::vector<Vertex> currentSeen;
            currentSeen.reserve(graphs[FWD]->numVertices());

            for (std::size_t i = 0; i < path.size(); ++i) {
              bestPosToPath[dir][path[i]] = i;
            }

            thisBFS.run(
                [&currentSeen](const Vertex v) {
                  currentSeen.emplace_back(v);
                  return false;
                },
                [&](const Vertex from, const Vertex to) {
                  if (dir == FWD) {
                    bestPosToPath[dir][to] = std::max(bestPosToPath[dir][to],
                                                      bestPosToPath[dir][from]);
                  } else {
                    bestPosToPath[dir][to] = std::min(bestPosToPath[dir][to],
                                                      bestPosToPath[dir][from]);
                  }

                  Vertex fromVertex =
                      (dir == FWD ? path[bestPosToPath[dir][from]] : to);
                  Vertex toVertex =
                      (dir == FWD ? to : path[bestPosToPath[dir][from]]);

                  return (
                      query(labels[FWD][fromVertex], labels[BWD][toVertex]));
                });

            for (const Vertex v : currentSeen) {
              assert(v < labels[!dir].size());
              assert(bestPosToPath[dir][v] < path.size());

              labels[!dir][v].emplace_back(p, bestPosToPath[dir][v]);
              labels[!dir][v].sort();
            }
          },
          bfs[dir]);
    };

    for (std::size_t p = 0; p < paths.size(); ++p) {
      std::fill(bestPosToPath[FWD].begin(), bestPosToPath[FWD].end(), 0);
      std::fill(bestPosToPath[BWD].begin(), bestPosToPath[BWD].end(),
                noPathPos);

      processChain(FWD, p);
      processChain(BWD, p);
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

    paths.resize(chains.size());

    parallelFor(0, chains.size(), [&](const std::size_t, const std::size_t i) {
      const auto& chain = chains[i];
      auto& p = paths[i];
      p.reserve(chain.size());

      for (Vertex v : chain.getVertices()) {
        p.push_back(v);
      }
    });
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

    std::sort(pathLengths.begin(), pathLengths.end());

    std::cout << "Path Statistics:\n";
    std::cout << "  Total Paths:    " << totalPaths << "\n";
    std::cout << "  Min Length:     " << minLength.load() << "\n";
    std::cout << "  Max Length:     " << maxLength.load() << "\n";
    std::cout << "  Average Length: " << avgLength << "\n";

    std::cout << "  Percentiles:\n";
    for (int p = 10; p <= 90; p += 10) {
      std::size_t index = (p * totalPaths) / 100;
      if (index >= totalPaths) index = totalPaths - 1;
      std::cout << "    " << p << "%: " << pathLengths[index] << "\n";
    }
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