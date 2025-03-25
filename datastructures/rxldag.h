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

#include <omp.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

#include "drawer.h"
#include "forest.h"
#include "graph.h"
#include "hub_labels.h"
#include "ips4o.hpp"
#include "pruned_landmark_labeling.h"
#include "status_log.h"
#include "topological_sort.h"
#include "utils.h"

template <int T = 32, int WIDTH = 256, class LABEL = Label>
struct RXLDAG {
 public:
  std::array<std::vector<LABEL>, 2> labels;

  std::vector<std::atomic<bool>> alreadyProcessed;
  std::array<const Graph *, 2> graph;
  std::vector<std::size_t> rank;

  std::vector<Edge> edges;

  std::array<std::vector<std::bitset<WIDTH>>, 2> reachability;

  std::vector<PLL<WIDTH, LABEL>> plls;

  Drawer drawer;
  Forest<> forest;

  std::vector<std::array<std::uint32_t, 16>> valuesPerElement;
  const int numThreads;

  RXLDAG(const Graph &fwdGraph, const Graph &bwdGraph,
         const std::vector<std::size_t> &rankPar, const int numThreads = 1)
      : labels{std::vector<LABEL>(), std::vector<LABEL>()},
        alreadyProcessed(),
        graph{&fwdGraph, &bwdGraph},
        rank(rankPar),
        edges(),
        reachability{std::vector<std::bitset<WIDTH>>(),
                     std::vector<std::bitset<WIDTH>>()},
        plls(numThreads,
             PLL<WIDTH, LABEL>(labels, reachability, alreadyProcessed, graph)),
        drawer(fwdGraph.numVertices()),
        forest(fwdGraph.numVertices(),
               std::make_shared<const std::vector<std::size_t>>(rank), T),
        valuesPerElement(fwdGraph.numVertices()),
        numThreads(numThreads) {
    init(fwdGraph.numVertices());
  };

  template <bool PARALL_TAIL = true>
  void run(const std::string &orderingFileName) {
    StatusLog log("Computing HLs");
    assert(graph[FWD]->numVertices() == graph[BWD]->numVertices());
    assert(graph[FWD]->numEdges() == graph[BWD]->numEdges());

    const std::size_t numVertices = graph[FWD]->numVertices();

    std::vector<Vertex> ordering = getOrdering(orderingFileName);

    assert(ordering.size() == numVertices);
    assert(isOrdering(ordering, numVertices));

    std::size_t i = 0;

    if (numThreads > 1) {
      for (; i + WIDTH < (numVertices >> 7); i += WIDTH) {
        runSweep(i, ordering);

#pragma omp parallel for schedule(dynamic, 1) num_threads(numThreads)
        for (std::size_t step = 0; step < WIDTH; ++step) {
          int threadId = omp_get_thread_num();
          plls[threadId].template runPrunedBFS<true>(i, step, ordering);
        }
      }
    }

    if constexpr (PARALL_TAIL) {
#pragma omp parallel for schedule(dynamic, 8) num_threads(numThreads)
      for (std::size_t j = i; j < numVertices; ++j) {
        int threadId = omp_get_thread_num();
        plls[threadId].template runPrunedBFS<false>(j, 0, ordering);
      }
    } else {
      for (std::size_t j = i; j < numVertices; ++j) {
        plls[0].runPrunedBFS(ordering[j]);
      }
    }
  }

  void updateTrees() {
    assert(drawer.hasNext());

    std::array<Vertex, T> pickedVertices;

    std::size_t limit = std::min(static_cast<std::size_t>(T), drawer.size());

    for (std::size_t i = 0; i < limit; ++i) {
      pickedVertices[i] = drawer.draw();
    }

    for (std::size_t i = 0; i < T; ++i) {
      auto &tree = forest[i];
      tree.clear();

      if (i < limit) {
        plls[0].growTree(pickedVertices[i], tree);
        // add vertex back to drawer
        drawer.add(pickedVertices[i]);
      }
    }

    forest.computeSubtreeSizes();

    updateDescendantCounter();
  }

  // Sampling stuff
  void sample() {
    // - pick T vertices
    // - grow the trees in parallel
    // - compute descendants count
    // - pick top K vertices one after the other
    assert(drawer.hasNext());
    assert(T <= drawer.size());

    std::array<Vertex, T> pickedVertices;

#pragma GCC unroll(4)
    for (std::size_t i = 0; i < T; ++i) {
      pickedVertices[i] = drawer.draw();
    }

#pragma omp parallel for num_threads(numThreads)
    for (std::size_t i = 0; i < T; ++i) {
      int threadId = omp_get_thread_num();

      auto &tree = forest[i];
      plls[threadId].growTree(pickedVertices[i], tree);

      drawer.add(pickedVertices[i]);
    }
    forest.computeSubtreeSizes(numThreads);
  }

  void updateDescendantCounter() {
#pragma omp parallel for num_threads(numThreads)
    for (std::size_t v = 0; v < graph[FWD]->numVertices(); ++v) {
      std::fill(std::begin(valuesPerElement[v]), std::end(valuesPerElement[v]),
                0);
      for (std::size_t j = 0; j < forest.numberOfTrees(); ++j) {
        valuesPerElement[v][j % 16] += forest[j].descendants[v];
      }
    }
  }

  void growTree(const Vertex v, const std::size_t treeIndex,
                const std::size_t threadId) {
    auto &tree = forest[treeIndex];
    plls[threadId].growTree(v, tree);
  }

  Vertex pickBestVertex() {
    std::atomic<std::uint64_t> importance(0);
    std::atomic<Vertex> result(noVertex);

#pragma omp parallel for num_threads(numThreads)
    for (std::size_t v = 0; v < graph[FWD]->numVertices(); ++v) {
      if (alreadyProcessed[v].load()) continue;

      auto currentImportance = getImportanceByAverage(valuesPerElement[v]);

      std::uint64_t thisImportance =
          (static_cast<std::uint64_t>(currentImportance.first) << 32) |
          (currentImportance.second);

      if (fetch_max(importance, thisImportance)) {
        result.store(v);
      }
    }

    Vertex v = result.load();
    assert(v != noVertex);
    drawer.remove(v);

    std::cout << "Vertex " << result.load() << " [" << importance.load() << "]"
              << std::endl;
    return result.load();
  }

  void runSweep(const std::size_t left, const std::vector<Vertex> &ordering) {
    assert(left + WIDTH < ordering.size());
    resetReachability();

#pragma GCC unroll(4)
    for (int i = 0; i < WIDTH; ++i) {
      const auto vertex = ordering[left + i];

      reachability[FWD][vertex][i] = 1;
      reachability[BWD][vertex][i] = 1;
    }

    auto fwdSweep = [&]() -> void {
      for (std::size_t i = 0; i < edges.size(); ++i) {
        const auto &edge = edges[i];
        reachability[FWD][edge.to] |= reachability[FWD][edge.from];
      }
    };

    auto bwdSweep = [&]() -> void {
      for (std::size_t i = edges.size(); i-- > 0;) {
        const auto &edge = edges[i];
        reachability[BWD][edge.from] |= reachability[BWD][edge.to];
      }
    };

    /* fwdSweep(); */
    /* bwdSweep(); */

    std::thread fwdThread(fwdSweep);
    std::thread bwdThread(bwdSweep);

    fwdThread.join();
    bwdThread.join();
  }

  std::size_t computeTotalBytes() const {
    std::size_t totalBytes = 0;

    for (const auto &labelSet : labels) {
      for (const auto &label : labelSet) {
        totalBytes += sizeof(LABEL);
        totalBytes += label.nodes.capacity() * sizeof(Vertex);
      }
    }

    return totalBytes;
  }

  void showStats() const {
    auto computeStats = [](const std::vector<LABEL> &currentLabels) {
      std::size_t minSize = std::numeric_limits<std::size_t>::max();
      std::size_t maxSize = 0;
      std::size_t totalSize = 0;

      for (const auto &label : currentLabels) {
        std::size_t size = label.size();
        minSize = std::min(minSize, size);
        maxSize = std::max(maxSize, size);
        totalSize += size;
      }

      double avgSize = static_cast<double>(totalSize) / currentLabels.size();
      return std::make_tuple(minSize, maxSize, avgSize, totalSize);
    };

    auto [inMin, inMax, inAvg, inTotal] = computeStats(labels[BWD]);
    auto [outMin, outMax, outAvg, outTotal] = computeStats(labels[FWD]);

    /* std::locale::global(std::locale("")); */
    /* std::cout.imbue(std::locale()); */

    std::cout << "Forward Labels Statistics:" << std::endl;
    std::cout << "  Min Size:     " << outMin << std::endl;
    std::cout << "  Max Size:     " << outMax << std::endl;
    std::cout << "  Avg Size:     " << outAvg << std::endl;

    std::cout << "Backward Labels Statistics:" << std::endl;
    std::cout << "  Min Size:     " << inMin << std::endl;
    std::cout << "  Max Size:     " << inMax << std::endl;
    std::cout << "  Avg Size:     " << inAvg << std::endl;

    std::cout << "FWD # count:    " << outTotal << std::endl;
    std::cout << "BWD # count:    " << inTotal << std::endl;
    std::cout << "Both # count:   " << (outTotal + inTotal) << std::endl;

    std::cout << "Total memory consumption [megabytes]:" << std::endl;
    std::cout << "  "
              << static_cast<double>(computeTotalBytes() / (1024.0 * 1024.0))
              << std::endl;
  }

  void print() const {
    for (std::size_t v = 0; v < graph[FWD]->numVertices(); ++v) {
      std::cout << " -> " << v << "\n\t";
      for (auto h : labels[FWD][v].nodes) std::cout << h << " ";
      std::cout << "\n <- " << v << "\n\t";
      for (auto h : labels[BWD][v].nodes) std::cout << h << " ";
      std::cout << std::endl;
    }
  }

  std::vector<Vertex> getOrdering(const std::string &fileName) {
    std::vector<Vertex> ordering;
    ordering.reserve(graph[FWD]->numVertices());

    if (fileName == "") {
      parallel_assign(ordering, graph[FWD]->numVertices(), Vertex(0));

      std::vector<std::size_t> randomNumber;
      parallel_assign_iota(randomNumber, graph[FWD]->numVertices(),
                           static_cast<std::size_t>(0));

      std::mt19937 g(42);

      std::shuffle(randomNumber.begin(), randomNumber.end(), g);

      auto degreeCompRandom = [&](const auto left, const auto right) {
        return std::forward_as_tuple(
                   graph[FWD]->degree(left) + graph[BWD]->degree(left),
                   randomNumber[left]) >
               std::forward_as_tuple(
                   graph[FWD]->degree(right) + graph[BWD]->degree(right),
                   randomNumber[right]);
      };

      parallel_iota(ordering, Vertex(0));
      ips4o::parallel::sort(ordering.begin(), ordering.end(), degreeCompRandom);
      /* std::sort(ordering.begin(), ordering.end(), degreeCompRandom); */
      assert(
          std::is_sorted(ordering.begin(), ordering.end(), degreeCompRandom));
    } else {
      std::ifstream file(fileName);
      if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + fileName);
      }

      std::string line;
      while (std::getline(file, line)) {
        std::istringstream iss(line);
        Vertex vertex;
        double centrality;

        if (iss >> vertex >> centrality) {
          ordering.push_back(vertex - 1);
        } else {
          throw std::runtime_error("Failed to parse line: " + line);
        }
      }
    }
    return ordering;
  }

  bool isOrdering(const std::vector<Vertex> &ordering,
                  const std::size_t numVertices) {
    std::set<Vertex> orderedSet(ordering.begin(), ordering.end());

    if (orderedSet.size() != numVertices) {
      std::cout << "The ordering does not contain all vertices!" << std::endl;
      std::cout << "Ordering has " << orderedSet.size() << ", but there are "
                << numVertices << " many vertices!" << std::endl;
      return false;
    }
    if (!orderedSet.contains(0)) {
      std::cout << "The ordering does not contain 0!" << std::endl;
      return false;
    }
    if (!orderedSet.contains(numVertices - 1)) {
      std::cout << "The ordering does not contain the last vertex!"
                << std::endl;
      return false;
    }

    return true;
  };

  void init(std::size_t numVertices) {
    StatusLog log("Init the datastructures");

    parallel_assign(labels[BWD], numVertices, LABEL());
    parallel_assign(labels[FWD], numVertices, LABEL());

    alreadyProcessed = std::vector<std::atomic<bool>>(numVertices);

    edges.reserve(graph[FWD]->numEdges());

    for (Vertex from = 0; from < numVertices; ++from) {
      for (std::size_t i = graph[FWD]->beginEdge(from);
           i < graph[FWD]->endEdge(from); ++i) {
        edges.emplace_back(from, graph[FWD]->toVertex[i]);
      }
    }

    /* std::sort(edges.begin(), edges.end(), */
    ips4o::parallel::sort(edges.begin(), edges.end(),
                          [&](const auto &left, const auto &right) {
                            return std::tie(rank[left.from], rank[left.to]) <
                                   std::tie(rank[right.from], rank[right.to]);
                          });
    assert(std::is_sorted(edges.begin(), edges.end(),
                          [&](const auto &left, const auto &right) {
                            return std::tie(rank[left.from], rank[left.to]) <
                                   std::tie(rank[right.from], rank[right.to]);
                          }));
  }

  void resetReachability() {
    parallel_assign(reachability[BWD], graph[FWD]->numVertices(),
                    std::bitset<WIDTH>());
    parallel_assign(reachability[FWD], graph[FWD]->numVertices(),
                    std::bitset<WIDTH>());
  }
};
