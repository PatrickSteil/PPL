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
#include "timetable.h"

class PPL {
 public:
  TimeTable& tt;
  std::array<std::vector<PathLabel>, 2> labels;

  bfs::FixedSizedQueue<Vertex> queue;
  bfs::GenerationChecker<> seen;

  std::vector<std::vector<Vertex>> paths;

  PPL(TimeTable& tt)
      : tt(tt),
        labels{std::vector<PathLabel>(tt.numberOfEvents()),
               std::vector<PathLabel>(tt.numberOfEvents())},
        queue(tt.numberOfEvents()),
        seen(tt.numberOfEvents()),
        paths(tt.eventsOfStop) {
    sortPaths();
  };

  void showStats() { showLabelStats(labels); }

  void clear() {
    StatusLog log("Clear Datastructures");

    assert(labels[FWD].size() == labels[BWD].size());
    assert(labels[FWD].size() == tt.numberOfEvents());

    for (std::size_t v = 0; v < labels[FWD].size(); ++v) {
      labels[FWD][v].clear();
      labels[BWD][v].clear();
    }
  }

  void run() {
    StatusLog log("Computing Path Labels");

    auto processVertexFromPath = [&](const DIRECTION dir, const Vertex root,
                                     const std::uint32_t path_id,
                                     const std::uint32_t position) -> void {
      assert(!seen.isMarked(root));
      std::size_t oldRead = queue.read;
      seen.mark(root);
      queue.push(root);

      while (!queue.isEmpty()) {
        const Vertex u = queue.pop();

        tt.relaxNextEventInTrip(dir, u, [&](const Vertex, const Vertex w) {
          if (seen.isMarked(w)) return;
          seen.mark(w);

          if (dir == FWD && query(labels[FWD][root], labels[BWD][w])) return;
          if (dir == BWD && query(labels[FWD][w], labels[BWD][root])) return;

          if (dir == FWD) queue.push(tt.toDepartureEventId(w));
          if (dir == BWD) queue.push(tt.toArrivalEventId(w));
        });

        tt.relaxTransferEdges(dir, u, [&](const Vertex, const Vertex w) {
          if (seen.isMarked(w)) return;
          seen.mark(w);

          if (dir == FWD && query(labels[FWD][root], labels[BWD][w])) return;
          if (dir == BWD && query(labels[FWD][w], labels[BWD][root])) return;

          if (dir == FWD) queue.push(tt.toDepartureEventId(w));
          if (dir == BWD) queue.push(tt.toArrivalEventId(w));
        });
      }

      std::size_t newRead = queue.read;
      assert(oldRead <= newRead);
      assert(oldRead < queue.data.size());
      assert(newRead <= queue.data.size());

      for (; oldRead < newRead; ++oldRead) {
        if (oldRead + 4 < newRead) {
          PREFETCH(&labels[!dir][queue.data[oldRead + 4]]);
        }
        Vertex other = queue.data[oldRead];

        assert(dir != FWD || tt.isDepartureEventId(other));
        assert(dir != BWD || tt.isArrivalEventId(other));

        assert(other < labels[!dir].size());
        labels[!dir][other].emplace_back(path_id, position);
        labels[!dir][other].sort();
      }
    };

    auto processChain = [&](const DIRECTION dir, const std::size_t p,
                            auto range) -> void {
      seen.reset();
      queue.reset();

      for (std::size_t i : range) {
        Vertex root = paths[p][i];

        if (dir == FWD && tt.isArrivalEventId(root)) continue;
        if (dir == BWD && tt.isDepartureEventId(root)) continue;

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

  void sortPaths() {
    const std::size_t n = tt.numberOfEvents();

    std::vector<std::size_t> value(n, 0);
    for (std::size_t v = 0; v < n; ++v) {
      value[v] = (tt.transferGraphs[FWD].degree(v) + 1) *
                 (tt.transferGraphs[BWD].degree(v) + 1);
    }

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
};