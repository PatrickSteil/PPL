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

#include <array>
#include <vector>

#include "flatbuffer.h"
#include "status_log.h"
#include "types.h"
#include "utils.h"

class CompressedLabels {
public:
  std::array<DeltaFlatBuffer, 2> pathIds;
  std::array<std::vector<std::vector<std::uint16_t>>, 2> pathPos;

  CompressedLabels(){};

  template <typename T>
  void loadLabels(const std::array<std::vector<T>, 2> &labels) {
    StatusLog log("Loading labels into DeltaFlatBuffer");
    std::vector<std::uint32_t> toLoad;
    toLoad.reserve(256);

    pathPos[FWD].resize(labels[FWD].size());

    for (std::size_t i = 0; i < labels[FWD].size(); ++i) {
      const auto &label = labels[FWD][i];
      pathPos[FWD][i].reserve(label.size());

      label.doForAllHubs([&](const auto &pathHUb) {
        toLoad.emplace_back(pathHUb.getPath());
        pathPos[FWD][i].emplace_back(pathHUb.getPathPos());
      });
      pathIds[FWD].addVector(toLoad);
      toLoad.clear();
    }

    pathPos[BWD].resize(labels[BWD].size());

    for (std::size_t i = 0; i < labels[BWD].size(); ++i) {
      const auto &label = labels[BWD][i];
      pathPos[BWD][i].reserve(label.size());

      label.doForAllHubs([&](const auto &pathHUb) {
        toLoad.emplace_back(pathHUb.getPath());
        pathPos[BWD][i].emplace_back(pathHUb.getPathPos());
      });

      pathIds[BWD].addVector(toLoad);
      toLoad.clear();
    }
    pathIds[BWD].addVector(toLoad);
  }

  void showStats() {
    auto bytesToMB = [](std::size_t bytes) {
      return bytes / (1024.0 * 1024.0);
    };

    // Memory for DeltaFlatBuffers
    std::size_t fwdPathIdsBytes = pathIds[FWD].memoryUsage();
    std::size_t bwdPathIdsBytes = pathIds[BWD].memoryUsage();

    // Memory for std::vector<uint16_t>
    std::size_t fwdPathPosBytes = 0;
    for (const auto &v : pathPos[FWD])
      fwdPathPosBytes += sizeof(uint16_t) * v.size();

    std::size_t bwdPathPosBytes = 0;
    for (const auto &v : pathPos[BWD])
      bwdPathPosBytes += sizeof(uint16_t) * v.size();

    double fwdMB = bytesToMB(fwdPathIdsBytes + fwdPathPosBytes);
    double bwdMB = bytesToMB(bwdPathIdsBytes + bwdPathPosBytes);
    double totalMB = fwdMB + bwdMB;

    std::cout << "FWD Memory: " << fwdMB << " MB" << std::endl;
    std::cout << "BWD Memory: " << bwdMB << " MB" << std::endl;
    std::cout << "Total Memory: " << totalMB << " MB" << std::endl;
  }
};

bool query(const CompressedLabels &compLabels, const Vertex source,
           const Vertex target) {
  if (source == target) [[unlikely]]
    return true;

  const auto fromIds = compLabels.pathIds[FWD][source];
  const auto toIds = compLabels.pathIds[BWD][target];

  assert(source < compLabels.pathPos[FWD].size());
  assert(target < compLabels.pathPos[BWD].size());

  const auto &fromPos = compLabels.pathPos[FWD][source];
  const auto &toPos = compLabels.pathPos[BWD][target];

  auto itF = fromIds.begin();
  auto itT = toIds.begin();
  std::size_t i = 0, j = 0;

  while (itF != fromIds.end() && itT != toIds.end()) {
    uint32_t fromPath = *itF;
    uint32_t toPath = *itT;

    if (fromPath < toPath) {
      ++itF;
      ++i;
    } else if (fromPath > toPath) {
      ++itT;
      ++j;
    } else {
      uint16_t fromPathPos = fromPos[i];
      uint16_t toPathPos = toPos[j];
      if (fromPathPos <= toPathPos) [[unlikely]] {
        return true;
      } else {
        ++itT;
        ++j;
      }
    }
  }
  return false;
}

void benchmark_compressedlabels(const CompressedLabels &compLabels,
                                const std::size_t numQueries = 100000) {
  using clock = std::chrono::high_resolution_clock;
  using ns = std::chrono::nanoseconds;
  assert(compLabels.pathPos[FWD].size() == compLabels.pathPos[BWD].size());

  auto queries = generateRandomQueries<Vertex>(numQueries, 0,
                                               compLabels.pathPos[FWD].size());
  std::size_t counter = 0;

  auto t1 = clock::now();
  for (const auto &[u, v] : queries)
    counter += query(compLabels, u, v);
  auto t2 = clock::now();

  double total_ns = std::chrono::duration_cast<ns>(t2 - t1).count();
  std::cout << numQueries << " queries: total " << total_ns << " ns, avg "
            << (total_ns / numQueries) << " ns/query, counter=" << counter
            << "\n";
}
