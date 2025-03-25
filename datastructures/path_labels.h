#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "types.h"
#include "utils.h"

// Represents a hub in a path with a unique id and position.
struct PathHub {
  std::uint16_t path_id;
  std::uint16_t path_pos;

  PathHub() : path_id(-1), path_pos(-1) {}

  PathHub(std::uint16_t pid, std::uint16_t pos) : path_id(pid), path_pos(pos) {}

  std::uint16_t getPath() const { return path_id; }
  std::uint16_t getPathPos() const { return path_pos; }

  bool isOnSamePath(const PathHub& other) const {
    return (path_id == other.getPath());
  }

  bool isReachable(const PathHub& other) const {
    return (path_id == other.getPath() && path_pos <= other.getPathPos());
  }

  auto operator<=>(const PathHub&) const = default;
};

// Holds a vector of PathHub entries.
struct PathLabel {
  std::vector<PathHub> hubs;

  PathLabel() = default;

  explicit PathLabel(const std::vector<PathHub>& init_hubs) : hubs(init_hubs) {}

  explicit PathLabel(std::vector<PathHub>&& init_hubs)
      : hubs(std::move(init_hubs)) {}

  PathLabel(const PathLabel& other) = default;
  PathLabel& operator=(const PathLabel& other) = default;
  PathLabel& operator=(PathLabel&& other) noexcept = default;

  PathHub& operator[](size_t index) {
    assert(index < hubs.size() && "Index out of bounds in PathLabel");
    return hubs[index];
  }
  const PathHub& operator[](size_t index) const {
    assert(index < hubs.size() && "Index out of bounds in PathLabel");
    return hubs[index];
  }

  size_t size() const { return hubs.size(); }
  bool empty() const { return hubs.empty(); }
  void push_back(const PathHub& label) { hubs.push_back(label); }
  void emplace_back(const std::uint32_t path_id, const std::uint32_t path_pos) {
    hubs.emplace_back(path_id, path_pos);
  }

  void sort() { std::sort(hubs.begin(), hubs.end()); }

  auto begin() { return hubs.begin(); }
  auto end() { return hubs.end(); }
  auto begin() const { return hubs.begin(); }
  auto end() const { return hubs.end(); }

  void clear() { hubs.clear(); }
};

// **** Query ****
bool query(const PathLabel& from, const PathLabel& to) {
  std::size_t i = 0, j = 0;

  assert(std::is_sorted(from.hubs.begin(), from.hubs.end()));
  assert(std::is_sorted(to.hubs.begin(), to.hubs.end()));

  while (i < from.size() && j < to.size()) {
    if (from[i].isReachable(to[j])) {
      return true;
    } else if (from[i] < to[j]) {
      ++i;
    } else {
      ++j;
    }
  }
  return false;
}

void benchmark_pathlabels(std::array<std::vector<PathLabel>, 2>& labels,
                          const std::size_t numQueries = 100000) {
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;

  assert(labels[FWD].size() == labels[BWD].size());

  auto queries =
      generateRandomQueries<Vertex>(numQueries, 0, labels[FWD].size());
  long double totalTime(0);

  for (std::pair<Vertex, Vertex>& paar : queries) {
    auto t1 = high_resolution_clock::now();
    query(labels[FWD][paar.first], labels[BWD][paar.second]);
    auto t2 = high_resolution_clock::now();
    duration<double, std::nano> nano_double = t2 - t1;
    totalTime += nano_double.count();
  }

  std::cout << "The " << numQueries << " random queries took in total "
            << totalTime << " [ms] and on average "
            << (double)(totalTime / numQueries) << " [ns]!\n";
}

// **** Stats ****
std::size_t computeTotalBytes(
    const std::array<std::vector<PathLabel>, 2>& labels) {
  std::size_t totalBytes = 0;
  for (const auto& labelSet : labels) {
    for (const auto& label : labelSet) {
      totalBytes += sizeof(PathLabel);
      totalBytes += label.size() * sizeof(std::uint32_t);
      totalBytes += label.size() * sizeof(std::uint32_t);
    }
  }
  return totalBytes;
}

void showLabelStats(const std::array<std::vector<PathLabel>, 2>& labels) {
  auto computeStats = [](const std::vector<PathLabel>& currentLabels) {
    std::size_t minSize = std::numeric_limits<std::size_t>::max();
    std::size_t maxSize = 0, totalSize = 0;
    for (const auto& label : currentLabels) {
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
            << static_cast<double>(computeTotalBytes(labels) /
                                   (1024.0 * 1024.0))
            << std::endl;
}

/// **** IO ****
void saveToFile(std::array<std::vector<PathLabel>, 2>& labels,
                const std::string& fileName) {
  std::ofstream outFile(fileName);
  if (!outFile.is_open()) {
    std::cerr << "Error: Unable to open file " << fileName << " for writing.\n";
    return;
  }

  std::size_t N = labels[FWD].size();
  outFile << "V " << N << "\n";

  for (std::size_t v = 0; v < N; ++v) {
    outFile << "o " << v;
    for (const auto& hub : labels[FWD][v]) {
      outFile << " " << hub.getPath() << " " << hub.getPathPos();
    }
    outFile << "\n";

    outFile << "i " << v;
    for (const auto& hub : labels[BWD][v]) {
      outFile << " " << hub.getPath() << " " << hub.getPathPos();
    }
    outFile << "\n";
  }

  outFile.close();
  if (outFile.fail()) {
    std::cerr << "Error: Writing to file " << fileName << " failed.\n";
  } else {
    std::cout << "Labels saved successfully to " << fileName << "\n";
  }
}

std::vector<std::vector<Vertex>> loadPathFile(const std::string& fileName) {
  std::ifstream inFile(fileName);
  if (!inFile.is_open()) {
    std::cerr << "Error: Unable to open file " << fileName << "\n";
    return {};
  }

  std::string line;
  if (!std::getline(inFile, line)) {
    std::cerr << "Error: File is empty or could not read the first line.\n";
    return {};
  }

  std::istringstream headerStream(line);
  char identifier;
  std::size_t numPaths;
  headerStream >> identifier >> numPaths;
  if (identifier != 'P') {
    std::cerr << "Error: Expected line to start with 'P'.\n";
    return {};
  }

  std::vector<std::vector<Vertex>> paths;
  paths.reserve(numPaths);

  while (std::getline(inFile, line)) {
    if (line.empty()) continue;

    std::istringstream lineStream(line);
    std::vector<Vertex> path;
    Vertex v;
    while (lineStream >> v) {
      path.push_back(v);
    }
    paths.push_back(std::move(path));
  }

  return paths;
}

void saveToFile(const std::vector<std::vector<Vertex>>& paths, const std::string& fileName) {
    std::ofstream outFile(fileName);
    if (!outFile.is_open()) {
        std::cerr << "Error: Unable to open file " << fileName << " for writing.\n";
        return;
    }
    
    // Write header line: 'P' followed by the number of paths.
    outFile << "P " << paths.size() << "\n";
    
    // Write each path on a new line.
    for (const auto& path : paths) {
        std::ostringstream lineStream;
        for (Vertex v : path) {
            lineStream << v << " ";
        }
        outFile << lineStream.str() << "\n";
    }
    
    outFile.close();
}
