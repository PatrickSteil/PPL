#pragma once

#ifdef __GNUC__
#define PREFETCH(addr) __builtin_prefetch(addr)
#else
#define PREFETCH(addr)
#endif

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
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

  template <typename Func>
  auto doForAllHubs(Func&& func) {
    for (auto hub : hubs) {
      func(hub);
    }
  }
};

class ThreadSafePathLabel {
 public:
  std::vector<PathHub> hubs;
  mutable std::mutex m;

  ThreadSafePathLabel() = default;

  explicit ThreadSafePathLabel(const std::vector<PathHub>& init_hubs)
      : hubs(init_hubs) {}

  explicit ThreadSafePathLabel(std::vector<PathHub>&& init_hubs)
      : hubs(std::move(init_hubs)) {}

  ThreadSafePathLabel(const ThreadSafePathLabel& other) {
    std::lock_guard<std::mutex> lock(other.m);
    hubs = other.hubs;
  }

  ThreadSafePathLabel& operator=(const ThreadSafePathLabel& other) {
    if (this != &other) {
      std::scoped_lock lock(m, other.m);
      hubs = other.hubs;
    }
    return *this;
  }

  ThreadSafePathLabel(ThreadSafePathLabel&& other) noexcept {
    std::lock_guard<std::mutex> lock(other.m);
    hubs = std::move(other.hubs);
  }

  ThreadSafePathLabel& operator=(ThreadSafePathLabel&& other) noexcept {
    if (this != &other) {
      std::scoped_lock lock(m, other.m);
      hubs = std::move(other.hubs);
    }
    return *this;
  }

  PathHub get(size_t index) const {
    std::lock_guard<std::mutex> lock(m);
    assert(index < hubs.size() && "Index out of bounds in ThreadSafePathLabel");
    return hubs[index];
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(m);
    return hubs.size();
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(m);
    return hubs.empty();
  }

  void push_back(const PathHub& label) {
    std::lock_guard<std::mutex> lock(m);
    hubs.push_back(label);
  }

  void emplace_back(const std::uint32_t path_id, const std::uint32_t path_pos) {
    std::lock_guard<std::mutex> lock(m);
    hubs.emplace_back(path_id, path_pos);
  }

  void sort() {
    std::lock_guard<std::mutex> lock(m);
    std::sort(hubs.begin(), hubs.end());
  }

  void clear() {
    std::lock_guard<std::mutex> lock(m);
    hubs.clear();
  }

  std::vector<PathHub> get_snapshot() const {
    std::lock_guard<std::mutex> lock(m);
    return hubs;
  }

  template <typename Func>
  auto doForAllHubs(Func&& func) {
    std::lock_guard<std::mutex> lock(m);
    for (auto hub : hubs) {
      func(hub);
    }
  }
};

bool query(const PathLabel& from, const PathLabel& to) {
  std::size_t i = 0, j = 0;
  while (i < from.size() && j < to.size()) {
    const auto& srcHub = from[i];
    const auto& tgtHub = to[j];

    if (srcHub.getPath() < tgtHub.getPath()) {
      ++i;
    } else if (srcHub.getPath() > tgtHub.getPath()) {
      ++j;
    } else {
      if (srcHub.getPathPos() <= tgtHub.getPathPos()) {
        return true;
      } else {
        ++j;
      }
    }
  }
  return false;
}

bool query(const ThreadSafePathLabel& from, const ThreadSafePathLabel& to) {
  std::scoped_lock lock(from.m, to.m);

  std::size_t i = 0, j = 0;
  while (i < from.hubs.size() && j < to.hubs.size()) {
    const auto& srcHub = from.hubs[i];
    const auto& tgtHub = to.hubs[j];

    // Compare path ids to sync the two labels
    if (srcHub.getPath() < tgtHub.getPath()) {
      ++i;
    } else if (srcHub.getPath() > tgtHub.getPath()) {
      ++j;
    } else {
      if (srcHub.getPathPos() <= tgtHub.getPathPos()) {
        return true;
      } else {
        ++j;
      }
    }
  }
  return false;
}

template <class PATHLABEL_TYPE = PathLabel>
void benchmark_pathlabels(std::array<std::vector<PATHLABEL_TYPE>, 2>& labels,
                          const std::size_t numQueries = 100000) {
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;

  assert(labels[FWD].size() == labels[BWD].size());

  auto queries =
      generateRandomQueries<Vertex>(numQueries, 0, labels[FWD].size());
  long double totalTime(0);
  std::size_t counter(0);

  for (std::pair<Vertex, Vertex>& paar : queries) {
    auto t1 = high_resolution_clock::now();
    counter += query(labels[FWD][paar.first], labels[BWD][paar.second]);
    auto t2 = high_resolution_clock::now();
    duration<double, std::nano> nano_double = t2 - t1;
    totalTime += nano_double.count();
  }

  std::cout << "The " << numQueries << " random queries took in total "
            << totalTime << " [ns] and on average "
            << (double)(totalTime / numQueries) << " [ns]! Counter: " << counter
            << "\n";
}

// **** Stats ****
template <class PATHLABEL_TYPE = PathLabel>
std::size_t computeTotalBytes(
    const std::array<std::vector<PATHLABEL_TYPE>, 2>& labels) {
  std::size_t totalBytes = 0;
  for (const auto& labelSet : labels) {
    for (const auto& label : labelSet) {
      totalBytes += sizeof(PATHLABEL_TYPE);
      totalBytes += label.size() * sizeof(decltype(PathHub{}.path_id));
      totalBytes += label.size() * sizeof(decltype(PathHub{}.path_pos));
    }
  }
  return totalBytes;
}

template <class PATHLABEL_TYPE = PathLabel>
void showLabelStats(const std::array<std::vector<PATHLABEL_TYPE>, 2>& labels) {
  auto computeStats = [](const std::vector<PATHLABEL_TYPE>& currentLabels) {
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
template <class PATHLABEL_TYPE = PathLabel>
void saveToFile(std::array<std::vector<PATHLABEL_TYPE>, 2>& labels,
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
    labels[FWD][v].doForAllHubs([&](const auto& hub) {
      outFile << " " << hub.getPath() << " " << hub.getPathPos();
    });
    outFile << "\n";

    outFile << "i " << v;
    labels[BWD][v].doForAllHubs([&](const auto& hub) {
      outFile << " " << hub.getPath() << " " << hub.getPathPos();
    });
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

void saveToFile(const std::vector<std::vector<Vertex>>& paths,
                const std::string& fileName) {
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
