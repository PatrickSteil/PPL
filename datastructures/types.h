/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

typedef std::uint32_t Vertex;
typedef std::uint32_t StopID;
typedef std::uint32_t LineID;
typedef std::uint32_t TripID;
typedef std::uint32_t EventID;
typedef std::uint32_t Time;
typedef std::size_t Index;
typedef std::uint8_t Distance;
typedef std::uint8_t StopPos;

constexpr std::uint32_t noVertex = std::uint32_t(-1);
constexpr std::uint32_t noStopID = std::uint32_t(-1);
constexpr std::uint32_t noLineID = std::uint32_t(-1);
constexpr std::uint32_t noTripID = std::uint32_t(-1);
constexpr std::uint32_t noEventID = std::uint32_t(-1);
constexpr std::uint32_t noTime = std::uint32_t(-1);
constexpr std::size_t noIndex = std::size_t(-1);
constexpr std::uint8_t noDistance = std::numeric_limits<std::uint8_t>::max();
constexpr std::uint8_t infinity = std::numeric_limits<std::uint8_t>::max() / 2;
constexpr std::uint8_t noStopPos = std::uint8_t(-1);
constexpr std::uint16_t noPathPos = std::uint16_t(-1);

enum DIRECTION : bool { FWD, BWD };

// Ranking methods for chains/paths.
enum class RankingMethod {
  SUM,      // Sum of vertex scores.
  AVERAGE,  // Average vertex score.
  MIN,      // Minimum (bottleneck) vertex score.
  MAX,      // Maximum vertex score.
  HYBRID,   // A hybrid score combining several metrics.
  LENGTH    // By Length
};

// Helper functions to compute scores for a given path.
// 'value' is a vector of vertex scores computed beforehand.

inline std::size_t computeSum(const std::vector<Vertex>& path,
                              const std::vector<std::size_t>& value) {
  return std::accumulate(
      path.begin(), path.end(), 0ULL,
      [&](std::size_t acc, Vertex v) { return acc + value[v]; });
}

inline double computeAverage(const std::vector<Vertex>& path,
                             const std::vector<std::size_t>& value) {
  if (path.empty()) return 0.0;
  return static_cast<double>(computeSum(path, value)) / path.size();
}

inline std::size_t computeMin(const std::vector<Vertex>& path,
                              const std::vector<std::size_t>& value) {
  if (path.empty()) return std::numeric_limits<std::size_t>::max();
  std::size_t minVal = std::numeric_limits<std::size_t>::max();
  for (Vertex v : path) minVal = std::min(minVal, value[v]);
  return minVal;
}

inline std::size_t computeMax(const std::vector<Vertex>& path,
                              const std::vector<std::size_t>& value) {
  std::size_t maxVal = 0;
  for (Vertex v : path) maxVal = std::max(maxVal, value[v]);
  return maxVal;
}

inline double computeHybrid(const std::vector<Vertex>& path,
                            const std::vector<std::size_t>& value) {
  // Example: weighted sum of the aggregate, average and bottleneck (min)
  // scores.
  double sum = static_cast<double>(computeSum(path, value));
  double avg = computeAverage(path, value);
  double minVal = static_cast<double>(computeMin(path, value));
  constexpr double alpha = 1.0;
  constexpr double beta = 1.0;
  constexpr double gamma = 1.0;
  return alpha * sum + beta * avg + gamma * minVal;
}