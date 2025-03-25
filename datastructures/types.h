#pragma once

#include <cstdint>
#include <limits>
#include <vector>

typedef std::uint32_t Vertex;
typedef std::size_t Index;
typedef std::uint8_t Distance;

constexpr std::uint32_t noVertex = std::uint32_t(-1);
constexpr std::size_t noIndex = std::size_t(-1);
constexpr std::uint8_t noDistance = std::numeric_limits<std::uint8_t>::max();
constexpr std::uint8_t infinity = std::numeric_limits<std::uint8_t>::max() / 2;

enum DIRECTION : bool { FWD, BWD };