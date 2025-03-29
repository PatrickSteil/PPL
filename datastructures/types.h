#pragma once

#include <cstdint>
#include <limits>
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

enum DIRECTION : bool { FWD, BWD };