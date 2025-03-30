#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "../external/csv.h"
#include "../external/status_log.h"
#include "graph.h"
#include "types.h"
#include "utils.h"

// TODO contract events at the same time

struct Event {
  EventID stopEventId;
  LineID lineId;
  TripID tripId;
  StopPos stopIndex;
  StopID stopId;
  Time arrivalTime;
  Time departureTime;

  Event(EventID se, LineID li, TripID ti, StopPos si, StopID st, Time at,
        Time dt)
      : stopEventId(se),
        lineId(li),
        tripId(ti),
        stopIndex(si),
        stopId(st),
        arrivalTime(at),
        departureTime(dt) {
    assert(arrivalTime <= departureTime);
  }

  // Comparison operator based on arrival and departure times.
  auto operator<=>(const Event& other) const {
    return std::tie(arrivalTime, departureTime, lineId, tripId, stopIndex) <=>
           std::tie(other.arrivalTime, other.departureTime, other.lineId,
                    other.tripId, other.stopIndex);
  }
};

struct TimeTable {
  // Trips still store event IDs.
  std::vector<std::vector<EventID>> trips;
  // For each stop we now store vertices (each event gives two vertices).
  std::vector<std::vector<Vertex>> eventsOfStop;
  std::vector<std::vector<std::pair<StopID, Time>>> footpaths;
  std::vector<Event> events;
  std::array<Graph, 2> transferGraphs;

  const int numThreads;

  TimeTable(const int numThreads = 1) : numThreads(numThreads) {}

  // Constructor that reads timetable data from CSV files.
  TimeTable(const std::string& fileName, const int numThreads = 1)
      : numThreads(numThreads) {
    readFromCSV(fileName);
  }

  // Helper methods for TEGraph vertices.
  // Returns true if the vertex represents a departure event.
  bool isDepartureVertex(const Vertex v) const { return (v & 1) == 1; }

  // Returns true if the vertex represents an arrival event.
  bool isArrivalVertex(const Vertex v) const { return (v & 1) == 0; }

  // Returns the associated time for the vertex (arrival time for arrival
  // vertex, departure time for departure vertex).
  Time getTime(const Vertex v) const {
    EventID eventId = v >> 1;
    assert(eventId < events.size());
    return isArrivalVertex(v) ? events[eventId].arrivalTime
                              : events[eventId].departureTime;
  }

  // Returns the number of stops.
  std::size_t numberOfStops() const { return eventsOfStop.size(); }

  // Returns the number of trips.
  std::size_t numberOfTrips() const { return trips.size(); }

  // Returns the number of events.
  std::size_t numberOfEvents() const { return events.size(); }

  // Clears all timetable data.
  void clear() {
    trips.clear();
    eventsOfStop.clear();
    footpaths.clear();
    events.clear();
    transferGraphs[FWD].clear();
    transferGraphs[BWD].clear();
  }

  // Applies a function to relax all transfer edges for a given vertex.
  template <typename Func>
  void relaxTransferEdges(const DIRECTION dir, const Vertex vertex,
                          Func&& function) {
    transferGraphs[dir].relaxAllEdges(vertex, function);
  }

  // Applies a function to relax all footpath edges for a given stop.
  template <typename Func>
  void relaxFootpathEdges(const StopID stopId, Func&& function) {
    assert(stopId < numberOfStops());
    for (auto& p : footpaths[stopId]) {
      function(stopId, p.first, p.second);
    }
  }

  // Applies a function to relax the next event in the same trip.
  template <typename Func>
  void relaxNextEventInTrip(const DIRECTION dir, const EventID eventId,
                            Func&& function) {
    TripID tripId = events[eventId].tripId;
    StopPos stopIndex = events[eventId].stopIndex;

    if (dir == FWD && stopIndex + 1 < trips[tripId].size()) {
      function((eventId << 1) + 1, trips[tripId][stopIndex + 1] << 1);
    }
    if (dir == BWD && stopIndex > 0) {
      function((eventId << 1), trips[tripId][stopIndex - 1] << 1);
    }
  }

  bool lessOrEqual(const Vertex left, const Vertex right) const {
    return std::forward_as_tuple(getTime(left), left) <=
           std::forward_as_tuple(getTime(right), right);
  }

  // Build Time Expanded Graph (TEGraph)
  void buildTEGraph() {
    StatusLog log("Build Time Expanded Graph");
    sortEventsOfStops();

    const std::size_t numTEVertices = numberOfEvents() * 2;
    // Build temporary TEGraph as an adjacency list.
    std::vector<std::vector<Vertex>> ted(numTEVertices);
    std::atomic<std::size_t> numberOfTEDEdges{0};

    // 1. Stop-transfer-chain:
    parallelFor(
        0, numberOfStops(),
        [&](const std::size_t, const std::size_t s) {
          const auto& vecVertices = eventsOfStop[s];
          assert(std::is_sorted(vecVertices.begin(), vecVertices.end(),
                                [&](const auto left, const auto right) {
                                  return lessOrEqual(left, right);
                                }));
          for (std::size_t i = 1; i < vecVertices.size(); ++i) {
            Vertex from = vecVertices[i - 1];
            Vertex to = vecVertices[i];
            ted[from].push_back(to);
            numberOfTEDEdges.fetch_add(1, std::memory_order_relaxed);
          }
        },
        numThreads);

    // 2. Trip-chain edges:
    parallelFor(
        0, numberOfTrips(),
        [&](const std::size_t, const std::size_t t) {
          const auto& tripEvents = trips[t];
          for (std::size_t i = 1; i < tripEvents.size(); ++i) {
            Vertex from = (tripEvents[i - 1] << 1) + 1;
            Vertex to = (tripEvents[i] << 1);
            assert(lessOrEqual(from, to));
            ted[from].push_back(to);
            numberOfTEDEdges.fetch_add(1, std::memory_order_relaxed);
          }
        },
        numThreads);

    // 3. Footpath edges:
    // Create a vector of hashmaps, one per thread.
    std::vector<std::unordered_map<StopID, Vertex>> threadLocalMap(numThreads);

    // For each stop, iterate over its events in reverse order.
    parallelFor(
        0, numberOfStops(),
        [&](const std::size_t threadId, const std::size_t s) {
          auto& localMap = threadLocalMap[threadId];
          localMap.clear();

          const auto& vecVertices = eventsOfStop[s];
          for (Vertex v : vecVertices | std::views::reverse) {
            if (isArrivalVertex(v)) {
              Time arrTime = getTime(v);
              relaxFootpathEdges(s, [&](const StopID, const StopID toStop,
                                        const Time duration) {
                assert(s != toStop);
                Time arrivalThreshold = arrTime + duration;
                const auto& toStopVertices = eventsOfStop[toStop];
                Vertex targetVertex = noVertex;
                for (Vertex x : toStopVertices) {
                  if (isArrivalVertex(x) &&
                      std::forward_as_tuple(getTime(x), x) >=
                          std::forward_as_tuple(arrivalThreshold, v)) {
                    targetVertex = x;
                    break;
                  }
                }
                if (targetVertex != noVertex) {
                  if (localMap.contains(toStop)) {
                    return;
                  }
                  localMap[toStop] = targetVertex;
                  ted[v].push_back(targetVertex);
                  numberOfTEDEdges.fetch_add(1, std::memory_order_relaxed);
                }
              });
            }
          }
        },
        numThreads);

    // Convert the temporary graph into an edge list.
    std::vector<std::pair<Vertex, Vertex>> edges;
    edges.reserve(numberOfTEDEdges.load(std::memory_order_relaxed));
    for (Vertex from = 0; from < ted.size(); ++from) {
      for (Vertex to : ted[from]) {
        assert(lessOrEqual(from, to));
        edges.emplace_back(from, to);
      }
    }

    transferGraphs[FWD].clear();
    transferGraphs[BWD].clear();

    transferGraphs[FWD].buildFromEdgeList(edges, numTEVertices);
    transferGraphs[BWD] = transferGraphs[FWD].reverseGraph();
  }

  // **** I/O Functions ****

  // Reads the complete timetable from CSV files.
  void readFromCSV(const std::string& fileName) {
    StatusLog log("Loading TimeTable from file");
    clear();

    readEventsFromCSV(fileName + "/trips.csv");
    // readMinChangeTimesFromCSV(fileName + "/stops.csv");
    readFootpathsFromCSV(fileName + "/footpaths.csv");
    // readEventGraphFromCSV(fileName + "/transfers.csv");

    sortEventsOfStops();
  }

  // Reads events from a CSV file and organizes them into trips and stops.
  void readEventsFromCSV(const std::string& fileName) {
    io::CSVReader<7, io::trim_chars<' '>, io::double_quote_escape<',', '\"'>>
        in(fileName);
    in.read_header(io::ignore_extra_column, "StopEventId", "LineId", "TripId",
                   "StopIndex", "StopId", "ArrivalTime", "DepartureTime");
    EventID stopEventId;
    LineID lineId;
    TripID tripId;
    StopPos stopIndex;
    StopID stopId;
    Time arrivalTime, departureTime;

    std::size_t numberOfStops = 0;
    std::size_t numberOfTrips = 0;

    // Reserve memory for events.
    events.reserve(10000);

    // Read each row from the CSV and create Event objects.
    while (in.read_row(stopEventId, lineId, tripId, stopIndex, stopId,
                       arrivalTime, departureTime)) {
      events.emplace_back(stopEventId, lineId, tripId, stopIndex, stopId,
                          arrivalTime, departureTime);

      // Keep track of the maximum stop and trip IDs.
      numberOfStops = std::max(numberOfStops, static_cast<std::size_t>(stopId));
      numberOfTrips = std::max(numberOfTrips, static_cast<std::size_t>(tripId));
    }

    numberOfStops++;
    numberOfTrips++;

    trips.resize(numberOfTrips);
    // Note: eventsOfStop now stores vertices, so resize accordingly.
    eventsOfStop.resize(numberOfStops);

    // Organize events by stop and by trip.
    for (std::size_t i = 0; i < events.size(); ++i) {
      const auto& event = events[i];
      assert(event.stopId < eventsOfStop.size());
      assert(event.tripId < trips.size());
      // For each event, add both an arrival and a departure vertex.
      eventsOfStop[event.stopId].push_back(static_cast<Vertex>(i << 1));
      eventsOfStop[event.stopId].push_back(static_cast<Vertex>((i << 1) + 1));
      trips[event.tripId].push_back(static_cast<EventID>(i));
    }
  }

  // Reads stop-specific data from a CSV file and adjusts departure times.
  void readMinChangeTimesFromCSV(const std::string& fileName) {
    io::CSVReader<2, io::trim_chars<' '>, io::double_quote_escape<',', '\"'>>
        in(fileName);
    in.read_header(io::ignore_extra_column, "StopId", "MinChangeTime");

    StopID stopId;
    int minChangeTime;

    std::vector<int> times(numberOfStops());

    while (in.read_row(stopId, minChangeTime)) {
      assert(stopId < eventsOfStop.size());
      times[stopId] = minChangeTime;
    }

    parallelFor(
        0, numberOfStops(),
        [&](const std::size_t, const std::size_t i) {
          // Adjust departure times for each event at stop i.
          for (Vertex v : eventsOfStop[i]) {
            // Only adjust if the vertex represents a departure.
            if (isDepartureVertex(v)) {
              EventID eventId = v >> 1;
              events[eventId].departureTime += times[i];
            }
          }
        },
        numThreads);
  }

  // Reads footpath data.
  void readFootpathsFromCSV(const std::string& fileName) {
    io::CSVReader<3, io::trim_chars<' '>, io::double_quote_escape<',', '\"'>>
        in(fileName);
    in.read_header(io::ignore_extra_column, "FromStopId", "ToStopId",
                   "TravelTime");

    StopID from, to;
    Time duration;

    footpaths.resize(numberOfStops());

    while (in.read_row(from, to, duration)) {
      assert(from < numberOfStops());
      assert(to < numberOfStops());
      if (from == to) continue;
      footpaths[from].emplace_back(to, duration);
    }
  }

  // Sorts events for each stop by their scheduled times using helper methods.
  void sortEventsOfStops() {
    parallelFor(
        0, eventsOfStop.size(),
        [&](const std::size_t, const std::size_t s) {
          auto& vec = eventsOfStop[s];
          std::sort(vec.begin(), vec.end(), [&](Vertex a, Vertex b) {
            return std::forward_as_tuple(getTime(a), a) <
                   std::forward_as_tuple(getTime(b), b);
          });
        },
        numThreads);
  }

  // Reads transfer graph data from a CSV file and builds the graphs.
  void readEventGraphFromCSV(const std::string& fileName) {
    io::CSVReader<2, io::trim_chars<' '>, io::double_quote_escape<',', '\"'>>
        in(fileName);
    in.read_header(io::ignore_extra_column, "FromVertex", "ToVertex");

    Vertex fromVertex, toVertex;
    std::vector<std::pair<Vertex, Vertex>> edges;
    edges.reserve((numberOfEvents() << 1));

    while (in.read_row(fromVertex, toVertex)) {
      assert(fromVertex < (numberOfEvents() << 1));
      assert(toVertex < (numberOfEvents() << 1));
      edges.emplace_back(fromVertex, toVertex);
    }

    transferGraphs[FWD].buildFromEdgeList(edges, numberOfEvents() << 1);
    transferGraphs[BWD] = transferGraphs[FWD].reverseGraph();
  }

  // Displays basic statistics about the timetable and transfer graph.
  void showStats() const {
    std::size_t totalNumFootpaths = 0;
    for (auto& f : footpaths) totalNumFootpaths += f.size();

    std::cout << "TimeTable Statistics:" << std::endl;
    std::cout << "  Number of events:      " << numberOfEvents() << std::endl;
    std::cout << "  Number of stops:       " << numberOfStops() << std::endl;
    std::cout << "  Number of trips:       " << numberOfTrips() << std::endl;
    std::cout << "  Number of footpaths:   " << totalNumFootpaths << std::endl;
    std::cout << "Transfer Graph Stats:" << std::endl;
    transferGraphs[FWD].showStats();
  }
};