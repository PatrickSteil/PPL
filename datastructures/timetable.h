/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "csv.h"
#include "csv_status_log.h"
#include "graph.h"
#include "types.h"
#include "utils.h"

struct Event {
  TripID tripId;
  StopPos stopIndex;
  StopID stopId;
  Time arrivalTime;
  Time departureTime;

  Event(TripID ti, StopPos si, StopID st, Time at, Time dt)
      : tripId(ti), stopIndex(si), stopId(st), arrivalTime(at),
        departureTime(dt) {
    assert(arrivalTime <= departureTime);
  }

  // Comparison operator based on arrival and departure times.
  auto operator<=>(const Event &other) const {
    return std::tie(arrivalTime, departureTime, tripId, stopIndex) <=>
           std::tie(other.arrivalTime, other.departureTime, other.tripId,
                    other.stopIndex);
  }
};

struct TimeTable {
  std::vector<std::vector<EventID>> trips;
  std::vector<std::vector<Vertex>> eventsOfStop;
  std::vector<std::vector<std::pair<StopID, Time>>> footpaths;
  std::vector<Event> events;
  std::vector<Time> times;
  std::array<Graph, 2> transferGraphs;

  const int numThreads;

  TimeTable(const int numThreads = 1) : numThreads(numThreads) {}

  // Constructor that reads timetable data from CSV files.
  TimeTable(const std::string &fileName, const int numThreads = 1)
      : numThreads(numThreads) {
    readFromCSV(fileName);
  }

  // Helper methods for TEGraph vertices.
  // Returns true if the vertex represents a departure event.
  bool isDepartureVertex(const Vertex v) const { return (v & 1); }

  // Returns true if the vertex represents an arrival event.
  bool isArrivalVertex(const Vertex v) const { return !isDepartureVertex(v); }

  // Returns the associated time for the vertex (arrival time for arrival
  // vertex, departure time for departure vertex).
  Time getTime(const Vertex v) const {
    assert(v < times.size());
    return times[v];
  }

  // Applies a function to relax all footpath edges for a given stop.
  template <typename Func>
  void relaxFootpathEdges(const StopID stopId, Func &&function) {
    assert(stopId < numberOfStops());
    for (auto &p : footpaths[stopId]) {
      function(stopId, p.first, p.second);
    }
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
    times.clear();
    transferGraphs[FWD].clear();
    transferGraphs[BWD].clear();
  }

  bool lessOrEqual(const Vertex left, const Vertex right) const {
    return std::forward_as_tuple(getTime(left), left) <=
           std::forward_as_tuple(getTime(right), right);
  }

  Vertex findEarliestEvent(const StopID stop, Time arrivalThreshold,
                           const Vertex v) {
    auto it = std::lower_bound(
        eventsOfStop[stop].begin(), eventsOfStop[stop].end(), arrivalThreshold,
        [&](Vertex vertex, Time time) {
          return std::forward_as_tuple(getTime(vertex), vertex) <=
                 std::forward_as_tuple(time, v);
        });

    if (it != eventsOfStop[stop].end()) {
      return *it;
    } else {
      return noVertex;
    }
  }

  void contract() {
    CSVStatusLog log("Compress Data");

    const std::size_t oldNumVertices = numberOfEvents() * 2;
    std::vector<Vertex> oldToNewMapping(oldNumVertices, 0);
    std::size_t newId = 0;

    for (StopID s = 0; s < numberOfStops(); ++s) {
      auto &vec = eventsOfStop[s];
      if (vec.empty())
        continue;

      oldToNewMapping[vec[0]] = newId++;
      Time prevTime = getTime(vec[0]);

      for (std::size_t i = 1; i < vec.size(); ++i) {
        Vertex v = vec[i];
        Time currTime = getTime(v);
        if (currTime == prevTime) {
          oldToNewMapping[v] = oldToNewMapping[vec[i - 1]];
        } else {
          oldToNewMapping[v] = newId++;
          prevTime = currTime;
        }
      }
    }

    std::vector<Time> newTimes(newId, infinity);
    for (Vertex u = 0; u < oldNumVertices; ++u) {
      Vertex new_v = oldToNewMapping[u];
      if (newTimes[new_v] == infinity) {
        newTimes[new_v] = times[u];
      } else {
        assert(newTimes[new_v] == times[u]);
      }
    }
    times = std::move(newTimes);

    std::vector<std::pair<Vertex, Vertex>> newEdges;
    newEdges.reserve(transferGraphs[FWD].numEdges());

    transferGraphs[FWD].doForAllEdges([&](const Vertex u, const Vertex v) {
      Vertex fromVertex = oldToNewMapping[u];
      Vertex toVertex = oldToNewMapping[v];

      if (fromVertex == toVertex)
        return;

      if (std::forward_as_tuple(times[fromVertex], fromVertex) >
          std::forward_as_tuple(times[toVertex], toVertex)) {
        std::swap(fromVertex, toVertex);
      }
      newEdges.emplace_back(fromVertex, toVertex);
    });

    sortAndRemoveDuplicates(newEdges);

    transferGraphs[FWD].buildFromEdgeList(newEdges, newId);
    transferGraphs[BWD] = transferGraphs[FWD].reverseGraph();

    for (StopID s = 0; s < numberOfStops(); ++s) {
      auto &vec = eventsOfStop[s];
      for (auto &v : vec) {
        v = oldToNewMapping[v];
      }
      sortAndRemoveDuplicates(vec);
    }

    sortEventsOfStops();
  }

  // Build Time Expanded Graph (TEGraph)
  void buildTEGraph() {
    CSVStatusLog log("Build Time Expanded Graph");

    const std::size_t numTEVertices = numberOfEvents() * 2;

    std::vector<std::vector<Vertex>> ted(numTEVertices);
    std::atomic_size_t numberOfTEDEdges{0};

    // 1. Waiting arcs
    parallelFor(
        0, numberOfStops(),
        [&](const std::size_t, const std::size_t s) {
          const auto &vecVertices = eventsOfStop[s];
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

    // 2. Connection arcs
    parallelFor(
        0, numberOfTrips(),
        [&](const std::size_t, const std::size_t t) {
          const auto &tripEvents = trips[t];
          for (std::size_t i = 1; i < tripEvents.size(); ++i) {
            Vertex from = (tripEvents[i - 1] << 1) + 1;
            Vertex to = (tripEvents[i] << 1);

            assert(isDepartureVertex(from));
            assert(isArrivalVertex(to));
            assert(lessOrEqual(from, to));

            ted[from].push_back(to);
            numberOfTEDEdges.fetch_add(1, std::memory_order_relaxed);
          }
        },
        numThreads);

    // 3. Trip Arr to Dep arcs
    parallelFor(
        0, numberOfEvents(),
        [&](const std::size_t, const std::size_t v) {
          Vertex from = (v << 1);
          Vertex to = (v << 1) + 1;

          assert(isArrivalVertex(from));
          assert(isDepartureVertex(to));
          assert(lessOrEqual(from, to));

          ted[from].push_back(to);
          numberOfTEDEdges.fetch_add(1, std::memory_order_relaxed);
        },
        numThreads);

    // 4. Footpath edges:
    // Create a vector of hashmaps, one per thread.
    // used to skip adding unnecessary footpath edges from earlier events to the
    // same event if
    std::vector<std::unordered_map<StopID, Vertex>> threadLocalMap(numThreads);

    // For each stop, iterate over its events in reverse order.
    parallelFor(
        0, numberOfStops(),
        [&](const std::size_t threadId, const std::size_t s) {
          auto &localMap = threadLocalMap[threadId];
          localMap.clear();

          const auto &vecVertices = eventsOfStop[s];
          for (Vertex v : vecVertices | std::views::reverse) {
            Time arrTime = getTime(v);
            relaxFootpathEdges(
                s, [&](const StopID, const StopID toStop, const Time duration) {
                  assert(s != toStop);
                  Time arrivalThreshold = arrTime + duration;

                  Vertex targetVertex =
                      findEarliestEvent(toStop, arrivalThreshold, v);

                  if (targetVertex != noVertex) {
                    if (localMap.contains(toStop) &&
                        localMap[toStop] == targetVertex) {
                      return;
                    }
                    localMap[toStop] = targetVertex;
                    ted[v].push_back(targetVertex);
                    numberOfTEDEdges.fetch_add(1, std::memory_order_relaxed);
                  }
                });
          }
        },
        numThreads);

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
  void readFromCSV(const std::string &fileName) {
    CSVStatusLog log("Loading TimeTable from file");
    clear();

    readEventsFromCSV(fileName + "/trips.csv");
    readFootpathsFromCSV(fileName + "/footpaths.csv");
    // readEventGraphFromCSV(fileName + "/transfers.csv");

    sortEventsOfStops();
  }

  // Reads events from a CSV file and organizes them into trips and stops.
  void readEventsFromCSV(const std::string &fileName) {
    io::CSVReader<5, io::trim_chars<' '>, io::double_quote_escape<',', '\"'>>
        in(fileName);
    in.read_header(io::ignore_extra_column, "TripId", "StopIndex", "StopId",
                   "ArrivalTime", "DepartureTime");

    TripID tripId;
    StopPos stopIndex;
    StopID stopId;
    Time arrivalTime, departureTime;

    std::size_t numberOfStops = 0;
    std::size_t numberOfTrips = 0;

    events.reserve(10000);
    times.reserve(10000);

    while (in.read_row(tripId, stopIndex, stopId, arrivalTime, departureTime)) {
      events.emplace_back(tripId, stopIndex, stopId, arrivalTime,
                          departureTime);
      times.emplace_back(arrivalTime);
      times.emplace_back(departureTime);

      numberOfStops = std::max(numberOfStops, static_cast<std::size_t>(stopId));
      numberOfTrips = std::max(numberOfTrips, static_cast<std::size_t>(tripId));
    }

    numberOfStops++;
    numberOfTrips++;

    trips.resize(numberOfTrips);
    eventsOfStop.resize(numberOfStops);

    for (std::size_t i = 0; i < events.size(); ++i) {
      const auto &event = events[i];
      assert(event.stopId < eventsOfStop.size());
      assert(event.tripId < trips.size());

      eventsOfStop[event.stopId].push_back(static_cast<Vertex>(i << 1));
      eventsOfStop[event.stopId].push_back(static_cast<Vertex>((i << 1) + 1));
      trips[event.tripId].push_back(static_cast<EventID>(i));
    }
  }

  // Reads footpath data.
  void readFootpathsFromCSV(const std::string &fileName) {
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
      if (from == to)
        continue;
      footpaths[from].emplace_back(to, duration);
    }
  }

  // Sorts events for each stop by their scheduled times using helper methods.
  void sortEventsOfStops() {
    parallelFor(
        0, eventsOfStop.size(),
        [&](const std::size_t, const std::size_t s) {
          auto &vec = eventsOfStop[s];
          std::sort(vec.begin(), vec.end(), [&](Vertex a, Vertex b) {
            return std::forward_as_tuple(getTime(a), a) <
                   std::forward_as_tuple(getTime(b), b);
          });
        },
        numThreads);
  }

  // Reads transfer graph data from a CSV file and builds the graphs.
  void readEventGraphFromCSV(const std::string &fileName) {
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
    for (auto &f : footpaths)
      totalNumFootpaths += f.size();

    std::cout << "TimeTable Statistics:" << std::endl;
    std::cout << "  Number of events:      " << numberOfEvents() << std::endl;
    std::cout << "  Number of stops:       " << numberOfStops() << std::endl;
    std::cout << "  Number of trips:       " << numberOfTrips() << std::endl;
    std::cout << "  Number of footpaths:   " << totalNumFootpaths << std::endl;
    std::cout << "Transfer Graph Stats:" << std::endl;
    transferGraphs[FWD].showStats();
  }
};

// Query

template <class PATHLABEL_TYPE = PathLabel>
Time plt_query(const TimeTable &tt,
               std::array<std::vector<PATHLABEL_TYPE>, 2> &, const StopID from,
               const StopID to, const Time depTime) {
  assert(from < tt.numberOfStops());
  assert(to < tt.numberOfStops());

  if (from == to)
    return depTime;
  // TODO
  return infinity;
}

template <class PATHLABEL_TYPE = PathLabel>
void benchmark_ptl(const TimeTable &&tt,
                   std::array<std::vector<PATHLABEL_TYPE>, 2> &labels,
                   const std::size_t numQueries = 100000) {
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;

  assert(labels[FWD].size() == labels[BWD].size());

  auto queries =
      generateRandomQueries<StopID>(numQueries, 0, tt.numberOfStops());
  long double totalTime(0);
  std::size_t counter(0);

  for (std::pair<StopID, StopID> &paar : queries) {
    auto t1 = high_resolution_clock::now();
    counter += plt_query(tt, labels, paar.first, paar.second, 8 * 60 * 60);
    auto t2 = high_resolution_clock::now();
    duration<double, std::nano> nano_double = t2 - t1;
    totalTime += nano_double.count();
  }

  std::cout << "The " << numQueries
            << " random stop-to-stop queries (depTime: 08:00) took in total "
            << totalTime << " [ns] and on average "
            << (double)(totalTime / numQueries) << " [ns]! Counter: " << counter
            << "\n";
}
