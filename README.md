# Pruned Path Labeling (PPL)
Author: Patrick Steil

This repository implements a hub labeling algorithm for efficient reachability queries on directed acyclic graphs (DAGs). Two variants are supported:

- Standard PPL: Reads a graph from a DIMACS file.
- Timetable Variant (TT_PPL): Reads a timetable from a directory, builds a time expanded graph (TEG), and then performs the PPL algorithm using the events per stop as a chain decomposition.

## Overview
### Timetable Reading:
For the timetable variant, the timetable is read from a specified directory. This timetable contains details about stops, trips, events, and footpaths.

**Timetable Format**:
The timetable data is organized into several CSV files, each serving a specific role in describing trips, stops, and transfer options. The code expects the following files (or a subset, as some are optional):

1. trips.csv
*Columns*:
- TripId: A unique identifier for the trip.
- StopIndex: The order in which stops are visited during the trip.
- StopId: A unique identifier for the stop where the event occurs.
- ArrivalTime: The scheduled arrival time at the stop.
- DepartureTime: The scheduled departure time from the stop.

*Notes*:
Each row represents an event (with both an arrival and a departure) at a stop. Events are later grouped into trips based on TripId and ordered by StopIndex.

2. footpaths.csv
*Columns*:
- FromStopId: The identifier of the originating stop.
- ToStopId: The identifier of the destination stop.
- TravelTime: The duration required to walk from the origin to the destination.

*Notes*:
These connections allow the algorithm to model transfers where a traveler may walk from one stop to another.

3. stops.csv (Optional)
*Columns*:
- StopId: The unique identifier for the stop.
- MinChangeTime: The minimum time needed to change services at that stop.

4. transfers.csv (Optional)
*Columns*:
- FromVertex: A vertex identifier representing the origin event (or connection point).
- ToVertex: A vertex identifier representing the destination event.

*Notes*:
This file is used to construct a graph that models transfer opportunities between different events.

### Time Expanded Graph Construction:
From the timetable data, a time expanded graph is built. Each event (arrival and departure) becomes a vertex in the graph. The graph is then augmented with:

- Stop-transfer chains: Edges between consecutive events at the same stop.
- Trip-chain edges: Edges connecting the departure event of one trip segment to the arrival event of the following segment.
- Footpath edges: Edges representing transfers between stops via footpaths.

The events per stop form a natural chain decomposition that is later used for the PPL algorithm.

### PPL Algorithm Execution:
After constructing the time expanded graph, the PPL algorithm is executed. This involves computing hub labels for both forward (outgoing) and backward (incoming) reachability queries on the DAG.

## Build Instructions
To build the project, run the provided compile.sh script:

```bash
./compile.sh
```
Alternatively, to build only the release version:

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

The project supports two build configurations:

- Release: Optimized for performance.
- Debug: Includes debug symbols and error checks.

To build and run the release versions:

```bash
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
./PPL -h
./TT_PPL -h
```

## Output File Format

The output file contains hub labels for all vertices in the graph. Each vertex has exactly two lines:

-   **`o` line** – outgoing hubs (forward labels).
-   **`i` line** – incoming hubs (backward labels).

### File Structure
1.  First line contains `V ` followed by the number of vertices.
3.  **Outgoing Hubs (`o` line)**
    -   Starts with `o`.
    -   Followed by the vertex ID.
    -   Followed by pairs of (path id, path position)
    -   Example: `o 0 1 3` → Vertex `0` has outgoing hubs path id `1` and path position `3`.
4.  **Incoming Hubs (`i` line)**
    -   Starts with `i`.
    -   Followed by the vertex ID.
    -   Followed by pairs of (path id, path position)
    -   Example: `i 0 2 4` → Vertex `0` has incoming hubs path id `2` and path position `4`.
5.  Each vertex contributes **exactly two lines** to the file (`o` and `i`).
6.  The total number of lines is `2 × number of vertices + 1`.

## Example Execution
### Standard PPL
Here is an example of running the PPL algorithm on a graph read from a DIMACS file:

```bash
./PPL -i ../example/icice.dimacs -s -b
```

Example Output:

```bash
Reading graph from dimacs ... done [406ms]
Graph Statistics:
  Number of vertices:            186068
  Number of edges:               586679
  Min degree:                    0
  Max degree:                    19
  Average degree:                3.15304
  Number of isolated vertices:   440
Reversing Graph ... done [7ms]
Computing Chain Decomposition ... done [75ms]
Path Statistics:
  Total Paths:    7876
  Min Length:     1
  Max Length:     248
  Average Length: 23.6247
Computing Path Labels ... done [7590ms]
Forward Labels Statistics:
  Min Size:     1
  Max Size:     193
  Avg Size:     47.8313
Backward Labels Statistics:
  Min Size:     1
  Max Size:     204
  Avg Size:     56.3595
FWD # count:    8899866
BWD # count:    10486706
Both # count:   19386572
Total memory consumption [megabytes]:
  82.4714
The 100000 random queries took in total 4.14783e+07 [ns] and on average 414.783 [ns]! Counter: 38960
```

### Timetable Variant (TT_PPL)
For the timetable variant, the process is as follows:

1. The timetable is loaded from a directory.
2. The time expanded graph is built using the timetable data and chain decomposition based on events per stop.
3. Finally, the PPL algorithm is applied on the constructed graph.

Run the timetable variant with:

```bash
./TT_PPL -i ../datasets/kvv/ -s -b
```

Example Output:

```bash
Loading TimeTable from file ... done [72ms]
Build Time Expanded Graph ... done [170ms]
TimeTable Statistics:
  Number of events:      214232
  Number of stops:       2740
  Number of trips:       10532
  Number of footpaths:   2774
Transfer Graph Stats:
Graph Statistics:
  Number of vertices:            428464
  Number of edges:               632192
  Min degree:                    0
  Max degree:                    11
  Average degree:                1.47548
  Number of isolated vertices:   139
Path Statistics:
  Total Paths:    2740
  Min Length:     2
  Max Length:     1620
  Average Length: 156.374
Computing Path Labels ... done [966ms]
Forward Labels Statistics:
  Min Size:     1
  Max Size:     42
  Avg Size:     12.1578
Backward Labels Statistics:
  Min Size:     1
  Max Size:     47
  Avg Size:     11.3196
FWD # count:    5209181
BWD # count:    4850034
Both # count:   10059215
Total memory consumption [megabytes]:
  57.9864
The 100000 random queries took in total 3.30743e+07 [ns] and on average 330.743 [ns]! Counter: 31397
```

## Reference
The Pruned Path Labeling algorithm is based on:
- [**"Fast and Scalable Reachability Queries on Graphs
by Pruned Labeling with Landmarks and Paths"** – Akiba et al.](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=a3d5e5d2fae76af242e40e35989237a8c3e6385f)