# Pruned Path Labeling (PPL)
Author: Patrick Steil

This repository implements a hub labeling algorithm for efficient reachability queries on directed acyclic graphs (DAGs).

## Overview

The events per stop form a natural chain decomposition that is later used for the PPL algorithm.

### PPL Algorithm Execution:
This involves computing hub labels for both forward (outgoing) and backward (incoming) reachability queries on the DAG.

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
### PPL + SIMD + Parallel + Compression
Here is an example of running the PPL algorithm on a graph read from a DIMACS file:

```bash
./PPL -b -i ../datasets/swiss_te.dimacs -p ../datasets/swiss_paths.txt -c -s
```

Example Output:

```bash
Reading graph from dimacs ... done [4203ms]
Graph Statistics:
  Number of vertices:            5860345
  Number of edges:               12713790
  Min degree:                    0
  Max degree:                    30
  Average degree:                2.16946
  Number of isolated vertices:   1503
Build and sort edges topologically ... done [1612ms]
Compute Path Stats ... Path Statistics:
  Total Paths:    29045
  Min Length:     1
  Max Length:     5020
  Average Length: 201.768
done [1ms]
Computing Iterative Centrality ... done [106904ms]
Computing Path Labels ... done [42412ms]
Compute Label Stats ... Forward Labels Statistics:
  Min Size:     1
  Max Size:     79
  Avg Size:     32.4293
Backward Labels Statistics:
  Min Size:     1
  Max Size:     79
  Avg Size:     32.0704
FWD # count:    190046825
BWD # count:    187943558
Both # count:   377990383
Total memory consumption [megabytes]:
  1799.61
done [278ms]
100000 queries: total 3.83034e+07 ns, avg 383.034 ns/query, counter=36558
Loading labels into DeltaFlatBuffer ... done [4194ms]
FWD Memory: 593.572 MB
BWD Memory: 587.39 MB
Total Memory: 1180.96 MB
100000 queries: total 4.57595e+07 ns, avg 457.595 ns/query, counter=36558
```

## Reference
The Pruned Path Labeling algorithm is based on:
- [**"Fast and Scalable Reachability Queries on Graphs
by Pruned Labeling with Landmarks and Paths"** – Akiba et al.](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=a3d5e5d2fae76af242e40e35989237a8c3e6385f)
