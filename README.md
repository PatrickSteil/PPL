# Pruned Path Labeling (PPL)

Author: Patrick Steil

This repository implements a hub labeling algorithm for reachability queries on directed acyclic graphs (DAGs).

## Build Instructions

To build the project, run the provided `compile.sh` script:

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

-   **Release:** Optimized for performance.
-   **Debug:** Includes debug symbols and error checks.

To build and run the release version:

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
```bash
./PPL -i ../example/icice.dimacs -s -b
```

**Example Output:**
```
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

## Reference

This project is based on:
- [**"Fast and Scalable Reachability Queries on Graphs
by Pruned Labeling with Landmarks and Paths"** – Akiba et al.](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=a3d5e5d2fae76af242e40e35989237a8c3e6385f)
