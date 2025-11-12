#!/usr/bin/env python3
import sys
from collections import defaultdict, deque

def read_dimacs_graph(filename):
    """Reads a DIMACS shortest path format graph (directed)."""
    n = m = 0
    adj = defaultdict(list)
    indeg = defaultdict(int)
    with open(filename) as f:
        for line in f:
            if not line.strip() or line[0] == 'c':
                continue  # comment or empty
            parts = line.strip().split()
            if parts[0] == 'p':
                # e.g. p sp 2527390 7955735
                n, m = int(parts[2]), int(parts[3])
            elif parts[0] == 'a':
                # e.g. a 1 2 0
                u, v = int(parts[1]), int(parts[2])
                adj[u].append(v)
                indeg[v] += 1
                # ensure nodes appear even if isolated
                _ = adj[u]; _ = indeg[u]
    return n, adj, indeg

def count_indegree_zero(indeg, n):
    """Counts vertices with indegree == 0."""
    count = sum(1 for v in range(1, n+1) if indeg[v] == 0)
    return count

def topological_sort(adj, indeg, n):
    """Computes a topological ordering of DAG using Kahn's algorithm."""
    q = deque([v for v in range(1, n+1) if indeg[v] == 0])
    topo = []
    while q:
        v = q.popleft()
        topo.append(v)
        for w in adj[v]:
            indeg[w] -= 1
            if indeg[w] == 0:
                q.append(w)
    if len(topo) != n:
        print("Warning: graph has a cycle (not a DAG)")
    return topo

def greedy_path_decomposition(adj, topo):
    """Greedy path decomposition using topological order."""
    used = set()
    chains = []
    pos = {v: i for i, v in enumerate(topo)}  # position in topo order

    for v in topo:
        if v in used:
            continue
        chain = [v]
        used.add(v)
        current = v
        while True:
            # pick next unused successor with smallest topo index
            next_candidates = [w for w in adj[current] if w not in used]
            if not next_candidates:
                break
            w = min(next_candidates, key=lambda x: pos[x])
            chain.append(w)
            used.add(w)
            current = w
        chains.append(chain)
    return chains

def main(filename):
    n, adj, indeg = read_dimacs_graph(filename)
    indeg_zero_count = count_indegree_zero(indeg, n)
    print(f"Number of vertices with indegree 0: {indeg_zero_count}")

    # Copy indeg (Kahn modifies it)
    indeg_copy = indeg.copy()
    topo = topological_sort(adj, indeg_copy, n)
    print(f"Topological order computed, length = {len(topo)}")

    chains = greedy_path_decomposition(adj, topo)
    print(f"Path decomposition: {len(chains)} chains")
    avg_len = sum(len(c) for c in chains) / len(chains)
    print(f"Average chain length: {avg_len:.2f}")

    # Optional: print first few chains
    for i, chain in enumerate(chains[:5]):
        print(f"  Chain {i+1}: {chain[:10]}{'...' if len(chain)>10 else ''}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <dimacs_graph_file>")
        sys.exit(1)
    main(sys.argv[1])

