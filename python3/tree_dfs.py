#!/usr/bin/env python3
import sys
from collections import defaultdict, deque

def read_dimacs_graph(filename):
    adj = defaultdict(list)
    indeg = defaultdict(int)
    n = m = 0
    with open(filename) as f:
        for line in f:
            if not line.strip() or line.startswith("c"):
                continue
            parts = line.strip().split()
            if parts[0] == "p":
                n, m = int(parts[2]), int(parts[3])
            elif parts[0] == "a":
                u, v = int(parts[1]), int(parts[2])
                adj[u].append(v)
                indeg[v] += 1
                indeg[u] += 0  # ensure present
    return n, adj, indeg

def topological_sort(adj, indeg, n):
    """Standard Kahn topological sort."""
    q = deque([v for v in range(1, n + 1) if indeg[v] == 0])
    topo = []
    indeg = indeg.copy()
    while q:
        v = q.popleft()
        topo.append(v)
        for w in adj[v]:
            indeg[w] -= 1
            if indeg[w] == 0:
                q.append(w)
    return topo

def dfs_tree_decomposition_iterative(adj, topo, max_length=(1<<16)):
    rank = {v: i for i, v in enumerate(topo)}
    visited = set()
    bags = []
    parent = {}

    for start in topo:
        if start in visited:
            continue

        stack = [start]
        current_bag = []

        while stack and len(current_bag) < max_length - 1:
            v = stack.pop()
            if v in visited:
                continue
            visited.add(v)
            current_bag.append(v)

            for w in adj[v]:
                if w not in visited:
                    parent[w] = v
                    stack.append(w)

        bags.append(current_bag)

    return bags, parent

def main(filename):
    n, adj, indeg = read_dimacs_graph(filename)
    topo = topological_sort(adj, indeg, n)
    print(f"Topological order computed ({len(topo)} vertices).")

    bags, parent = dfs_tree_decomposition_iterative(adj, topo)
    print(f"Constructed {len(bags)} DFS trees (bags).")

    max_bag_size = max(len(b) for b in bags)
    print(f"Largest bag size: {max_bag_size}")
    print(f"Example bag (first 10 vertices): {bags[0][:10]}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <dimacs_graph_file>")
        sys.exit(1)
    main(sys.argv[1])

