/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <cassert>
#include <queue>
#include <stdexcept>
#include <vector>

#include "../external/status_log.h"
#include "graph.h"
#include "utils.h"

struct TopologicalSort {
  std::vector<Vertex> ordering;

  TopologicalSort(const Graph& graph) {
    const std::size_t numVertices = graph.numVertices();
    ordering.reserve(numVertices);

    std::vector<std::size_t> inDegree(numVertices, 0);

    for (Vertex v = 0; v < numVertices; ++v) {
      for (std::size_t edge = graph.beginEdge(v); edge < graph.endEdge(v);
           ++edge) {
        ++inDegree[graph.toVertex[edge]];
      }
    }

    std::queue<Vertex> zeroInDegree;
    for (Vertex v = 0; v < numVertices; ++v) {
      if (inDegree[v] == 0) {
        zeroInDegree.push(v);
      }
    }

    while (!zeroInDegree.empty()) {
      Vertex v = zeroInDegree.front();
      zeroInDegree.pop();
      ordering.push_back(v);

      for (std::size_t edge = graph.beginEdge(v); edge < graph.endEdge(v);
           ++edge) {
        Vertex neighbor = graph.toVertex[edge];
        --inDegree[neighbor];
        if (inDegree[neighbor] == 0) {
          zeroInDegree.push(neighbor);
        }
      }
    }

    if (ordering.size() != numVertices) {
      throw std::runtime_error("The graph is not a DAG (cycle detected)");
    }
  }

  Vertex& operator[](size_t index) {
    assert(index < ordering.size() && "Index out of bounds in PathLabel");
    return ordering[index];
  }

  const Vertex& operator[](size_t index) const {
    assert(index < ordering.size() && "Index out of bounds in PathLabel");
    return ordering[index];
  }

  std::vector<Vertex>& getOrdering() { return ordering; }
};
