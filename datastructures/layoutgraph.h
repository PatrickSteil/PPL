/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <cassert>

#include "graph.h"

template <typename PPL_CLASS>
static Graph createLayoutGraph(PPL_CLASS &ppl) {
  assert(!ppl.paths.empty());

  const std::size_t numVertices = ppl.paths.size();

  std::vector<Vertex> lookupVertices(ppl.graphs[FWD]->numVertices(), 0);
  std::vector<std::pair<Vertex, Vertex>> edges;
  edges.reserve(numVertices * 2);

  for (Vertex pId = 0; pId < numVertices; ++pId) {
    for (const auto &vertex : ppl.paths[pId]) {
      assert(vertex < lookupVertices.size());
      lookupVertices[vertex] = pId;
    }
  }

  ppl.graphs[FWD]->doForAllEdges([&](const Vertex from, const Vertex to) {
    if (lookupVertices[from] == lookupVertices[to]) return;

    edges.emplace_back(lookupVertices[from], lookupVertices[to]);
    edges.emplace_back(lookupVertices[to], lookupVertices[from]);
  });

  Graph layoutGraph;
  layoutGraph.buildFromEdgeList(edges, numVertices);

  return layoutGraph;
}
