#pragma once

#include <cassert>
#include <list>
#include <stdexcept>

#include "types.h"

struct Chain {
 private:
  std::list<Vertex> chain;

 public:
  Chain() = default;

  void push(const Vertex& n) { chain.push_back(n); }

  std::size_t size() const { return chain.size(); }

  bool empty() const { return chain.empty(); }

  const std::list<Vertex>& getVertices() const { return chain; }

  const Vertex& getLastVertex() const {
    assert(!chain.empty());
    return chain.back();
  }
};

using Chains = std::list<Chain>;