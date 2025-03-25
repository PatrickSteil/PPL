/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#include "../datastructures/graph.h"

#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "../datastructures/topological_sort.h"

void writeTestFile(const std::string &fileName, const std::string &content) {
  std::ofstream file(fileName);
  file << content;
  file.close();
}

class GraphTest : public ::testing::Test {
 protected:
  void SetUp() override {
    writeTestFile("test_edge_list.txt",
                  "1 2\n"
                  "1 3\n"
                  "2 3\n"
                  "3 4\n"
                  "4 1\n");

    writeTestFile("test_dimacs.txt",
                  "c This is a test DIMACS graph file\n"
                  "c with comments\n"
                  "p edge 4 5\n"
                  "a 1 2\n"
                  "a 1 3\n"
                  "a 2 3\n"
                  "a 3 4\n"
                  "a 4 1\n");

    writeTestFile("test_dag.txt",
                  "5 6\n"
                  "1 2\n"
                  "1 3\n"
                  "3 4\n"
                  "4 5\n"
                  "2 4\n"
                  "2 5\n");

    writeTestFile("test_cycle.txt",
                  "4 4\n"
                  "1 2\n"
                  "2 3\n"
                  "3 4\n"
                  "4 1");
  }

  void TearDown() override {
    std::remove("test_edge_list.txt");
    std::remove("test_dimacs.txt");
    std::remove("test_dag.txt");
    std::remove("test_cycle.txt");
  }
};

TEST_F(GraphTest, ReadFromEdgeList) {
  Graph graph;
  graph.readFromEdgeList("test_edge_list.txt");

  EXPECT_EQ(graph.numVertices(), 4);
  EXPECT_EQ(graph.numEdges(), 5);

  EXPECT_EQ(graph.beginEdge(0), 0);
  EXPECT_EQ(graph.endEdge(0), 2);

  EXPECT_EQ(graph.beginEdge(1), 2);
  EXPECT_EQ(graph.endEdge(1), 3);

  EXPECT_EQ(graph.beginEdge(2), 3);
  EXPECT_EQ(graph.endEdge(2), 4);

  EXPECT_EQ(graph.beginEdge(3), 4);
  EXPECT_EQ(graph.endEdge(3), 5);
}

TEST_F(GraphTest, ReadFromDimacs) {
  Graph graph;
  graph.readDimacs("test_dimacs.txt");

  EXPECT_EQ(graph.numVertices(), 4);
  EXPECT_EQ(graph.numEdges(), 5);

  EXPECT_EQ(graph.beginEdge(0), 0);
  EXPECT_EQ(graph.endEdge(0), 2);

  EXPECT_EQ(graph.beginEdge(1), 2);
  EXPECT_EQ(graph.endEdge(1), 3);

  EXPECT_EQ(graph.beginEdge(2), 3);
  EXPECT_EQ(graph.endEdge(2), 4);

  EXPECT_EQ(graph.beginEdge(3), 4);
  EXPECT_EQ(graph.endEdge(3), 5);
}

TEST_F(GraphTest, ReverseGraph) {
  Graph graph;
  graph.readFromEdgeList("test_edge_list.txt");
  Graph reversed = graph.reverseGraph();

  EXPECT_EQ(reversed.numVertices(), graph.numVertices());
  EXPECT_EQ(reversed.numEdges(), graph.numEdges());

  EXPECT_EQ(reversed.beginEdge(0), 0);
  EXPECT_EQ(reversed.endEdge(0), 1);

  EXPECT_EQ(reversed.beginEdge(1), 1);
  EXPECT_EQ(reversed.endEdge(1), 2);

  EXPECT_EQ(reversed.beginEdge(2), 2);
  EXPECT_EQ(reversed.endEdge(2), 4);

  EXPECT_EQ(reversed.beginEdge(3), 4);
  EXPECT_EQ(reversed.endEdge(3), 5);
}

TEST_F(GraphTest, TopologicalSortValidDAG) {
  Graph graph;
  graph.readFromEdgeList("test_dag.txt");

  TopologicalSort topoSort(graph);

  const std::vector<Vertex> &order = topoSort.ordering;
  EXPECT_EQ(order.size(), graph.numVertices());

  std::vector<std::size_t> position(graph.numVertices());
  for (std::size_t i = 0; i < order.size(); ++i) {
    position[order[i]] = i;
  }
  for (Vertex u = 0; u < graph.numVertices(); ++u) {
    for (std::size_t edge = graph.beginEdge(u); edge < graph.endEdge(u);
         ++edge) {
      Vertex v = graph.toVertex[edge];
      EXPECT_LT(position[u], position[v]);
    }
  }
}

TEST_F(GraphTest, TopologicalSortCycle) {
  Graph graph;
  graph.readFromEdgeList("test_cycle.txt");

  EXPECT_THROW(TopologicalSort topoSort(graph), std::runtime_error);
}
