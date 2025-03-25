/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#include <stdlib.h>

#include <cstdlib>
#include <iostream>
#include <numeric>
#include <thread>
#include <utility>

#include "datastructures/graph.h"
#include "datastructures/ppl.h"
#include "external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_graph", "Input graph file.");
  parser.set_optional<std::string>("p", "path_file", "", "File to write the path decomposition into.");
  parser.set_optional<std::string>("o", "output_file", "",
                                   "Output file to save hub labels into.");
  parser.set_optional<int>("t", "num_threads",
                           std::thread::hardware_concurrency(),
                           "Number of threads to use.");
  parser.set_optional<bool>("s", "show_stats", false,
                            "Show statistics about the computed hub labels.");
  parser.set_optional<bool>("b", "benchmark", false,
                            "Run 100 000 random vertex-to-vertex queries.");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string outputFileName = parser.get<std::string>("o");
  const std::string outputPathFile = parser.get<std::string>("p");
  const bool showstats = parser.get<bool>("s");
  const bool run_benchmark = parser.get<bool>("b");

  Graph g;
  g.readDimacs(inputFileName);

  if (showstats) g.showStats();

  Graph bwdGraph = g.reverseGraph();

  PPL ppl(&g, &bwdGraph);

  ppl.paths = loadPathFile(outputPathFile);
  ppl.sortPaths();

  // ppl.chainDecomposition();

  if (showstats) ppl.showPathStats();

  // if (outputPathFile != "") saveToFile(ppl.paths, outputPathFile);

  ppl.run();

  if (showstats) ppl.showStats();

  if (outputFileName != "") saveToFile(ppl.labels, outputFileName);

  if (run_benchmark) benchmark_pathlabels(ppl.labels);

  return 0;
}
