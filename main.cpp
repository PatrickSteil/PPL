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

#include "cmdparser.hpp"
#include "compressed_labels.h"
#include "csv_status_log.h"
#include "graph.h"
#include "ppl.h"
#include "ppl_simd.h"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_graph", "Input graph file.");
  parser.set_required<std::string>("p", "path_file",
                                   "File to read the path decomposition from.");
  parser.set_optional<std::string>("o", "output_file", "",
                                   "Output file to save hub labels into.");
  parser.set_optional<std::string>("l", "log_file", "status_ppl.log",
                                   "Output file to write the logs into.");
  parser.set_optional<int>("t", "num_threads",
                           std::thread::hardware_concurrency(),
                           "Number of threads to use.");
  parser.set_optional<bool>("s", "show_stats", false,
                            "Show statistics about the computed hub labels.");
  parser.set_optional<bool>("b", "benchmark", false,
                            "Run 100000 random vertex-to-vertex queries.");
  parser.set_optional<bool>("c", "compress", false,
                            "Evaluate compressed DeltaFlatBuffers.");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(
      argc, argv,
      "Pruned Path Labeling (PPL).\nThe "
      "paths are processed by importance (here: weighted sum of "
      "vertices).\nThis program can compute pathlabels, execute random queries "
      "and show the overall memory consumption.");
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string inputPathFile = parser.get<std::string>("p");
  const std::string outputFileName = parser.get<std::string>("o");
  const std::string outputLogFileName = parser.get<std::string>("l");
  const int numThreads = parser.get<int>("t");
  const bool showstats = parser.get<bool>("s");
  const bool run_benchmark = parser.get<bool>("b");
  const bool eval_compressed = parser.get<bool>("c");

  CSVStatusLog::init(outputLogFileName);

  Graph g;
  g.readDimacs(inputFileName);

  if (showstats) g.showStats();

  Graph bwdGraph = g.reverseGraph();

  PPLSimd<simd<256>> ppl(&g, &bwdGraph, numThreads);
  /* PPL ppl(&g, &bwdGraph, numThreads); */

  ppl.paths = loadPathFile(inputPathFile);

  ppl.sortPaths();

  if (showstats) ppl.showPathStats();

  ppl.run();

  if (showstats) ppl.showStats();

  if (outputFileName != "") saveToFile(ppl.labels, outputFileName);

  if (run_benchmark) benchmark_pathlabels(ppl.labels);

  if (eval_compressed) {
    CompressedLabels comp;

    comp.loadLabels(ppl.labels);
    comp.showStats();

    if (run_benchmark) benchmark_compressedlabels(comp);
  }
  return 0;
}
