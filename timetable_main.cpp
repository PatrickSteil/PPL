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

#include "datastructures/ppl.h"
// #include "datastructures/ppl_bit.h"
#include "datastructures/timetable.h"
#include "external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_timetable",
                                   "Input TimeTable directory.");
  parser.set_optional<std::string>("o", "output_file", "",
                                   "Output file to save hub labels into.");
  parser.set_optional<std::string>(
      "d", "output_dimacs_teg", "",
      "Output file to save the time exapnded graph as DIMACS.");
  parser.set_optional<std::string>(
      "p", "output_path_decomposition", "",
      "Output file to save the path decomposition into.");
  parser.set_optional<int>("t", "num_threads",
                           std::thread::hardware_concurrency(),
                           "Number of threads to use.");
  parser.set_optional<bool>("c", "contract_graph", false,
                            "Contract vertices in the TEG which have the same "
                            "time at the same stop.");
  parser.set_optional<bool>("s", "show_stats", false,
                            "Show statistics about the computed hub labels.");
  parser.set_optional<bool>("b", "benchmark", false,
                            "Run 100000 random vertex-to-vertex queries.");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(
      argc, argv,
      "Timetable Pruned Path Labeling (TT_PPL).\nThe paths choosen are all "
      "events (arrival- and departureevents) at a stop sorted by time.\nThe "
      "paths are processed by importance (here: weighted sum of "
      "vertices).\nThis program can compute pathlabels, save them, execute "
      "random queries "
      "and show the overall memory consumption.");
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string outputFileName = parser.get<std::string>("o");
  const std::string outputFileNameDimacs = parser.get<std::string>("d");
  const std::string outputFileNamePathDecomposition =
      parser.get<std::string>("p");
  const int numThreads = parser.get<int>("t");
  const bool contract = parser.get<bool>("c");
  const bool showstats = parser.get<bool>("s");
  const bool run_benchmark = parser.get<bool>("b");

  TimeTable tt(inputFileName, numThreads);

  tt.buildTEGraph();

  if (contract) tt.contract();
  if (showstats) tt.showStats();

  PPL ppl(&tt.transferGraphs[FWD], &tt.transferGraphs[BWD], numThreads);
  ppl.paths = tt.eventsOfStop;

  // ppl.reorderByRank();
  ppl.sortPaths(RankingMethod::SUM);

  if (showstats) ppl.showPathStats();

  ppl.run();

  if (showstats) ppl.showStats();

  if (outputFileName != "") saveToFile(ppl.labels, outputFileName);

  if (run_benchmark) benchmark_pathlabels(ppl.labels);

  if (outputFileNameDimacs != "")
    tt.transferGraphs[FWD].toDimacs(outputFileNameDimacs);

  if (outputFileNamePathDecomposition != "")
    saveToFile(tt.eventsOfStop, outputFileNamePathDecomposition);

  return 0;
}