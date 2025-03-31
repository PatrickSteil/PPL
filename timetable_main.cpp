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
#include "datastructures/timetable.h"
#include "external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_timetable",
                                   "Input TimeTable directory.");
  parser.set_optional<std::string>("o", "output_file", "",
                                   "Output file to save hub labels into.");
  parser.set_optional<int>("t", "num_threads",
                           std::thread::hardware_concurrency(),
                           "Number of threads to use.");
  parser.set_optional<bool>("c", "contract_graph", false,
                            "Contract vertices in the TEG which have the same "
                            "time at the same stop.");
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

  ppl.sortPaths(RankingMethod::SUM);

  if (showstats) ppl.showPathStats();

  ppl.run();

  if (showstats) ppl.showStats();

  if (outputFileName != "") saveToFile(ppl.labels, outputFileName);

  if (run_benchmark) benchmark_pathlabels(ppl.labels);

  return 0;
}