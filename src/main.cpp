#include "label/label.h"
#include "run/run.h"
#include "vis/vis.h"
#include <boost/program_options.hpp>
#include <iostream>

#define VERSION "0.1"

namespace po = boost::program_options;

static void printInvalid(const std::vector<std::string> &opts);
static void printUsage(const po::options_description &desc);

int main(int argc, char *argv[]) {
  if (argc > 1) {
    if (std::string(argv[1]) == "run") {
      Run run;
      return run.run(argc - 1, argv + 1);
    } else if (std::string(argv[1]) == "dist") {
      // TODO
    } else if (std::string(argv[1]) == "vis") {
      return vis(argc - 1, argv + 1);
    } else if (std::string(argv[1]) == "label") {
      return label(argc - 1, argv + 1);
    }
  }

  po::options_description global("global options");
  global.add_options() // use comment to force new line using formater
      ("help,h", "print help message") //
      ("version", "print version number");

  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv)
                                  .options(global)
                                  .allow_unregistered()
                                  .run();
  po::store(parsed, vm);

  // print invalid options
  std::vector<std::string> unrecog =
      collect_unrecognized(parsed.options, po::exclude_positional);
  if (unrecog.size() > 0) {
    printInvalid(unrecog);
    printUsage(global);
    return 1;
  }

  if (vm.count("help")) {
    printUsage(global);
    return 0;
  }

  // check whether required options exist after handling help
  po::notify(vm);

  if (vm.count("version")) {
    std::cout << "cellmate version " << VERSION << std::endl;
    return 0;
  }

  std::cout << global << std::endl;
  return 1;
}

static void printInvalid(const std::vector<std::string> &opts) {
  std::cerr << "invalid options: ";
  for (const auto &opt : opts) {
    std::cerr << opt << " ";
  }
  std::cerr << std::endl;
}

static void printUsage(const po::options_description &desc) {
  std::cout << "cellmate [global options] command [command options]"
            << std::endl
            << std::endl
            << desc << std::endl
            << "commands:" << std::endl
            << "  run        run cellmate" << std::endl
            << "  dist       run a stage of cellmate" << std::endl
            << "  vis        visualize a datobase" << std::endl
            << "  label      label a database" << std::endl;
}
