#pragma once

#include <boost/program_options.hpp>

namespace po = boost::program_options;

class Vis final {
public:
  int run(int argc, char *argv[]);

private:
  static void printInvalid(const std::vector<std::string> &opts);
  static void printUsage(const po::options_description &desc);
};
