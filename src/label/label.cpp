#include "label/widget.h"
#include <QApplication>
#include <QResource>
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;

static void printInvalid(const std::vector<std::string> &opts);
static void printUsage(const po::options_description &desc);

int label(int argc, char *argv[]) {
  // Parse arguments
  std::string dbFile;

  po::options_description label("command options");
  label.add_options() // use comment to force new line using formater
      ("help,h", "print help message") //
      ("dbfile", po::value<std::string>(&dbFile)->required(), "database file");

  po::positional_options_description pos;
  pos.add("dbfile", 1);

  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv)
                                  .options(label)
                                  .positional(pos)
                                  .allow_unregistered()
                                  .run();
  po::store(parsed, vm);
  po::notify(vm);

  // print invalid options
  std::vector<std::string> unrecog =
      collect_unrecognized(parsed.options, po::exclude_positional);
  if (unrecog.size() > 0) {
    printInvalid(unrecog);
    printUsage(label);
    return 1;
  }

  if (vm.count("help")) {
    printUsage(label);
    return 0;
  }

  // Run the program
  QResource::registerResource("label_dot.rcc");

  QApplication a(argc, argv);
  Widget w;

  // open database
  if (!w.init(dbFile)) {
    return 1;
  }

  w.show();

  return a.exec();
}

static void printInvalid(const std::vector<std::string> &opts) {
  std::cerr << "invalid options: ";
  for (const auto &opt : opts) {
    std::cerr << opt << " ";
  }
  std::cerr << std::endl;
}

static void printUsage(const po::options_description &desc) {
  std::cout << "cellmate label [command options]" << std::endl
            << std::endl
            << desc << std::endl;
}
