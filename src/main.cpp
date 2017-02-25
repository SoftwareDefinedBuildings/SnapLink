#include "run/run.h"
#include <cstdio>

#define CMD_RUN "run"

int main(int argc, char *argv[]) {

  if (argc <= 1) {
    return 1;
  }
  
  if (std::string(argv[1]) == CMD_RUN) {
    return run(argc - 1, argv + 1);
  }

  return 1;
}
