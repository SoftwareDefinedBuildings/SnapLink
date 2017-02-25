#pragma once

#include <string>
#include <vector>

int run(int argc, char *argv[]);
void showUsage();
void parseOpt(int argc, char *argv[], bool &http, bool &bosswave,
              int &sampleSize, int &corrSize, double &distRatio,
              std::vector<std::string> &dbfiles);
