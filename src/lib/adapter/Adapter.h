#pragma once

#include <set>

class Labels;
class Words;
class Images;

class Adapter final {
public:
  virtual ~Adapter() = default;

  virtual bool init(const std::set<std::string> &dbPaths) = 0;

  virtual const Words &getWords() = 0;

  virtual const Images &getImages() = 0;

  virtual const Labels &getLabels() = 0;
};
