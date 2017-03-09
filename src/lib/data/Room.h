#pragma once

#include "lib/data/Word.h"
#include <map>
#include <vector>
#include <set>
#include <opencv2/core/core.hpp>

class Room final {
public:
  explicit Room(int id);

  int getId() const;
  void addWordIds(std::vector<int> &&wordIds);
  const std::set<int> &getWordIds() const;

private:
  int _id;
  std::set<int> _wordIds;
};
