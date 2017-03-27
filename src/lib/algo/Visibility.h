#pragma once

#include "lib/data/Label.h"
#include <list>
#include <map>
#include <memory>
#include <numeric>

class CameraModel;
class Transform;

class Visibility final {
public:
  explicit Visibility(const std::map<int, std::vector<Label>> &labels);

  std::vector<std::string> process(int dbId, const CameraModel &camera,
                                   const Transform &pose) const;

private:
  const std::map<int, std::vector<Label>> &_labels;
};

struct CompareMeanDist {
  typedef std::pair<std::string, std::vector<double>> PairType;

  static double meanDist(const std::vector<double> &vec);
  bool operator()(const PairType &left, const PairType &right) const;
};
