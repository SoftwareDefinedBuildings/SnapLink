#pragma once

#include "lib/data/Labels.h"
#include <memory>
#include <numeric>

class CameraModel;
class Transform;

class Visibility final {
public:
  explicit Visibility(std::unique_ptr<Labels> &&labels);

  std::vector<std::string> process(int dbId, const CameraModel &camera,
                                   const Transform &pose) const;

private:
  std::unique_ptr<Labels> _labels;
};

struct CompareMeanDist {
  typedef std::pair<std::string, std::vector<double>> PairType;

  static double meanDist(const std::vector<double> &vec);
  bool operator()(const PairType &left, const PairType &right) const;
};
