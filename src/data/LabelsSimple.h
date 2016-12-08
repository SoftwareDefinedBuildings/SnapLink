#pragma once

#include "data/Labels.h"

class LabelsSimple : public Labels {
public:
  /**
   * Add labels, ownership transfer
   */
  void putLabels(std::list<std::unique_ptr<Label>> &&labels);

  /**
   * get all labels
   */
  const std::map<int, std::list<std::unique_ptr<Label>>> &getLabels() const;

private:
  std::map<int, std::list<std::unique_ptr<Label>>> _labels;
};
