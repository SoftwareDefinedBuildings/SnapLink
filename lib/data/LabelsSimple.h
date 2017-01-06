#pragma once

#include "data/Labels.h"

class LabelsSimple final : public Labels {
public:
  /**
   * Add labels, ownership transfer
   */
  void putLabels(std::list<std::unique_ptr<Label>> &&labels) final;

  /**
   * get all labels
   */
  const std::map<int, std::list<std::unique_ptr<Label>>> &getLabels() const final;

private:
  std::map<int, std::list<std::unique_ptr<Label>>> _labels;
};
