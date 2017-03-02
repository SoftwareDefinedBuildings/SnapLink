#pragma once

#include "lib/data/Label.h"
#include <list>
#include <map>
#include <memory>

class Labels {
public:
  virtual ~Labels() = default;

  /**
   * Add labels, ownership transfer
   */
  virtual void putLabels(std::list<std::unique_ptr<Label>> &&labels) = 0;

  /**
   * get all labels
   */
  virtual const std::map<int, std::list<std::unique_ptr<Label>>> &
  getLabels() const = 0;
};
