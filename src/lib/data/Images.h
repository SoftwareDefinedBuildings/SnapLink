#pragma once

#include "lib/data/Image.h"
#include <list>
#include <map>
#include <memory>

class Images {
public:
  virtual ~Images() = default;

  /**
   * Add images, ownership transfer
   */
  virtual void putImages(std::list<std::unique_ptr<Image>> &&images) = 0;

  /**
   * get all images
   */
  virtual const std::map<int, std::list<std::unique_ptr<Image>>> &
  getImages() const = 0;
};
