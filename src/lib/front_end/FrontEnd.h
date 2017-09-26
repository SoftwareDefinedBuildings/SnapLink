#pragma once

#include "lib/data/CameraModel.h"
#include "lib/data/FoundItem.h"
#include "lib/data/Transform.h"
#include <functional>
#include <memory>
#include <vector>

class FrontEnd {
public:
  virtual ~FrontEnd() = default;

  /**
   * start the front end asynchronously
   */
  virtual bool start() = 0;

  /**
   * stop front end synchronously
   */
  virtual void stop() = 0;

  /**
   * register a callback function
   */
  void registerOnQuery(std::function<std::pair<Transform, std::vector<FoundItem>>(
                           const cv::Mat &image, const CameraModel &camera)>
                           onQuery) {
    _onQuery = onQuery;
  }

  /**
   * call the callback function
   */
  std::function<std::pair<Transform, std::vector<FoundItem>>(const cv::Mat &image,
                                         const CameraModel &camera)>
  getOnQuery() {
    return _onQuery;
  }

private:
  std::function<std::pair<Transform, std::vector<FoundItem>>(const cv::Mat &image,
                                         const CameraModel &camera)>
      _onQuery;
};
