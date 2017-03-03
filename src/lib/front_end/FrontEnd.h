#pragma once

#include <functional>
#include <vector>
#include <memory>
#include "lib/data/CameraModel.h"

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
  void registerOnQuery(std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> onQuery) {_onQuery = onQuery;}

  /**
   * call the callback function
   */
  std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> getOnQuery() {return _onQuery;}

private:
  std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> _onQuery;
};
