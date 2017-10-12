#pragma once

#include "lib/data/CameraModel.h"
#include "lib/data/FoundItem.h"
#include "lib/data/Transform.h"
#include "lib/data/Label.h"
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
   * register a callback function for identify
   */
  void registerOnIdentify(std::function<std::pair<Transform, std::vector<FoundItem>>(
                           const cv::Mat &image, const CameraModel &camera)>
                           onIdentify) {
    _onIdentify = onIdentify;
  }
  
  /**
   * register a callback function for getLabels
   */
  void registerOnGetLabels(std::function<std::map<int, std::vector<Label>>()>
                           onGetLabels) {
    _onGetLabels = onGetLabels;
  }



  /**
   * call the identify callback function
   */
  std::function<std::pair<Transform, std::vector<FoundItem>>(const cv::Mat &image,
                                         const CameraModel &camera)>
  getOnIdentify() {
    return _onIdentify;
  }

 
  /**
   * call the getLabels callback function
   */
  std::function<std::map<int, std::vector<Label>>()>
  getOnGetLabels() {
    return _onGetLabels;
  }


private:
  std::function<std::pair<Transform, std::vector<FoundItem>>(const cv::Mat &image,
                                         const CameraModel &camera)>
      _onIdentify;

  std::function<std::map<int, std::vector<Label>>()>
      _onGetLabels;
};
