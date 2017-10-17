#pragma once

#include "lib/data/CameraModel.h"
#include "lib/data/FoundItem.h"
#include "lib/data/Transform.h"
#include "lib/data/Label.h"
#include <functional>
#include <memory>
#include <vector>

// TODO: items are for test purpose only, remove remove in future releases
typedef std::function<std::pair<int, Transform>(const cv::Mat &image, const CameraModel &camera, std::vector<FoundItem> *items)> LocalizeFunc;
typedef std::function<std::map<int, std::vector<Label>>()> GetLabelsFunc;

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
   * register a callback function for localization
   */
  void registerLocalizeFunc(LocalizeFunc localizeFunc) {
    _localizeFunc = localizeFunc;
  }
  
  /**
   * register a callback function for getLabels of all databases
   */
  void registerGetLabelsFunc(GetLabelsFunc getLabelsFunc) {
    _getLabelsFunc = getLabelsFunc;
  }

  /**
   * call the localize callback function
   */
  LocalizeFunc localizeFunc() {
    return _localizeFunc;
  }

  /**
   * call the getLabels callback function
   */
  GetLabelsFunc getLabelsFunc() {
    return _getLabelsFunc;
  }


private:
  LocalizeFunc _localizeFunc;
  GetLabelsFunc _getLabelsFunc;
};
