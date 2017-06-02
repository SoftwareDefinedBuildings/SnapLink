#include "lib/algo/Visibility.h"
#include "lib/data/CameraModel.h"
#include "lib/data/Transform.h"
#include "lib/data/FoundItem.h"
#include "lib/data/Label.h"
#include "lib/util/Utility.h"
#include <fstream>
#include <iostream>
#include <opencv/cv.h>
#include <pcl/point_types.h>

Visibility::Visibility(const std::map<int, std::vector<Label>> &labels)
    : _labels(labels) {}

std::vector<FoundItem> Visibility::process(int dbId,
                                             const CameraModel &camera,
                                             const Transform &pose) const {
  std::vector<cv::Point3f> points;
  std::vector<std::string> names;
  for (auto &label : _labels.at(dbId)) {
    points.emplace_back(label.getPoint3());
    names.emplace_back(label.getName());
  }

  std::vector<FoundItem> results;

  std::cout << "processing transform" << std::endl << pose << std::endl;

  std::vector<cv::Point2f> planePoints;
  std::vector<std::string> visibleLabels;

  cv::Mat K = camera.K();
  cv::Mat R = (cv::Mat_<double>(3, 3) << (double)pose.r11(), (double)pose.r12(),
               (double)pose.r13(),                                         //
               (double)pose.r21(), (double)pose.r22(), (double)pose.r23(), //
               (double)pose.r31(), (double)pose.r32(), (double)pose.r33());
  cv::Mat rvec(1, 3, CV_64FC1);
  cv::Rodrigues(R, rvec);
  cv::Mat tvec = (cv::Mat_<double>(1, 3) << (double)pose.x(), (double)pose.y(),
                  (double)pose.z());

  // do the projection
  cv::projectPoints(points, rvec, tvec, K, cv::Mat(), planePoints);

  // find points in the image
  int width = camera.getImageSize().width;
  int height = camera.getImageSize().height;
  std::map<std::string, std::vector<double>> distances;
  std::map<std::string, std::vector<cv::Point2f>> labelPoints;
  cv::Point2f center(width / 2, height / 2);

  for (unsigned int i = 0; i < points.size(); ++i) {
    std::string name = names[i];
    // if (uIsInBounds(int(planePoints[i].x), 0, width) &&
    //        uIsInBounds(int(planePoints[i].y), 0, height))
    if (true) {
      if (Utility::isInFrontOfCamera(points[i], pose)) {
        visibleLabels.emplace_back(name);
        double dist = cv::norm(planePoints[i] - center);
        distances[name].emplace_back(dist);
        labelPoints[name].emplace_back(planePoints[i]);
        std::cout << "Find label " << name << " at (" << planePoints[i].x << ","
                  << planePoints[i].y << ")" << std::endl;
      } else {
        std::cout << "Label " << name << " invalid at (" << planePoints[i].x
                  << "," << planePoints[i].y << ")"
                  << " because it is from the back of the camera" << std::endl;
      }
    } else {
      std::cout << "label " << name << " invalid at (" << planePoints[i].x
                << "," << planePoints[i].y << ")" << std::endl;
    }
  }

  std::string minlabel;
  if (!distances.empty()) {
    // find the label with minimum mean distance
    std::pair<std::string, std::vector<double>> minDist =
        *min_element(distances.begin(), distances.end(), CompareMeanDist());
    std::string minlabel = minDist.first;
    std::cout << "Nearest label " << minlabel << " with mean distance "
              << CompareMeanDist::meanDist(minDist.second) << std::endl;
    //results.emplace_back(minlabel);
  } else {
    std::cout << "No label is qualified" << std::endl;
  }

  //TODO: right now x and y are hardcoded, and it only return 1 nearst result, need to modify it to return multiple, as well as position
  results.push_back(FoundItem(minlabel, -1, -1));
  return results;
}

double CompareMeanDist::meanDist(const std::vector<double> &vec) {
  double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  double mean = sum / vec.size();
  return mean;
}

bool CompareMeanDist::operator()(const PairType &left,
                                 const PairType &right) const {
  return meanDist(left.second) < meanDist(right.second);
}
