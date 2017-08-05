#include "lib/algo/Visibility.h"
#include "lib/data/CameraModel.h"
#include "lib/data/FoundItem.h"
#include "lib/data/Label.h"
#include "lib/data/Transform.h"
#include "lib/util/Utility.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

Visibility::Visibility(const std::map<int, std::vector<Label>> &labels)
    : _labels(labels) {}

std::vector<FoundItem> Visibility::process(int dbId, const CameraModel &camera,
                                           const Transform &pose) const {
  std::vector<FoundItem> results;
  if (_labels.at(dbId).empty()) {
    return results;
  }

  std::vector<cv::Point3f> points;
  std::vector<std::string> names;
  for (auto &label : _labels.at(dbId)) {
    points.emplace_back(label.getPoint3());
    names.emplace_back(label.getName());
  }

  std::cout << "processing transform" << std::endl << pose << std::endl;

  std::vector<cv::Point2f> planePoints;

  cv::Mat K = camera.K();
  // change the base coordiate of the transform from the world coordinate to the
  // image coordinate
  Transform poseInCamera = pose.inverse();
  cv::Mat R = (cv::Mat_<double>(3, 3) << (double)poseInCamera.r11(),
               (double)poseInCamera.r12(),
               (double)poseInCamera.r13(), //
               (double)poseInCamera.r21(), (double)poseInCamera.r22(),
               (double)poseInCamera.r23(), //
               (double)poseInCamera.r31(), (double)poseInCamera.r32(),
               (double)poseInCamera.r33());
  cv::Mat rvec(1, 3, CV_64FC1);
  cv::Rodrigues(R, rvec);
  cv::Mat tvec = (cv::Mat_<double>(1, 3) << (double)poseInCamera.x(),
                  (double)poseInCamera.y(), (double)poseInCamera.z());

  // do the projection
  cv::projectPoints(points, rvec, tvec, K, cv::Mat(), planePoints);

  // find points in the image
  int width = camera.getImageSize().width;
  int height = camera.getImageSize().height;
  cv::Point2f center(width / 2, height / 2);
  std::map<double, std::pair<std::string, cv::Point2f>> resultMap;
  for (unsigned int i = 0; i < points.size(); ++i) {
    std::string name = names[i];
    // if (uIsInBounds(int(planePoints[i].x), 0, width) &&
    //        uIsInBounds(int(planePoints[i].y), 0, height))
    if (true) {
      if (Utility::isInFrontOfCamera(points[i], poseInCamera)) {
        double dist = cv::norm(planePoints[i] - center);
        resultMap[dist] = std::pair<std::string, cv::Point2f>(name, planePoints[i]);
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
  double size;
  if(width>height) {
    size = height/10;
  } else {
    size = width/10;
  }
  for(std::map<double, std::pair<std::string, cv::Point2f>>::iterator it=resultMap.begin(); it!=resultMap.end(); ++it) {
    std::pair<std::string, cv::Point2f> result = it->second;
    results.push_back(FoundItem(result.first, result.second.x, result.second.y, size , width, height));
  } 
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
