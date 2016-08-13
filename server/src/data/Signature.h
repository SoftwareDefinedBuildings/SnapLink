#pragma once

#include <list>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <set>
#include <vector>

#include "data/SensorData.h"
#include "data/Transform.h"

class Signature {
public:
  Signature(int id, int mapId, int dbId, Transform pose,
            const SensorData &sensorData,
            std::multimap<int, cv::KeyPoint> words,
            std::multimap<int, cv::Point3f> words3);
  virtual ~Signature();

  int getId() const;
  int getMapId() const;
  int getDbId() const;
  const Transform &getPose() const;
  const SensorData &getSensorData() const;
  const std::multimap<int, cv::KeyPoint> &getWords() const;
  const std::multimap<int, cv::Point3f> &getWords3() const;

private:
  int _id;
  int _mapId;
  int _dbId;
  Transform _pose;
  SensorData _sensorData;

  // Contains all words (Some can be duplicates -> if a word appears 2
  // times in the signature, it will be 2 times in this list)
  // Words match with the CvSeq keypoints and descriptors
  std::multimap<int, cv::KeyPoint> _words; // word <id, keypoint>
  std::multimap<int, cv::Point3f> _words3; // word <id, 3D point> // in
                                           // base_link frame (localTransform
                                           // applied))
};
