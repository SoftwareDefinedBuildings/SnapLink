#pragma once

#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include "lib/data/CameraModel.h"

class Image {
public:
  explicit Image(int id, cv::Mat &&image, cv::Mat &&depth, Transform &&pose, CameraModel &&camera);

  int getId() const;
  const cv::Mat &getImage() const;
  const cv::Mat &getDepth() const;
  const Transform &getPose() const;
  const CameraModel &getCameraModel() const;

private:
  int _id;
  cv::Mat _image;
  cv::Mat _depth;
  Transform _pose;
  CameraModel _camera;
};
