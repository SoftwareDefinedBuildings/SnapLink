#pragma once

#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include "lib/data/CameraModel.h"

class Image {
public:
  explicit Image(int id, int roomId, int cv::Mat &&image, cv::Mat &&depth, Transform &&pose, CameraModel &&camera);

  int getId() const;
  int getRoomId() const;
  const cv::Mat &getImage() const;
  const cv::Mat &getDepth() const;
  const Transform &getPose() const;
  const CameraModel &getCameraModel() const;

private:
  int _id;
  int _roomId;
  cv::Mat _image;
  cv::Mat _depth;
  Transform _pose;
  CameraModel _camera;
};
