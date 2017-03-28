#pragma once

#include "lib/data/CameraModel.h"
#include "lib/data/Transform.h"
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>

class Image {
public:
  explicit Image(int id, int roomId, const cv::Mat &image, const cv::Mat &depth,
                 const Transform &pose, const CameraModel &camera);

  int getId() const;
  int getRoomId() const;
  const cv::Mat &getImage() const;
  const cv::Mat &getDepth() const;
  const Transform &getPose() const;
  const CameraModel &getCameraModel() const;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(int decimation) const;

private:
  float getDepth(const cv::Mat &depthImage, float x, float y, bool smoothing,
                 float maxZError, bool estWithNeighborsIfNull) const;
  pcl::PointXYZ projectDepthTo3D(const cv::Mat &depthImage, float x, float y,
                                 float cx, float cy, float fx, float fy,
                                 bool smoothing, float maxZError) const;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  cloudFromDepthRGB(const cv::Mat &imageRgb, const cv::Mat &imageDepthIn,
                    const CameraModel &model, int decimation, float maxDepth,
                    float minDepth) const;

private:
  int _id;
  int _roomId;
  cv::Mat _image;
  cv::Mat _depth;
  Transform _pose;
  CameraModel _camera;
};
