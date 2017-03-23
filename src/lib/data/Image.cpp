#include "lib/data/Image.h"

Image::Image(int id, int roomId, const cv::Mat &image, const cv::Mat &depth,
             Transform &&pose, CameraModel &&camera)
    : _id(id), _roomId(roomId), _image(image), _depth(depth),
      _pose(std::move(pose)), _camera(std::move(camera)) {}

int Image::getId() const { return _id; }

int Image::getRoomId() const { return _roomId; }

const cv::Mat &Image::getImage() const { return _image; }

const cv::Mat &Image::getDepth() const { return _depth; }

const Transform &Image::getPose() const { return _pose; }

const CameraModel &Image::getCameraModel() const { return _camera; }
