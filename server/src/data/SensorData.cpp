#include "data/SensorData.h"

SensorData::SensorData() = default;

SensorData::SensorData(
    const cv::Mat &image,
    const cv::Mat &depth,
    const rtabmap::CameraModel &cameraModel,
    int id,
    double stamp) :
    _image(image),
    _depth(depth),
    _cameraModel(cameraModel),
    _id(id),
    _stamp(stamp)
{
    assert(!image.empty());
    assert(image.type() == CV_8UC1 || image.type() == CV_8UC3);  // Mono or RGB
}

SensorData::~SensorData() = default;

int SensorData::getId() const
{
    return _id;
}

void SensorData::setId(int id)
{
    _id = id;
}

double SensorData::getStamp() const
{
    return _stamp;
}

const cv::Mat &SensorData::getImage() const
{
    return _image;
}

const cv::Mat &SensorData::getDepth() const
{
    return _depth;
}

const rtabmap::CameraModel &SensorData::getCameraModel() const
{
    return _cameraModel;
}
