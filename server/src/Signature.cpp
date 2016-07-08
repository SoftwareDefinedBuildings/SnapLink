#include <cassert>
#include <rtabmap/core/EpipolarGeometry.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Compression.h>
#include <opencv2/highgui/highgui.hpp>
#include "Signature.h"

Signature::Signature(
    int id,
    int mapId,
    int dbId,
    const rtabmap::Transform &pose,
    const rtabmap::SensorData &sensorData,
    const std::multimap<int, cv::KeyPoint> &words,
    const std::multimap<int, cv::Point3f> &words3D,
    const std::multimap<int, cv::Mat> &descriptors):
    _id(id),
    _mapId(mapId),
    _dbId(dbId),
    _pose(pose),
    _sensorData(sensorData),
    _words(words),
    _words3D(words3D),
    _wordsDescriptors(descriptors)
{
    if (_sensorData.id() == 0)
    {
        _sensorData.setId(id);
    }
    assert(_sensorData.id() == _id);
}

Signature::~Signature()
{
}

int Signature::getId() const
{
    return _id;
}

int Signature::getMapId() const
{
    return _mapId;
}

int Signature::getDbId() const
{
    return _dbId;
}

const rtabmap::Transform &Signature::getPose() const
{
    return _pose;
}

const rtabmap::SensorData &Signature::getSensorData() const
{
    return _sensorData;
}

const std::multimap<int, cv::KeyPoint> &Signature::getWords() const
{
    return _words;
}

const std::multimap<int, cv::Point3f> &Signature::getWords3D() const
{
    return _words3D;
}

const std::multimap<int, cv::Mat> &Signature::getWordsDescriptors() const
{
    return _wordsDescriptors;
}
