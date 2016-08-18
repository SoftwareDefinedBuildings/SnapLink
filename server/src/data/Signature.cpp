#include "data/Signature.h"
#include <cassert>

Signature::Signature(int id, int mapId, int dbId, Transform pose,
                     const SensorData &sensorData,
                     std::multimap<int, cv::KeyPoint> words,
                     std::multimap<int, cv::Point3f> words3)
    : _id(id), _mapId(mapId), _dbId(dbId), _pose(std::move(pose)),
      _sensorData(sensorData), _words(std::move(words)),
      _words3(std::move(words3)) {
  assert(_pose.isNull() == false);
  assert(_words.size() == _words3.size());
}

Signature::~Signature() = default;

int Signature::getId() const { return _id; }

int Signature::getMapId() const { return _mapId; }

int Signature::getDbId() const { return _dbId; }

const Transform &Signature::getPose() const { return _pose; }

const SensorData &Signature::getSensorData() const { return _sensorData; }

const std::multimap<int, cv::KeyPoint> &Signature::getWords() const {
  return _words;
}

const std::multimap<int, cv::Point3f> &Signature::getWords3() const {
  return _words3;
}
