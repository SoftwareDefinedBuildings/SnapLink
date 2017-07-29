#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include "lib/algo/WordCluster.h"
#include "lib/data/Room.h"
#include "lib/data/Transform.h"
#include "lib/data/Word.h"
#include "lib/util/Utility.h"
#include <opencv2/xfeatures2d.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/VWDictionary.h>
#include <sqlite3.h>
#include <utility>

RTABMapAdapter::RTABMapAdapter(float distRatio)
    : _nextImageId(0), _distRatio(distRatio) {}

bool RTABMapAdapter::init(const std::set<std::string> &dbPaths) {
  int roomId = 0;
  for (const auto &dbPath : dbPaths) {
    // a room is a DB (for now)
    auto roomImages = readRoomImages(dbPath, roomId);
    _images.emplace(roomId, std::move(roomImages));

    auto roomLabels = readRoomLabels(dbPath, roomId);
    _labels.emplace(roomId, std::move(roomLabels));

    _roomPaths.emplace(roomId, std::string(dbPath));

    createAprilTagMap(dbPath, roomId);
    roomId++;

  }
  _dbCounts = roomId;

  return true;
}

const std::map<int, std::map<int, Image>> &RTABMapAdapter::getImages() {
  return _images;
}

const std::map<int, Word> &RTABMapAdapter::getWords() {
  // create words the first time it's accessed
  if (_words.empty()) {
    createWords();
  }
  return _words;
}

const std::map<int, Room> &RTABMapAdapter::getRooms() {
  if (_words.empty()) {
    createWords();
  }
  // create rooms the first time it's accessed
  if (_rooms.empty()) {
    createRooms();
  }
  return _rooms;
}

const std::map<int, std::vector<Label>> &RTABMapAdapter::getLabels() {
  return _labels;
}

std::map<int, Image> RTABMapAdapter::readRoomImages(const std::string &dbPath,
                                                    int roomId) {
  std::cerr << "reading images from database " << dbPath << std::endl;

  rtabmap::Memory memory;
  // TODO print more error message
  assert(memory.init(dbPath));
  assert(memory.getLastWorkingSignature() != nullptr);

  // optimize signaure poses using the graph save in the database
  int sigId = memory.getLastWorkingSignature()->id();
  int maxGraphDepth = 0;
  std::map<int, int> idMarginMap = memory.getNeighborsId(sigId, maxGraphDepth);
  std::set<int> sigIds;
  for (const auto &pair : idMarginMap) {
    sigIds.emplace(pair.first);
  }

  std::map<int, rtabmap::Transform> posesRtabmap;
  std::multimap<int, rtabmap::Link> links;
  bool lookInDatabase = true;
  memory.getMetricConstraints(sigIds, posesRtabmap, links, lookInDatabase);

  // we take the ownership of optimizer
  std::unique_ptr<rtabmap::Optimizer> optimizer(
      rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeTORO));
  auto optimizedPoses =
      optimizer->optimize(posesRtabmap.begin()->first, posesRtabmap, links);
  std::map<int, Transform> poses;
  for (const auto &pose : optimizedPoses) {
    const rtabmap::Transform &p = pose.second;
    Transform t(p.r11(), p.r12(), p.r13(), p.o14(), //
                p.r21(), p.r22(), p.r23(), p.o24(), //
                p.r31(), p.r32(), p.r33(), p.o34());
    poses.emplace(pose.first, t);
  }

  // get signatures
  std::map<int, Image> images;
  for (const auto &sigId : sigIds) {
    const rtabmap::Signature *sig = memory.getSignature(sigId);
    assert(sig != nullptr);

    const auto &iter = poses.find(sig->id());
    // only use signatures with optimized poses
    assert(iter != poses.end());
    Transform pose = iter->second;

    bool uncompressedData = true;
    const rtabmap::SensorData data =
        memory.getNodeData(sigId, uncompressedData);
    const cv::Mat &image = data.imageRaw();
    const cv::Mat &depth = data.depthRaw();
    assert(!image.empty() && !depth.empty());

    const std::vector<rtabmap::CameraModel> &cameras = data.cameraModels();
    assert(cameras.size() == 1); // TODO is this true for RTABMap?
    const rtabmap::CameraModel &c = cameras[0];
    CameraModel camera(c.name(), c.fx(), c.fy(), c.cx(), c.cy(), c.imageSize());

    const rtabmap::Transform &lt = c.localTransform();
    Transform t(lt.r11(), lt.r12(), lt.r13(), lt.o14(), //
                lt.r21(), lt.r22(), lt.r23(), lt.o24(), //
                lt.r31(), lt.r32(), lt.r33(), lt.o34());
    pose = pose * t;

    int imageId = _nextImageId;
    _nextImageId++;
    images.emplace(imageId, Image(imageId, roomId, image, depth, pose, camera));
    // insert if not exists
    _sigImageIdMap[roomId][sig->id()] = imageId;
    _imageSigIdMap[roomId][imageId] = sig->id();
  }

  return images;
}

std::vector<Label> RTABMapAdapter::readRoomLabels(const std::string &dbPath,
                                                  int roomId) {
  std::cerr << "reading labels from database " << dbPath << std::endl;

  std::vector<Label> labels;

  // SQLite C API
  sqlite3 *db = nullptr;
  sqlite3_stmt *stmt = nullptr;
  int rc;

  rc = sqlite3_open(dbPath.c_str(), &db);
  if (rc != SQLITE_OK) {
    std::cerr << "Could not open database " << sqlite3_errmsg(db) << std::endl;
    sqlite3_close(db);
    return labels;
  }

  std::string sql = "SELECT * from Labels";
  rc = sqlite3_prepare(db, sql.c_str(), -1, &stmt, nullptr);
  if (rc == SQLITE_OK) {
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      std::string name(
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
      int sigId = sqlite3_column_int(stmt, 1);
      int x = sqlite3_column_int(stmt, 2);
      int y = sqlite3_column_int(stmt, 3);

      // throws out_of_range
      int imageId = _sigImageIdMap.at(roomId).at(sigId);
      cv::Point3f point3;
      const Image &image = _images.at(roomId).at(imageId);

      if (Utility::getPoint3World(image, cv::Point2f(x, y), point3)) {
        labels.emplace_back(roomId, point3, name);
        // std::cerr << "Read point (" << point3.x << "," << point3.y << ","
        //          << point3.z << ") with label " << name << " in database "
        //          << dbPath << std::endl;
      }
    }
  } else {
    std::cerr << "Could not read database " << dbPath << ":"
              << sqlite3_errmsg(db) << std::endl;
  }

  sqlite3_finalize(stmt);
  sqlite3_close(db);

  return labels;
}

void RTABMapAdapter::createWords() {
  std::cerr << "creating words for all rooms" << std::endl;

  assert(_images.empty() == false);

  rtabmap::VWDictionary vwd;
  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();

  std::vector<int> roomIds;
  std::vector<cv::Point3f> points3;
  cv::Mat descriptors;
  for (const auto &roomImages : _images) {
    int roomId = roomImages.first;
    for (const auto &image : roomImages.second) {
      // compute 2D features
      std::vector<cv::KeyPoint> imgKeyPoints;
      cv::Mat imgDescriptors;
      detector->detectAndCompute(image.second.getImage(), cv::Mat(),
                                 imgKeyPoints, imgDescriptors);

      for (unsigned int i = 0; i < imgKeyPoints.size(); i++) {
        cv::KeyPoint kp = imgKeyPoints.at(i);
        cv::Mat descriptor = imgDescriptors.row(i);
        cv::Point3f point3;
        if (Utility::getPoint3World(image.second, kp.pt, point3)) {
          roomIds.emplace_back(roomId);
          points3.emplace_back(point3);
          descriptors.push_back(descriptor);
        }
      }
    }
  }

  // convert them to 3D points in words
  WordCluster wordCluster(_distRatio);
  _words = wordCluster.cluster(roomIds, points3, descriptors);

  std::cerr << "total number of words: " << _words.size() << std::endl;
  long count = 0;
  for (const auto &word : _words) {
    for (const auto &desc : word.second.getDescriptorsByDb()) {
      count += desc.second.rows;
    }
  }
  std::cerr << "total number of 3D points: " << count << std::endl;
}

void RTABMapAdapter::createRooms() {
  assert(_words.empty() == false);
  for (const auto word : _words) {
    int wordId = word.first;
    for (const auto points : word.second.getPoints3Map()) {
      int roomId = points.first;
      auto iter = _rooms.find(roomId);
      if (iter == _rooms.end()) {
        auto ret = _rooms.emplace(roomId, Room(roomId));
        iter = ret.first;
      }
      iter->second.addWordIds(std::vector<int>(1, wordId));
    }
  }
}

sqlite3 *RTABMapAdapter::createLabelTable(int roomId) {
  sqlite3 *labelDB;
  if (sqlite3_open(this->_roomPaths.at(roomId).c_str(), &labelDB) !=
      SQLITE_OK) {
    std::cerr << "RTABMapAdapter::createLabelTable()::Could not open database"
              << std::endl;
    return nullptr;
  }
  std::string query;
  query = "CREATE TABLE IF NOT EXISTS Labels (\n\t"
          "labelName VARCHAR(255),\n\t"
          "imgId INT,\n\t"
          "x INT,\n\t"
          "y INT\n); ";
  int rc = sqlite3_exec(labelDB, query.c_str(), NULL, NULL, NULL);
  if (rc == SQLITE_OK) {
    return labelDB;
  } else {
    return nullptr;
  }
}

bool RTABMapAdapter::putLabel(int roomId, std::string label_name,
                              std::string label_id, std::string label_x,
                              std::string label_y) {
  cv::Point3f point3;
  int sigId, imageId, x, y;
  imageId = std::stoi(label_id);
  sigId = _imageSigIdMap.at(roomId).at(imageId);
  x = std::stoi(label_x);
  y = std::stoi(label_y);
  if (Utility::getPoint3World(_images.at(roomId).at(imageId), cv::Point2f(x, y),
                              point3)) {
    sqlite3 *labelDB = createLabelTable(roomId);
    if (!labelDB) {
      std::cerr << "RTABMapAdapter::createLabelTable()::Could not open database"
                << std::endl;
      return false;
    }
    std::stringstream saveQuery;
    saveQuery << "INSERT INTO Labels VALUES ('" << label_name << "', '" << sigId
              << "', '" << label_x << "', '" << label_y << "');";
    int rc = sqlite3_exec(labelDB, saveQuery.str().c_str(), NULL, NULL, NULL);
    sqlite3_close(labelDB);
    if (rc == SQLITE_OK) {
      Label newLabel(roomId, point3, label_name);
      _labels.at(roomId).push_back(newLabel);
    }
    return rc == SQLITE_OK;
  } else {
    std::cerr << "Could not convert label" << std::endl;
    return false;
  }
}

bool RTABMapAdapter::saveAprilTagPose(int roomId, long time, int code,
                                      Transform pose) {
  {
    std::lock_guard<std::mutex> lock(_aprilTagMapMutex);
    if (_aprilTagMap.count(code) > 0) {
      _aprilTagMap[code].emplace(roomId, pose);
    } else {
      std::multimap<int, Transform> value;
      value.emplace(roomId, pose);
      _aprilTagMap.emplace(code, value);
    }
  }

  sqlite3 *labelDB = createAprilTagPoseTable(roomId);
  if (!labelDB) {
    std::cerr << "RTABMapAdapter::createLabelTable()::Could not open database"
              << std::endl;
    return false;
  }
  std::stringstream saveQuery;
  saveQuery << "INSERT INTO AprilTagPoses VALUES ('" << time << "', '" << code
            << "', '" << pose.r11() << "', '" << pose.r12() << "', '"
            << pose.r13() << "', '" << pose.r21() << "', '" << pose.r22()
            << "', '" << pose.r23() << "', '" << pose.r31() << "', '"
            << pose.r32() << "', '" << pose.r33() << "', '" << pose.x()
            << "', '" << pose.y() << "', '" << pose.z() << "');";
  int rc = sqlite3_exec(labelDB, saveQuery.str().c_str(), NULL, NULL, NULL);
  sqlite3_close(labelDB);
  return rc == SQLITE_OK;
}

sqlite3 *RTABMapAdapter::createAprilTagPoseTable(int roomId) {
  sqlite3 *labelDB;
  if (sqlite3_open(this->_roomPaths.at(roomId).c_str(), &labelDB) !=
      SQLITE_OK) {
    std::cerr
        << "RTABMapAdapter::createAprilTagPoseTable()::Could not open database"
        << std::endl;
    return nullptr;
  }
  std::string query;
  query = "CREATE TABLE IF NOT EXISTS AprilTagPoses (\n\t"
          "timeStamp INT,\n\t"
          "code INT,\n\t"
          "r11 REAL,\n\t"
          "r12 REAL,\n\t"
          "r13 REAL,\n\t"
          "r21 REAL,\n\t"
          "r22 REAL,\n\t"
          "r23 REAL,\n\t"
          "r31 REAL,\n\t"
          "r32 REAL,\n\t"
          "r33 REAL,\n\t"
          "x REAL,\n\t"
          "y REAL,\n\t"
          "z REAL\n); ";
  int rc = sqlite3_exec(labelDB, query.c_str(), NULL, NULL, NULL);
  if (rc == SQLITE_OK) {
    return labelDB;
  } else {
    return nullptr;
  }
}

void RTABMapAdapter::createAprilTagMap(std::string dbPath, int roomId) {
  std::cerr << "reading aprilTag poses from database " << dbPath << std::endl;

  // SQLite C API
  sqlite3 *db = nullptr;
  sqlite3_stmt *stmt = nullptr;
  int rc;

  rc = sqlite3_open(dbPath.c_str(), &db);
  if (rc != SQLITE_OK) {
    std::cerr << "Could not open database " << sqlite3_errmsg(db) << std::endl;
    sqlite3_close(db);
    return;
  }

  std::string sql = "SELECT * from AprilTagPoses";
  rc = sqlite3_prepare(db, sql.c_str(), -1, &stmt, nullptr);
  if (rc == SQLITE_OK) {
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      int code = sqlite3_column_int(stmt, 1);
      double r11 = sqlite3_column_double(stmt, 2);
      double r12 = sqlite3_column_double(stmt, 3);
      double r13 = sqlite3_column_double(stmt, 4);
      double r21 = sqlite3_column_double(stmt, 5);
      double r22 = sqlite3_column_double(stmt, 6);
      double r23 = sqlite3_column_double(stmt, 7);
      double r31 = sqlite3_column_double(stmt, 8);
      double r32 = sqlite3_column_double(stmt, 9);
      double r33 = sqlite3_column_double(stmt, 10);
      double x = sqlite3_column_double(stmt, 11);
      double y = sqlite3_column_double(stmt, 12);
      double z = sqlite3_column_double(stmt, 13);
      Transform pose(r11, r12, r13, x, 
                     r21,
                     r22, r23, y,
                     r31,
                     r32, r33, z);
      if (_aprilTagMap.count(code) > 0) {
        _aprilTagMap[code].emplace(roomId, pose);
      } else {
        std::multimap<int, Transform> value;
        value.emplace(roomId, pose);
        _aprilTagMap.emplace(code, value);
      }
    }
  } else {
    std::cerr << "Could not read database " << dbPath << ":"
              << sqlite3_errmsg(db) << std::endl;
  }

  sqlite3_finalize(stmt);
  sqlite3_close(db);
}

std::pair<int, Transform> RTABMapAdapter::lookupAprilCode(int code) {
  std::multimap<int, Transform> value;
  {
    std::lock_guard<std::mutex> lock(_aprilTagMapMutex);
    if (_aprilTagMap.count(code) < 1) {
      // the first time this tag code appears
      // return roomId = -1 indicating not found
      return std::pair<int, Transform>(-1, Transform());
    }
    value = _aprilTagMap[code];
  }

  int maxCount = 0;
  std::multimap<int, Transform>::iterator maxRoomPosesIter;
  //given a code, find the room with most occurence of this tag code
  for (std::multimap<int, Transform>::iterator it = value.begin(), end = value.end();
       it != end; it = value.upper_bound(it->first)) {
    int roomId = it->first;
    int count = value.count(roomId);
    if(count > maxCount) {
      maxCount = count;
      maxRoomPosesIter = it;
    }
  }
  int resultRoomId = maxRoomPosesIter->first;
  Transform resultPose = maxRoomPosesIter->second;
  return std::pair<int, Transform>(resultRoomId, resultPose);
}


int RTABMapAdapter::getDBCounts() {
  return _dbCounts;
}