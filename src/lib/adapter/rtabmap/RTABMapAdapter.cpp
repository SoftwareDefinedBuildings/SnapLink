#include "lib/adapter/rtabmap/RTABMapAdapter.h"
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

RTABMapAdapter::RTABMapAdapter() : _nextImageId(0) {}

bool RTABMapAdapter::init(const std::set<std::string> &dbPaths) {
  int roomId = 0;
  for (const auto &dbPath : dbPaths) {
    // a room is a DB (for now)
    auto roomImages = readRoomImages(dbPath, roomId);
    _images.emplace(roomId, std::move(roomImages));

    auto roomLabels = readRoomLabels(dbPath, roomId);
    _labels.emplace(roomId, std::move(roomLabels));

    _roomPaths.emplace(roomId, std::string(dbPath));

    roomId++;
  }

  return true;
}

const std::map<int, std::vector<Image>> &RTABMapAdapter::getImages() {
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

std::vector<Image> RTABMapAdapter::readRoomImages(const std::string &dbPath,
                                                  int roomId) {
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
  std::vector<Image> images;
  std::cerr << "Read signatures from database..." << std::endl;
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

    int imageId = _nextImageId++;
    images.emplace_back(imageId, roomId, image, depth, pose, camera);
    // insert if not exists
    _sigImageIdMap[roomId][sig->id()] = imageId;
  }

  return images;
}

std::vector<Label> RTABMapAdapter::readRoomLabels(const std::string &dbPath,
                                                  int roomId) {
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
        std::cerr << "Read point (" << point3.x << "," << point3.y << ","
                  << point3.z << ") with label " << name << " in database "
                  << dbPath << std::endl;
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
  assert(_images.empty() == false);

  std::cerr << "Building Index for Words" << std::endl;
  rtabmap::VWDictionary vwd;
  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();

  for (const auto &roomImages : _images) {
    int roomId = roomImages.first;
    std::cerr << "Creating words for signatures in room " << roomId
              << std::endl;
    for (const auto &image : roomImages.second) {
      // compute 2D features
      std::vector<cv::KeyPoint> keyPoints;
      cv::Mat descriptors;
      detector->detectAndCompute(image.getImage(), cv::Mat(), keyPoints,
                                 descriptors);

      // convert them to 3D points in words
      int dummySigId = 1;
      // TODO: write our own word clustering
      std::list<int> wordIds = vwd.addNewWords(descriptors, dummySigId);
      vwd.update();
      size_t i = 0;
      for (int wordId : wordIds) {
        cv::KeyPoint kp = keyPoints.at(i);
        cv::Mat descriptor = descriptors.row(i);
        cv::Point3f point3;
        if (Utility::getPoint3World(image, kp.pt, point3)) {
          auto iter = _words.find(wordId);
          if (iter == _words.end()) {
            auto ret = _words.emplace(wordId, Word(wordId));
            iter = ret.first;
          }
          std::vector<cv::Point3f> points3(1, point3);
          iter->second.addPoints3(roomId, points3, descriptor);
        }
        i++;
      }
    }
  }

  std::cerr << "Total Number of words: " << _words.size() << std::endl;
  long count = 0;
  for (const auto &word : _words) {
    for (const auto &desc : word.second.getDescriptorsByDb()) {
      count += desc.second.rows;
    }
  }
  std::cerr << "Total Number of points: " << count << std::endl;
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
  int imageId, x, y;
  imageId = std::stoi(label_id);
  x = std::stoi(label_x);
  y = std::stoi(label_y);
  if (Utility::getPoint3World(this->_images.at(roomId).at(imageId),
                              cv::Point2f(x, y), point3)) {
    sqlite3 *labelDB = createLabelTable(roomId);
    if (!labelDB) {
      std::cerr << "RTABMapAdapter::createLabelTable()::Could not open database"
                << std::endl;
      return false;
    }
    std::stringstream saveQuery;
    saveQuery << "INSERT INTO Labels VALUES ('" << label_name << "', '"
              << label_id << "', '" << label_x << "', '" << label_y << "');";
    int rc = sqlite3_exec(labelDB, saveQuery.str().c_str(), NULL, NULL, NULL);
    sqlite3_close(labelDB);
    if(rc == SQLITE_OK) {
      Label newLabel(roomId, point3, label_name);
      _labels.at(roomId).push_back(newLabel);
    }
    return rc == SQLITE_OK;
  } else {
    std::cerr << "Could not convert label" << std::endl;
    return false;
  }
}
