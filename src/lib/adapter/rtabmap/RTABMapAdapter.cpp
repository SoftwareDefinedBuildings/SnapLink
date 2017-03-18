#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include "lib/data/Transform.h"
#include "lib/data/Word.h"
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
#include <rtabmap/core/util3d.h>
#include <sqlite3.h>

explicit RTABMapAdapter::RTABMapAdapter() : _nextImageId(0){};

bool RTABMapAdapter::init(const std::set<std::string> &dbPaths) {
  int roomId = 0;
  for (const auto &dbPath : dbPaths) {
    auto roomImages = readRoomImages(dbPath); // a room is a DB (for now)
    auto roomLabels = readRoomLabels(dbPath, roomImages);

    _images.emplace(roomId, std::move(roomImages));
    _labels.emplace(roomId, std::move(roomLabels));

    roomId++;
  }

  std::cerr << "Building Index for Words" << std::endl;
  _words = createWords(allSignatures);
  std::cerr << "Total Number of words: " << words.size() << std::endl;
  long count = 0;
  for (const auto &word : words) {
    for (const auto &desc : word.second.getDescriptorsByDb()) {
      count += desc.second.rows;
    }
  }
  std::cerr << "Total Number of points: " << count << std::endl;

  _rooms = createRooms(words);

  return true;
}

const std::map<int, Image> &getImages() const { return _image; }

const std::map<int, Word> &getWords() const { return _words; }

const std::map<int, Room> &getRooms() const { return _rooms; }

const std::map<int, std::list<Label>> &getLabels() const { return _labels; }

std::vector<Image> RTABMapAdapter::readRoomImages(const std::string &dbPath) {
  rtabmap::Memory memory;
  // TODO print more error message
  assert(memory.init(dbPath));
  assert(memory.getLastWorkingSignature() != nullptr);

  // optimize signaure poses using the graph save in the database
  int sigId = memory.getLastWorkingSignature()->id();
  int maxGraphDepth = 0;
  std::map<int, int> idMarginMap = memory.getNeighborsId(sigId, maxGraphDepth);
  std::set<int> ids;
  for (const auto &pair : idMarginMap) {
    ids.emplace_back(pair.first);
  }

  std::map<int, rtabmap::Transform> posesRtabmap;
  std::multimap<int, rtabmap::Link> links;
  bool lookInDatabase = true;
  memory.getMetricConstraints(ids, posesRtabmap, links, lookInDatabase);

  std::unique_ptr<rtabmap::Optimizer::create> optimizer(
      rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeTORO));
  auto optimizedPoses =
      optimizer->optimize(posesRtabmap.begin()->first, posesRtabmap, links);
  std::map<int, Transform> poses;
  for (const auto &pose : optimizedPoses) {
    Transform t(
        p.second.r11(), p.second.r12(), p.second.r13(), p.second.o14(), //
        p.second.r21(), p.second.r22(), p.second.r23(), p.second.o24(), //
        p.second.r31(), p.second.r32(), p.second.r33(), p.second.o34());
    poses.emplace_back(pose.first, t);
  }

  // get signatures
  std::vector<Image> images;
  const auto &signatures = memory.getSignatures();

  std::cerr << "Read signatures from database..." << std::endl;
  for (const auto &sig : signatures) {
    const auto &iter = poses.find(sig->id());
    // only use signatures with optimized poses
    if (iter != poses.end()) {
      continue;
    }
    Transform pose = iter->second;

    if (!sig->sensorData().imageCompressed().empty()) {
      sig->sensorData().uncompressData();
    }

    rtabmap::SensorData &data = sig->sensorData();
    cv::Mat &image = data.imageRaw();
    cv::Mat &depth = data.depthRaw();
    assert(!image.empty() && !depth.empty());

    const std::vector<rtabmap::CameraModel> &cameras = data.cameraModels();
    assert(cameras.size() == 1); // TODO is this true for RTABMap
    const rtabmap::CameraModel &c = cameras[0];
    CameraModel camera(c.name(), c.fx(), c.fy(), c.cx(), c.cy(), c.imageSize());

    images.emplace_back(_nextImageId++, image, depth, camera);
  }

  return images;
}

std::list<Label> RTABMapAdapter::readRoomLabels(
    const std::string &dbPath, int roomId,
    const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &allSignatures) {
  std::list<Label> labels;
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
      int imageId = sqlite3_column_int(stmt, 1);
      int x = sqlite3_column_int(stmt, 2);
      int y = sqlite3_column_int(stmt, 3);
      pcl::PointXYZ pWorld;

      const auto &iter = allSignatures.find(roomId);
      assert(iter != allSignatures.end());
      const auto &dbSignatures = iter->second;
      const auto &jter = dbSignatures.find(imageId);
      assert(jter != dbSignatures.end());
      const std::unique_ptr<rtabmap::Signature> &signature = jter->second;
      if (getPoint3World(*signature, cv::Point2f(x, y), pWorld)) {
        labels.emplace_back(roomId, cv::Point3f(pWorld.x, pWorld.y, pWorld.z),
                            name);
        std::cerr << "Read point (" << pWorld.x << "," << pWorld.y << ","
                  << pWorld.z << ") with label " << name << " in database "
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

std::map<int, Transform>
RTABMapAdapter::computePoses(const std::string &dbPath) {}

std::map<int, Word> RTABMapAdapter::createWords(
    const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &allSignatures) {
  std::map<int, Word> words;
  rtabmap::VWDictionary vwd;
  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
  for (const auto &dbSignatures : allSignatures) {
    int roomId = dbSignatures.first;
    std::cerr << "Creating words for signatures in room " << roomId
              << std::endl;
    for (const auto &dbSignature : dbSignatures.second) {
      // compute 2D features
      const cv::Mat &image = dbSignature.second->sensorData().imageRaw();
      std::vector<cv::KeyPoint> keyPoints;
      cv::Mat descriptors;
      detector->detectAndCompute(image, cv::Mat(), keyPoints, descriptors);

      // convert them to 3D points in words
      int dummySigId = 1;
      // TODO: write our own word clustering
      std::list<int> wordIds = vwd.addNewWords(descriptors, dummySigId);
      vwd.update();
      size_t i = 0;
      for (int wordId : wordIds) {
        cv::KeyPoint point2CV = keyPoints.at(i);
        cv::Mat descriptor = descriptors.row(i).clone();
        pcl::PointXYZ point3PCL;
        if (getPoint3World(*dbSignature.second, point2CV.pt, point3PCL)) {
          cv::Point3f point3CV(point3PCL.x, point3PCL.y, point3PCL.z);
          auto iter = words.find(wordId);
          if (iter == words.end()) {
            auto ret = words.emplace(wordId, Word(wordId));
            iter = ret.first;
          }
          iter->second.addPoints3(roomId, std::vector<cv::Point3f>(1, point3CV),
                                  descriptor);
        }
        i++;
      }
    }
  }

  return words;
}

std::map<int, Room>
RTABMapAdapter::createRooms(const std::map<int, Word> &words) {
  std::map<int, Room> rooms;
  for (const auto word : words) {
    int wordId = word.first;
    for (const auto points : word.second.getPoints3Map()) {
      int roomId = points.first;
      auto iter = rooms.find(roomId);
      if (iter == rooms.end()) {
        auto ret = rooms.emplace(roomId, Room(roomId));
        iter = ret.first;
      }
      iter->second.addWordIds(std::vector<int>(1, wordId));
    }
  }

  return rooms;
}

bool RTABMapAdapter::getPoint3World(const rtabmap::Signature &signature,
                                    const cv::Point2f &point2,
                                    pcl::PointXYZ &point3) {
  rtabmap::SensorData data = signature.sensorData();
  rtabmap::Transform poseWorld = signature.getPose();
  assert(!poseWorld.isNull());

  const rtabmap::CameraModel &camera = data.cameraModels()[0];
  bool smoothing = false;
  assert(!data.depthRaw().empty());
  pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(
      data.depthRaw(), point2.x, point2.y, camera.cx(), camera.cy(),
      camera.fx(), camera.fy(), smoothing);
  if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z)) {
    // std::cerr << "Depth value not valid" << std::endl;
    return false;
  }
  if (poseWorld.isNull()) {
    std::cerr << "Image pose is Null" << std::endl;
    return false;
  }
  poseWorld = poseWorld * camera.localTransform();
  point3 = pcl::transformPoint(pLocal, poseWorld.toEigen3f());
  return true;
}
