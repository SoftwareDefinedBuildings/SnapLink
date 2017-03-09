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
#include <rtabmap/utilite/UStl.h>
#include <sqlite3.h>

bool RTABMapAdapter::readData(const std::vector<std::string> &dbPaths,
                              std::map<int, Word> &words, std::map<int, Room> &rooms,
                              std::map<int, std::list<Label>> &labels) {
  // Read data from databases
  std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
      allSignatures;
  int dbId = 0;
  for (const auto &dbPath : dbPaths) {
    auto dbSignatures = readSignatures(dbPath);
    allSignatures.emplace(dbId, std::move(dbSignatures));

    auto dbLabels = readDBLabels(dbPath, dbId, allSignatures);
    labels.emplace(dbId, std::move(dbLabels));

    dbId++;
  }

  std::cerr << "Building Index for Words" << std::endl;
  words = createWords(allSignatures);
  std::cerr << "Total Number of words: " << words.size() << std::endl;
  long count = 0;
  for (const auto &word : words) {
    for (const auto &desc : word.second.getDescriptorsByDb()) {
      count += desc.second.rows;
    }
  }
  std::cerr << "Total Number of points: " << count << std::endl;

  rooms = createRooms(words);

  return true;
}

std::map<int, std::unique_ptr<rtabmap::Signature>>
RTABMapAdapter::readSignatures(const std::string &dbPath) {
  std::map<int, std::unique_ptr<rtabmap::Signature>> signatures;

  // get optimized poses of signatures
  std::cerr << "Optimize poses of signatures..." << std::endl;
  const std::map<int, rtabmap::Transform> &optimizedPoseMap =
      getOptimizedPoseMap(dbPath);

  rtabmap::DBDriver *dbDriver = rtabmap::DBDriver::create();
  if (!dbDriver->openConnection(dbPath)) {
    std::cerr << "Connecting to database " << dbPath << ", path is invalid!"
              << std::endl;
    return signatures;
  }

  // Read signatures from database
  std::cerr << "Read signatures from database..." << std::endl;
  std::list<rtabmap::Signature *> rtabmapSignatures;
  std::set<int> sigIds;
  dbDriver->getAllNodeIds(sigIds, true);
  dbDriver->loadSignatures(std::list<int>(sigIds.begin(), sigIds.end()),
                           rtabmapSignatures);
  dbDriver->loadNodeData(rtabmapSignatures);
  for (auto &signature : rtabmapSignatures) {
    int id = signature->id();

    const auto &iter = optimizedPoseMap.find(id);
    assert(iter != optimizedPoseMap.end());

    signature->setPose(iter->second);
    if (!signature->sensorData().imageCompressed().empty()) {
      signature->sensorData().uncompressData();
    }
    signatures.emplace(id, std::unique_ptr<rtabmap::Signature>(signature));
    signature = nullptr;
  }

  std::cerr << "Closing database " << dbDriver->getUrl() << "..." << std::endl;
  dbDriver->closeConnection();
  dbDriver->join();
  delete dbDriver;
  dbDriver = nullptr;

  return signatures;
}

std::list<Label> RTABMapAdapter::readDBLabels(
    const std::string &dbPath, int dbId,
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

      const auto &iter = allSignatures.find(dbId);
      assert(iter != allSignatures.end());
      const auto &dbSignatures = iter->second;
      const auto &jter = dbSignatures.find(imageId);
      assert(jter != dbSignatures.end());
      const std::unique_ptr<rtabmap::Signature> &signature = jter->second;
      if (getPoint3World(*signature, cv::Point2f(x, y), pWorld)) {
        labels.emplace_back(dbId, cv::Point3f(pWorld.x, pWorld.y, pWorld.z),
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

std::map<int, rtabmap::Transform>
RTABMapAdapter::getOptimizedPoseMap(const std::string &dbPath) {
  rtabmap::Memory memory;
  memory.init(dbPath);

  std::map<int, rtabmap::Transform> optimizedPoseMap;
  if (memory.getLastWorkingSignature()) {
    // Get all IDs linked to last signature (including those in Long-Term
    // Memory)
    std::map<int, int> ids =
        memory.getNeighborsId(memory.getLastWorkingSignature()->id(), 0, -1);

    // Get all metric constraints (the graph)
    std::map<int, rtabmap::Transform> poses;
    std::multimap<int, rtabmap::Link> links;
    memory.getMetricConstraints(uKeysSet(ids), poses, links, true);

    // Optimize the graph
    rtabmap::Optimizer *graphOptimizer =
        rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeTORO);
    optimizedPoseMap =
        graphOptimizer->optimize(poses.begin()->first, poses, links);
    delete graphOptimizer;
  }

  return optimizedPoseMap;
}

std::map<int, Word> RTABMapAdapter::createWords(
    const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &allSignatures) {
  std::map<int, Word> words;
  rtabmap::VWDictionary vwd;
  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
  for (const auto &dbSignatures : allSignatures) {
    int dbId = dbSignatures.first;
    std::cerr << "Creating words for signatures in DB " << dbId << std::endl;
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
          iter->second.addPoints3(dbId, std::vector<cv::Point3f>(1, point3CV),
                                   descriptor);
        }
        i++;
      }
    }
  }

  return words;
}

std::map<int, Room> createRooms(const std::map<int, Word> &words) {
  std::map<int, Room> rooms;
  for (const auto word : words) {
    int wordId = word.first;
    for (const auto points : word.second.getPoints3Map()) {
      int roomId = points.first; // same as DB ID for now
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
