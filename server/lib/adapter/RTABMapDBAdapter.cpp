#include "adapter/RTABMapDBAdapter.h"
#include "data/Labels.h"
#include "data/Transform.h"
#include "data/Words.h"
#include "util/Time.h"
#include <QDebug>
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

bool RTABMapDBAdapter::readData(const std::vector<std::string> &dbPaths,
                                Words &words, Labels &labels) {
  // Read data from databases
  std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
      allSignatures;
  std::list<std::unique_ptr<Label>> allLabels;
  int dbId = 0;
  for (const auto &dbPath : dbPaths) {
    auto dbSignatures = readSignatures(dbPath);
    allSignatures.insert(std::make_pair(dbId, std::move(dbSignatures)));

    auto dbLabels = readLabels(dbPath, dbId, allSignatures);
    std::move(dbLabels.begin(), dbLabels.end(), std::back_inserter(allLabels));

    dbId++;
  }

  qDebug() << "Building Index for Words";
  std::list<std::unique_ptr<Word>> allWords = createWords(allSignatures);
  qDebug() << "Total Number of words: " << allWords.size();
  long count = 0;
  for (const auto &word : allWords) {
    for (const auto &desc : word->getDescriptorsByDb()) {
      count += desc.second.rows;
    }
  }
  qDebug() << "Total Number of descriptors: " << count;
  allWords = clusterPointsInWords(allWords);
  count = 0;
  for (const auto &word : allWords) {
    for (const auto &desc : word->getDescriptorsByDb()) {
      count += desc.second.rows;
    }
  }
  qDebug() << "Total Number of points: " << count;
  words.putWords(std::move(allWords));

  qDebug() << "Building Index for Labels";
  labels.putLabels(std::move(allLabels));

  return true;
}

std::map<int, std::unique_ptr<rtabmap::Signature>>
RTABMapDBAdapter::readSignatures(const std::string &dbPath) {
  std::map<int, std::unique_ptr<rtabmap::Signature>> signatures;

  // get optimized poses of signatures
  qDebug() << "Optimize poses of signatures...";
  const std::map<int, rtabmap::Transform> &optimizedPoseMap =
      getOptimizedPoseMap(dbPath);

  rtabmap::DBDriver *dbDriver = rtabmap::DBDriver::create();
  if (!dbDriver->openConnection(dbPath)) {
    qDebug() << "Connecting to database " << dbPath.c_str()
             << ", path is invalid!";
    return signatures;
  }

  // Read signatures from database
  qDebug() << "Read signatures from database...";
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
    signatures.insert(
        std::make_pair(id, std::unique_ptr<rtabmap::Signature>(signature)));
    signature = nullptr;
  }

  qDebug() << "Closing database " << dbDriver->getUrl().c_str() << "...";
  dbDriver->closeConnection();
  dbDriver->join();
  delete dbDriver;
  dbDriver = nullptr;

  return signatures;
}

std::list<std::unique_ptr<Label>> RTABMapDBAdapter::readLabels(
    const std::string &dbPath, int dbId,
    const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &allSignatures) {
  std::list<std::unique_ptr<Label>> labels;
  sqlite3 *db = nullptr;
  sqlite3_stmt *stmt = nullptr;
  int rc;

  rc = sqlite3_open(dbPath.c_str(), &db);
  if (rc != SQLITE_OK) {
    qCritical() << "Could not open database " << sqlite3_errmsg(db);
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
      if (getPoint3World(*signature, x, y, pWorld)) {
        labels.emplace_back(std::unique_ptr<Label>(
            new Label(dbId, cv::Point3f(pWorld.x, pWorld.y, pWorld.z), name)));
        std::cout << "Read point (" << pWorld.x << "," << pWorld.y << ","
                  << pWorld.z << ") with label " << name << " in database "
                  << dbPath << std::endl;
      }
    }
  } else {
    qWarning() << "Could not read database " << dbPath.c_str() << ":"
               << sqlite3_errmsg(db);
  }

  sqlite3_finalize(stmt);
  sqlite3_close(db);

  return labels;
}

std::map<int, rtabmap::Transform>
RTABMapDBAdapter::getOptimizedPoseMap(const std::string &dbPath) {
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

std::list<std::unique_ptr<Word>> RTABMapDBAdapter::createWords(
    const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &allSignatures) {
  std::map<int, std::unique_ptr<Word>> wordsMap; // wordId: word pointer
  rtabmap::VWDictionary vwd;
  int minHessian = 400;
  cv::Ptr<cv::xfeatures2d::SURF> detector =
      cv::xfeatures2d::SURF::create(minHessian);
  for (const auto &dbSignatures : allSignatures) {
    int dbId = dbSignatures.first;
    qDebug() << "Creating words for signatures in DB " << dbId;
    for (const auto &dbSignature : dbSignatures.second) {
      const cv::Mat &image = dbSignature.second->sensorData().imageRaw();
      std::vector<cv::KeyPoint> keyPoints;
      cv::Mat descriptors;
      detector->detectAndCompute(image, cv::Mat(), keyPoints, descriptors);
      int dummySigId = 1;
      std::list<int> wordIds = vwd.addNewWords(descriptors, dummySigId);
      size_t i = 0;
      for (int wordId : wordIds) {
        cv::KeyPoint point2CV = keyPoints.at(i);
        cv::Mat descriptor = descriptors.row(i).clone();
        pcl::PointXYZ point3PCL;
        if (getPoint3World(*dbSignature.second, point2CV.pt.x, point2CV.pt.y,
                           point3PCL)) {
          cv::Point3f point3CV(point3PCL.x, point3PCL.y, point3PCL.z);

          auto iter = wordsMap.find(wordId);
          if (iter == wordsMap.end()) {
            auto ret = wordsMap.insert(std::make_pair(
                wordId, std::unique_ptr<Word>(new Word(wordId))));
            iter = ret.first;
          }
          iter->second->addPoints3(dbId, std::vector<cv::Point3f>(1, point3CV),
                                   descriptor);
        }
        i++;
      }
    }
  }

  std::list<std::unique_ptr<Word>> words;
  for (auto &word : wordsMap) {
    words.emplace_back(std::move(word.second));
  }

  return words;
}

std::list<std::unique_ptr<Word>> RTABMapDBAdapter::clusterPointsInWords(
    std::list<std::unique_ptr<Word>> &words) {
  std::list<std::unique_ptr<Word>> newWords;
  for (auto &word : words) {
    std::unique_ptr<Word> newWord =
        std::unique_ptr<Word>(new Word(word->getId()));
    // cluster points and calculate mean descriptor for every cluster
    for (auto points3 : word->getPoints3Map()) {
      int dbId = points3.first;
      std::vector<pcl::PointXYZ> points3PCL;
      for (auto point3 : points3.second) {
        points3PCL.emplace_back(point3.x, point3.y, point3.z);
      }
      std::vector<pcl::PointIndices> clusterIndices;
      points3PCL = clusterPoints3(points3PCL, &clusterIndices);
      std::vector<cv::Point3f> points3CV;
      for (auto point3PCL : points3PCL) {
        points3CV.emplace_back(point3PCL.x, point3PCL.y, point3PCL.z);
      }

      // create new descriptors based on indices
      cv::Mat oldDescriptors = word->getDescriptorsByDb().at(dbId);
      cv::Mat newDescriptors; // descriptors of clustered points
      int j = 0;
      for (std::vector<pcl::PointIndices>::const_iterator iter =
               clusterIndices.begin();
           iter != clusterIndices.end(); iter++) {
        cv::Mat clusterDescriptors;
        cv::Mat meanDescriptor;
        for (std::vector<int>::const_iterator jter = iter->indices.begin();
             jter != iter->indices.end(); jter++) {
          clusterDescriptors.push_back(oldDescriptors.row(*jter));
        }
        cv::reduce(clusterDescriptors, meanDescriptor, 0, CV_REDUCE_AVG);
        newDescriptors.push_back(meanDescriptor);
        j++;
      }
      newWord->addPoints3(dbId, points3CV, newDescriptors);
    }

    newWords.emplace_back(std::move(newWord));
  }

  return newWords;
}

bool RTABMapDBAdapter::getPoint3World(const rtabmap::Signature &signature,
                                      int x, int y, pcl::PointXYZ &pWorld) {
  rtabmap::SensorData data = signature.sensorData();
  rtabmap::Transform poseWorld = signature.getPose();
  assert(!poseWorld.isNull());

  const rtabmap::CameraModel &camera = data.cameraModels()[0];
  bool smoothing = false;
  assert(!data.depthRaw().empty());
  pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(
      data.depthRaw(), x, y, camera.cx(), camera.cy(), camera.fx(), camera.fy(),
      smoothing);
  if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z)) {
    // qWarning() << "Depth value not valid";
    return false;
  }
  if (poseWorld.isNull()) {
    qWarning() << "Image pose is Null";
    return false;
  }
  poseWorld = poseWorld * camera.localTransform();
  pWorld = pcl::transformPoint(pLocal, poseWorld.toEigen3f());
  return true;
}

std::vector<pcl::PointXYZ> RTABMapDBAdapter::clusterPoints3(
    const std::vector<pcl::PointXYZ> &points3,
    std::vector<pcl::PointIndices> *clusterIndicesOut) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &point3 : points3) {
    cloud->push_back(point3);
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1); // in meter
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  std::vector<pcl::PointXYZ> clusteredPoints3;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator iter =
           clusterIndices.begin();
       iter != clusterIndices.end(); iter++) {
    pcl::CentroidPoint<pcl::PointXYZ> cloudCluster;
    for (std::vector<int>::const_iterator jter = iter->indices.begin();
         jter != iter->indices.end(); jter++) {
      cloudCluster.add(cloud->points[*jter]);
    }
    pcl::PointXYZ centroid;
    cloudCluster.get(centroid);
    clusteredPoints3.emplace_back(std::move(centroid));
    j++;
  }

  if (clusterIndicesOut != nullptr) {
    *clusterIndicesOut = clusterIndices;
  }

  return clusteredPoints3;
}
