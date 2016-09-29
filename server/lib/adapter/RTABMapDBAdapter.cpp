#include "adapter/RTABMapDBAdapter.h"
#include "data/Labels.h"
#include "data/Signatures.h"
#include "data/Transform.h"
#include "data/Words.h"
#include "util/Time.h"
#include <QDebug>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/UStl.h>
#include <sqlite3.h>

bool RTABMapDBAdapter::readData(const std::vector<std::string> &dbPaths,
                                Words &words, Signatures &signatures,
                                Labels &labels) {
  std::ofstream pRawFile;
  pRawFile.open("points_raw.txt");
  pRawFile.close();
  std::ofstream pCookedFile;
  pCookedFile.open("points_cooked.txt");
  pCookedFile.close();

  // Read data from databases
  std::map<int, std::list<std::unique_ptr<Word>>> allWords;
  std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
      allSignatures;
  std::list<std::unique_ptr<Label>> allLabels;
  int dbId = 0;
  for (const auto &dbPath : dbPaths) {
    auto dbSignatures = readSignatures(dbPath);
    allSignatures.insert(std::make_pair(dbId, std::move(dbSignatures)));

    auto dbWords = readWords(dbPath, dbId, allSignatures);
    allWords.insert(std::make_pair(dbId, std::move(dbWords)));

    auto dbLabels = readLabels(dbPath, dbId, allSignatures);
    std::move(dbLabels.begin(), dbLabels.end(), std::back_inserter(allLabels));

    dbId++;
  }

  // merge data from all databases
  auto mergeWordsIdMap = getMergeWordsIdMap(allWords);
  auto mergeSignaturesIdMap = getMergeSignaturesIdMap(allSignatures);

  auto mergedSignatures = mergeSignatures(
      std::move(allSignatures), mergeSignaturesIdMap, mergeWordsIdMap);
  signatures.putSignatures(std::move(mergedSignatures));

  auto mergedWords =
      mergeWords(std::move(allWords), mergeWordsIdMap, mergeSignaturesIdMap);
  words.putWords(std::move(mergedWords));

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

std::list<std::unique_ptr<Word>> RTABMapDBAdapter::readWords(
    const std::string &dbPath, int dbId,
    const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &allSignatures) {
  std::list<std::unique_ptr<Word>> words;

  rtabmap::DBDriver *dbDriver = rtabmap::DBDriver::create();
  if (!dbDriver->openConnection(dbPath)) {
    qDebug() << "Connecting to database " << dbPath.c_str()
             << ", path is invalid!";
    return words;
  }

  // Read words from database
  qDebug() << "Read words from database...";
  std::set<int> wordIds;
  const auto &iter = allSignatures.find(dbId);
  assert(iter != allSignatures.end());
  for (const auto &signature : iter->second) {
    const std::multimap<int, cv::KeyPoint> &signatureWords =
        signature.second->getWords();
    for (const auto &word : signatureWords) {
      wordIds.insert(word.first);
    }
  }
  std::list<rtabmap::VisualWord *> rtabmapWords;
  dbDriver->loadWords(wordIds, rtabmapWords);
  for (auto &rtabmapWord : rtabmapWords) {
    assert(rtabmapWord != nullptr);
    int id = rtabmapWord->id();
    const cv::Mat &descriptor = rtabmapWord->getDescriptor();
    const std::vector<cv::Point3f> &points3 =
        getWordPoints3(*rtabmapWord, dbId, allSignatures);
    words.emplace_back(
        std::unique_ptr<Word>(new Word(id, descriptor, dbId, points3)));
    delete rtabmapWord;
    rtabmapWord = nullptr;
  }

  qDebug() << "Closing database " << dbDriver->getUrl().c_str() << "...";
  dbDriver->closeConnection();
  dbDriver->join();
  delete dbDriver;
  dbDriver = nullptr;

  return words;
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
      if (getPoint3World(allSignatures, dbId, imageId, x, y, pWorld)) {
        labels.emplace_back(std::unique_ptr<Label>(
            new Label(dbId, imageId, cv::Point2f(x, y),
                      cv::Point3f(pWorld.x, pWorld.y, pWorld.z), name)));
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

bool RTABMapDBAdapter::getPoint3World(
    const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &allSignatures,
    int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld) {
  // TODO: Use map of map for both signature and words
  const auto &iter = allSignatures.find(dbId);
  assert(iter != allSignatures.end());
  const auto &dbSignatures = iter->second;
  const auto &jter = dbSignatures.find(imageId);
  assert(jter != dbSignatures.end());
  const std::unique_ptr<rtabmap::Signature> &signature = jter->second;
  rtabmap::SensorData data = signature->sensorData();
  rtabmap::Transform poseWorld = signature->getPose();
  assert(!poseWorld.isNull());

  const rtabmap::CameraModel &camera = data.cameraModels()[0];
  bool smoothing = false;
  assert(!data.depthRaw().empty());
  pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(
      data.depthRaw(), x, y, camera.cx(), camera.cy(), camera.fx(), camera.fy(),
      smoothing);
  if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z)) {
    qWarning() << "Depth value not valid";
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

std::vector<cv::Point3f> RTABMapDBAdapter::getWordPoints3(
    const rtabmap::VisualWord &word, int dbId,
    const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &allSignatures) {
  std::vector<pcl::PointXYZ> points3PCL;

  std::ofstream pRawFile;
  pRawFile.open("points_raw.txt", std::ofstream::app);
  pRawFile << "word id: " << word.id() << std::endl;
  const auto &iter = allSignatures.find(dbId);
  assert(iter != allSignatures.end());
  const auto &dbSignatures = iter->second;
  for (const auto &dbSignature : dbSignatures) {
    const rtabmap::Transform &pose = dbSignature.second->getPose();
    const auto &range = dbSignature.second->getWords3().equal_range(word.id());
    for (auto jter = range.first; jter != range.second; jter++) {
      const cv::Point3f &localPointCV = jter->second;
      pcl::PointXYZ localPointPCL(localPointCV.x, localPointCV.y,
                                  localPointCV.z);
      pcl::PointXYZ globalPointPCL =
          pcl::transformPoint(localPointPCL, pose.toEigen3f());
      pRawFile << globalPointPCL.x << " " << globalPointPCL.y << " "
               << globalPointPCL.z << " " << pose.prettyPrint() << " "
               << dbSignature.second->id() << std::endl;
      points3PCL.emplace_back(std::move(globalPointPCL));
    }
  }
  pRawFile << std::endl;
  pRawFile.close();

  points3PCL = clusterPoints3(points3PCL);

  std::ofstream pCookedFile;
  pCookedFile.open("points_cooked.txt", std::ofstream::app);
  pCookedFile << "word id: " << word.id() << std::endl;
  std::vector<cv::Point3f> points3CV;
  for (const auto &point3PCL : points3PCL) {
    cv::Point3f point3CV = cv::Point3f(point3PCL.x, point3PCL.y, point3PCL.z);
    pCookedFile << point3CV.x << " " << point3CV.y << " " << point3CV.z << " "
                << std::endl;
    points3CV.emplace_back(std::move(point3CV));
  }
  pCookedFile << std::endl;
  pCookedFile.close();

  return points3CV;
}

std::vector<pcl::PointXYZ>
RTABMapDBAdapter::clusterPoints3(const std::vector<pcl::PointXYZ> &points3) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &point3 : points3) {
    cloud->push_back(point3);
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.3); // in meter
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

  return clusteredPoints3;
}

std::map<std::pair<int, int>, int> RTABMapDBAdapter::getMergeWordsIdMap(
    const std::map<int, std::list<std::unique_ptr<Word>>> &allWords) {
  std::map<std::pair<int, int>, int> mergeWordsIdMap;
  int nextWordId = 1;
  for (const auto &dbWords : allWords) {
    int dbId = dbWords.first;
    for (const auto &word : dbWords.second) {
      int wordId = word->getId();
      mergeWordsIdMap.insert(
          std::make_pair(std::make_pair(dbId, wordId), nextWordId));
      nextWordId++;
    }
  }
  return mergeWordsIdMap;
}

std::map<std::pair<int, int>, int> RTABMapDBAdapter::getMergeSignaturesIdMap(
    const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &allSignatures) {
  std::map<std::pair<int, int>, int> mergeSignaturesIdMap;
  int nextSignatureId = 1;
  for (const auto &dbSignatures : allSignatures) {
    int dbId = dbSignatures.first;
    for (const auto &signature : dbSignatures.second) {
      int signatureId = signature.second->id();
      mergeSignaturesIdMap.insert(
          std::make_pair(std::make_pair(dbId, signatureId), nextSignatureId));
      nextSignatureId++;
    }
  }
  return mergeSignaturesIdMap;
}

std::list<std::unique_ptr<Word>> RTABMapDBAdapter::mergeWords(
    std::map<int, std::list<std::unique_ptr<Word>>> &&allWords,
    const std::map<std::pair<int, int>, int> &mergeWordsIdMap,
    const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap) {
  std::list<std::unique_ptr<Word>> mergedWords;

  for (const auto &dbWords : allWords) {
    int dbId = dbWords.first;
    for (const auto &word : dbWords.second) {
      int wordId = word->getId();
      auto wordIdIter = mergeWordsIdMap.find(std::make_pair(dbId, wordId));
      assert(wordIdIter != mergeWordsIdMap.end());

      int newId = wordIdIter->second;
      const cv::Mat &descriptor = word->getDescriptor();
      int dbId = word->getDbId();
      const std::vector<cv::Point3f> &points3 = word->getPoints3();
      mergedWords.emplace_back(
          std::unique_ptr<Word>(new Word(newId, descriptor, dbId, points3)));
    }
  }
  return mergedWords;
}

std::list<std::unique_ptr<Signature>> RTABMapDBAdapter::mergeSignatures(
    std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
        &&allSignatures,
    const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap,
    const std::map<std::pair<int, int>, int> &mergeWordsIdMap) {
  std::list<std::unique_ptr<Signature>> mergedSignatures;

  for (const auto &dbSignatures : allSignatures) {
    int dbId = dbSignatures.first;
    for (const auto &signature : dbSignatures.second) {
      int signatureId = signature.second->id();
      auto signatureIdIter =
          mergeSignaturesIdMap.find(std::make_pair(dbId, signatureId));
      assert(signatureIdIter != mergeSignaturesIdMap.end());

      int newId = signatureIdIter->second;
      int mapId = signature.second->mapId();
      Transform pose =
          Transform::fromEigen4f(signature.second->getPose().toEigen4f());

      std::multimap<int, cv::KeyPoint> words;
      for (const auto &word : signature.second->getWords()) {
        auto wordIdIter =
            mergeWordsIdMap.find(std::make_pair(dbId, word.first));
        assert(wordIdIter != mergeWordsIdMap.end());
        words.insert(std::make_pair(wordIdIter->second, word.second));
      }

      std::multimap<int, cv::Point3f> words3;
      for (const auto &word3 : signature.second->getWords3()) {
        auto wordIdIter =
            mergeWordsIdMap.find(std::make_pair(dbId, word3.first));
        assert(wordIdIter != mergeWordsIdMap.end());
        words3.insert(std::make_pair(wordIdIter->second, word3.second));
      }
      mergedSignatures.emplace_back(std::unique_ptr<Signature>(
          new Signature(newId, mapId, dbId, std::move(pose), std::move(words),
                        std::move(words3))));
    }
  }

  return mergedSignatures;
}
