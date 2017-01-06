#include "adapter/rtabmap/RTABMapDBAdapter.h"
#include "data/Labels.h"
#include "data/Transform.h"
#include "data/Words.h"
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
    allSignatures.emplace(dbId, std::move(dbSignatures));

    auto dbLabels = readLabels(dbPath, dbId, allSignatures);
    std::move(dbLabels.begin(), dbLabels.end(), std::back_inserter(allLabels));

    dbId++;
  }

  std::cout << "Building Index for Words" << std::endl;
  std::list<std::unique_ptr<Word>> allWords = createWords(allSignatures);
  std::cout << "Total Number of words: " << allWords.size() << std::endl;
  long count = 0;
  for (const auto &word : allWords) {
    for (const auto &desc : word->getDescriptorsByDb()) {
      count += desc.second.rows;
    }
  }
  std::cout << "Total Number of descriptors: " << count << std::endl;
  allWords = clusterPointsInWords(allWords);
  count = 0;
  for (const auto &word : allWords) {
    for (const auto &desc : word->getDescriptorsByDb()) {
      count += desc.second.rows;
    }
  }
  std::cout << "Total Number of points: " << count << std::endl;
  words.putWords(std::move(allWords));

  std::cout << "Building Index for Labels" << std::endl;
  labels.putLabels(std::move(allLabels));

  return true;
}

std::map<int, std::unique_ptr<rtabmap::Signature>>
RTABMapDBAdapter::readSignatures(const std::string &dbPath) {
  std::map<int, std::unique_ptr<rtabmap::Signature>> signatures;

  // get optimized poses of signatures
  std::cout << "Optimize poses of signatures..." << std::endl;
  const std::map<int, rtabmap::Transform> &optimizedPoseMap =
      getOptimizedPoseMap(dbPath);

  rtabmap::DBDriver *dbDriver = rtabmap::DBDriver::create();
  if (!dbDriver->openConnection(dbPath)) {
    std::cout << "Connecting to database " << dbPath << ", path is invalid!"
              << std::endl;
    return signatures;
  }

  // Read signatures from database
  std::cout << "Read signatures from database..." << std::endl;
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

  std::cout << "Closing database " << dbDriver->getUrl() << "..." << std::endl;
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
    std::cout << "Could not open database " << sqlite3_errmsg(db) << std::endl;
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
        labels.emplace_back(std::unique_ptr<Label>(
            new Label(dbId, cv::Point3f(pWorld.x, pWorld.y, pWorld.z), name)));
        std::cout << "Read point (" << pWorld.x << "," << pWorld.y << ","
                  << pWorld.z << ") with label " << name << " in database "
                  << dbPath << std::endl;
      }
    }
  } else {
    std::cout << "Could not read database " << dbPath << ":"
              << sqlite3_errmsg(db) << std::endl;
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
  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
  for (const auto &dbSignatures : allSignatures) {
    int dbId = dbSignatures.first;
    std::cout << "Creating words for signatures in DB " << dbId << std::endl;
    for (const auto &dbSignature : dbSignatures.second) {
      // get image normal vector
      const rtabmap::Transform &poseWorld = dbSignature.second->getPose();
      assert(!poseWorld.isNull());
      pcl::PointXYZ normalPCL = pcl::transformPoint(
          pcl::PointXYZ(0, 0, 1), poseWorld.rotation().toEigen3f());
      cv::Point3f normalCV(normalPCL.x, normalPCL.y, normalPCL.z);

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

          auto iter = wordsMap.find(wordId);
          if (iter == wordsMap.end()) {
            auto ret = wordsMap.emplace(
                wordId, std::unique_ptr<Word>(new Word(wordId)));
            iter = ret.first;
          }
          iter->second->addPoints3(dbId, std::vector<cv::Point3f>(1, point3CV),
                                   std::vector<cv::Point3f>(1, normalCV),
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
  for (const auto &word : words) {
    std::unique_ptr<Word> newWord =
        std::unique_ptr<Word>(new Word(word->getId()));
    // cluster points and calculate mean descriptor for every cluster
    for (const auto &oldPoints3CV : word->getPoints3Map()) {
      int dbId = oldPoints3CV.first;
      std::vector<pcl::PointXYZ> oldPoints3PCL;
      for (const auto &oldPoint3CV : oldPoints3CV.second) {
        oldPoints3PCL.emplace_back(oldPoint3CV.x, oldPoint3CV.y, oldPoint3CV.z);
      }
      std::vector<pcl::PointIndices> clusterIndices;
      std::vector<pcl::PointXYZ> newPoints3PCL =
          clusterPoints3(oldPoints3PCL, &clusterIndices);
      std::vector<cv::Point3f> newPoints3CV;
      for (const auto &newPoint3PCL : newPoints3PCL) {
        newPoints3CV.emplace_back(newPoint3PCL.x, newPoint3PCL.y,
                                  newPoint3PCL.z);
      }

      // create new descriptors based on indices
      cv::Mat oldDescriptors = word->getDescriptorsByDb().at(dbId);
      cv::Mat newDescriptors; // descriptors of clustered points
      const auto &oldNormals = word->getNormalsMap().at(dbId);
      std::vector<cv::Point3f> newNormals;
      for (const auto &clusterIndice : clusterIndices) {
        cv::Mat clusterDescriptors;
        cv::Mat meanDescriptor;
        std::vector<cv::Point3f> clusterNormals;
        for (int index : clusterIndice.indices) {
          clusterDescriptors.push_back(oldDescriptors.row(index));
          clusterNormals.emplace_back(oldNormals.at(index));
        }
        cv::reduce(clusterDescriptors, meanDescriptor, 0, CV_REDUCE_AVG);
        cv::Point3f sum = std::accumulate(
            clusterNormals.begin(), clusterNormals.end(), cv::Point3f(0, 0, 0));
        newDescriptors.push_back(meanDescriptor);
        newNormals.emplace_back(sum.x / clusterNormals.size(),
                                sum.y / clusterNormals.size(),
                                sum.z / clusterNormals.size());
      }

      newWord->addPoints3(dbId, newPoints3CV, newNormals, newDescriptors);
    }

    newWords.emplace_back(std::move(newWord));
  }

  return newWords;
}

bool RTABMapDBAdapter::getPoint3World(const rtabmap::Signature &signature,
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
    // std::cout << "Depth value not valid" << std::endl;
    return false;
  }
  if (poseWorld.isNull()) {
    std::cout << "Image pose is Null" << std::endl;
    return false;
  }
  poseWorld = poseWorld * camera.localTransform();
  point3 = pcl::transformPoint(pLocal, poseWorld.toEigen3f());
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
