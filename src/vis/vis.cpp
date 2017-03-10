#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Memory.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/projection_matrix.h>
#include "vis/vis.h"
#include <rtabmap/utilite/UStl.h>
#include "rtabmap/core/RtabmapEvent.h"
#include <iostream>
#include <rtabmap/core/util3d.h>
/*
int main1() {
  std::cout<<"hello\n";
  //get signatures
  DBDriver * driver = DBDriver::create();
  driver->openConnection("/root/workspace/tempdata/face_down.db");
  std::set<int> ids;
  driver->getAllNodeIds(ids, true);
  std::cout<<"Number of pictures : "<<ids.size()<<std::endl;
  std::list<Signature*> signaturesList;
  driver->loadSignatures(std::list<int>(ids.begin(), ids.end()), signaturesList);
  std::map<int, Signature> signatures;
  driver->loadNodeData(signaturesList);
  for(std::list<Signature *>::iterator iter=signaturesList.begin(); iter!=signaturesList.end(); ++iter) {
      signatures.insert(std::make_pair((*iter)->id(), *(*iter)));
      delete *iter;
  }
  //get posesMap and linksMap
  rtabmap::Memory memory;
  memory.init(dbPath);
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> links;
  if (memory.getLastWorkingSignature()) {
      // Get all IDs linked to last signature (including those in Long-Term
      //  Memory)
      std::map<int, int> ids =
          memory.getNeighborsId(memory.getLastWorkingSignature()->id(), 0, -1);
      // Get all metric constraints (the graph)
      memory.getMetricConstraints(uKeysSet(ids), poses, links, true);
  }
  // Optimize the graph
  std::map<int, rtabmap::Transform> optimizedPoseMap;
  rtabmap::Optimizer *graphOptimizer =
                rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeTORO);
  optimizedPoseMap =
                graphOptimizer->optimize(poses.begin()->first, poses, links);
  delete graphOptimizer;
  //process updateCache
  //
  std::map<int, int> mapIds;
  std::map<int, Transform> groundTruth;
  std::map<int, std::string> labels;
  for(std::map<int, Signature>::const_iterator iter = signatures.begin();
          iter!=signatures.end();
          ++iter)
  {
    mapIds.insert(std::make_pair(iter->first, iter->second.mapId()));
    if(!iter->second.getGroundTruthPose().isNull()) {
      groundTruth.insert(std::make_pair(iter->first, iter->second.getGroundTruthPose()));
    }
    if(!iter->second.getLabel().empty()) {
      labels.insert(std::make_pair(iter->first, iter->second.getLabel()));
    }
  }
  // filter duplicated poses
  // Mainwindow.cpp 1864, do we need to do filtering for poses?

  for(std::map<int, Transform>::const_iterator iter = optimizedPoseMap.begin(); iter!=poses.end(); ++iter)
  {
    if(!iter->second.isNull()){
      std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::IndicesPtr>
                createdCloud = this->createAndAddCloudToMap(iter->first,
                                                            iter->second, uValue(mapIds, iter->first, -1));
    }


  }    

  return 0;  
}
*/

int main1() {
  std::string dbPath = "/root/workspace/tempdata/face_down.db"; 
  std::cout<<"hello\n";
  rtabmap::DBDriver * driver = rtabmap::DBDriver::create();
  driver->openConnection(dbPath);
  std::set<int> ids;
  driver->getAllNodeIds(ids, true);
  std::cout<<"Number of pictures : "<<ids.size()<<std::endl;
  rtabmap::Memory memory;
  memory.init(dbPath);
  for(auto id : ids) {
    std::cout<<"id is "<<id<<std::endl;
  
  
  int imageId = id;
  bool uncompressedData = true; 
  rtabmap::SensorData data = memory.getNodeData(imageId, uncompressedData);
  const rtabmap::CameraModel &cm = data.cameraModels()[0];
  bool smoothing = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cv::Mat depthRaw = data.depthRaw();
  cv::Mat imageRaw = data.imageRaw();
  cloud->height = depthRaw.rows;
  cloud->width  = depthRaw.cols;
  cloud->resize(cloud->height * cloud->width);
    
  for(int i = 0; i < depthRaw.rows; i++) {
    for(int j = 0; j < depthRaw.cols; j++) {
      pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(
                data.depthRaw(), i, j, cm.cx(), cm.cy(), cm.fx(), cm.fy(), smoothing);
      const unsigned char * bgr = imageRaw.ptr<unsigned char>(i,j);
      pcl::PointXYZRGB & pt = cloud->at(i*cloud->width + j);
      pt.x = pLocal.x;
      pt.y = pLocal.y;
      pt.z = pLocal.z;
      pt.b = bgr[0];
      pt.g = bgr[1];
      pt.r = bgr[2];
      if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z)) {
        //std::cout<<"Depth value not valid\n";
        
        pt.x = pt.y = pt.z = pt.b = pt.g = pt.r = 0;;
      }
    }
  }

  pcl::io::savePLYFile("output"+std::to_string(id)+".ply", *cloud, false);
  }
  return 0; 

}

