#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Memory.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/projection_matrix.h>
#include "vis/vis.h"
#include <rtabmap/utilite/UStl.h>
#include "rtabmap/core/RtabmapEvent.h"
#include <rtabmap/core/Optimizer.h>
#include <iostream>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <opencv2/viz.hpp>
#include <fstream>


// static cv::Mat cvcloud_load()
// {
//     std::ifstream ifs("output.ply");
//     int lineCount = 0;
//     std::string buffer;
//     while(getline(ifs, buffer);) {
//       lineCount++;
//     }
//     cv::Mat cloud(1, lineCount, CV_32FC3);
//
//     for(size_t i = 0; i < 32; ++i)
//         getline(ifs, str);
//     cv::Point3f* data = cloud.ptr<cv::Point3f>();
//     float dummy1, dummy2;
//     for(size_t i = 0; i < 1889; ++i)
//         ifs >> data[i].x >> data[i].y >> data[i].z >> dummy1 >> dummy2;
//     cloud *= 5.0f;
//     return cloud;
// }


int main1() {
  std::string dbPath = "face_down.db";
  std::cout<<"hello\n";


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

  rtabmap::DBDriver * driver = rtabmap::DBDriver::create();
  driver->openConnection(dbPath);
  std::set<int> ids;
  driver->getAllNodeIds(ids, true);
  std::cout<<"Number of pictures : "<<ids.size()<<std::endl;
  //rtabmap::Memory memory;
  //memory.init(dbPath);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(auto id : ids) {
    if(optimizedPoseMap.count(id) == 0) {
      //this image is being optimized out
      continue;
    }
    std::cout<<"id is "<<id<<std::endl;
    int imageId = id;
    bool uncompressedData = true;
    rtabmap::SensorData data = memory.getNodeData(imageId, uncompressedData);
    const rtabmap::CameraModel &cm = data.cameraModels()[0];
    bool smoothing = false;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Mat depthRaw = data.depthRaw();
    cv::Mat imageRaw = data.imageRaw();
    //Todo Check what's the difference between my implementation and rtabmap's cloudFromDepthRGB()
    // cloud->height = depthRaw.rows;
    // cloud->width  = depthRaw.cols;
    // cloud->resize(cloud->height * cloud->width);
    //
    // for(int i = 0; i < depthRaw.rows; i++) {
    //   for(int j = 0; j < depthRaw.cols; j++) {
    //     pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(
    //               data.depthRaw(), i, j, cm.cx(), cm.cy(), cm.fx(), cm.fy(), smoothing);
    //     const unsigned char * bgr = imageRaw.ptr<unsigned char>(i,j);
    //     pcl::PointXYZRGB & pt = cloud->at(i*cloud->width + j);
    //     pt.x = pLocal.x;
    //     pt.y = pLocal.y;
    //     pt.z = pLocal.z;
    //     pt.b = bgr[0];
    //     pt.g = bgr[1];
    //     pt.r = bgr[2];
    //     if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z)) {
    //       //std::cout<<"Depth value not valid\n";
    //
    //       pt.x = pt.y = pt.z = pt.b = pt.g = pt.r = 0;;
    //     }
    //   }
    // }
    cloud = rtabmap::util3d::cloudFromDepthRGB(imageRaw, depthRaw,cm,4,0,0,nullptr);
    cloud = rtabmap::util3d::removeNaNFromPointCloud(cloud);
    cloud = rtabmap::util3d::transformPointCloud(cloud, cm.localTransform());
    cloud = rtabmap::util3d::transformPointCloud(cloud, optimizedPoseMap.at(id));
    *assembledCloud += *cloud;
    std::cout<<optimizedPoseMap.at(id);
  }
  // pcl::io::savePLYFile("output.ply", *assembledCloud, false);
  int totalSize = assembledCloud->size();
  cv::Mat cloudXYZ(1, totalSize, CV_32FC3);
  cv::Mat cloudBGR(1, totalSize, CV_32FC3);
  cv::Point3f* XYZdata = cloudXYZ.ptr<cv::Point3f>();
  cv::Point3f* BGRdata = cloudBGR.ptr<cv::Point3f>();
  for(int i = 0; i < totalSize; i++) {
    pcl::PointXYZRGB & pt = assembledCloud->at(i);
    XYZdata[i].x = pt.x;
    XYZdata[i].y = pt.y;
    XYZdata[i].z = pt.z;
    BGRdata[i].x = pt.b;
    BGRdata[i].x = pt.g;
    BGRdata[i].x = pt.r;
  }
  bool camera_pov = false;
  cv::viz::Viz3d myWindow("Coordinate Frame");
  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  cv::Vec3f cam_pos(3.0f,3.0f,3.0f), cam_focal_point(3.0f,3.0f,2.0f), cam_y_dir(-1.0f,0.0f,0.0f);
  cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
  cv::Affine3f transform = cv::viz::makeTransformToGlobal(cv::Vec3f(0.0f,-1.0f,0.0f), cv::Vec3f(-1.0f,0.0f,0.0f), cv::Vec3f(0.0f,0.0f,-1.0f), cam_pos);
  cv::viz::WCloud cloud_widget(cloudXYZ,  cv::viz::Color::green());
  cv::Affine3f cloud_pose = cv::Affine3f().translate(cv::Vec3f(0.0f,0.0f,3.0f));
  cv::Affine3f cloud_pose_global = transform * cloud_pose;
  if (!camera_pov)
  {
      cv::viz::WCameraPosition cpw(0.5); // Coordinate axes
      cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599)); // Camera frustum
      myWindow.showWidget("CPW", cpw, cam_pose);
      myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
  }
  myWindow.showWidget("bunny", cloud_widget, cloud_pose_global);
  if (camera_pov)
      myWindow.setViewerPose(cam_pose);
  myWindow.spin();
  return 0;
}
