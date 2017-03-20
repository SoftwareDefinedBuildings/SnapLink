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
#include <boost/program_options.hpp>
namespace po = boost::program_options;


static void printInvalid(const std::vector<std::string> &opts);
static void printUsage(const po::options_description &desc);
std::vector<float> getPoseFromFileVector(std::string camaraPoseFile);
cv::Mat_<float> getPoseFromFileMat(std::string);
cv::Mat_<float> makeVector(float a, float b, float c);

int vis(int argc, char* argv[]) {
  std::string dbFile = argv[1];
  std::string camaraPoseFile = argv[2];

  //get posesMap and linksMap
  rtabmap::Memory memory;
  memory.init(dbFile);
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
  driver->openConnection(dbFile);
  std::set<int> ids;
  driver->getAllNodeIds(ids, true);
  std::cout<<"Number of pictures : "<<ids.size()<<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  rtabmap::CameraModel cm;
  for(auto id : ids) {
    if(optimizedPoseMap.count(id) == 0) {
      //this image is being optimized out
      continue;
    }
    std::cout<<"id is "<<id<<std::endl;
    int imageId = id;
    bool uncompressedData = true;
    rtabmap::SensorData data = memory.getNodeData(imageId, uncompressedData);
    cm = data.cameraModels()[0];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Mat depthRaw = data.depthRaw();
    cv::Mat imageRaw = data.imageRaw();
    //Todo Check what's the difference between my implementation and rtabmap's cloudFromDepthRGB()
    //My implementation is at the botom of this file
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
  cv::Mat cloudBGR(1, totalSize, CV_8UC3);
  cv::Point3f* XYZdata = cloudXYZ.ptr<cv::Point3f>();
  for(int i = 0; i < totalSize; i++) {
    pcl::PointXYZRGB & pt = assembledCloud->at(i);
    XYZdata[i].x = pt.x;
    XYZdata[i].y = pt.y;
    XYZdata[i].z = pt.z;
    cloudBGR.at<cv::Vec3b>(i)[0] = pt.b;
    cloudBGR.at<cv::Vec3b>(i)[1] = pt.g;
    cloudBGR.at<cv::Vec3b>(i)[2] = pt.r;
  }


  cv::viz::Viz3d myWindow("Coordinate Frame");
  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  std::vector<float> datas = getPoseFromFileVector(camaraPoseFile);
  std::cout<<"happen-4?\n";
  // std::cout<<cm.localTransform().rotationMatrix();
  // cv::Mat_<float> focal_point = cm.localTransform().rotation().dataMatrix() * makeVector(datas[2],datas[6],datas[10]);
  // cv::Vec3f cam_pos(0,0,0), cam_focal_point(0.0f,0.0f,1.0f), cam_y_dir(0.0f,1.0f,0.0f);
  std::cout<<"happen-3?\n";
  // c, -a, -b
  // 2, 6,  10
  // a, b, c
  // 10, -2. -6
  cv::Vec3f cam_pos(datas[3],datas[7],datas[11]), cam_focal_point(datas[0], datas[4], datas[8]), cam_y_dir(0.0f,1.0f,0.0f);
  std::cout<<"happen-2?\n";
  cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);


  std::cout<<"happen-1?\n";

  // cv::Affine3f transform = cv::viz::makeTransformToGlobal(cv::Vec3f(0.0f,-1.0f,0.0f), cv::Vec3f(-1.0f,0.0f,0.0f), cv::Vec3f(0.0f,0.0f,-1.0f), cam_pos);
  // cv::Affine3f cam_pose(getPoseFromFileMat(camaraPoseFile));
  std::cout<<"camara pose\n";
  std::cout<<cam_pose.rotation()<<std::endl;
  std::cout<<cam_pose.translation()<<std::endl;
  std::cout<<"happen1?\n";
  cv::viz::WCloud cloud_widget(cloudXYZ,  cloudBGR);
  std::cout<<"happen2?\n";
  // cv::Affine3f cloud_pose = cv::Affine3f().translate(cv::Vec3f(0.0f,0.0f,0.0f));
  // cv::Affine3f cloud_pose_global = transform * cloud_pose;
  cv::Affine3f cloud_pose_global = cv::Affine3f().translate(cv::Vec3f(0.0f,0.0f,0.0f));
  std::cout<<"happen3?\n";

  cv::viz::WCameraPosition cpw(0.5); // Coordinate axes
  std::cout<<"happen4?\n";
  cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599)); // Camera frustum
  std::cout<<"happen5?\n";
  myWindow.showWidget("CPW", cpw, cam_pose);
  std::cout<<"happen6?\n";
  myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
  std::cout<<"happen7?\n";
  myWindow.showWidget("bunny", cloud_widget, cloud_pose_global);
  std::cout<<"happen8?\n";
  myWindow.spin();
  std::cout<<"happen9?\n";
  return 0;
}


std::vector<float> getPoseFromFileVector(std::string camaraPoseFile) {
  std::ifstream fin(camaraPoseFile);
  if(fin.fail()) {
    std::cout<<"Target file open failed";
  }
  std::string dummy;
  fin>>dummy;
  std::vector<float> datas;

  std::cout<<"Target file is "<<camaraPoseFile<<"\n";
  std::cout<<"Pose read is:\n";
  for(int i = 0; i < 12; i++) {
    float temp;
    fin>>temp;
    datas.push_back(temp);
    std::cout<<datas[i]<<"  ";
    if((i+1)%4 == 0) {
      std::cout<<"\n";
    }
  }
  fin.close();
  // return cv::Mat_<float>(3, 4) << datas[0], datas[1], datas[2], datas[3],
  //                                 datas[4], datas[5], datas[6], datas[7],
  //                                 datas[8], datas[9], datas[10], datas[11];
  return datas;
}

cv::Mat_<float> makeVector(float a, float b, float c) {
  return cv::Mat_<float>(3,1) << a, b, c;
}


cv::Mat_<float> getPoseFromFileMat(std::string camaraPoseFile) {
  std::ifstream fin(camaraPoseFile);
  if(fin.fail()) {
    std::cout<<"Target file open failed";
  }
  std::string dummy;
  fin>>dummy;
  float datas[12];

  std::cout<<"Target file is "<<camaraPoseFile<<"\n";
  std::cout<<"Pose read is:\n";
  for(int i = 0; i < 12; i++) {
    fin>>datas[i];
    std::cout<<datas[i]<<"  ";
    if((i+1)%4 == 0) {
      std::cout<<"\n";
    }
  }
  fin.close();
  return cv::Mat_<float>(3, 4) << datas[0], datas[1], datas[2], datas[3],
                                  datas[4], datas[5], datas[6], datas[7],
                                  datas[8], datas[9], datas[10], datas[11];
  // return cv::Mat_<float>(3, 4) << 1,0,0,0,
  //                                 0,1,0,0,
  //                                 0,0,1,1;
}

static void printInvalid(const std::vector<std::string> &opts) {
  std::cerr << "invalid options: ";
  for (const auto &opt : opts) {
    std::cerr << opt << " ";
  }
  std::cerr << std::endl;
}

static void printUsage(const po::options_description &desc) {
  std::cout << "cellmate vis [command options]" << std::endl
            << std::endl
            << desc << std::endl;
}


//tried to add code to parse arguments
// Parse arguments
// std::string dbFile;
//
// po::options_description label("command options");
// label.add_options() // use comment to force new line using formater
//     ("help,h", "print help message") //
//     ("dbfile", po::value<std::string>(&dbFile)->required(), "database file")
//     ("P", po::value< std::vector<double> >(), "pose");
//
// po::positional_options_description pos;
// pos.add("dbfile", 1);
//
// po::variables_map vm;
// po::parsed_options parsed = po::command_line_parser(argc, argv)
//                                 .options(label)
//                                 .positional(pos)
//                                 .allow_unregistered()
//                                 .run();
// po::store(parsed, vm);
// po::notify(vm);
//
// // print invalid options
// std::vector<std::string> unrecog =
//     collect_unrecognized(parsed.options, po::exclude_positional);
// if (unrecog.size() > 0) {
//   printInvalid(unrecog);
//   printUsage(label);
//   return 1;
// }
//
// if (vm.count("help")) {
//   printUsage(label);
//   return 0;
// }

// My implementation
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
