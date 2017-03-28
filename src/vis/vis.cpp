#include "vis/vis.h"
#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include "rtabmap/core/RtabmapEvent.h"
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/viz.hpp>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/ply_io.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UStl.h>
namespace po = boost::program_options;

void printTransformMat(Transform t);
static void printInvalid(const std::vector<std::string> &opts);
static void printUsage(const po::options_description &desc);
std::vector<float> getPoseFromFileVector(std::string camaraPoseFile,
                                         int resultId);
cv::Mat_<float> makeCvMat(float a1, float a2, float a3, float a4, float a5,
                          float a6, float a7, float a8, float a9, float a10,
                          float a11, float a12);
cv::Mat_<float> makeCvMatRotation(float a1, float a2, float a3, float a4,
                                  float a5, float a6, float a7, float a8,
                                  float a9, float a10, float a11, float a12);

int vis(int argc, char *argv[]) {
  std::set<std::string> dbFile;
  dbFile.push_back(argv[1]);
  std::string camaraPoseFile = argv[2];
  int resultId = atoi(argv[3]);
  int decimation = 4;
  RTABMapAdapter adapter;
  if (!adapter.init(std::set<std::string>(dbFiles.begin(), dbFiles.end()))) {
    std::cerr << "reading data failed";
    return 1;
  }
  const std::map<int, std::vector<Image>> &images = adapter.getImages();
  Image targetRoomImages = images.begin();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (auto image : targetRoomImages) {
    const CameraModel cm = image.getCameraModel();
    const Transform pose = image.getPose();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = image.getCloud(4);
    cloud = rtabmap::util3d::transformPointCloud(cloud, cm.localTransform());
    cloud = rtabmap::util3d::transformPointCloud(cloud, pose);
    *assembledCloud += *cloud;
  }

  // // get posesMap and linksMap
  // rtabmap::Memory memory;
  // memory.init(dbFile);
  // std::map<int, rtabmap::Transform> poses;
  // std::multimap<int, rtabmap::Link> links;
  // if (memory.getLastWorkingSignature()) {
  //   // Get all IDs linked to last signature (including those in Long-Term
  //   //  Memory)
  //   std::map<int, int> ids =
  //       memory.getNeighborsId(memory.getLastWorkingSignature()->id(), 0, -1);
  //   // Get all metric constraints (the graph)
  //   memory.getMetricConstraints(uKeysSet(ids), poses, links, true);
  // }
  // // Optimize the graph
  // std::map<int, rtabmap::Transform> optimizedPoseMap;
  // rtabmap::Optimizer *graphOptimizer =
  //     rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeTORO);
  // optimizedPoseMap =
  //     graphOptimizer->optimize(poses.begin()->first, poses, links);
  // delete graphOptimizer;
  //
  // rtabmap::DBDriver *driver = rtabmap::DBDriver::create();
  // driver->openConnection(dbFile);
  // std::set<int> ids;
  // driver->getAllNodeIds(ids, true);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(
  //     new pcl::PointCloud<pcl::PointXYZRGB>);
  //
  // rtabmap::CameraModel cm;
  // for (auto id : ids) {
  //   if (optimizedPoseMap.count(id) == 0) {
  //     // this image is being optimized out
  //     continue;
  //   }
  //   int imageId = id;
  //   bool uncompressedData = true;
  //   rtabmap::SensorData data = memory.getNodeData(imageId, uncompressedData);
  //   cm = data.cameraModels()[0];
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
  //       new pcl::PointCloud<pcl::PointXYZRGB>);
  //   cv::Mat depthRaw = data.depthRaw();
  //   cv::Mat imageRaw = data.imageRaw();
  //   // Todo Check what's the difference between my implementation and
  //   rtabmap's
  //   // cloudFromDepthRGB()
  //   // My implementation is at the botom of this file
  //   cloud = rtabmap::util3d::cloudFromDepthRGB(imageRaw, depthRaw, cm, 4, 0,
  //   0,
  //                                              nullptr);
  //   cloud = rtabmap::util3d::removeNaNFromPointCloud(cloud);
  //   cloud = rtabmap::util3d::transformPointCloud(cloud, cm.localTransform());
  //   cloud =
  //       rtabmap::util3d::transformPointCloud(cloud, optimizedPoseMap.at(id));
  //   *assembledCloud += *cloud;
  // }
  float maxX = 0.0f;
  float maxY = 0.0f;
  float maxZ = 0.0f;
  int totalSize = assembledCloud->size();
  cv::Mat cloudXYZ(1, totalSize, CV_32FC3);
  cv::Mat cloudBGR(1, totalSize, CV_8UC3);
  cv::Point3f *XYZdata = cloudXYZ.ptr<cv::Point3f>();
  for (int i = 0; i < totalSize; i++) {
    pcl::PointXYZRGB &pt = assembledCloud->at(i);
    XYZdata[i].x = pt.x;
    XYZdata[i].y = pt.y;
    XYZdata[i].z = pt.z;
    if (pt.x > maxX) {
      maxX = pt.x;
    }
    if (pt.y > maxY) {
      maxY = pt.y;
    }
    if (pt.z > maxZ) {
      maxZ = pt.z;
    }
    cloudBGR.at<cv::Vec3b>(i)[0] = pt.b;
    cloudBGR.at<cv::Vec3b>(i)[1] = pt.g;
    cloudBGR.at<cv::Vec3b>(i)[2] = pt.r;
  }
  std::cout << "Max X is " << maxX << "\n";
  std::cout << "Max Y is " << maxY << "\n";
  std::cout << "Max Z is " << maxZ << "\n";

  cv::viz::Viz3d myWindow("Coordinate Frame");
  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  std::vector<float> datas = getPoseFromFileVector(camaraPoseFile, resultId);
  // std::vector<float> datas;
  // datas.push_back(0.992269);
  // datas.push_back(-0.055068);
  // datas.push_back(-0.111220);
  // datas.push_back(-0.052803);
  // datas.push_back(0.050433);
  // datas.push_back(0.997755);
  // datas.push_back(-0.044067);
  // datas.push_back(-0.140940);
  // datas.push_back(0.113397);
  // datas.push_back(0.038117);
  // datas.push_back(0.992819);
  // datas.push_back(0.021211);
  Transform transP(datas[0], datas[1], datas[2], datas[3], datas[4], datas[5],
                   datas[6], datas[7], datas[8], datas[9], datas[10],
                   datas[11]);
  Transform transL(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
  Transform transPL = transP * transL * transL;
  std::cout << "transofrm PL is:\n";
  printTransformMat(transPL);
  cv::Affine3f cam_pose(
      makeCvMatRotation(transPL.r11(), transPL.r12(), transPL.r13(),
                        transPL.x(), transPL.r21(), transPL.r22(),
                        transPL.r23(), transPL.y(), transPL.r31(),
                        transPL.r32(), transPL.r33(), transPL.z()),
      cv::Vec3f(transPL.x(), transPL.y(), transPL.z()));
  std::cout << "camara pose\n";
  std::cout << cam_pose.rotation() << std::endl;
  std::cout << cam_pose.translation() << std::endl;
  cv::viz::WCloud cloud_widget(cloudXYZ, cloudBGR);
  cv::Affine3f cloud_pose_global =
      cv::Affine3f().translate(cv::Vec3f(0.0f, 0.0f, 0.0f));
  cv::viz::WCameraPosition cpw(0.5); // Coordinate axes
  cv::viz::WCameraPosition cpw_frustum(
      cv::Vec2f(0.889484, 0.523599)); // Camera frustum
  myWindow.showWidget("CPW", cpw, cam_pose);
  myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
  myWindow.showWidget("Cellmate vis", cloud_widget, cloud_pose_global);
  myWindow.spin();
  return 0;
}

std::vector<float> getPoseFromFileVector(std::string camaraPoseFile,
                                         int resultId) {
  std::ifstream fin(camaraPoseFile);
  if (fin.fail()) {
    std::cout << "Target file open failed";
  }

  std::string dummy;
  for (int i = 0; i < resultId; i++) {
    std::getline(fin, dummy);
    std::getline(fin, dummy);
    std::getline(fin, dummy);
    std::getline(fin, dummy);
  }
  fin >> dummy;
  std::vector<float> datas;

  std::cout << "Target item is " << dummy << "\n";
  std::cout << "Pose read is:\n";
  for (int i = 0; i < 12; i++) {
    float temp;
    fin >> temp;
    datas.push_back(temp);
    std::cout << datas[i] << "  ";
    if ((i + 1) % 4 == 0) {
      std::cout << "\n";
    }
  }
  fin.close();
  return datas;
}

cv::Mat_<float> makeCvMat(float a1, float a2, float a3, float a4, float a5,
                          float a6, float a7, float a8, float a9, float a10,
                          float a11, float a12) {
  return cv::Mat_<float>(3, 4) << a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11,
         a12, 0, 0, 0, 1;
}

cv::Mat_<float> makeCvMatRotation(float a1, float a2, float a3, float a4,
                                  float a5, float a6, float a7, float a8,
                                  float a9, float a10, float a11, float a12) {
  return cv::Mat_<float>(3, 3) << a1, a2, a3, a5, a6, a7, a9, a10, a11;
}

void printTransformMat(Transform t) {
  std::cout << " " << t.r11() << " " << t.r12() << " " << t.r13() << " "
            << t.x() << "\n";
  std::cout << " " << t.r21() << " " << t.r22() << " " << t.r23() << " "
            << t.y() << "\n";
  std::cout << " " << t.r31() << " " << t.r32() << " " << t.r33() << " "
            << t.z() << "\n";
}

// tried to add code to parse arguments
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
//               data.depthRaw(), i, j, cm.cx(), cm.cy(), cm.fx(), cm.fy(),
//               smoothing);
//     const unsigned char * bgr = imageRaw.ptr<unsigned char>(i,j);
//     pcl::PointXYZRGB & pt = cloud->at(i*cloud->width + j);
//     pt.x = pLocal.x;
//     pt.y = pLocal.y;
//     pt.z = pLocal.z;
//     pt.b = bgr[0];
//     pt.g = bgr[1];
//     pt.r = bgr[2];
//     if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z))
//     {
//       //std::cout<<"Depth value not valid\n";
//
//       pt.x = pt.y = pt.z = pt.b = pt.g = pt.r = 0;;
//     }
//   }
// }
