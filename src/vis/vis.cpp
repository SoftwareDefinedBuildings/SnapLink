#include "vis/vis.h"
#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include "rtabmap/core/RtabmapEvent.h"
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

void printTransformMat(Transform t);
std::vector<float> getPoseFromFileVector(std::string camaraPoseFile,
                                         int resultId);
cv::Mat_<float> makeCvMat(float a1, float a2, float a3, float a4, float a5,
                          float a6, float a7, float a8, float a9, float a10,
                          float a11, float a12);
cv::Mat_<float> makeCvMatRotation(float a1, float a2, float a3, float a4,
                                  float a5, float a6, float a7, float a8,
                                  float a9, float a10, float a11, float a12);

int Vis::run(int argc, char *argv[]) {
  // Parse arguments
  std::string dbFile;
  std::string poseFile;
  int poseIndex;

  po::options_description visible("command options");
  visible.add_options() // use comment to force new line using formater
      ("help,h", "print help message");

  po::options_description hidden;
  hidden.add_options() // use comment to force new line using formater
      ("dbfile", po::value<std::string>(&dbFile)->required(),
       "database file") //
      ("posefile", po::value<std::string>(&poseFile)->required(),
       "pose file") //
      ("poseindex", po::value<int>(&poseIndex)->required(), "pose index");

  po::options_description all;
  all.add(visible).add(hidden);

  po::positional_options_description pos;
  pos.add("dbfile", 1);
  pos.add("posefile", 1);
  pos.add("poseindex", 1);

  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv)
                                  .options(all)
                                  .positional(pos)
                                  .allow_unregistered()
                                  .run();
  po::store(parsed, vm);

  // print invalid options
  std::vector<std::string> unrecog =
      collect_unrecognized(parsed.options, po::exclude_positional);
  if (unrecog.size() > 0) {
    printInvalid(unrecog);
    printUsage(visible);
    return 1;
  }

  if (vm.count("help")) {
    printUsage(visible);
    return 0;
  }

  // check whether required options exist after handling help
  po::notify(vm);

  // Run the program
  RTABMapAdapter adapter;
  std::set<std::string> dbFiles{dbFile};
  if (!adapter.init(dbFiles)) {
    std::cerr << "reading data failed";
    return 1;
  }

  std::cerr << "DEBUG: cloud" << std::endl;
  const std::map<int, std::vector<Image>> &images = adapter.getImages();
  assert(images.size() == 1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (auto image : images.begin()->second) {
    auto cloud = image.getCloud(4);
    rtabmap::Transform pose(image.getPose().r11(), image.getPose().r12(),
                            image.getPose().r13(), image.getPose().x(), //
                            image.getPose().r21(), image.getPose().r22(),
                            image.getPose().r23(), image.getPose().y(), //
                            image.getPose().r31(), image.getPose().r32(),
                            image.getPose().r33(), image.getPose().z());
    cloud = rtabmap::util3d::transformPointCloud(cloud, pose);
    std::cerr << "DEBUG: cloud " << image.getId() << std::endl;
    *assembledCloud += *cloud;
  }

  int totalSize = assembledCloud->size();
  cv::Mat cloudXYZ(1, totalSize, CV_32FC3);
  cv::Mat cloudBGR(1, totalSize, CV_8UC3);
  cv::Point3f *XYZdata = cloudXYZ.ptr<cv::Point3f>();
  for (int i = 0; i < totalSize; i++) {
    pcl::PointXYZRGB &pt = assembledCloud->at(i);
    XYZdata[i].x = pt.x;
    XYZdata[i].y = pt.y;
    XYZdata[i].z = pt.z;
    cloudBGR.at<cv::Vec3b>(i)[0] = pt.b;
    cloudBGR.at<cv::Vec3b>(i)[1] = pt.g;
    cloudBGR.at<cv::Vec3b>(i)[2] = pt.r;
  }

  cv::viz::Viz3d myWindow("Coordinate Frame");
  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  std::vector<float> datas = getPoseFromFileVector(poseFile, poseIndex);
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

void Vis::printInvalid(const std::vector<std::string> &opts) {
  std::cerr << "invalid options: ";
  for (const auto &opt : opts) {
    std::cerr << opt << " ";
  }
  std::cerr << std::endl;
}

void Vis::printUsage(const po::options_description &desc) {
  std::cout << "cellmate vis [command options] db_file posefile poseindex"
            << std::endl
            << std::endl
            << desc << std::endl;
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
