#include "vis/vis.h"
#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include <fstream>
#include <iostream>
#include <opencv2/viz.hpp>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

void printTransformMat(Transform t);
Transform readPoseFile(std::string camaraPoseFile, int resultId);
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

  const std::map<int, std::vector<Image>> &images = adapter.getImages();
  assert(images.size() == 1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto &image : images.begin()->second) {
    auto cloud = image.getCloud(4);
    const Transform &pose = image.getPose();
    pcl::transformPointCloud(*cloud, *cloud, pose.toEigen4f());
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
  Tramsfrom transP = readPoseFile(poseFile, poseIndex);
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

Transform readPoseFile(std::string camaraPoseFile, int resultId) {
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

  Transform pose(datas[0], datas[1], datas[2], datas[3], datas[4], datas[5],
                 datas[6], datas[7], datas[8], datas[9], datas[10], datas[11]);
  return pose;
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
