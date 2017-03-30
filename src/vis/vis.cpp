#include "vis/vis.h"
#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include <fstream>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

int Vis::run(int argc, char *argv[]) {
  // Parse arguments
  std::string dbFile;
  std::string poseStr;

  po::options_description visible("command options");
  visible.add_options() // use comment to force new line using formater
      ("help,h", "print help message");

  po::options_description hidden;
  hidden.add_options() // use comment to force new line using formater
      ("pose", po::value<std::string>(&poseStr)->required(),
       "a string with 12 values of a pose") //
      ("dbfile", po::value<std::string>(&dbFile)->required(), "database file");

  po::options_description all;
  all.add(visible).add(hidden);

  po::positional_options_description pos;
  pos.add("dbfile", 1);

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

  // get point cloud
  const std::map<int, std::vector<Image>> &images = adapter.getImages();
  assert(images.size() == 1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto &image : images.begin()->second) {
    auto c = image.getCloud(4);
    const Transform &p = image.getPose();
    pcl::transformPointCloud(*c, *c, p.toEigen4f());
    *cloud += *c;
  }

  int size = cloud->size();
  cv::Mat cloudXYZ(1, size, CV_32FC3);
  cv::Mat cloudBGR(1, size, CV_8UC3);
  cv::Point3f *XYZdata = cloudXYZ.ptr<cv::Point3f>();
  for (int i = 0; i < size; i++) {
    pcl::PointXYZRGB &pt = cloud->at(i);
    XYZdata[i].x = pt.x;
    XYZdata[i].y = pt.y;
    XYZdata[i].z = pt.z;
    cloudBGR.at<cv::Vec3b>(i)[0] = pt.b;
    cloudBGR.at<cv::Vec3b>(i)[1] = pt.g;
    cloudBGR.at<cv::Vec3b>(i)[2] = pt.r;
  }

  // visualize camera
  cv::viz::Viz3d window("CellMate");
  window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  Transform pose;
  if (!(std::istringstream(poseStr) >> pose)) {
    std::cerr << "invalid pose input" << std::endl;
    return 1;
  }

  Transform local(0, 0, 1, 0,  //
                  -1, 0, 0, 0, //
                  0, -1, 0, 0);
  pose = pose * local.inverse();
  std::cerr << "camera pose is:" << std::endl << pose << std::endl;

  // construct and show widgets
  float horizontal = 0.8;
  float vertical = 0.6;
  double scale = 0.5;
  cv::viz::WCameraPosition frustum(cv::Vec2f(horizontal, vertical), scale);
  cv::Affine3f poseAffine(pose.toEigen3f().data());
  window.showWidget("frustum", frustum, poseAffine);

  cv::viz::WCloud cloudWidget(cloudXYZ, cloudBGR);
  window.showWidget("Cellmate vis", cloudWidget);
  window.spin();

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
