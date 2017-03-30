#include "vis/vis.h"
#include "lib/algo/Feature.h"
#include "lib/algo/Perspective.h"
#include "lib/algo/RoomSearch.h"
#include "lib/algo/Visibility.h"
#include "lib/algo/WordSearch.h"
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
  std::string camPoseStr;
  std::string imagePath;
  std::string intrinsic;

  po::options_description visible("command options");
  visible.add_options() // use comment to force new line using formater
      ("campose", po::value<std::string>(&camPoseStr),
       "a string of 12 values of the 3x4 camera pose") //
      ("image", po::value<std::string>(&imagePath),
       "image to be localized (overrides campose)") //
      ("intrinsic", po::value<std::string>(&intrinsic),
       "a string of 4 values of fx fy cx cy, required with image") //
      ("help,h", "print help message");

  po::options_description hidden;
  hidden.add_options() // use comment to force new line using formater
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
  std::set<std::string> dbFiles{dbFile};
  if (!_adapter.init(dbFiles)) {
    std::cerr << "reading data failed";
    return 1;
  }

  // create a visualization window
  cv::viz::Viz3d window("CellMate");

  // get point cloud
  const std::map<int, std::vector<Image>> &images = _adapter.getImages();
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

  // visualize point cloud
  // window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  cv::viz::WCloud cloudWidget(cloudXYZ, cloudBGR);
  window.showWidget("Cellmate vis", cloudWidget);

  // get camera pose and frustum
  Transform camPose;
  const double scale = 0.5;
  if (!imagePath.empty()) {
    if (!camPoseStr.empty()) {
      std::cerr << "using image, campose ignored" << std::endl;
    }

    cv::Mat image = cv::imread(imagePath);
    float fx, fy, cx, cy;
    auto in = std::istringstream(intrinsic);
    if (!(in >> fx >> fy >> cx >> cy) || !in.eof()) {
      std::cerr << "invalid intrinsic input" << std::endl;
      return 1;
    }

    CameraModel camera("camera", fx, fy, cx, cy, image.size());
    camPose = localize(image, camera);

    if (camPose.isNull()) {
      std::cerr << "image localization failed" << std::endl;
      return 1;
    }

    // visualize camera
    cv::viz::WCameraPosition coord(scale);
    cv::viz::WCameraPosition frustum(cv::Matx33f(camera.K()), image, scale);
    std::cerr << "K:" << std::endl << camera.K() << std::endl;
    cv::Affine3f poseAffine(camPose.toEigen3f().data());
    window.showWidget("coord", coord, poseAffine);
    window.showWidget("frustum", frustum, poseAffine);
  } else if (!camPoseStr.empty()) {
    if (!(std::istringstream(camPoseStr) >> camPose)) {
      std::cerr << "invalid pose input" << std::endl;
      return 1;
    }

    // visualize camera
    const float horizontal = 0.889484;
    const float vertical = 0.523599;
    cv::viz::WCameraPosition coord(scale);
    cv::viz::WCameraPosition frustum(cv::Vec2f(horizontal, vertical), scale);
    cv::Affine3f poseAffine(camPose.toEigen3f().data());
    window.showWidget("coord", coord, poseAffine);
    window.showWidget("frustum", frustum, poseAffine);
  }
  std::cerr << "image pose:" << std::endl << camPose << std::endl;
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
  std::cout << "cellmate vis [command options] db_file" << std::endl
            << std::endl
            << desc << std::endl;
}

Transform Vis::localize(const cv::Mat &image, const CameraModel &camera) {
  const std::map<int, Word> &words = _adapter.getWords();
  const std::map<int, Room> &rooms = _adapter.getRooms();
  const std::map<int, std::vector<Label>> &labels = _adapter.getLabels();
  Feature feature;
  WordSearch wordSearch(words);
  RoomSearch roomSearch(rooms, words);
  Perspective perspective(rooms, words);
  Visibility visibility(labels);

  // feature extraction
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  feature.extract(image, keyPoints, descriptors);

  // word search
  std::vector<int> wordIds = wordSearch.search(descriptors);

  // room search
  int roomId = roomSearch.search(wordIds);

  // PnP
  Transform pose =
      perspective.localize(wordIds, keyPoints, descriptors, camera, roomId);

  return pose;
}
