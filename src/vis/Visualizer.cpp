#include "vis/Visualizer.h"
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

int Visualizer::run(int argc, char *argv[]) {
  // Parse arguments
  std::string dbFile;
  std::string camPoseStr;
  std::string imagePath;
  std::string intrinsic;
  int featureLimit;
  int corrLimit;
  double distRatio;

  po::options_description visible("command options");
  visible.add_options() // use comment to force new line using formater
      ("help,h", "print help message") //
      ("cam-pose", po::value<std::string>(&camPoseStr),
       "a string of 12 values of the 3x4 camera pose") //
      ("image", po::value<std::string>(&imagePath),
       "image to be localized (overrides cam-pose)") //
      ("intrinsic", po::value<std::string>(&intrinsic),
       "a string of 4 values of fx fy cx cy, required with image") //
      ("feature-limit", po::value<int>(&featureLimit)->default_value(0),
       "limit the number of features used") //
      ("corr-limit", po::value<int>(&corrLimit)->default_value(0),
       "limit the number of corresponding 2D-3D points used") //
      ("dist-ratio", po::value<double>(&distRatio)->default_value(0.7),
       "distance ratio used to create words");

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
  const auto &images = _adapter.getImages();
  assert(images.size() == 1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (const auto &image : images.at(0)) {
    int decimation = 4;
    auto c = image.second.getCloud(decimation);
    const Transform &p = image.second.getPose();
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
  if (!imagePath.empty()) {
    if (!camPoseStr.empty()) {
      std::cerr << "using image, cam-pose ignored" << std::endl;
    }

    cv::Mat image = cv::imread(imagePath);
    float fx, fy, cx, cy;
    auto in = std::istringstream(intrinsic);
    if (!(in >> fx >> fy >> cx >> cy) || !in.eof()) {
      std::cerr << "invalid intrinsic input" << std::endl;
      return 1;
    }

    double scale;
    if (!downsample(image, scale)) {
      std::cerr << "image downsample failed" << std::endl;
      return 1;
    }
    std::cerr << "image downsample ratio: " << scale << std::endl;
    imwrite("image.jpg", image);
    CameraModel camera("camera", fx / scale, fy / scale, cx / scale, cy / scale,
                       image.size());
    std::cerr << "image intrinsic matrix:" << std::endl
              << camera.K() << std::endl;
    Transform camPose =
        localize(image, camera, featureLimit, corrLimit, distRatio);
    if (camPose.isNull()) {
      std::cerr << "image localization failed (did you provide the correct "
                   "intrinsic matrix?)"
                << std::endl;
      return 1;
    }

    // visualize camera
    const double visScale = 0.5;
    cv::viz::WCameraPosition coord(visScale);
    cv::viz::WCameraPosition frustum(cv::Matx33f(camera.K()), image, visScale);
    cv::Affine3f poseAffine(camPose.toEigen3f().data());
    window.showWidget("coord", coord, poseAffine);
    window.showWidget("frustum", frustum, poseAffine);

    std::cerr << "image pose:" << std::endl << camPose << std::endl;
  } else if (!camPoseStr.empty()) {
    Transform camPose;
    if (!(std::istringstream(camPoseStr) >> camPose)) {
      std::cerr << "invalid pose input" << std::endl;
      return 1;
    }

    // visualize camera
    const double visScale = 0.5;
    const float horizontal = 0.889484;
    const float vertical = 0.523599;
    cv::viz::WCameraPosition coord(visScale);
    cv::viz::WCameraPosition frustum(cv::Vec2f(horizontal, vertical), visScale);
    cv::Affine3f poseAffine(camPose.toEigen3f().data());
    window.showWidget("coord", coord, poseAffine);
    window.showWidget("frustum", frustum, poseAffine);

    std::cerr << "image pose:" << std::endl << camPose << std::endl;
  }

  window.spin();

  return 0;
}

void Visualizer::printInvalid(const std::vector<std::string> &opts) {
  std::cerr << "invalid options: ";
  for (const auto &opt : opts) {
    std::cerr << opt << " ";
  }
  std::cerr << std::endl;
}

void Visualizer::printUsage(const po::options_description &desc) {
  std::cout << "cellmate vis [command options] db_file" << std::endl
            << std::endl
            << desc << std::endl;
}

Transform Visualizer::localize(const cv::Mat &image, const CameraModel &camera,
                               int featureLimit, int corrLimit,
                               double distRatio) {
  const std::map<int, Word> &words = _adapter.getWords();
  const std::map<int, Room> &rooms = _adapter.getRooms();
  const std::map<int, std::vector<Label>> &labels = _adapter.getLabels();
  Feature feature(featureLimit);
  WordSearch wordSearch(words);
  RoomSearch roomSearch(rooms, words);
  Perspective perspective(rooms, words, corrLimit, distRatio);
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

bool Visualizer::downsample(cv::Mat &image, double &scale) {
  cv::Size size = image.size();
  std::cerr << "downsampling image. image width: " << image.size().width
            << ", height: " << image.size().height << std::endl;
  scale = 1;
  const double width = 640;
  const double height = 480;
  if ((double)size.width / size.height == width / height) {
    if (size.width > width) {
      // downsample to target aspect ratio
      scale = size.width / width;
      cv::resize(image, image, cv::Size(width, height), 0, 0, cv::INTER_AREA);
    }
    return true;
  } else if ((double)size.width / size.height == height / width) {
    if (size.width > height) {
      // downsample to target aspect ratio
      scale = size.width / height;
      cv::resize(image, image, cv::Size(height, width), 0, 0, cv::INTER_AREA);
    }
    return true;
  }
  std::cerr << "image downsample failed. image width: " << image.size().width
            << ", height: " << image.size().height << std::endl;
  // TODO image can have other aspect ratio
  return false;
}
