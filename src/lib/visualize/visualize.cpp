#include "lib/visualize/visualize.h"
#include "lib/data/Transform.h"
#include "lib/data/CameraModel.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>

#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include <QtConcurrent>

Visualize::Visualize(std::map<int, std::map<int, Image>> images) {
  _images = images;
  _totalCount = _images.size();
}

Visualize::~Visualize() {
  stopVis();
}

void Visualize::setPose(int roomId, Transform camPose, cv::Mat image, CameraModel camera) {
  std::vector<cv::Mat> matChannels;
  cv::split(image, matChannels);
  // create alpha channel
  cv::Mat alpha = (matChannels.at(0) * 0) + 100;
  matChannels.push_back(alpha);
  cv::merge(matChannels, image);
  {
    std::lock_guard<std::mutex> lock(_posesMutex);
    _posesAndImagesAndCameras[roomId] = std::make_tuple(camPose, image, camera);
  }
}

void Visualize::startVis() {
  
  for(unsigned int roomId = 0; roomId < _totalCount; roomId++) {
    Transform camPose = _images[roomId].begin()->second.getPose();
    cv::Mat image = _images[roomId].begin()->second.getImage();
    CameraModel camera = _images[roomId].begin()->second.getCameraModel();
    {
      std::lock_guard<std::mutex> lock(_posesMutex);
      _posesAndImagesAndCameras[roomId] = std::make_tuple(camPose, image, camera);
    }
  	
  	_windows[roomId] = cv::viz::Viz3d("Room " + std::to_string(roomId));

    // get point cloud
    const std::map<int, Image> &images = _images[roomId];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &image : images) {
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
    _windows[roomId].showWidget("cloudWidget", cloudWidget);


    //visualize a camera
    const double visScale = 0.5;
    cv::viz::WCameraPosition frustum;
    cv::Affine3f poseAffine;
    {
      std::lock_guard<std::mutex> lock(_posesMutex);
      poseAffine = (std::get<0>(_posesAndImagesAndCameras[roomId])).toAffine3f();
      frustum = cv::viz::WCameraPosition(cv::Matx33f(std::get<2>(_posesAndImagesAndCameras[roomId]).K()), std::get<1>(_posesAndImagesAndCameras[roomId]), visScale);
    }
    cv::viz::WCameraPosition coord(visScale);
    _windows[roomId].showWidget("coord", coord, poseAffine);
    _windows[roomId].showWidget("frustum", frustum, poseAffine);

  }

  while(true) {
    for(unsigned int roomId = 0; roomId < _totalCount; roomId++) {
      {
        std::lock_guard<std::mutex> lock(_windowMapMutex);
        if(_windows[roomId].wasStopped()) {
          return;
        } else {
          const double visScale = 0.5;
          cv::viz::WCameraPosition frustum;
          cv::Affine3f poseAffine;
          {
            std::lock_guard<std::mutex> lock(_posesMutex);
            poseAffine = (std::get<0>(_posesAndImagesAndCameras[roomId])).toAffine3f();
            frustum = cv::viz::WCameraPosition(cv::Matx33f(std::get<2>(_posesAndImagesAndCameras[roomId]).K()), std::get<1>(_posesAndImagesAndCameras[roomId]), visScale);
          }
          _windows[roomId].removeWidget("frustum");
          _windows[roomId].showWidget("frustum", frustum, poseAffine);
          _windows[roomId].setWidgetPose("coord", poseAffine);
          _windows[roomId].spinOnce(1, false);
        }
      }
    }
  }
}


void Visualize::stopVis() {
	for(unsigned int i = 0; i < _totalCount; i++) {
    {
      std::lock_guard<std::mutex> lock(_windowMapMutex);
      _windows[i].close();
    }
  }
}