#include "lib/visualize/visualize.h"
#include "lib/data/Transform.h"
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

void Visualize::setPose(int roomId, Transform camPose) {
  {
    std::lock_guard<std::mutex> lock(_posesMutex);
    _poses[roomId] = camPose;
  }
}

void Visualize::startVis() {
  
  for(int roomId = 0; roomId < _totalCount; roomId++) {
  	Transform camPose(1,0,0,0,0,1,0,0,0,0,1,0);
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
    const float horizontal = 0.889484;
    const float vertical = 0.523599;
    cv::viz::WCameraPosition coord(visScale);
    cv::viz::WCameraPosition frustum(cv::Vec2f(horizontal, vertical), visScale);
    cv::Affine3f poseAffine = camPose.toAffine3f();
    _windows[roomId].showWidget("coord", coord, poseAffine);
    _windows[roomId].showWidget("frustum", frustum, poseAffine);

  }

  while(true) {
    for(int roomId = 0; roomId < _totalCount; roomId++) {
      {
        std::lock_guard<std::mutex> lock(_windowMapMutex);
        if(_windows[roomId].wasStopped()) {
          return;
        } else {
          cv::Affine3f poseAffine = _poses[roomId].toAffine3f();
          _windows[roomId].setWidgetPose("coord", poseAffine);
          _windows[roomId].setWidgetPose("frustum", poseAffine);
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