#include "lib/visualize/visualize.h"
#include "lib/data/Transform.h"
#include "lib/data/CameraModel.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>

#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include <QtConcurrent>

Visualize::Visualize(std::map<int, std::map<int, Image>> images, int count) {
  _images = images;
  _totalCount = _images.size();
  _visCount = count;
  _widgetNameCounter = 0;
}

Visualize::~Visualize() {
  stopVis();
}

void Visualize::setPose(int roomId, Transform camPose, cv::Mat image, CameraModel camera) {
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
  }

  while(true) {
    for(unsigned int roomId = 0; roomId < _totalCount; roomId++) {
      {
        std::lock_guard<std::mutex> lock(_windowMapMutex);
        if(_windows[roomId].wasStopped()) {
          return;
        } else {
          //visualize a camera
          const double visScale = 0.5;
          Transform pose;
          cv::Mat im;
          CameraModel camera;
          {
            std::lock_guard<std::mutex> lock(_posesMutex);
            pose = std::get<0>(_posesAndImagesAndCameras[roomId]);
            im = std::get<1>(_posesAndImagesAndCameras[roomId]);
            camera = std::get<2>(_posesAndImagesAndCameras[roomId]);
          }
          if(!_widgetNameMap[roomId].empty() && _widgetNameMap[roomId].back().second == pose) {
            // do nothing, because this pose is already showed in window
          } else {
            // pop front if the queue is full
            if(_widgetNameMap[roomId].size() >= _visCount) {
              std::string name = _widgetNameMap[roomId].front().first;
              _widgetNameMap[roomId].pop();
              _windows[roomId].removeWidget("frustum"+name);
              _windows[roomId].removeWidget("coord"+name);
            } 
            // add new pose to queue
            std::string name = std::to_string(_widgetNameCounter++);
            cv::Affine3f poseAffine = pose.toAffine3f();
            _widgetNameMap[roomId].push(std::pair<std::string, Transform>(name, pose));
            cv::viz::WCameraPosition frustum(cv::Matx33f(camera.K()), im, visScale);
            cv::viz::WCameraPosition coord(visScale);
            _windows[roomId].showWidget("frustum"+name, frustum, poseAffine);
            _windows[roomId].showWidget("coord"+name, coord, poseAffine);
          }
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