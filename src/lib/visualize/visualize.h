#pragma once

#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include <mutex>
#include <atomic>
#include <tuple>

// class RTABMapAdapter;
class Transform;
class CameraModel;
class Visualize final {
public:
	Visualize(std::map<int, std::map<int, Image>> images);
	~Visualize();
	void setPose(int roomId, Transform camPose, cv::Mat image, CameraModel camera);
	void startVis();    
	void stopVis();

private:
	//{roomId, visWindow}
	std::map<int, cv::viz::Viz3d> _windows;
	std::mutex _windowMapMutex;
	unsigned _totalCount;
	std::map<int, std::map<int, Image>> _images;
	std::map<int, std::tuple<Transform, cv::Mat, CameraModel>> _posesAndImagesAndCameras;
	std::mutex _posesMutex;


};