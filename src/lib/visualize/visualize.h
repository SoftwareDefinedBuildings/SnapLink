#pragma once

#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include <mutex>
#include <atomic>

// class RTABMapAdapter;
class Transform;
class Visualize final {
public:
	Visualize(std::map<int, std::map<int, Image>> images);
	~Visualize();
	void setPose(int roomId, Transform camPose);
	void startVis();    
	void stopVis();

private:
	//{roomId, visWindow}
	std::map<int, cv::viz::Viz3d> _windows;
	std::mutex _windowMapMutex;
	unsigned _totalCount;
	std::map<int, std::map<int, Image>> _images;
	std::map<int, Transform> _poses;
	std::mutex _posesMutex;


};