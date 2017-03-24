#include "vis/vis.h"
#include "lib/data/Transform.h"
#include "rtabmap/core/RtabmapEvent.h"
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/viz.hpp>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UStl.h>
namespace po = boost::program_options;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);

float getDepth(
		const cv::Mat & depthImage,
		float x, float y,
		bool smoothing,
		float maxZError,
		bool estWithNeighborsIfNull);
pcl::PointXYZ projectDepthTo3D(
		const cv::Mat & depthImage,
		float x, float y,
		float cx, float cy,
		float fx, float fy,
		bool smoothing,
		float maxZError);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepthIn,
		const rtabmap::CameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth);

void printTransformMat(Transform t);
static void printInvalid(const std::vector<std::string> &opts);
static void printUsage(const po::options_description &desc);
std::vector<float> getPoseFromFileVector(std::string camaraPoseFile,
                                         int resultId);
cv::Mat_<float> makeCvMat(float a1, float a2, float a3, float a4, float a5,
                          float a6, float a7, float a8, float a9, float a10,
                          float a11, float a12);
cv::Mat_<float> makeCvMatRotation(float a1, float a2, float a3, float a4,
                                  float a5, float a6, float a7, float a8,
                                  float a9, float a10, float a11, float a12);

int vis(int argc, char *argv[]) {
  std::string dbFile = argv[1];
  std::string camaraPoseFile = argv[2];
  int resultId = atoi(argv[3]);
  // get posesMap and linksMap
  rtabmap::Memory memory;
  memory.init(dbFile);
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> links;
  if (memory.getLastWorkingSignature()) {
    // Get all IDs linked to last signature (including those in Long-Term
    //  Memory)
    std::map<int, int> ids =
        memory.getNeighborsId(memory.getLastWorkingSignature()->id(), 0, -1);
    // Get all metric constraints (the graph)
    memory.getMetricConstraints(uKeysSet(ids), poses, links, true);
  }
  // Optimize the graph
  std::map<int, rtabmap::Transform> optimizedPoseMap;
  rtabmap::Optimizer *graphOptimizer =
      rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeTORO);
  optimizedPoseMap =
      graphOptimizer->optimize(poses.begin()->first, poses, links);
  delete graphOptimizer;

  rtabmap::DBDriver *driver = rtabmap::DBDriver::create();
  driver->openConnection(dbFile);
  std::set<int> ids;
  driver->getAllNodeIds(ids, true);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  rtabmap::CameraModel cm;
  for (auto id : ids) {
    if (optimizedPoseMap.count(id) == 0) {
      // this image is being optimized out
      continue;
    }
    int imageId = id;
    bool uncompressedData = true;
    rtabmap::SensorData data = memory.getNodeData(imageId, uncompressedData);
    cm = data.cameraModels()[0];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Mat depthRaw = data.depthRaw();
    cv::Mat imageRaw = data.imageRaw();
    // Todo Check what's the difference between my implementation and rtabmap's
    // cloudFromDepthRGB()
    // My implementation is at the botom of this file
    cloud = cloudFromDepthRGB(imageRaw, depthRaw, cm, 2, 0, 0);
    cloud = removeNaNFromPointCloud(cloud);
    cloud = rtabmap::util3d::transformPointCloud(cloud, cm.localTransform());
    cloud =
        rtabmap::util3d::transformPointCloud(cloud, optimizedPoseMap.at(id));
    *assembledCloud += *cloud;
  }
  float maxX = 0.0f;
  float maxY = 0.0f;
  float maxZ = 0.0f;
  int totalSize = assembledCloud->size();
  cv::Mat cloudXYZ(1, totalSize, CV_32FC3);
  cv::Mat cloudBGR(1, totalSize, CV_8UC3);
  cv::Point3f *XYZdata = cloudXYZ.ptr<cv::Point3f>();
  for (int i = 0; i < totalSize; i++) {
    pcl::PointXYZRGB &pt = assembledCloud->at(i);
    XYZdata[i].x = pt.x;
    XYZdata[i].y = pt.y;
    XYZdata[i].z = pt.z;
    if (pt.x > maxX) {
      maxX = pt.x;
    }
    if (pt.y > maxY) {
      maxY = pt.y;
    }
    if (pt.z > maxZ) {
      maxZ = pt.z;
    }
    cloudBGR.at<cv::Vec3b>(i)[0] = pt.b;
    cloudBGR.at<cv::Vec3b>(i)[1] = pt.g;
    cloudBGR.at<cv::Vec3b>(i)[2] = pt.r;
  }
  std::cout << "Max X is " << maxX << "\n";
  std::cout << "Max Y is " << maxY << "\n";
  std::cout << "Max Z is " << maxZ << "\n";

  cv::viz::Viz3d myWindow("Coordinate Frame");
  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  std::vector<float> datas = getPoseFromFileVector(camaraPoseFile, resultId);
  // std::vector<float> datas;
  // datas.push_back(0.992269);
  // datas.push_back(-0.055068);
  // datas.push_back(-0.111220);
  // datas.push_back(-0.052803);
  // datas.push_back(0.050433);
  // datas.push_back(0.997755);
  // datas.push_back(-0.044067);
  // datas.push_back(-0.140940);
  // datas.push_back(0.113397);
  // datas.push_back(0.038117);
  // datas.push_back(0.992819);
  // datas.push_back(0.021211);
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


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepthIn,
		const rtabmap::CameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(decimation == 0)
	{
		decimation = 1;
	}
	if(decimation < 0)
	{
		if(imageRgb.rows % decimation != 0 || imageRgb.cols % decimation != 0)
		{
			int oldDecimation = decimation;
			while(decimation <= -1)
			{
				if(imageRgb.rows % decimation == 0 && imageRgb.cols % decimation == 0)
				{
					break;
				}
				++decimation;
			}

			if(imageRgb.rows % oldDecimation != 0 || imageRgb.cols % oldDecimation != 0)
			{
				std::cerr << "Decimation "<< oldDecimation <<" is not valid for current image size (rgb=" << imageRgb.cols << " x " << imageRgb.rows <<". Highest compatible decimation used=" << decimation;
			}
		}
	}
	else
	{
		if(imageDepthIn.rows % decimation != 0 || imageDepthIn.cols % decimation != 0)
		{
			int oldDecimation = decimation;
			while(decimation >= 1)
			{
				if(imageDepthIn.rows % decimation == 0 && imageDepthIn.cols % decimation == 0)
				{
					break;
				}
				--decimation;
			}

			if(imageDepthIn.rows % oldDecimation != 0 || imageDepthIn.cols % oldDecimation != 0)
			{
				std::cerr << "Decimation "<< oldDecimation <<" is not valid for current image size (rgb=" << imageRgb.cols << " x " << imageRgb.rows <<". Highest compatible decimation used=" << decimation;
			}
		}
	}

	cv::Mat imageDepth = imageDepthIn;

	bool mono;
	if(imageRgb.channels() == 3) // BGR
	{
		mono = false;
	}
	else if(imageRgb.channels() == 1) // Mono
	{
		mono = true;
	}
	else
	{
		return cloud;
	}

	//cloud.header = cameraInfo.header;
	cloud->height = imageDepth.rows/decimation;
	cloud->width  = imageDepth.cols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);

	float rgbToDepthFactorX = float(imageRgb.cols) / float(imageDepth.cols);
	float rgbToDepthFactorY = float(imageRgb.rows) / float(imageDepth.rows);
	float depthFx = model.fx() / rgbToDepthFactorX;
	float depthFy = model.fy() / rgbToDepthFactorY;
	float depthCx = model.cx() / rgbToDepthFactorX;
	float depthCy = model.cy() / rgbToDepthFactorY;

	int oi = 0;
	for(int h = 0; h < imageDepth.rows && h/decimation < (int)cloud->height; h+=decimation)
	{
		for(int w = 0; w < imageDepth.cols && w/decimation < (int)cloud->width; w+=decimation)
		{
			pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

			int x = int(w*rgbToDepthFactorX);
			int y = int(h*rgbToDepthFactorY);
			assert(x >=0 && x<imageRgb.cols && y >=0 && y<imageRgb.rows);
			if(!mono)
			{
				const unsigned char * bgr = imageRgb.ptr<unsigned char>(y,x);
				pt.b = bgr[0];
				pt.g = bgr[1];
				pt.r = bgr[2];
			}
			else
			{
				unsigned char v = imageRgb.at<unsigned char>(y,x);
				pt.b = v;
				pt.g = v;
				pt.r = v;
			}

			pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w, h, depthCx, depthCy, depthFx, depthFy, false, 0.02f);
			if (pcl::isFinite(ptXYZ) && ptXYZ.z >= minDepth && (maxDepth <= 0.0f || ptXYZ.z <= maxDepth))
			{
				pt.x = ptXYZ.x;
				pt.y = ptXYZ.y;
				pt.z = ptXYZ.z;
				++oi;
			}
			else
			{
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}

	if(oi == 0)
	{
		std::cerr << "Cloud with only NaN values created!\n";
	}
	return cloud;
}



float getDepth(
		const cv::Mat & depthImage,
		float x, float y,
		bool smoothing,
		float maxZError,
		bool estWithNeighborsIfNull)
{
	UASSERT(!depthImage.empty());
	UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);

	int u = int(x+0.5f);
	int v = int(y+0.5f);
	if(u == depthImage.cols && x<float(depthImage.cols))
	{
		u = depthImage.cols - 1;
	}
	if(v == depthImage.rows && y<float(depthImage.rows))
	{
		v = depthImage.rows - 1;
	}

	if(!(u >=0 && u<depthImage.cols && v >=0 && v<depthImage.rows))
	{
		UDEBUG("!(x >=0 && x<depthImage.cols && y >=0 && y<depthImage.rows) cond failed! returning bad point. (x=%f (u=%d), y=%f (v=%d), cols=%d, rows=%d)",
				x,u,y,v,depthImage.cols, depthImage.rows);
		return 0;
	}

	bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

	// Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
	// https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
	// Window weights:
	//  | 1 | 2 | 1 |
	//  | 2 | 4 | 2 |
	//  | 1 | 2 | 1 |
	int u_start = std::max(u-1, 0);
	int v_start = std::max(v-1, 0);
	int u_end = std::min(u+1, depthImage.cols-1);
	int v_end = std::min(v+1, depthImage.rows-1);

	float depth = 0.0f;
	if(isInMM)
	{
		if(depthImage.at<unsigned short>(v,u) > 0 &&
		   depthImage.at<unsigned short>(v,u) < std::numeric_limits<unsigned short>::max())
		{
			depth = float(depthImage.at<unsigned short>(v,u))*0.001f;
		}
	}
	else
	{
		depth = depthImage.at<float>(v,u);
	}

	if((depth==0.0f || !std::isfinite(depth)) && estWithNeighborsIfNull)
	{
		// all cells no2 must be under the zError to be accepted
		float tmp = 0.0f;
		int count = 0;
		for(int uu = u_start; uu <= u_end; ++uu)
		{
			for(int vv = v_start; vv <= v_end; ++vv)
			{
				if((uu == u && vv!=v) || (uu != u && vv==v))
				{
					float d = 0.0f;
					if(isInMM)
					{
						if(depthImage.at<unsigned short>(vv,uu) > 0 &&
						   depthImage.at<unsigned short>(vv,uu) < std::numeric_limits<unsigned short>::max())
						{
							d = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
						}
					}
					else
					{
						d = depthImage.at<float>(vv,uu);
					}
					if(d!=0.0f && std::isfinite(d))
					{
						if(tmp == 0.0f)
						{
							tmp = d;
							++count;
						}
						else if(fabs(d - tmp/float(count)) < maxZError)
						{
							tmp += d;
							++count;
						}
					}
				}
			}
		}
		if(count > 1)
		{
			depth = tmp/float(count);
		}
	}

	if(depth!=0.0f && std::isfinite(depth))
	{
		if(smoothing)
		{
			float sumWeights = 0.0f;
			float sumDepths = 0.0f;
			for(int uu = u_start; uu <= u_end; ++uu)
			{
				for(int vv = v_start; vv <= v_end; ++vv)
				{
					if(!(uu == u && vv == v))
					{
						float d = 0.0f;
						if(isInMM)
						{
							if(depthImage.at<unsigned short>(vv,uu) > 0 &&
							   depthImage.at<unsigned short>(vv,uu) < std::numeric_limits<unsigned short>::max())
							{
								d = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
							}
						}
						else
						{
							d = depthImage.at<float>(vv,uu);
						}

						// ignore if not valid or depth difference is too high
						if(d != 0.0f && std::isfinite(d) && fabs(d - depth) < maxZError)
						{
							if(uu == u || vv == v)
							{
								sumWeights+=2.0f;
								d*=2.0f;
							}
							else
							{
								sumWeights+=1.0f;
							}
							sumDepths += d;
						}
					}
				}
			}
			// set window weight to center point
			depth *= 4.0f;
			sumWeights += 4.0f;

			// mean
			depth = (depth+sumDepths)/sumWeights;
		}
	}
	else
	{
		depth = 0;
	}
	return depth;
}



pcl::PointXYZ projectDepthTo3D(
		const cv::Mat & depthImage,
		float x, float y,
		float cx, float cy,
		float fx, float fy,
		bool smoothing,
		float maxZError)
{

	pcl::PointXYZ pt;

	float depth = getDepth(depthImage, x, y, smoothing, maxZError, false);
	if(depth > 0.0f)
	{
		// Use correct principal point from calibration
		cx = cx > 0.0f ? cx : float(depthImage.cols/2) - 0.5f; //cameraInfo.K.at(2)
		cy = cy > 0.0f ? cy : float(depthImage.rows/2) - 0.5f; //cameraInfo.K.at(5)

		// Fill in XYZ
		pt.x = (x - cx) * depth / fx;
		pt.y = (y - cy) * depth / fy;
		pt.z = depth;
	}
	else
	{
		pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
	}
	return pt;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *output, indices);
	return output;
}

// #include "vis/vis.h"
// #include "lib/data/Transform.h"
// #include "rtabmap/core/RtabmapEvent.h"
// #include <boost/program_options.hpp>
// #include <fstream>
// #include <iostream>
// #include <opencv2/viz.hpp>
// #include <pcl/common/projection_matrix.h>
// #include <pcl/io/ply_io.h>
// #include <rtabmap/core/DBDriver.h>
// #include <rtabmap/core/Link.h>
// #include <rtabmap/core/Memory.h>
// #include <rtabmap/core/Optimizer.h>
// #include <rtabmap/core/util3d.h>
// #include <rtabmap/core/util3d_filtering.h>
// #include <rtabmap/core/util3d_transforms.h>
// #include <rtabmap/utilite/UStl.h>
// #include "lib/adapter/rtabmap/RTABMapAdapter.h"
// #include "lib/data/Image.h"
// namespace po = boost::program_options;
//
// void printTransformMat(Transform t);
// static void printInvalid(const std::vector<std::string> &opts);
// static void printUsage(const po::options_description &desc);
// std::vector<float> getPoseFromFileVector(std::string camaraPoseFile,
//                                          int resultId);
// cv::Mat_<float> makeCvMat(float a1, float a2, float a3, float a4, float a5,
//                           float a6, float a7, float a8, float a9, float a10,
//                           float a11, float a12);
// cv::Mat_<float> makeCvMatRotation(float a1, float a2, float a3, float a4,
//                                   float a5, float a6, float a7, float a8,
//                                   float a9, float a10, float a11, float a12);
//
// int vis(int argc, char *argv[]) {
//   std::set<std::string> dbFile;
//   dbFile.push_back(argv[1]);
//   std::string camaraPoseFile = argv[2];
//   int resultId = atoi(argv[3]);
//   int decimation = 4;
//   RTABMapAdapter adapter;
//   if (!adapter.init(std::set<std::string>(dbFiles.begin(), dbFiles.end()))) {
//     std::cerr << "reading data failed";
//     return 1;
//   }
//   const std::map<int, std::vector<Image>> &images = adapter.getImages();
//   Image targetRoomImages = images.begin();
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(
//       new pcl::PointCloud<pcl::PointXYZRGB>);
//   for(auto image: targetRoomImages) {
//     const CameraModel cm = image.getCameraModel();
//     const cv::Mat depthRaw = image.getDepth();
//     const cv::Mat imageRaw = image.getImage();
//     const Transform pose = image.getPose();
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
//         new pcl::PointCloud<pcl::PointXYZRGB>);
//     cloud = rtabmap::util3d::cloudFromDepthRGB(imageRaw, depthRaw, cm, decimation, 0, 0, nullptr);
//     cloud = rtabmap::util3d::removeNaNFromPointCloud(cloud);
//     cloud = rtabmap::util3d::transformPointCloud(cloud, cm.localTransform());
//     cloud = rtabmap::util3d::transformPointCloud(cloud, pose);
//     *assembledCloud += *cloud;
//   }
//
//   // // get posesMap and linksMap
//   // rtabmap::Memory memory;
//   // memory.init(dbFile);
//   // std::map<int, rtabmap::Transform> poses;
//   // std::multimap<int, rtabmap::Link> links;
//   // if (memory.getLastWorkingSignature()) {
//   //   // Get all IDs linked to last signature (including those in Long-Term
//   //   //  Memory)
//   //   std::map<int, int> ids =
//   //       memory.getNeighborsId(memory.getLastWorkingSignature()->id(), 0, -1);
//   //   // Get all metric constraints (the graph)
//   //   memory.getMetricConstraints(uKeysSet(ids), poses, links, true);
//   // }
//   // // Optimize the graph
//   // std::map<int, rtabmap::Transform> optimizedPoseMap;
//   // rtabmap::Optimizer *graphOptimizer =
//   //     rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeTORO);
//   // optimizedPoseMap =
//   //     graphOptimizer->optimize(poses.begin()->first, poses, links);
//   // delete graphOptimizer;
//   //
//   // rtabmap::DBDriver *driver = rtabmap::DBDriver::create();
//   // driver->openConnection(dbFile);
//   // std::set<int> ids;
//   // driver->getAllNodeIds(ids, true);
//   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(
//   //     new pcl::PointCloud<pcl::PointXYZRGB>);
//   //
//   // rtabmap::CameraModel cm;
//   // for (auto id : ids) {
//   //   if (optimizedPoseMap.count(id) == 0) {
//   //     // this image is being optimized out
//   //     continue;
//   //   }
//   //   int imageId = id;
//   //   bool uncompressedData = true;
//   //   rtabmap::SensorData data = memory.getNodeData(imageId, uncompressedData);
//   //   cm = data.cameraModels()[0];
//   //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
//   //       new pcl::PointCloud<pcl::PointXYZRGB>);
//   //   cv::Mat depthRaw = data.depthRaw();
//   //   cv::Mat imageRaw = data.imageRaw();
//   //   // Todo Check what's the difference between my implementation and rtabmap's
//   //   // cloudFromDepthRGB()
//   //   // My implementation is at the botom of this file
//   //   cloud = rtabmap::util3d::cloudFromDepthRGB(imageRaw, depthRaw, cm, 4, 0, 0,
//   //                                              nullptr);
//   //   cloud = rtabmap::util3d::removeNaNFromPointCloud(cloud);
//   //   cloud = rtabmap::util3d::transformPointCloud(cloud, cm.localTransform());
//   //   cloud =
//   //       rtabmap::util3d::transformPointCloud(cloud, optimizedPoseMap.at(id));
//   //   *assembledCloud += *cloud;
//   // }
//   float maxX = 0.0f;
//   float maxY = 0.0f;
//   float maxZ = 0.0f;
//   int totalSize = assembledCloud->size();
//   cv::Mat cloudXYZ(1, totalSize, CV_32FC3);
//   cv::Mat cloudBGR(1, totalSize, CV_8UC3);
//   cv::Point3f *XYZdata = cloudXYZ.ptr<cv::Point3f>();
//   for (int i = 0; i < totalSize; i++) {
//     pcl::PointXYZRGB &pt = assembledCloud->at(i);
//     XYZdata[i].x = pt.x;
//     XYZdata[i].y = pt.y;
//     XYZdata[i].z = pt.z;
//     if (pt.x > maxX) {
//       maxX = pt.x;
//     }
//     if (pt.y > maxY) {
//       maxY = pt.y;
//     }
//     if (pt.z > maxZ) {
//       maxZ = pt.z;
//     }
//     cloudBGR.at<cv::Vec3b>(i)[0] = pt.b;
//     cloudBGR.at<cv::Vec3b>(i)[1] = pt.g;
//     cloudBGR.at<cv::Vec3b>(i)[2] = pt.r;
//   }
//   std::cout << "Max X is " << maxX << "\n";
//   std::cout << "Max Y is " << maxY << "\n";
//   std::cout << "Max Z is " << maxZ << "\n";
//
//   cv::viz::Viz3d myWindow("Coordinate Frame");
//   myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
//   std::vector<float> datas = getPoseFromFileVector(camaraPoseFile, resultId);
//   // std::vector<float> datas;
//   // datas.push_back(0.992269);
//   // datas.push_back(-0.055068);
//   // datas.push_back(-0.111220);
//   // datas.push_back(-0.052803);
//   // datas.push_back(0.050433);
//   // datas.push_back(0.997755);
//   // datas.push_back(-0.044067);
//   // datas.push_back(-0.140940);
//   // datas.push_back(0.113397);
//   // datas.push_back(0.038117);
//   // datas.push_back(0.992819);
//   // datas.push_back(0.021211);
//   Transform transP(datas[0], datas[1], datas[2], datas[3], datas[4], datas[5],
//                    datas[6], datas[7], datas[8], datas[9], datas[10],
//                    datas[11]);
//   Transform transL(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
//   Transform transPL = transP * transL * transL;
//   std::cout << "transofrm PL is:\n";
//   printTransformMat(transPL);
//   cv::Affine3f cam_pose(
//       makeCvMatRotation(transPL.r11(), transPL.r12(), transPL.r13(),
//                         transPL.x(), transPL.r21(), transPL.r22(),
//                         transPL.r23(), transPL.y(), transPL.r31(),
//                         transPL.r32(), transPL.r33(), transPL.z()),
//       cv::Vec3f(transPL.x(), transPL.y(), transPL.z()));
//   std::cout << "camara pose\n";
//   std::cout << cam_pose.rotation() << std::endl;
//   std::cout << cam_pose.translation() << std::endl;
//   cv::viz::WCloud cloud_widget(cloudXYZ, cloudBGR);
//   cv::Affine3f cloud_pose_global =
//       cv::Affine3f().translate(cv::Vec3f(0.0f, 0.0f, 0.0f));
//   cv::viz::WCameraPosition cpw(0.5); // Coordinate axes
//   cv::viz::WCameraPosition cpw_frustum(
//       cv::Vec2f(0.889484, 0.523599)); // Camera frustum
//   myWindow.showWidget("CPW", cpw, cam_pose);
//   myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
//   myWindow.showWidget("Cellmate vis", cloud_widget, cloud_pose_global);
//   myWindow.spin();
//   return 0;
// }
//
// std::vector<float> getPoseFromFileVector(std::string camaraPoseFile,
//                                          int resultId) {
//   std::ifstream fin(camaraPoseFile);
//   if (fin.fail()) {
//     std::cout << "Target file open failed";
//   }
//
//   std::string dummy;
//   for (int i = 0; i < resultId; i++) {
//     std::getline(fin, dummy);
//     std::getline(fin, dummy);
//     std::getline(fin, dummy);
//     std::getline(fin, dummy);
//   }
//   fin >> dummy;
//   std::vector<float> datas;
//
//   std::cout << "Target item is " << dummy << "\n";
//   std::cout << "Pose read is:\n";
//   for (int i = 0; i < 12; i++) {
//     float temp;
//     fin >> temp;
//     datas.push_back(temp);
//     std::cout << datas[i] << "  ";
//     if ((i + 1) % 4 == 0) {
//       std::cout << "\n";
//     }
//   }
//   fin.close();
//   return datas;
// }
//
// cv::Mat_<float> makeCvMat(float a1, float a2, float a3, float a4, float a5,
//                           float a6, float a7, float a8, float a9, float a10,
//                           float a11, float a12) {
//   return cv::Mat_<float>(3, 4) << a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11,
//          a12, 0, 0, 0, 1;
// }
//
// cv::Mat_<float> makeCvMatRotation(float a1, float a2, float a3, float a4,
//                                   float a5, float a6, float a7, float a8,
//                                   float a9, float a10, float a11, float a12) {
//   return cv::Mat_<float>(3, 3) << a1, a2, a3, a5, a6, a7, a9, a10, a11;
// }
//
// void printTransformMat(Transform t) {
//   std::cout << " " << t.r11() << " " << t.r12() << " " << t.r13() << " "
//             << t.x() << "\n";
//   std::cout << " " << t.r21() << " " << t.r22() << " " << t.r23() << " "
//             << t.y() << "\n";
//   std::cout << " " << t.r31() << " " << t.r32() << " " << t.r33() << " "
//             << t.z() << "\n";
// }
//
// // tried to add code to parse arguments
// // Parse arguments
// // std::string dbFile;
// //
// // po::options_description label("command options");
// // label.add_options() // use comment to force new line using formater
// //     ("help,h", "print help message") //
// //     ("dbfile", po::value<std::string>(&dbFile)->required(), "database file")
// //     ("P", po::value< std::vector<double> >(), "pose");
// //
// // po::positional_options_description pos;
// // pos.add("dbfile", 1);
// //
// // po::variables_map vm;
// // po::parsed_options parsed = po::command_line_parser(argc, argv)
// //                                 .options(label)
// //                                 .positional(pos)
// //                                 .allow_unregistered()
// //                                 .run();
// // po::store(parsed, vm);
// // po::notify(vm);
// //
// // // print invalid options
// // std::vector<std::string> unrecog =
// //     collect_unrecognized(parsed.options, po::exclude_positional);
// // if (unrecog.size() > 0) {
// //   printInvalid(unrecog);
// //   printUsage(label);
// //   return 1;
// // }
// //
// // if (vm.count("help")) {
// //   printUsage(label);
// //   return 0;
// // }
//
// // My implementation
// // cloud->height = depthRaw.rows;
// // cloud->width  = depthRaw.cols;
// // cloud->resize(cloud->height * cloud->width);
// //
// // for(int i = 0; i < depthRaw.rows; i++) {
// //   for(int j = 0; j < depthRaw.cols; j++) {
// //     pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(
// //               data.depthRaw(), i, j, cm.cx(), cm.cy(), cm.fx(), cm.fy(),
// //               smoothing);
// //     const unsigned char * bgr = imageRaw.ptr<unsigned char>(i,j);
// //     pcl::PointXYZRGB & pt = cloud->at(i*cloud->width + j);
// //     pt.x = pLocal.x;
// //     pt.y = pLocal.y;
// //     pt.z = pLocal.z;
// //     pt.b = bgr[0];
// //     pt.g = bgr[1];
// //     pt.r = bgr[2];
// //     if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z))
// //     {
// //       //std::cout<<"Depth value not valid\n";
// //
// //       pt.x = pt.y = pt.z = pt.b = pt.g = pt.r = 0;;
// //     }
// //   }
// // }
