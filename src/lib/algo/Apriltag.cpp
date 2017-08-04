#include "apriltags/CameraUtil.h"
#include "apriltags/TagDetector.h"
#include "lib/data/Transform.h"
#include "lib/algo/Apriltag.h"
#include "lib/data/CameraModel.h"
#include <opencv2/opencv.hpp>
#include "lib/util/Utility.h"

Apriltag::Apriltag(double tagSize) {
  _tagFamilyName =  "Tag36h11";
  _tagSize = tagSize;
}

std::vector<std::pair<int, Transform>> Apriltag::aprilLocalize(
        std::vector<Transform> tagPoseInCamFrame,
        std::vector<std::pair<int, Transform>>  tagPoseInModelFrame) {
  long totalStartTime = Utility::getTime();
  std::vector<std::pair<int, Transform>> camPoseInModelFrame;
  assert(tagPoseInCamFrame.size() == tagPoseInModelFrame.size());
  for(unsigned int i = 0; i < tagPoseInCamFrame.size(); i++) {
    // first = -1 means the tag is not found in db and tagPoseMap
    if(tagPoseInModelFrame[i].first >= 0) {
    	camPoseInModelFrame.push_back(
          std::make_pair(tagPoseInModelFrame[i].first,
                         tagPoseInModelFrame[i].second * tagPoseInCamFrame[i].inverse()));
    }
  }

  long totalTime = Utility::getTime() - totalStartTime;
  std::cout << "Time Apriltag localize overall " << totalTime << " ms" <<std::endl;

  return camPoseInModelFrame;
}

std::pair<std::vector<int>, std::vector<Transform>> Apriltag::aprilDetect(const cv::Mat &im, const CameraModel &camera) {
  std::vector<Transform> tagPoseInCamFrame;
  std::vector<int> tagCodes;

  long totalStartTime = Utility::getTime();
  
  cv::Point2d opticalCenter;
  if(camera.isValid()) {
    opticalCenter.x = camera.cx();
    opticalCenter.y = camera.cy();
  } else {
    opticalCenter.x = im.cols/2;
    opticalCenter.y = im.rows/2;
  }
  
  TagDetectionArray detections;

  TagDetectorParams params;
  TagFamily family(_tagFamilyName);
  TagDetector detector(family, params);
  detector.process(im, opticalCenter, detections);
  cv::Mat rVec, t;
  for (unsigned int i = 0; i < detections.size(); i++) {
    CameraUtil::homographyToPoseCV(camera.fx(), camera.fy(), _tagSize,
                                   detections[i].homography, rVec, t);
    cv::Mat r,dump;
    cv::Rodrigues(rVec,r,dump);
    Transform pose(r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2), t.at<double>(0, 0), //
                   r.at<double>(1, 0), r.at<double>(1, 1), r.at<double>(1, 2), t.at<double>(0, 1), //
                   r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2), t.at<double>(0, 2));
    tagPoseInCamFrame.push_back(pose);
    tagCodes.push_back(detections[i].code);
  }

  long totalTime = Utility::getTime() - totalStartTime;
  std::cout << "Time Apriltag detect overall " << totalTime << " ms" <<std::endl;
 
  return std::pair<std::vector<int>, std::vector<Transform>>(tagCodes, tagPoseInCamFrame);
}


Transform Apriltag::calculateNewAprilTagPoseInModelFrame(
    Transform camPoseInModelFrame,
    Transform tagPoseInCamFrame) {

  return camPoseInModelFrame * tagPoseInCamFrame;
}