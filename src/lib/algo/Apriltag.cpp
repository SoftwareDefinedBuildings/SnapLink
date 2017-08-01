#include "apriltags/CameraUtil.h"
#include "apriltags/TagDetector.h"
#include "lib/data/Transform.h"
#include "lib/algo/Apriltag.h"
#include "lib/data/CameraModel.h"
#include <opencv2/opencv.hpp>
Apriltag::Apriltag(double tagSize) {
  _tagFamilyName =  "Tag36h11";
  _tagSize = tagSize;
}

std::vector<std::pair<int, Transform>> Apriltag::aprilLocalize(
        std::vector<Transform> tagPoseInCamFrame,
        std::vector<std::pair<int, Transform>>  tagPoseInModelFrame) {
  std::vector<std::pair<int, Transform>> camPoseInModelFrame;
  assert(tagPoseInCamFrame.size() == tagPoseInModelFrame.size());
  for(int i = 0; i < tagPoseInCamFrame.size(); i++) {
    if(tagPoseInModelFrame[i].first >= 0) {
    	camPoseInModelFrame.push_back(
          std::make_pair(tagPoseInModelFrame[i].first,
                         tagPoseInModelFrame[i].second * tagPoseInCamFrame[i].inverse()));
    }
  }

  return camPoseInModelFrame;
}

std::pair<std::vector<int>, std::vector<Transform>> Apriltag::aprilDetect(const cv::Mat &im, const CameraModel &camera) {
  std::vector<Transform> tagPoseInCamFrame;
  std::vector<int> tagCodes;
  cv::Point2d opticalCenter;
  opticalCenter.x = camera.cx();
  opticalCenter.y = camera.cy();

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

  return std::pair<std::vector<int>, std::vector<Transform>>(tagCodes, tagPoseInCamFrame);
}


Transform Apriltag::calculateNewAprilTagPoseInModelFrame(
    Transform camPoseInModelFrame,
    Transform tagPoseInCamFrame) {

  return camPoseInModelFrame * tagPoseInCamFrame;
}