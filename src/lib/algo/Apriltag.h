#pragma once

class Transform;
class CameraModel;
class Apriltag final {
public:
  explicit Apriltag(double tagSize); 

  std::vector<std::pair<int, Transform>> aprilLocalize(
  	std::vector<Transform> tagPoseInCamFrame,
    std::vector<std::pair<int, Transform>>  tagPoseInModelFrame);

  //This function take in a query image and camera model
  //Return a list of apriltags' code and pose inside the image
  //The pose returned is the tag pose in camera frame
  std::pair<std::vector<int>, std::vector<Transform>> aprilDetect(const cv::Mat &im, const CameraModel &camera);

  
  Transform calculateNewAprilTagPoseInModelFrame(
    Transform camPoseInModelFrame,
    Transform tagPoseInCamFrame);
private:
  std::string _tagFamilyName;
  double _tagSize;
};
