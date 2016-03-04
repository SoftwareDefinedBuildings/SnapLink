#pragma once

#include <rtabmap/core/Camera.h>

class UTimer;

namespace rtabmap
{

class CameraNetwork :
    public Camera
{
public:
    CameraNetwork(bool rectifyImages = false,
                  bool isDepth = false,
                  float imageRate = 0,
                  const Transform &localTransform = Transform::getIdentity());
    virtual ~CameraNetwork();

    virtual bool init(const std::string &calibrationFolder = ".", const std::string &cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;
    // ownership transferred
    bool addImage(std::vector<unsigned char> *);

protected:
    virtual SensorData captureImage();

private: 
    static cv::Mat dataToImage(std::vector<unsigned char> *data);

private:
    cv::Mat _img;
    bool _rectifyImages;
    bool _isDepth;

    std::string _cameraName;
    CameraModel _model;
};

} // namespace rtabmap
