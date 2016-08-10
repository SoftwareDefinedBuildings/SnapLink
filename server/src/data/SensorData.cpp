#include "SensorData.h"
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>

SensorData::SensorData() = default;

virtual SensorData::~SensorData() = default;

// Mono constructor
SensorData::SensorData(
        const cv::Mat & image,
        const CameraModel & cameraModel,
        int id,
        double stamp,
        const cv::Mat & userData) :
        _id(id),
        _stamp(stamp),
        _laserScanMaxPts(0),
        _laserScanMaxRange(0.0f),
        _cameraModels(std::vector<CameraModel>(1, cameraModel))
{
    if(image.rows == 1)
    {
        UASSERT(image.type() == CV_8UC1); // Bytes
        _imageCompressed = image;
    }
    else if(!image.empty())
    {
        UASSERT(image.type() == CV_8UC1 || // Mono
                image.type() == CV_8UC3);  // RGB
        _imageRaw = image;
    }

    if(userData.type() == CV_8UC1) // Bytes
    {
        _userDataCompressed = userData; // assume compressed
    }
    else
    {
        _userDataRaw = userData;
    }
}

void SensorData::uncompressData()
{
    cv::Mat tmpA, tmpB, tmpC, tmpD;
    uncompressData(_imageCompressed.empty()?0:&tmpA,
                _depthOrRightCompressed.empty()?0:&tmpB,
                _laserScanCompressed.empty()?0:&tmpC,
                _userDataCompressed.empty()?0:&tmpD);
}

void SensorData::uncompressData(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw, cv::Mat * userDataRaw)
{
    uncompressDataConst(imageRaw, depthRaw, laserScanRaw, userDataRaw);
    if(imageRaw && !imageRaw->empty() && _imageRaw.empty())
    {
        _imageRaw = *imageRaw;
        //backward compatibility, set image size in camera model if not set
        if(!_imageRaw.empty() && _cameraModels.size())
        {
            cv::Size size(_imageRaw.cols/_cameraModels.size(), _imageRaw.rows/_cameraModels.size());
            for(unsigned int i=0; i<_cameraModels.size(); ++i)
            {
                if(_cameraModels[i].isValidForProjection() && _cameraModels[i].imageWidth() == 0)
                {
                    _cameraModels[i].setImageSize(size);
                }
            }
        }
    }
    if(depthRaw && !depthRaw->empty() && _depthOrRightRaw.empty())
    {
        _depthOrRightRaw = *depthRaw;
    }
    if(laserScanRaw && !laserScanRaw->empty() && _laserScanRaw.empty())
    {
        _laserScanRaw = *laserScanRaw;
    }
    if(userDataRaw && !userDataRaw->empty() && _userDataRaw.empty())
    {
        _userDataRaw = *userDataRaw;
    }
}

void SensorData::uncompressDataConst(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw, cv::Mat * userDataRaw) const
{
    if(imageRaw)
    {
        *imageRaw = _imageRaw;
    }
    if(depthRaw)
    {
        *depthRaw = _depthOrRightRaw;
    }
    if(laserScanRaw)
    {
        *laserScanRaw = _laserScanRaw;
    }
    if(userDataRaw)
    {
        *userDataRaw = _userDataRaw;
    }
    if( (imageRaw && imageRaw->empty()) ||
        (depthRaw && depthRaw->empty()) ||
        (laserScanRaw && laserScanRaw->empty()) ||
        (userDataRaw && userDataRaw->empty()))
    {
        rtabmap::CompressionThread ctImage(_imageCompressed, true);
        rtabmap::CompressionThread ctDepth(_depthOrRightCompressed, true);
        rtabmap::CompressionThread ctLaserScan(_laserScanCompressed, false);
        rtabmap::CompressionThread ctUserData(_userDataCompressed, false);
        if(imageRaw && imageRaw->empty())
        {
            ctImage.start();
        }
        if(depthRaw && depthRaw->empty())
        {
            ctDepth.start();
        }
        if(laserScanRaw && laserScanRaw->empty())
        {
            ctLaserScan.start();
        }
        if(userDataRaw && userDataRaw->empty())
        {
            ctUserData.start();
        }
        ctImage.join();
        ctDepth.join();
        ctLaserScan.join();
        ctUserData.join();
        if(imageRaw && imageRaw->empty())
        {
            *imageRaw = ctImage.getUncompressedData();
            if(imageRaw->empty())
            {
                UWARN("Requested raw image data, but the sensor data (%d) doesn't have image.", this->id());
            }
        }
        if(depthRaw && depthRaw->empty())
        {
            *depthRaw = ctDepth.getUncompressedData();
            if(depthRaw->empty())
            {
                UWARN("Requested depth/right image data, but the sensor data (%d) doesn't have depth/right image.", this->id());
            }
        }
        if(laserScanRaw && laserScanRaw->empty())
        {
            *laserScanRaw = ctLaserScan.getUncompressedData();

            if(laserScanRaw->empty())
            {
                UWARN("Requested laser scan data, but the sensor data (%d) doesn't have laser scan.", this->id());
            }
        }
        if(userDataRaw && userDataRaw->empty())
        {
            *userDataRaw = ctUserData.getUncompressedData();

            if(userDataRaw->empty())
            {
                UWARN("Requested user data, but the sensor data (%d) doesn't have user data.", this->id());
            }
        }
    }
}
