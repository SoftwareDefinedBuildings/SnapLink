#pragma once

#include <rtabmap/utilite/UEvent.h>
#include "rtabmap/core/SensorData.h"

namespace rtabmap
{

class ImageEvent :
    public UEvent
{
public:
    ImageEvent(const cv::Mat &image, int seq = 0, double stamp = 0.0, const std::string &cameraName = "") :
        UEvent(0),
        data_(image, seq, stamp),
        cameraName_(cameraName)
    {
    }

    ImageEvent(const SensorData &data, const std::string &cameraName = "") :
        UEvent(0),
        data_(data),
        cameraName_(cameraName)
    {
    }

    // Image or descriptors
    const SensorData &data() const
    {
        return data_;
    }
    const std::string &cameraName() const
    {
        return cameraName_;
    }
    void *context() const
    {
        return context;
    }

    virtual std::string getClassName() const
    {
        return std::string("ImageEvent");
    }

private:
    SensorData data_;
    std::string cameraName_;
    void *context;
};

} // namespace rtabmap
