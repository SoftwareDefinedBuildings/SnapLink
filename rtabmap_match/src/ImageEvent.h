#pragma once

#include <rtabmap/utilite/UEvent.h>
#include "rtabmap/core/SensorData.h"

namespace rtabmap
{

class ImageEvent :
    public UEvent
{
public:
    ImageEvent(const cv::Mat &image, int seq = 0, double stamp = 0.0, const std::string &cameraName = "", void *context = NULL) :
        UEvent(0),
        _data(image, seq, stamp),
        _cameraName(cameraName),
        _context(context)
    {
    }

    ImageEvent(const SensorData &data, const std::string &cameraName = "", void *context = NULL) :
        UEvent(0),
        _data(data),
        _cameraName(cameraName),
        _context(context)
    {
    }

    // Image or descriptors
    const SensorData &data() const
    {
        return _data;
    }
    const std::string &cameraName() const
    {
        return _cameraName;
    }
    void *context() const
    {
        return _context;
    }

    virtual std::string getClassName() const
    {
        return std::string("ImageEvent");
    }

private:
    SensorData _data;
    std::string _cameraName;
    void *_context;
};

} // namespace rtabmap
