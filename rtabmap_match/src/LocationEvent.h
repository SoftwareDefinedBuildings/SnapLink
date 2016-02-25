#pragma once

#include "rtabmap/utilite/UEvent.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/OdometryInfo.h"

namespace rtabmap
{

class OdometryEvent :
    public UEvent
{
public:
    OdometryEvent(const SensorData &data, const Transform &pose) :
        UEvent(0),
        _data(data),
        _pose(pose)
    {
    }

    SensorData &data()
    {
        return _data;
    }
    const SensorData &data() const
    {
        return _data;
    }
    const Transform &pose() const
    {
        return _pose;
    }
    void *context() const
    {
        return context;
    }

    virtual std::string getClassName() const
    {
        return "OdometryEvent";
    }

private:
    SensorData _data;
    Transform _pose;
    void *context;
};

}
