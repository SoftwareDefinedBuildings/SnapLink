#pragma once

#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>

namespace rtabmap
{

class LocationEvent :
    public UEvent
{
public:
    LocationEvent(const SensorData &data, const Transform &pose, void *context = NULL) :
        UEvent(0),
        _data(data),
        _pose(pose),
        _context(context)
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
        return _context;
    }

    virtual std::string getClassName() const
    {
        return "LocationEvent";
    }

private:
    SensorData _data;
    Transform _pose;
    void *_context;
};

}
