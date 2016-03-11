#pragma once

#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>

class LocationEvent :
    public UEvent
{
public:
    LocationEvent(const rtabmap::SensorData &data, const rtabmap::Transform &pose, void *context = NULL) :
        UEvent(0),
        _data(data),
        _pose(pose),
        _context(context)
    {
    }

    rtabmap::SensorData &data()
    {
        return _data;
    }
    const rtabmap::SensorData &data() const
    {
        return _data;
    }
    const rtabmap::Transform &pose() const
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
    rtabmap::SensorData _data;
    rtabmap::Transform _pose;
    void *_context;
};
