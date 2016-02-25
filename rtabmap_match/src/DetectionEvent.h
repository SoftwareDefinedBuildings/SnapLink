#pragma once

#include <rtabmap/utilite/UEvent.h>
#include <vector>

namespace rtabmap
{

class DetectionEvent :
    public UEvent
{
public:
    DetectionEvent(const std::vector<std::string> &names) :
        UEvent(0),
        _names(names)
    {
    }

    // get the names of the detected objects
    std::vector<std::string> getNames() const
    {
        return _names;
    }
    void *context() const
    {
        return context;
    }

    virtual std::string getClassName() const
    {
        return std::string("DetectionEvent");
    }

private:
    std::vector<std::string> _names;
    void *context;
};

} // namespace rtabmap
