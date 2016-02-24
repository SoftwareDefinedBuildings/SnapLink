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
        _names(names)
    {
    }

    //virtual ~DetectionEvent() {}
    // get the names of the detected objects
    std::vector<std::string> getNames() const
    {
        return _names;
    }

    virtual std::string getClassName() const
    {
        return std::string("DetectionEvent");
    }

private:
    std::vector<std::string> _names;
};

} // namespace rtabmap
