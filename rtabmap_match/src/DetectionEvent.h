#pragma once

#include <rtabmap/utilite/UEvent.h>
#include <vector>

class DetectionEvent :
    public UEvent
{
public:
    DetectionEvent(const std::vector<std::string> &names, void *context = NULL) :
        UEvent(0),
        _names(names),
        _context(context)
    {
    }

    // get the names of the detected objects
    std::vector<std::string> getNames() const
    {
        return _names;
    }
    void *context() const
    {
        return _context;
    }

    virtual std::string getClassName() const
    {
        return std::string("DetectionEvent");
    }

private:
    std::vector<std::string> _names;
    void *_context;
};
