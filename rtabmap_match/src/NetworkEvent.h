#pragma once

#include <rtabmap/utilite/UEvent.h>

namespace rtabmap
{

class NetworkEvent :
    public UEvent
{
public:
    // ownership transfer
    NetworkEvent(std::vector<unsigned char> *data) :
        _data(data)
    {
    }

    std::vector<unsigned char> *getData() const
    {
        return _data;
    }

    virtual std::string getClassName() const
    {
        return std::string("NetworkEvent");
    }

private:
    std::vector<unsigned char> *_data;
};

} // namespace rtabmap
