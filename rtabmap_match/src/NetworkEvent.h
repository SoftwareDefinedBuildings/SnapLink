#pragma once

#include <rtabmap/utilite/UEvent.h>

namespace rtabmap
{

class NetworkEvent :
    public UEvent
{
public:
    // ownership transfer
    NetworkEvent(std::vector<char> *data) :
        _data(data)
    {
    }

    std::vector<char> *getData() const {return _data;}

    virtual std::string getClassName() const {return std::string("NetworkEvent");}

private:
    std::vector<char> *_data;
};

} // namespace rtabmap
