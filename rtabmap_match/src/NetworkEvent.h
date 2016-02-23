#pragma once

#include <rtabmap/utilite/UEvent.h>

namespace rtabmap
{

class NetworkEvent :
    public UEvent
{
public:
    NetworkEvent(void *data, size_t len) :
        _data(data),
        _len(len)
    {
    }

    void *getData() const {return _data;}
    size_t getLen() const {return _len;}

    virtual std::string getClassName() const {return std::string("NetworkEvent");}

private:
    void *_data;
    size_t _len;
};

} // namespace rtabmap
