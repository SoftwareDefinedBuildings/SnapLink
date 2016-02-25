#pragma once

#include <rtabmap/utilite/UEvent.h>

namespace rtabmap
{

class NetworkEvent :
    public UEvent
{
public:
    // ownership transfer
    NetworkEvent(std::vector<unsigned char> *payload) :
        UEvent(0),
        _payload(payload)
    {
    }

    std::vector<unsigned char> *payload() const
    {
        return _payload;
    }
    void *context() const
    {
        return context;
    }

    virtual std::string getClassName() const
    {
        return std::string("NetworkEvent");
    }

private:
    std::vector<unsigned char> *_payload;
    void *context;
};

} // namespace rtabmap
