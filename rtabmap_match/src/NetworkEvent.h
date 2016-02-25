#pragma once

#include <rtabmap/utilite/UEvent.h>

namespace rtabmap
{

class NetworkEvent :
    public UEvent
{
public:
    // ownership transfer
    NetworkEvent(std::vector<unsigned char> *payload, void *context = NULL) :
        UEvent(0),
        _payload(payload),
        _context(context)
    {
    }

    std::vector<unsigned char> *payload() const
    {
        return _payload;
    }
    void *context() const
    {
        return _context;
    }

    virtual std::string getClassName() const
    {
        return std::string("NetworkEvent");
    }

private:
    std::vector<unsigned char> *_payload;
    void *_context;
};

} // namespace rtabmap
