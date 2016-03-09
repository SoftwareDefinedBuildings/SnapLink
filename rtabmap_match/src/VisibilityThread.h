#pragma once

#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/Transform.h>
#include <list>

#include "Visibility.h"

namespace rtabmap
{

class Visibility;

class VisibilityThread :
    public UThread,
    public UEventsHandler
{
public:
    // ownership of Visibility
    VisibilityThread(Visibility *visibility, unsigned int dataBufferMaxSize = 1);
    virtual ~VisibilityThread();

protected:
    virtual void handleEvent(UEvent *event);

private:
    void mainLoopKill();
    void mainLoop();
    void addData(const SensorData &data, const Transform &pose, void *context = NULL);
    bool getData(SensorData &data, Transform &pose, void *&context);

private:
    USemaphore _dataAdded;
    UMutex _dataMutex;
    std::list<SensorData> _dataBuffer;
    std::list<Transform> _poseBuffer;
    std::list<void *> _contextBuffer;
    unsigned int _dataBufferMaxSize;
    Visibility *_visibility;
};

} // namespace rtabmap
