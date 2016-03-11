#pragma once

#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <list>

#include "Localization.h"

class LocalizationThread :
    public UThread,
    public UEventsHandler
{
public:
    // take ownership of Localization
    LocalizationThread(Localization *loc, unsigned int dataBufferMaxSize = 1);
    virtual ~LocalizationThread();

protected:
    virtual void handleEvent(UEvent *event);

private:
    void mainLoopKill();
    void mainLoop();
    void addData(const rtabmap::SensorData &data, void *context = NULL);
    bool getData(rtabmap::SensorData &data, void *&context);

private:
    USemaphore _dataAdded;
    UMutex _dataMutex;
    std::list<rtabmap::SensorData> _dataBuffer;
    std::list<void *> _contextBuffer;
    Localization *_loc;
    unsigned int _dataBufferMaxSize;
};
