#pragma once

#include <rtabmap/core/RtabmapExp.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <list>

namespace rtabmap {

class OdometrySporadic;

class RTABMAP_EXP OdometrySporadicThread : public UThread, public UEventsHandler {
public:
    // take ownership of Odometry
    OdometrySporadicThread(OdometrySporadic *odometry, unsigned int dataBufferMaxSize = 1);
    virtual ~OdometrySporadicThread();

protected:
    virtual void handleEvent(UEvent * event);

private:
    void mainLoopKill();

    void mainLoop();
    void addData(const SensorData & data);
    bool getData(SensorData & data);

private:
    USemaphore _dataAdded;
    UMutex _dataMutex;
    std::list<SensorData> _dataBuffer;
    OdometrySporadic * _odometry;
    unsigned int _dataBufferMaxSize;
};

} // namespace rtabmap
