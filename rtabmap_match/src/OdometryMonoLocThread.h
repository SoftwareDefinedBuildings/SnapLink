#ifndef ODOMETRYMONOLOCTHREAD_H_
#define ODOMETRYMONOLOCTHREAD_H_

#include <rtabmap/core/RtabmapExp.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <list>

#include <iostream>
#include <fstream>

namespace rtabmap {

class Odometry;

class RTABMAP_EXP OdometryMonoLocThread : public UThread, public UEventsHandler {
public:
    // take ownership of Odometry
    OdometryMonoLocThread(Odometry * odometry, unsigned int dataBufferMaxSize = 1);
    virtual ~OdometryMonoLocThread();

protected:
    virtual void handleEvent(UEvent * event);

private:
    void mainLoopKill();

    //============================================================
    // MAIN LOOP
    //============================================================
    void mainLoop();
    void addData(const SensorData & data, const std::string & fileName="");
    bool getData(SensorData & data, std::string & fileName);

private:
    USemaphore _dataAdded;
    UMutex _dataMutex;
    std::list<SensorData> _dataBuffer;
    std::list<std::string> _fileNameBuffer;
    Odometry * _odometry;
    unsigned int _dataBufferMaxSize;
    bool _resetOdometry;
};

} // namespace rtabmap


#endif /* ODOMETRYMONOLOCTHREAD_H_ */
