/*
 * Author: Kaifei Chen <kaifei@berkeley.edu>
 */

#ifndef VISIBILITYTHREAD_H_
#define VISIBILITYTHREAD_H_

#include <rtabmap/core/RtabmapExp.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/core/OdometryEvent.h>
#include <list>

#include "Visibility.h"
#include <rtabmap/core/Transform.h>

namespace rtabmap {

class Visibility;

class RTABMAP_EXP VisibilityThread : public UThread, public UEventsHandler {
public:
    // ownership of Visibility
    VisibilityThread(Visibility *visibility, unsigned int dataBufferMaxSize = 1);
    virtual ~VisibilityThread();

protected:
    virtual void handleEvent(UEvent * event);

private:
    void mainLoopKill();

    //============================================================
    // MAIN LOOP
    //============================================================
    void mainLoop();
    void addData(const SensorData & data, const Transform & pose, const std::string & imgName);
    bool getData(SensorData & data, Transform & pose, std::string & imgName);

private:
    USemaphore _dataAdded;
    UMutex _dataMutex;
    std::list<SensorData> _dataBuffer;
    std::list<Transform> _poseBuffer;
    std::list<std::string> _imgNameBuffer;
    unsigned int _dataBufferMaxSize;
    Visibility * _visibility;
};

} // namespace rtabmap


#endif /* VISIBILITYTHREAD_H_ */
