/*
 * Author: Kaifei Chen <kaifei@berkeley.edu>
 */

#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/utilite/ULogger.h"

#include "VisibilityThread.h"
#include "CameraCalibratedEvent.h"
#include "OdometryMonoLoc.h"
#include "OdometryInfoErr.h"

namespace rtabmap {

VisibilityThread::VisibilityThread(Visibility *visibility, unsigned int dataBufferMaxSize) :
    _visibility(visibility),
    _dataBufferMaxSize(dataBufferMaxSize)
{
    UASSERT(_visibility != 0);
}

VisibilityThread::~VisibilityThread()
{
    this->unregisterFromEventsManager();
    this->join(true);
    if(_visibility)
    {
        delete _visibility;
    }
    UDEBUG("");
}

void VisibilityThread::handleEvent(UEvent * event)
{
    if(this->isRunning())
    {
        if(event->getClassName().compare("OdometryEvent") == 0)
        {
            OdometryEvent * odomEvent = (OdometryEvent*)event;
            SensorData data = odomEvent->data();
            Transform pose = odomEvent->pose();
            std::string imgName = odomEvent->imageName;
            this->addData(data, pose, imgName);
        }
    }
}

void VisibilityThread::mainLoopKill()
{
    _dataAdded.release();
}

//============================================================
// MAIN LOOP
//============================================================
void VisibilityThread::mainLoop()
{
    SensorData data;
    Transform pose;
    std::string imgName;
    if(getData(data, pose, imgName))
    {
        // HDR
        _visibility->process(data, pose, imgName); 
    }
}

void VisibilityThread::addData(const SensorData & data, const Transform & pose, const std::string & imgName)
{
    bool notify = true;
    _dataMutex.lock();
    {
        _dataBuffer.push_back(data);
        _poseBuffer.push_back(pose);
        _imgNameBuffer.push_back(imgName);

        while(_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
        {
            UDEBUG("Data buffer is full, the oldest data is removed to add the new one.");
            _dataBuffer.pop_front();
            _poseBuffer.pop_front();
            _imgNameBuffer.pop_front();
            notify = false;
        }
    }
    _dataMutex.unlock();

    if(notify)
    {
        _dataAdded.release();
    }
}

bool VisibilityThread::getData(SensorData & data, Transform & pose, std::string & imgName)
{
    bool dataFilled = false;
    _dataAdded.acquire();
    _dataMutex.lock();
    {
        if(!_dataBuffer.empty())
        {
            data = _dataBuffer.front();
            pose = _poseBuffer.front();
            imgName = _imgNameBuffer.front();
            _dataBuffer.pop_front();
            _poseBuffer.pop_front();
            _imgNameBuffer.pop_front();
            dataFilled = true;
        }
    }
    _dataMutex.unlock();
    return dataFilled;
}

} // namespace rtabmap
