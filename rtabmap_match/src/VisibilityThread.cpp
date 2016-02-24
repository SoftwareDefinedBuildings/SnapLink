#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/utilite/ULogger.h>

#include "VisibilityThread.h"
#include "DetectionEvent.h"

namespace rtabmap
{

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
    if (_visibility)
    {
        delete _visibility;
    }
    UDEBUG("");
}

void VisibilityThread::handleEvent(UEvent *event)
{
    if (this->isRunning())
    {
        if (event->getClassName().compare("OdometryEvent") == 0)
        {
            OdometryEvent *odomEvent = (OdometryEvent *)event;
            SensorData data = odomEvent->data();
            Transform pose = odomEvent->pose();
            this->addData(data, pose);
        }
    }
}

void VisibilityThread::mainLoopKill()
{
    _dataAdded.release();
}

void VisibilityThread::mainLoop()
{
    SensorData data;
    Transform pose;
    if (getData(data, pose))
    {
        std::vector<std::string> names = _visibility->process(data, pose);
        this->post(new DetectionEvent(names));
    }
}

void VisibilityThread::addData(const SensorData &data, const Transform &pose)
{
    bool notify = true;
    _dataMutex.lock();
    {
        _dataBuffer.push_back(data);
        _poseBuffer.push_back(pose);

        while (_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
        {
            UWARN("Data buffer is full, the oldest data is removed to add the new one.");
            _dataBuffer.pop_front();
            _poseBuffer.pop_front();
            notify = false;
        }
    }
    _dataMutex.unlock();

    if (notify)
    {
        _dataAdded.release();
    }
}

bool VisibilityThread::getData(SensorData &data, Transform &pose)
{
    bool dataFilled = false;
    _dataAdded.acquire();
    _dataMutex.lock();
    {
        if (!_dataBuffer.empty())
        {
            data = _dataBuffer.front();
            pose = _poseBuffer.front();
            _dataBuffer.pop_front();
            _poseBuffer.pop_front();
            dataFilled = true;
        }
    }
    _dataMutex.unlock();
    return dataFilled;
}

} // namespace rtabmap
