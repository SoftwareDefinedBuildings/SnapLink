#include <rtabmap/core/Odometry.h>
#include <rtabmap/utilite/ULogger.h>

#include "VisibilityThread.h"
#include "LocationEvent.h"
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
}

void VisibilityThread::handleEvent(UEvent *event)
{
    if (this->isRunning())
    {
        if (event->getClassName().compare("LocationEvent") == 0)
        {
            LocationEvent *locEvent = (LocationEvent *)event;
            SensorData data = locEvent->data();
            Transform pose = locEvent->pose();
            void *context = locEvent->context();
            this->addData(data, pose, context);
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
    void *context = NULL;
    if (getData(data, pose, context))
    {
        std::vector<std::string> names = _visibility->process(data, pose);
        this->post(new DetectionEvent(names, context));
    }
}

void VisibilityThread::addData(const SensorData &data, const Transform &pose, void *context)
{
    bool notify = true;
    _dataMutex.lock();
    {
        _dataBuffer.push_back(data);
        _poseBuffer.push_back(pose);
        _contextBuffer.push_back(context);

        while (_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
        {
            UWARN("Data buffer is full, the oldest data is removed to add the new one.");
            _dataBuffer.pop_front();
            _poseBuffer.pop_front();
            _contextBuffer.pop_front();
            notify = false;
        }
    }
    _dataMutex.unlock();

    if (notify)
    {
        _dataAdded.release();
    }
}

bool VisibilityThread::getData(SensorData &data, Transform &pose, void *&context)
{
    bool dataFilled = false;
    _dataAdded.acquire();
    _dataMutex.lock();
    {
        if (!_dataBuffer.empty() && _dataBuffer.size() == _contextBuffer.size())
        {
            data = _dataBuffer.front();
            pose = _poseBuffer.front();
            context = _contextBuffer.front();
            _dataBuffer.pop_front();
            _poseBuffer.pop_front();
            _contextBuffer.pop_front();
            dataFilled = true;
        }
    }
    _dataMutex.unlock();
    return dataFilled;
}

} // namespace rtabmap
