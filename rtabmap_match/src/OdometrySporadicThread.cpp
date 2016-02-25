#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/utilite/ULogger.h>

#include "OdometrySporadicThread.h"
#include "ImageEvent.h"

namespace rtabmap
{

OdometrySporadicThread::OdometrySporadicThread(OdometrySporadic *odometry, unsigned int dataBufferMaxSize) :
    _odometry(odometry),
    _dataBufferMaxSize(dataBufferMaxSize)
{
    UASSERT(_odometry != NULL);
}

OdometrySporadicThread::~OdometrySporadicThread()
{
    this->unregisterFromEventsManager();
    this->join(true);
    if (_odometry)
    {
        delete _odometry;
    }
}

void OdometrySporadicThread::handleEvent(UEvent *event)
{
    if (this->isRunning())
    {
        if (event->getClassName().compare("ImageEvent") == 0)
        {
            ImageEvent *imageEvent = (ImageEvent *)event;
            this->addData(imageEvent->data(), imageEvent->context());
        }
    }
}

void OdometrySporadicThread::mainLoopKill()
{
    _dataAdded.release();
}

void OdometrySporadicThread::mainLoop()
{
    // because the image is from sporadic time and location
    _odometry->reset(Transform::getIdentity());

    SensorData data;
    void *context = NULL;
    if (getData(data, context))
    {
        OdometryInfo info;
        Transform pose = _odometry->process(data, &info);
        // a null pose notify that odometry could not be computed
        double variance = info.variance > 0 ? info.variance : 1;
        this->post(new LocationEvent(data, pose, context));
    }
}

void OdometrySporadicThread::addData(const SensorData &data, void *context)
{
    if (data.imageRaw().empty() || (data.cameraModels().size() == 0 && !data.stereoCameraModel().isValid()))
    {
        ULOGGER_ERROR("Missing some information (image empty or missing calibration)!?");
        return;
    }

    bool notify = true;
    _dataMutex.lock();
    {
        _dataBuffer.push_back(data);
        _contextBuffer.push_back(context);

        while (_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
        {
            UWARN("Data buffer is full, the oldest data is removed to add the new one.");
            _dataBuffer.pop_front();
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

bool OdometrySporadicThread::getData(SensorData &data, void *&context)
{
    bool dataFilled = false;
    _dataAdded.acquire();
    _dataMutex.lock();
    {
        if (!_dataBuffer.empty() && _dataBuffer.size() == _contextBuffer.size())
        {
            data = _dataBuffer.front();
            context = _contextBuffer.front();
            _dataBuffer.pop_front();
            _contextBuffer.pop_front();
            dataFilled = true;
        }
    }
    _dataMutex.unlock();
    return dataFilled;
}

} // namespace rtabmap
