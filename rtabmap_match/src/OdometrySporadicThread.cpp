#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/utilite/ULogger.h>

#include "OdometrySporadicThread.h"
#include "OdometrySporadic.h"

namespace rtabmap {

OdometrySporadicThread::OdometrySporadicThread(OdometrySporadic * odometry, unsigned int dataBufferMaxSize) :
_odometry(odometry),
_dataBufferMaxSize(dataBufferMaxSize)
{
    UASSERT(_odometry != NULL);
}

OdometrySporadicThread::~OdometrySporadicThread()
{
    this->unregisterFromEventsManager();
    this->join(true);
    if(_odometry)
    {
        delete _odometry;
    }
}

void OdometrySporadicThread::handleEvent(UEvent * event)
{
    if(this->isRunning())
    {
        if(event->getClassName().compare("CameraEvent") == 0)
        {
            CameraEvent * cameraEvent = (CameraEvent*)event;
            if(cameraEvent->getCode() == CameraEvent::kCodeData)
            {
                this->addData(cameraEvent->data());
            }
        }
    }
}

void OdometrySporadicThread::mainLoopKill()
{
    _dataAdded.release();
}

//============================================================
// MAIN LOOP
//============================================================
void OdometrySporadicThread::mainLoop()
{
    // because the image is from sporadic time and location
    _odometry->reset(Transform::getIdentity());

    SensorData data;
    if(getData(data))
    {
        OdometryInfo info; 
        Transform pose = _odometry->process(data, &info);
        // a null pose notify that odometry could not be computed
        double variance = info.variance>0?info.variance:1;
        this->post(new OdometryEvent(data, pose, variance, variance, info));
    }
}

void OdometrySporadicThread::addData(const SensorData & data)
{
    if(data.imageRaw().empty() || (data.cameraModels().size()==0 && !data.stereoCameraModel().isValid()))
    {
        ULOGGER_ERROR("Missing some information (image empty or missing calibration)!?");
        return;
    }

    bool notify = true;
    _dataMutex.lock();
    {
        _dataBuffer.push_back(data);
        while(_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
        {
            UDEBUG("Data buffer is full, the oldest data is removed to add the new one.");
            _dataBuffer.pop_front();
            notify = false;
        }
    }
    _dataMutex.unlock();

    if(notify)
    {
        _dataAdded.release();
    }
}

bool OdometrySporadicThread::getData(SensorData & data)
{
    bool dataFilled = false;
    _dataAdded.acquire();
    _dataMutex.lock();
    {
        if(!_dataBuffer.empty())
        {
            data = _dataBuffer.front();
            _dataBuffer.pop_front();
            dataFilled = true;
        }
    }
    _dataMutex.unlock();
    return dataFilled;
}

} // namespace rtabmap
