#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/utilite/ULogger.h>

#include "OdometryMonoLocThread.h"
#include "CameraCalibratedEvent.h"
#include "OdometryMonoLoc.h"

namespace rtabmap {

OdometryMonoLocThread::OdometryMonoLocThread(Odometry * odometry, unsigned int dataBufferMaxSize) :
    _odometry(odometry),
    _dataBufferMaxSize(dataBufferMaxSize),
    _resetOdometry(false)
{
    UASSERT(_odometry != NULL);
}

OdometryMonoLocThread::~OdometryMonoLocThread()
{
    this->unregisterFromEventsManager();
    this->join(true);
    if(_odometry)
    {
        delete _odometry;
    }
}

void OdometryMonoLocThread::handleEvent(UEvent * event)
{
    if(this->isRunning())
    {
        if(event->getClassName().compare("CameraCalibratedEvent") == 0)
        {
            CameraCalibratedEvent * cameraEvent = (CameraCalibratedEvent*)event;
            if(cameraEvent->getCode() == CameraCalibratedEvent::kCodeImageCalibrated)
            {
                this->addData(cameraEvent->data(), cameraEvent->cameraName()); // camera name is file name for CameraCalibratedEvent
            }
        }
    }
}

void OdometryMonoLocThread::mainLoopKill()
{
    _dataAdded.release();
}

//============================================================
// MAIN LOOP
//============================================================
void OdometryMonoLocThread::mainLoop()
{
    if(_resetOdometry)
    {
        _odometry->reset();
        _resetOdometry = false;
    }

    // reset the super odometry because we use arbitary images
    OdometryMonoLoc * odomMonoLoc = dynamic_cast<OdometryMonoLoc*>(_odometry);
    odomMonoLoc->resetSuperOdom();

    SensorData data;
    std::string fileName;
    if(getData(data, fileName))
    {
        
        info.fileName = fileName;
        Transform pose = _odometry->process(data, &info);
        // a null pose notify that odometry could not be computed
        UDEBUG("processing transform = %s", pose.prettyPrint().c_str());
        double variance = info.variance>0?info.variance:1;
        if(!pose.isNull())
        {
            OdometryEvent * odomEvent = new OdometryEvent(data, pose, variance, variance, info);
            this->post(odomEvent);
        }
    }
}

void OdometryMonoLocThread::addData(const SensorData & data, const std::string & fileName)
{
    if(dynamic_cast<OdometryMonoLoc*>(_odometry) != 0)
    {
        if(data.imageRaw().empty() || (data.cameraModels().size()==0 && !data.stereoCameraModel().isValid()))
        {
            ULOGGER_ERROR("Missing some information (image empty or missing calibration)!?");
            return;
        }
    }
    else
    {
        ULOGGER_ERROR("OdometryMonoLocThread can only work with OdometryMonoLoc");
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
        // Added file name buffer
        _fileNameBuffer.push_back(fileName);
        while(_dataBufferMaxSize > 0 && _fileNameBuffer.size() > _dataBufferMaxSize)
        {
            UDEBUG("Data buffer is full, the oldest data is removed to add the new one.");
            _fileNameBuffer.pop_front();
            notify = false;
        }
    }
    _dataMutex.unlock();

    if(notify)
    {
        _dataAdded.release();
    }
}

bool OdometryMonoLocThread::getData(SensorData & data, std::string & fileName)
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
        // Added file name buffer
        if(!_fileNameBuffer.empty())
        {
            fileName = _fileNameBuffer.front();
            _fileNameBuffer.pop_front();
            dataFilled = true;
        }
    }
    _dataMutex.unlock();
    return dataFilled;
}

} // namespace rtabmap
