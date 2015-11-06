#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/utilite/ULogger.h>

#include "OdometryMonoLocThread.h"
#include "CameraCalibratedEvent.h"
#include "OdometryMonoLoc.h"
#include "OdometryInfoErr.h"

namespace rtabmap {

OdometryMonoLocThread::OdometryMonoLocThread(Odometry * odometry, unsigned int dataBufferMaxSize) :
    _odometry(odometry),
    _dataBufferMaxSize(dataBufferMaxSize),
    _resetOdometry(false)
{
    UASSERT(_odometry != 0);
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
        else if(event->getClassName().compare("OdometryResetEvent") == 0)
        {
            _resetOdometry = true;
        }
        else if(event->getClassName().compare("OdometryEvent") == 0)
        {
            OdometryEvent * odomEvent = (OdometryEvent*)event;
            this->addData(odomEvent->data());
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
        
        OdometryInfoErr info;
        info.fileName = fileName;
        Transform pose = _odometry->process(data, &info);
        // a null pose notify that odometry could not be computed
        UDEBUG("processing transform = %s", pose.prettyPrint().c_str());
        double variance = info.variance>0?info.variance:1;
        if(pose.isNull())
        {
            if (info.err == 0) {
                // TODO
            }
            else if(info.err == 1)
            {
                UWARN("Fail to localize %s because the new image has no enough words", fileName.c_str());
            }
            else if (info.err == 2)
            {
                UWARN("Fail to localize %s because they have no enough inliers", fileName.c_str());
            }
            else if (info.err == 3)
            {
                UWARN("Fail to localize %s because the old image has no enough words", fileName.c_str());
            }
            else if (info.err == 4)
            {
                UWARN("Fail to localize %s because the rotation is too large", fileName.c_str());
            }
            else
            {
                UWARN("Unkown error: %d", info.err);
                exit(1);
            }
        }
        else
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
