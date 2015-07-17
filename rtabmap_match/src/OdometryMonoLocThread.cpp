/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/utilite/ULogger.h"

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
    errFileNoWordNewImg.open("errFileNoWordNewImg.txt");
    errFileInlier.open("errFileInlier.txt");
    errFileNoWordOldImg.open("errFileNoWordOldImg.txt");
    errFileLargeRot.open("errFileLargeRot.txt");
}

OdometryMonoLocThread::~OdometryMonoLocThread()
{
    this->unregisterFromEventsManager();
    this->join(true);
    if(_odometry)
    {
        delete _odometry;
    }
    errFileNoWordNewImg.close();
    errFileInlier.close();
    errFileNoWordOldImg.close();
    errFileLargeRot.close();
    UDEBUG("");
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
        this->post(new OdometryEvent(data, pose, variance, variance, info));
        if(pose.isNull())
        {
            if(info.err == 1)
            {
                UWARN("Fail to localize %s because the new image has no enough words", fileName.c_str());
                errFileNoWordNewImg << fileName << ", " << info.oldImgId << std::endl;
            }
            else if (info.err == 2)
            {
                UWARN("Fail to localize %s because they have no enough inliers", fileName.c_str());
                errFileInlier << fileName << ", " << info.oldImgId << std::endl;
            }
            else if (info.err == 3)
            {
                UWARN("Fail to localize %s because the old image has no enough words", fileName.c_str());
                errFileNoWordOldImg << fileName << ", " << info.oldImgId << std::endl;
            }
            else if (info.err == 4)
            {
                UWARN("Fail to localize %s because the rotation is too large", fileName.c_str());
                errFileLargeRot << fileName << ", " << info.oldImgId << std::endl;
            }
            else
            {
                UWARN("Unkown error: %d", info.err);
                exit(1);
            }
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
