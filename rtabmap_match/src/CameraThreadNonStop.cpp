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

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>

#include "CameraThreadNonStop.h"
#include "CameraCalibrated.h"
#include "CameraCalibratedEvent.h"

namespace rtabmap
{

// ownership transferred
CameraThreadNonStop::CameraThreadNonStop(CameraCalibrated * camera) :
        _cameraCalibrated(camera),
        _seq(0)
{
    UASSERT(_cameraCalibrated != 0);
}

CameraThreadNonStop::~CameraThreadNonStop()
{
    join(true);
    if(_cameraCalibrated)
    {
        delete _cameraCalibrated;
    }
}

void CameraThreadNonStop::setImageRate(float imageRate)
{
    _imageRate = imageRate;
    if(_cameraCalibrated)
    {
        _cameraCalibrated->setImageRate(imageRate);
    }
}

bool CameraThreadNonStop::init()
{
    if(!this->isRunning())
    {
        _seq = 0;
        if(_cameraCalibrated)
        {
            return _cameraCalibrated->init();
        }

        // Added sleep time to ignore first frames (which are darker)
        uSleep(1000);
    }
    else
    {
        UERROR("Cannot initialize the camera because it is already running...");
    }
    return false;
}

void CameraThreadNonStop::mainLoop()
{
    UTimer timer;
    UDEBUG("");
    cv::Mat rgb, depth, gray;
    float fx = 0.0f;
    float fyOrBaseline = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    double stamp = UTimer::now();
    if(_cameraCalibrated)
    {
        _cameraCalibrated->takeImage(rgb, fx, fyOrBaseline, cx, cy);
    }

    if(!rgb.empty())
    {
        // enforce gray scale images
        if (rgb.channels() > 1) {
            cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = rgb.clone();
        }

        if(_cameraCalibrated)
        {
            // hardcoded local transform that's applied to all RGBD cameras (I guess)
            Transform localTransform(0,0,1,0,-1,0,0,0,0,-1,0,0);
            SensorData data(gray, depth, CameraModel(fx, fyOrBaseline, cx, cy, localTransform), ++_seq, stamp);
            this->post(new CameraCalibratedEvent(data, "Image"));
        }
    }
    else if(!this->isKilled())
    {
        uSleep(1.0f/_imageRate*1000);
    }
}

} // namespace rtabmap
