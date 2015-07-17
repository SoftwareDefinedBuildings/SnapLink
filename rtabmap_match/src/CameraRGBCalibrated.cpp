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

#include "rtabmap/core/DBDriver.h"

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <cmath>

#include "CameraRGBCalibrated.h"

namespace rtabmap
{

/////////////////////////
// CameraCalibratedImages
/////////////////////////
CameraCalibratedImages::CameraCalibratedImages(const std::string & path,
                     int startAt,
                     bool refreshDir,
                     float imageRate,
                     const Transform & localTransform,
                     float fx,
                     float fyOrBaseline,
                     float cx,
                     float cy) :
    Camera(imageRate, localTransform),
    _path(path),
    _startAt(startAt),
    _refreshDir(refreshDir),
    _count(0),
    _dir(0),
    _model(fx, fyOrBaseline, cx, cy, localTransform)
{

}

CameraCalibratedImages::~CameraCalibratedImages(void)
{
    if(_dir)
    {
        delete _dir;
    }
}

bool CameraCalibratedImages::init(const std::string & calibrationFolder, const std::string & cameraName)
{
    UDEBUG("");
    if(_dir)
    {
        _dir->setPath(_path, "jpg ppm png bmp pnm tiff");
    }
    else
    {
        _dir = new UDirectory(_path, "jpg ppm png bmp pnm tiff");
    }
    _count = 0;
    if(_path[_path.size()-1] != '\\' && _path[_path.size()-1] != '/')
    {
        _path.append("/");
    }
    if(!_dir->isValid())
    {
        ULOGGER_ERROR("Directory path is not valid \"%s\"", _path.c_str());
    }
    else if(_dir->getFileNames().size() == 0)
    {
        UWARN("Directory is empty \"%s\"", _path.c_str());
    }
    else
    {
        UINFO("path=%s images=%d", _path.c_str(), (int)this->imagesCount());
    }
    return _dir->isValid();
}

bool CameraCalibratedImages::isCalibrated() const
{
    return false;
}

std::string CameraCalibratedImages::getSerial() const
{
    return _lastFileName;  // return last file name for processing later
}

unsigned int CameraCalibratedImages::imagesCount() const
{
    if(_dir)
    {
        return _dir->getFileNames().size();
    }
    return 0;
}

SensorData CameraCalibratedImages::captureImage()
{
    cv::Mat img;
    UDEBUG("");
    if(_dir->isValid())
    {
        if(_refreshDir)
        {
            _dir->update();
        }
        if(_startAt == 0)
        {
            const std::list<std::string> & fileNames = _dir->getFileNames();
            if(fileNames.size())
            {
                if(_lastFileName.empty() || uStrNumCmp(_lastFileName,*fileNames.rbegin()) < 0)
                {
                    _lastFileName = *fileNames.rbegin();
                    std::string fullPath = _path + _lastFileName;
                    img = cv::imread(fullPath.c_str());
                }
            }
        }
        else
        {
            std::string fileName;
            std::string fullPath;
            fileName = _dir->getNextFileName();
            if(fileName.size())
            {
                fullPath = _path + fileName;
                while(++_count < _startAt && (fileName = _dir->getNextFileName()).size())
                {
                    fullPath = _path + fileName;
                }
                _lastFileName = fileName;
                if(fileName.size())
                {
                    ULOGGER_DEBUG("Loading image : %s", fullPath.c_str());

#if CV_MAJOR_VERSION >2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
                    img = cv::imread(fullPath.c_str(), cv::IMREAD_UNCHANGED);
#else
                    img = cv::imread(fullPath.c_str(), -1);
#endif
                    UDEBUG("width=%d, height=%d, channels=%d, elementSize=%d, total=%d",
                            img.cols, img.rows, img.channels(), img.elemSize(), img.total());

#if CV_MAJOR_VERSION < 3
                    // FIXME : it seems that some png are incorrectly loaded with opencv c++ interface, where c interface works...
                    if(img.depth() != CV_8U)
                    {
                        // The depth should be 8U
                        UWARN("Cannot read the image correctly, falling back to old OpenCV C interface...");
                        IplImage * i = cvLoadImage(fullPath.c_str());
                        img = cv::Mat(i, true);
                        cvReleaseImage(&i);
                    }
#endif

                    if(img.channels()>3)
                    {
                        UWARN("Conversion from 4 channels to 3 channels (file=%s)", fullPath.c_str());
                        cv::Mat out;
                        cv::cvtColor(img, out, CV_BGRA2BGR);
                        img = out;
                    }
                }
            }
        }
    }
    else
    {
        UWARN("Directory is not set, camera must be initialized.");
    }

    cv::Mat depth, gray;
    
    // enforce gray scale images
    if (img.channels() > 1) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img.clone();
    }
    
    double stamp = UTimer::now();
    int seq = 0;

    SensorData data(gray, depth, _model, seq, stamp);

    return data;
}

} // namespace rtabmap
