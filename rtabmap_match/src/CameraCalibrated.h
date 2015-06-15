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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include "rtabmap/core/SensorData.h"
#include <set>
#include <stack>
#include <list>
#include <vector>

class UDirectory;
class UTimer;

namespace rtabmap
{

/**
 * Class CameraCalibrated
 *
 */
class RTABMAP_EXP CameraCalibrated
{
public:
    virtual ~CameraCalibrated();
	void takeImage(cv::Mat & img, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);
    virtual bool init() = 0;

    //getters
    void getImageSize(unsigned int & width, unsigned int & height);
    float getImageRate() const {return _imageRate;}
    bool isMirroringEnabled() const {return _mirroring;}

    //setters
    void setImageRate(float imageRate) {_imageRate = imageRate;}
    void setImageSize(unsigned int width, unsigned int height);
    void setMirroringEnabled(bool enabled) {_mirroring = enabled;}

    void setCalibration(const std::string & fileName);
    void setCalibration(const cv::Mat & cameraMatrix, const cv::Mat & distorsionCoefficients);
    void resetCalibration();

protected:
    /**
     * Constructor
     *
     * @param imageRate : image/second , 0 for fast as the camera can
     */
    CameraCalibrated(float imageRate = 0,
            unsigned int imageWidth = 0,
            unsigned int imageHeight = 0);

	virtual void captureImage(cv::Mat & img, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy) = 0;

private:
    float _imageRate;
    unsigned int _imageWidth;
    unsigned int _imageHeight;
    bool _mirroring;
    UTimer * _frameRateTimer;
    cv::Mat _k; // camera_matrix
    cv::Mat _d; // distorsion_coefficients
};


/////////////////////////
// CameraCalibratedImages
/////////////////////////
class RTABMAP_EXP CameraCalibratedImages :
    public CameraCalibrated
{
public:
    CameraCalibratedImages(const std::string & path,
            int startAt = 1,
            bool refreshDir = false,
            float imageRate = 0,
            unsigned int imageWidth = 0,
            unsigned int imageHeight = 0);
    virtual ~CameraCalibratedImages();

    virtual bool init();
    std::string getPath() const {return _path;}

protected:
	virtual void captureImage(cv::Mat & img, cv::Mat & depth, float & fx, float & fy, float & cx, float & cy);

private:
    std::string _path;
    int _startAt;
    // If the list of files in the directory is refreshed
    // on each call of takeImage()
    bool _refreshDir;
    int _count;
    UDirectory * _dir;
    std::string _lastFileName;
};


} // namespace rtabmap
