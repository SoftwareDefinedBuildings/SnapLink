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

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/Parameters.h"
#include "OdometryMonoLoc.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <QApplication>
#include <stdio.h>

#include "CameraThreadNonStop.h"
#include "CameraCalibrated.h"
#include "OdometryMonoLocThread.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/utilite/UConversion.h"


void showUsage()
{
    printf("\nUsage:\n"
            "rtabmap-rgbd_mapping database_file image_folder\n");
    exit(1);
}

using namespace rtabmap;
int main(int argc, char * argv[])
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kDebug);

    std::string dbfile;
    std::string imgpath;
    if(argc != 3)
    {
        showUsage();
    }
    else
    {
        dbfile = std::string(argv[argc-2]);
        imgpath = std::string(argv[argc-1]);
    }

    int startAt = 1;
    bool refreshDir = true;
    float imageRate = 1.0f;
    unsigned int imageWidth = 0;
    unsigned int imageHeight = 0;
    Camera *camera = new CameraImages(imgpath, startAt, refreshDir, imageRate, imageWidth, imageHeight);
    CameraThread cameraThread(camera);
    if(!cameraThread.init())
    {
        UERROR("Camera thread init failed!");
        exit(1);
    }

    // Create an odometry thread to process camera events, it will send OdometryEvent.
    rtabmap::ParametersMap parameters;
    parameters.insert(ParametersPair(Parameters::kOdomFeatureType(), uNumber2Str(Feature2D::kFeatureSurf)));
    parameters.insert(ParametersPair(Parameters::kLccBowPnPEstimation(), uBool2Str(true)));
    parameters.insert(ParametersPair(Parameters::kLccBowMinInliers(), uNumber2Str(20)));
    OdometryMonoLocThread odomThread(new OdometryMonoLoc(dbfile, parameters));


    // Create RTAB-Map to process OdometryEvent
    Rtabmap * rtabmap = new Rtabmap();
    rtabmap->init();
    RtabmapThread rtabmapThread(rtabmap); // ownership is transfered
    rtabmapThread.setDetectorRate(1.0f);

    // Setup handlers
    odomThread.registerToEventsManager();
    rtabmapThread.registerToEventsManager();

    // The RTAB-Map is subscribed by default to CameraEvent, but we want
    // RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
    // We can do that by creating a "pipe" between the camera and odometry, then
    // only the odometry will receive CameraEvent from that camera. RTAB-Map is
    // also subscribed to OdometryEvent by default, so no need to create a pipe between
    // odometry and RTAB-Map.
    UEventsManager::createPipe(&cameraThread, &odomThread, "CameraCalibratedEvent");

    // Let's start the threads
    rtabmapThread.start();
    odomThread.start();
    cameraThread.start();

    pause();

    // remove handlers
    rtabmapThread.unregisterFromEventsManager();
    odomThread.unregisterFromEventsManager();

    // Kill all threads
    cameraThread.join(true);
    odomThread.join(true);
    rtabmapThread.join(true);

    return 0;
}
