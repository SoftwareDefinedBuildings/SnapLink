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
#include "OdometryMonoLoc.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <QApplication>
#include <stdio.h>

#include "MapBuilder.h"

#include "WaitCameraThread.h"
#include "CameraCalibrated.h"

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
    ULogger::setLevel(ULogger::kWarning);

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

    // Here is the pipeline that we will use:
    // DBReader -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

    // Create the OpenNI camera, it will send a CameraEvent at the rate specified.
    // Set transform to camera so z is up, y is left and x going forward
    DBReader * dbReader = NULL;
    float frameRate = 1.0f;
    bool odometryIgnored = false;
    bool ignoreGoalDelay = true;
    dbReader = new DBReader(dbfile, frameRate, odometryIgnored, ignoreGoalDelay);
    if(!dbReader->init())
    {
        UERROR("Database Reader init failed!");
        exit(1);
    }


    int startAt = 1;
    bool refreshDir = true;
    float imageRate = 1.0f;
    unsigned int imageWidth = 0;
    unsigned int imageHeight = 0;
    CameraCalibrated *camera = new CameraCalibratedImages(imgpath, startAt, refreshDir, imageRate, imageWidth, imageHeight);
    WaitCameraThread *cameraThread = new WaitCameraThread(camera);
    if(!cameraThread->init())
    {
        UERROR("Camera thread init failed!");
        exit(1);
    }

    // GUI stuff, there the handler will receive RtabmapEvent and construct the map
    // We give it the camera so the GUI can pause/resume the camera
    QApplication app(argc, argv);
    MapBuilder mapBuilder(dbReader);

    // Create an odometry thread to process camera events, it will send OdometryEvent.
    OdometryThread odomThread(new OdometryMonoLoc());


    // Create RTAB-Map to process OdometryEvent
    Rtabmap * rtabmap = new Rtabmap();
    rtabmap->init();
    RtabmapThread rtabmapThread(rtabmap); // ownership is transfered
    rtabmapThread.setDetectorRate(1.0f);

    // Setup handlers
    odomThread.registerToEventsManager();
    rtabmapThread.registerToEventsManager();
    mapBuilder.registerToEventsManager();

    // The RTAB-Map is subscribed by default to CameraEvent, but we want
    // RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
    // We can do that by creating a "pipe" between the camera and odometry, then
    // only the odometry will receive CameraEvent from that camera. RTAB-Map is
    // also subscribed to OdometryEvent by default, so no need to create a pipe between
    // odometry and RTAB-Map.
    UEventsManager::createPipe(dbReader, &odomThread, "CameraEvent");
    UEventsManager::createPipe(cameraThread, &odomThread, "CameraEvent");

    // Let's start the threads
    rtabmapThread.start();
    odomThread.start();
    dbReader->start();
    cameraThread->start();

    mapBuilder.show();
    app.exec(); // main loop

    // remove handlers
    mapBuilder.unregisterFromEventsManager();
    rtabmapThread.unregisterFromEventsManager();
    odomThread.unregisterFromEventsManager();

    // Kill all threads
    dbReader->join(true);
    cameraThread->join(true);
    odomThread.join(true);
    rtabmapThread.join(true);

    delete dbReader;
    delete cameraThread;

    return 0;
}
