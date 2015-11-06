#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/CameraRGBD.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <QApplication>
#include <stdio.h>

#include "OdometryMonoLoc.h"
#include "CameraThreadStream.h"
#include "CameraRGBCalibrated.h"
#include "OdometryMonoLocThread.h"
#include "Visibility.h"
#include "VisibilityThread.h"


void showUsage()
{
    printf("\nUsage:\n"
            "rtabmap-rgbd_mapping database_file image_folder label_folder topk\n");
    exit(1);
}

using namespace rtabmap;
int main(int argc, char * argv[])
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kInfo);
    //ULogger::setLevel(ULogger::kDebug);

    std::string dbfile;
    std::string imgpath;
    std::string labelpath;
    int topk;
    if(argc != 5)
    {
        showUsage();
    }
    else
    {
        dbfile = std::string(argv[argc-4]);
        imgpath = std::string(argv[argc-3]);
        labelpath = std::string(argv[argc-2]);
        topk = atoi(argv[argc-1]);
    }

    // Hardcoded for CameraRGBImages for Android LG G2 Mini
    // TODO read fx and fy from EXIF
    int cameraType = 2; // lg g2 mini = 1, kinect v1 = 2

    float fx;
    float fyOrBaseline;
    float cx;
    float cy;
    Transform localTransform;
    
    if (cameraType == 1)
    {
        // now it is hardcoded for lg g2 mini
        fx = 2248.90280131777f;
        fyOrBaseline = 2249.05827505121f;
        cx = 1303.16905149739f;
        cy = 936.309085911272f;
        Transform tempTransform(0,0,1,0,-1,0,0,0,0,-1,0,0);
    
        localTransform = tempTransform;

        // TODO undistort img (or call it rectify here, not same rectification as eipometry)
        // k1 = 0.134408880645970, k2 = -0.177147104797916
    }
    else if (cameraType == 2)
    {
        // hardcoded for map1_10Hz
        fx = 525.0f;
        fyOrBaseline = 525.0f;
        cx = 320.0f;
        cy = 240.0f;
        Transform tempTransform(0,0,1,0.105000,-1,0,0,0,0,-1,0,0.431921);
    
        localTransform = tempTransform;
    }
    int startAt = 1;
    bool refreshDir = true;
    float imageRate = 10.0f;
    Camera *camera = new CameraCalibratedImages(imgpath, startAt, refreshDir, imageRate, localTransform, fx, fyOrBaseline, cx, cy);
    CameraThreadStream cameraThread(camera);
    if(!camera->init())
    {
        UERROR("Camera init failed!");
        exit(1);
    }

    // Create an odometry thread to process camera events, it will send OdometryEvent.
    OdometryMonoLocThread odomThread(new OdometryMonoLoc(dbfile, topk), 1000);

    CameraCalibratedImages *cameraCalibrated = dynamic_cast<CameraCalibratedImages *>(camera); 
    Visibility * visibility = new Visibility(cameraCalibrated->cameraModel());
    visibility->init(labelpath);
    VisibilityThread visThread(visibility);

    // Setup handlers
    odomThread.registerToEventsManager();
    visThread.registerToEventsManager();

    // The RTAB-Map is subscribed by default to CameraEvent, but we want
    // RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
    // We can do that by creating a "pipe" between the camera and odometry, then
    // only the odometry will receive CameraEvent from that camera. RTAB-Map is
    // also subscribed to OdometryEvent by default, so no need to create a pipe between
    // odometry and RTAB-Map.
    UEventsManager::createPipe(&cameraThread, &odomThread, "CameraCalibratedEvent");
    UEventsManager::createPipe(&odomThread, &visThread, "OdometryEvent");

    // Let's start the threads
    odomThread.start();
    cameraThread.start();
    visThread.start();

    pause();

    // remove handlers
    visThread.unregisterFromEventsManager();
    odomThread.unregisterFromEventsManager();

    // Kill all threads
    cameraThread.join(true);
    odomThread.join(true);
    visThread.join(true);

    return 0;
}
