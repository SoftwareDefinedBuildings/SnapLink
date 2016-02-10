#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/CameraRGB.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <cstdio>

#include "OdometrySporadic.h"
#include "CameraThreadStream.h"
#include "OdometrySporadicThread.h"
#include "Visibility.h"
#include "VisibilityThread.h"


void showUsage()
{
    printf("\nUsage:\n"
            "rtabmap-rgbd_mapping database_file image_folder label_folder\n");
    exit(1);
}

using namespace rtabmap;
int main(int argc, char * argv[])
{
    ULogger::setType(ULogger::kTypeConsole);
    //ULogger::setLevel(ULogger::kInfo);
    ULogger::setLevel(ULogger::kDebug);

    std::string dbfile;
    std::string imgpath;
    std::string labelpath;
    if(argc != 4)
    {
        showUsage();
    }
    else
    {
        dbfile = std::string(argv[argc-3]);
        imgpath = std::string(argv[argc-2]);
        labelpath = std::string(argv[argc-1]);
    }

    // Hardcoded for CameraRGBImages for Android LG G2 Mini
    // TODO read fx and fy from EXIF
    int cameraType = 1; // lg g2 mini = 1, kinect v1 = 2

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
    bool rectifyImages = false;
    bool isDepth = false;
    float imageRate = 10.0f;
    Camera *camera = new CameraImages(imgpath, startAt, refreshDir, rectifyImages, isDepth, imageRate, localTransform);
    CameraThreadStream cameraThread(camera);
    if(!camera->init("../cameras/", "kinect"))
    {
        UERROR("Camera init failed!");
        exit(1);
    }

    // Create an odometry thread to process camera events, it will send OdometryEvent.
    OdometrySporadicThread odomThread(new OdometrySporadic(dbfile), 1000);

    Visibility * visibility = new Visibility();
    if(!visibility->init(labelpath)) {
        UERROR("Visibility init failed!");
        exit(1);
    }
    VisibilityThread visThread(visibility);

    // Setup handlers
    odomThread.registerToEventsManager();
    visThread.registerToEventsManager();

    // build "pipes" between threads
    UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");
    UEventsManager::createPipe(&odomThread, &visThread, "OdometryEvent");

    // Let's start the threads
    cameraThread.start();
    odomThread.start();
    visThread.start();

    pause();

    // remove handlers
    odomThread.unregisterFromEventsManager();
    visThread.unregisterFromEventsManager();

    // Kill all threads
    cameraThread.join(true);
    odomThread.join(true);
    visThread.join(true);

    return 0;
}
