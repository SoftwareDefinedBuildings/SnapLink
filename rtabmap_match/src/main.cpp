#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <cstdio>

#include "HTTPServer.h"
#include "OdometrySporadic.h"
#include "CameraNetwork.h"
#include "CameraNetworkThread.h"
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
    ULogger::setLevel(ULogger::kInfo);
    //ULogger::setLevel(ULogger::kDebug);

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

    uint16_t port = 8080;
    unsigned int maxClients = 2;
    HTTPServer httpServer(port, maxClients);

    // Hardcoded for CameraRGBImages for Android LG G2 Mini
    // TODO read fx and fy from EXIF
    int cameraType = 1; // lg g2 mini = 1, kinect v1 = 2

    Transform localTransform;
    
    if (cameraType == 1)
    {
        // now it is hardcoded for lg g2 mini
        Transform tempTransform(0,0,1,0,-1,0,0,0,0,-1,0,0);
    
        localTransform = tempTransform;

        // TODO undistort img (or call it rectify here, not same rectification as eipometry)
        // k1 = 0.134408880645970, k2 = -0.177147104797916
    }
    else if (cameraType == 2)
    {
        // hardcoded for map1_10Hz
        Transform tempTransform(0,0,1,0.105000,-1,0,0,0,0,-1,0,0.431921);
    
        localTransform = tempTransform;
    }
    bool rectifyImages = false;
    bool isDepth = false;
    float imageRate = 10.0f;
    CameraNetwork *camera = new CameraNetwork(rectifyImages, isDepth, imageRate, localTransform);
    CameraNetworkThread cameraThread(camera, 10);
    if (!camera->init("../cameras/", "lg_g2_mini_640_480"))
    {
        UERROR("Camera init failed!");
        exit(1);
    }

    // Create an odometry thread to process camera events, it will send OdometryEvent.
    OdometrySporadicThread odomThread(new OdometrySporadic(dbfile), 10);

    Visibility * visibility = new Visibility();
    if(!visibility->init(labelpath)) {
        UERROR("Visibility init failed!");
        exit(1);
    }
    VisibilityThread visThread(visibility, 10);

    // Setup handlers
    httpServer.registerToEventsManager();
    cameraThread.registerToEventsManager();
    odomThread.registerToEventsManager();
    visThread.registerToEventsManager();

    // build "pipes" between threads
    UEventsManager::createPipe(&httpServer, &cameraThread, "NetworkEvent");
    UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");
    UEventsManager::createPipe(&odomThread, &visThread, "OdometryEvent");
    UEventsManager::createPipe(&visThread, &httpServer, "DetectionEvent");

    // Let's start the threads
    httpServer.start();
    cameraThread.start();
    odomThread.start();
    visThread.start();

    pause();

    // remove handlers
    httpServer.unregisterFromEventsManager();
    cameraThread.unregisterFromEventsManager();
    odomThread.unregisterFromEventsManager();
    visThread.unregisterFromEventsManager();

    // Kill all threads
    httpServer.stop();
    cameraThread.join(true);
    odomThread.join(true);
    visThread.join(true);

    return 0;
}
