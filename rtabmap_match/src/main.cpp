#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <cstdio>
#include <QCoreApplication>
#include <QThread>
#include "HTTPServer.h"
#include "Localization.h"
#include "CameraNetwork.h"
#include "Visibility.h"


void showUsage()
{
    printf("\nUsage:\n"
           "rtabmap-rgbd_mapping database_file image_folder label_folder\n");
    exit(1);
}

int main(int argc, char *argv[])
{
    ULogger::setType(ULogger::kTypeConsole);
    //ULogger::setLevel(ULogger::kInfo);
    ULogger::setLevel(ULogger::kDebug);

    std::string dbfile;
    std::string labelpath;
    if (argc != 3)
    {
        showUsage();
    }
    else
    {
        dbfile = std::string(argv[argc - 2]);
        labelpath = std::string(argv[argc - 1]);
    }

    QCoreApplication a(argc, argv);

    uint16_t port = 8080;
    unsigned int maxClients = 2;
    HTTPServer httpServer(port, maxClients);

    // Hardcoded for CameraRGBImages for Android LG G2 Mini
    // TODO read fx and fy from EXIF
    int cameraType = 1; // lg g2 mini = 1, kinect v1 = 2

    rtabmap::Transform localTransform;

    if (cameraType == 1)
    {
        // now it is hardcoded for lg g2 mini
        rtabmap::Transform tempTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);

        localTransform = tempTransform;

        // TODO undistort img (or call it rectify here, not same rectification as eipometry)
        // k1 = 0.134408880645970, k2 = -0.177147104797916
    }
    else if (cameraType == 2)
    {
        // hardcoded for map1_10Hz
        rtabmap::Transform tempTransform(0, 0, 1, 0.105000, -1, 0, 0, 0, 0, -1, 0, 0.431921);

        localTransform = tempTransform;
    }
    bool rectifyImages = false;
    bool isDepth = false;
    float imageRate = 10.0f;
    CameraNetwork camera(rectifyImages, isDepth, imageRate, localTransform);
    if (!camera.init("../cameras/", "lg_g2_mini_640_480"))
    {
        UERROR("Camera init failed!");
        exit(1);
    }
    QThread cameraThread;
    camera.moveToThread(&cameraThread);
    cameraThread.start();

    QThread locThread;
    Localization loc(dbfile);
    loc.moveToThread(&locThread);
    locThread.start();

    camera._loc = &loc;

    Visibility visibility;
    if (!visibility.init(labelpath))
    {
        UERROR("Visibility init failed!");
        exit(1);
    }
    QThread visThread;
    visibility.moveToThread(&visThread);
    visThread.start();

    loc._vis = &visibility;

    visibility._httpserver = &httpServer;

    httpServer._camera = &camera;
    httpServer.start();

    
    return a.exec();
}
