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

    QCoreApplication app(argc, argv);

    uint16_t port = 8080;
    unsigned int maxClients = 2;
    HTTPServer httpServer(port, maxClients);

    // Hardcoded for CameraRGBImages for Android LG G2 Mini
    // TODO read fx and fy from EXIF
    rtabmap::Transform localTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
    bool rectifyImages = false;
    bool isDepth = false;
    float imageRate = 10.0f;
    CameraNetwork camera(localTransform, "../cameras/", "lg_g2_mini_640_480");
    QThread cameraThread;
    camera.moveToThread(&cameraThread);

    Localization loc(dbfile);
    QThread locThread;
    loc.moveToThread(&locThread);

    Visibility visibility;
    if (!visibility.init(labelpath))
    {
        UERROR("Visibility init failed!");
        exit(1);
    }
    QThread visThread;
    visibility.moveToThread(&visThread);
    visThread.start();

    httpServer.setCamera(&camera);
    camera.setLocalizer(&loc);
    loc.setVisibility(&visibility);
    visibility.setHTTPServer(&httpServer);

    cameraThread.start();
    locThread.start();
    httpServer.start();

    return app.exec();
}
