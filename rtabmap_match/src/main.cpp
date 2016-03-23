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

    HTTPServer httpServer;
    if (!httpServer.start())
    {
        return 1;
    }

    // Hardcoded for CameraRGBImages for Android LG G2 Mini
    rtabmap::Transform localTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
    const std::string &calibrationFolder = "../cameras/";
    const std::string &cameraName = "lg_g2_mini_640_480";
    CameraNetwork camera;
    if (!camera.init(localTransform, calibrationFolder, cameraName))
    {
        return 1;
    }
    httpServer.setCamera(&camera);
    camera.setHTTPServer(&httpServer);

    Localization loc;
    if (!loc.init(dbfile))
    {
        return 1;
    }
    camera.setLocalizer(&loc);
    loc.setHTTPServer(&httpServer);

    Visibility vis;
    if (!vis.init(labelpath))
    {
        return 1;
    }
    loc.setVisibility(&vis);
    vis.setHTTPServer(&httpServer);

    QThread visThread;
    vis.moveToThread(&visThread);
    visThread.start();

    QThread locThread;
    loc.moveToThread(&locThread);
    locThread.start();

    QThread cameraThread;
    camera.moveToThread(&cameraThread);
    cameraThread.start();

    return app.exec();
}
