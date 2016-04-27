#include <rtabmap/utilite/UConversion.h>
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
#include "MemoryLoc.h"


void showUsage()
{
    printf("\nUsage:\n"
           "rtabmap-rgbd_mapping database_file\n");
    exit(1);
}

int main(int argc, char *argv[])
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kInfo);
    //ULogger::setLevel(ULogger::kDebug);

    std::string dbfile;
    if (argc != 2)
    {
        showUsage();
    }
    else
    {
        dbfile = std::string(argv[argc - 1]);
    }

    QCoreApplication app(argc, argv);

    HTTPServer httpServer;
    if (!httpServer.start())
    {
        UERROR("Starting HTTP Server failed");
        return 1;
    }

    // Hardcoded for CameraRGBImages for Android LG G2 Mini
    CameraNetwork camera;
    camera.setHTTPServer(&httpServer);
    httpServer.setCamera(&camera);

    rtabmap::ParametersMap memoryParams;
    // Setup memory
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImageKept(), "true"));
    memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpDetectorStrategy(), uNumber2Str(rtabmap::Feature2D::kFeatureSurf)));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisEstimationType(), "1")); // Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)
    memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), "4"));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpIncrementalDictionary(), "true")); // make sure it is incremental
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNewWordsComparedTogether(), "false"));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNNStrategy(), uNumber2Str(rtabmap::VWDictionary::kNNBruteForce))); // bruteforce
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNndrRatio(), "0.3"));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxFeatures(), "1500"));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpBadSignRatio(), "0"));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpRoiRatios(), "0.0 0.0 0.0 0.0"));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemGenerateIds(), "true"));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisIterations(), "2000"));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPReprojError(), "1.0"));
    // memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPFlags(), "0")); // 0=Iterative, 1=EPNP, 2=P3P
    memoryParams.insert(rtabmap::ParametersPair(rtabmap::Parameters::kSURFGpuVersion(), "true"));

    MemoryLoc memory;
    if (!memory.init(dbfile, memoryParams))
    {
        UERROR("Initializing memory failed");
        return 1;
    }

    Localization loc;
    loc.setMemory(&memory);
    loc.setHTTPServer(&httpServer);
    camera.setLocalizer(&loc);

    Visibility vis;
    vis.setMemory(&memory);
    vis.setHTTPServer(&httpServer);
    if (!vis.init(dbfile))
    {
        UERROR("Initializing visibility failed");
        return 1;
    }
    loc.setVisibility(&vis);

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
