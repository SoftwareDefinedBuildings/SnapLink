#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UConversion.h>
#include <cstdio>
#include <utility>
#include <QCoreApplication>
#include <QThread>
#include <QDebug>
#include "data/WordsKdTree.h"
#include "data/SignaturesSimple.h"
#include "data/LabelsSimple.h"
#include "stage/HTTPServer.h"
#include "adapter/RTABMapDBAdapter.h"
#include "stage/FeatureExtraction.h"
#include "stage/Perspective.h"
#include "stage/SignatureSearch.h"
#include "stage/Visibility.h"
#include "stage/WordSearch.h"


void showUsage()
{
    printf("\nUsage:\n"
           "CellMate database_file1 [database_file2 ...]\n");
    exit(1);
}

int main(int argc, char *argv[])
{
    //ULogger::setType(ULogger::kTypeConsole);
    //ULogger::setLevel(ULogger::kInfo);
    //ULogger::setLevel(ULogger::kDebug);

    std::vector<std::string> dbfiles;
    for (int i = 1; i < argc; i++)
    {
        dbfiles.emplace_back(argv[i]);
    }

    QCoreApplication app(argc, argv);

    std::unique_ptr<WordsKdTree> words(new WordsKdTree());
    std::unique_ptr<SignaturesSimple> signatures(new SignaturesSimple());
    std::unique_ptr<LabelsSimple> labels(new LabelsSimple());
    HTTPServer httpServer;
    FeatureExtraction feature;
    WordSearch wordSearch;
    SignatureSearch signatureSearch;
    Perspective perspective;
    Visibility vis;

    QThread featureThread;
    QThread wordSearchThread;
    QThread signatureSearchThread;
    QThread perspectiveThread;
    QThread visThread;

    rtabmap::ParametersMap params;
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpDetectorStrategy(), uNumber2Str(rtabmap::Feature2D::kFeatureSurf)));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), "3"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpIncrementalDictionary(), "false")); // do not create new word because we don't know whether extedning BOW dimension is good...
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNewWordsComparedTogether(), "false")); // do not compare with last signature's words
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNNStrategy(), uNumber2Str(rtabmap::VWDictionary::kNNBruteForce))); // bruteforce
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNndrRatio(), "0.3"));
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxFeatures(), "50000"));
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpBadSignRatio(), "0"));
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpRoiRatios(), "0.0 0.0 0.0 0.0"));
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemGenerateIds(), "true"));
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisIterations(), "2000"));
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPReprojError(), "1.0"));
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisPnPFlags(), "0")); // 0=Iterative, 1=EPNP, 2=P3P
    // params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kSURFGpuVersion(), "true"));

    std::cout << "Reading data" << std::endl;
    if (!RTABMapDBAdapter::readData(dbfiles, *words, *signatures, *labels))
    {
        qCritical() << "Reading data failed";
        return 1;
    }

    // Visibility
    std::cout << "Initializing Visibility" << std::endl;
    vis.putLabels(std::move(labels));
    vis.setHTTPServer(&httpServer);
    vis.moveToThread(&visThread);
    visThread.start();

    // Perspective
    std::cout << "Initializing Perspective" << std::endl;
    perspective.setHTTPServer(&httpServer);
    perspective.setVisibility(&vis);
    if (!perspective.init(params))
    {
        qCritical() << "Initializing Perspective failed";
        return 1;
    }
    perspective.moveToThread(&perspectiveThread);
    perspectiveThread.start();

    // Signature Search
    std::cout << "Initializing Signature Search" << std::endl;
    signatureSearch.putSignatures(std::move(signatures));
    signatureSearch.setPerspective(&perspective);
    signatureSearch.moveToThread(&signatureSearchThread);
    signatureSearchThread.start();

    // Word Search
    std::cout << "Initializing Word Search" << std::endl;
    wordSearch.putWords(std::move(words));
    wordSearch.setSignatureSearch(&signatureSearch);
    wordSearch.moveToThread(&wordSearchThread);
    wordSearchThread.start();

    // FeatureExtraction
    std::cout << "Initializing feature extraction" << std::endl;
    feature.init(params);
    feature.setWordSearch(&wordSearch);
    feature.moveToThread(&featureThread);
    featureThread.start();

    // HTTPServer
    std::cout << "Initializing HTTP server" << std::endl;
    httpServer.setFeatureExtraction(&feature);
    if (!httpServer.start())
    {
        qCritical() << "Starting HTTP Server failed";
        return 1;
    }

    return app.exec();
}
