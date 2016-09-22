#include "adapter/RTABMapDBAdapter.h"
#include "data/LabelsSimple.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "front/HTTPServer.h"
#include "stage/FeatureStage.h"
#include "stage/PerspectiveStage.h"
#include "stage/SignatureSearchStage.h"
#include "stage/VisibilityStage.h"
#include "stage/WordSearchStage.h"
#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include <cstdio>
#include <utility>

// void showUsage() {
//   printf("\nUsage:\n"
//          "CellMate database_file1 [database_file2 ...]\n");
//   exit(1);
// }

int main(int argc, char *argv[]) {
  // ULogger::setType(ULogger::kTypeConsole);
  // ULogger::setLevel(ULogger::kInfo);
  // ULogger::setLevel(ULogger::kDebug);
 std::cout << "test 1" << std::endl;

  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }
  
  QCoreApplication app(argc, argv);

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::shared_ptr<SignaturesSimple> signatures(new SignaturesSimple());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  QThread featureThread;
  QThread wordSearchThread;
  QThread signatureSearchThread;
  QThread perspectiveThread;
  QThread visThread;

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *signatures, *labels)) {
    qCritical() << "Reading data failed";
    return 1;
  }

  HTTPServer httpServer;
  FeatureStage feature;
  WordSearchStage wordSearch(std::move(words));
  SignatureSearchStage signatureSearch(signatures);
  PerspectiveStage perspective(signatures);
  VisibilityStage vis(std::move(labels));

  // VisibilityStage
  std::cout << "Initializing VisibilityStage" << std::endl;
  vis.setHTTPServer(&httpServer);
  vis.moveToThread(&visThread);
  visThread.start();

  // PerspectiveStage
  std::cout << "Initializing PerspectiveStage" << std::endl;
  perspective.setHTTPServer(&httpServer);
  perspective.setVisibilityStage(&vis);
  perspective.moveToThread(&perspectiveThread);
  perspectiveThread.start();

  // Signature Search
  std::cout << "Initializing Signature Search" << std::endl;
  signatureSearch.setPerspectiveStage(&perspective);
  signatureSearch.moveToThread(&signatureSearchThread);
  signatureSearchThread.start();

  // Word Search
  std::cout << "Initializing Word Search" << std::endl;
  wordSearch.setSignatureSearchStage(&signatureSearch);
  wordSearch.moveToThread(&wordSearchThread);
  wordSearchThread.start();

  // FeatureStage
  std::cout << "Initializing feature extraction" << std::endl;
  feature.setWordSearchStage(&wordSearch);
  feature.moveToThread(&featureThread);
  featureThread.start();

  // HTTPServer
  std::cout << "Initializing HTTP server" << std::endl;
  httpServer.setFeatureStage(&feature);
  if (!httpServer.start()) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  return app.exec();
}
