#include "adapter/RTABMapDBAdapter.h"
#include "data/LabelsSimple.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "stage/FeatureExtraction.h"
#include "stage/HTTPServer.h"
#include "stage/Perspective.h"
#include "stage/SignatureSearch.h"
#include "stage/Visibility.h"
#include "stage/WordSearch.h"
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

  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  QCoreApplication app(argc, argv);

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::shared_ptr<SignaturesSimple> signatures(new SignaturesSimple());
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

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *signatures, *labels)) {
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
  perspective.setSignatures(signatures);
  perspective.setHTTPServer(&httpServer);
  perspective.setVisibility(&vis);
  perspective.moveToThread(&perspectiveThread);
  perspectiveThread.start();

  // Signature Search
  std::cout << "Initializing Signature Search" << std::endl;
  signatureSearch.setSignatures(signatures);
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
  feature.init();
  feature.setWordSearch(&wordSearch);
  feature.moveToThread(&featureThread);
  featureThread.start();

  // HTTPServer
  std::cout << "Initializing HTTP server" << std::endl;
  httpServer.setFeatureExtraction(&feature);
  if (!httpServer.start()) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  return app.exec();
}
