#include "stage/SignatureSearch.h"
#include "data/PerfData.h"
#include "data/Signature.h"
#include "event/FailureEvent.h"
#include "event/SignatureEvent.h"
#include "event/WordEvent.h"
#include "stage/Perspective.h"
#include "util/Time.h"
#include <QCoreApplication>
#include <QtDebug>

SignatureSearch::SignatureSearch() : _perspective(nullptr) {}

SignatureSearch::~SignatureSearch() { _perspective = nullptr; }

void SignatureSearch::setSignatures(
    const std::shared_ptr<Signatures> &signatures) {
  _signatures = signatures;
}

void SignatureSearch::setPerspective(Perspective *perspective) {
  _perspective = perspective;
}

bool SignatureSearch::event(QEvent *event) {
  if (event->type() == WordEvent::type()) {
    WordEvent *wordEvent = static_cast<WordEvent *>(event);
    std::unique_ptr<std::vector<int>> wordIds = wordEvent->takeWordIds();
    std::unique_ptr<SensorData> sensorData = wordEvent->takeSensorData();
    std::unique_ptr<PerfData> perfData = wordEvent->takePerfData();
    const void *session = wordEvent->getSession();

    std::unique_ptr<std::vector<int>> signatureIds(new std::vector<int>());
    perfData->signaturesStart = getTime();
    *signatureIds = search(*wordIds);
    perfData->signaturesEnd = getTime();

    std::unique_ptr<std::multimap<int, cv::KeyPoint>> words(
        new std::multimap<int, cv::KeyPoint>());
    *words = createWords(*wordIds, sensorData->keypoints());
    std::unique_ptr<CameraModel> camera(
        new CameraModel(sensorData->getCameraModel()));

    QCoreApplication::postEvent(
        _perspective, new SignatureEvent(std::move(words), std::move(camera),
                                         std::move(signatureIds),
                                         std::move(perfData), session));
    return true;
  }
  return QObject::event(event);
}

std::vector<int>
SignatureSearch::search(const std::vector<int> &wordIds) const {
  return _signatures->findKNN(wordIds, TOP_K);
}

std::multimap<int, cv::KeyPoint>
SignatureSearch::createWords(const std::vector<int> &wordIds,
                             const std::vector<cv::KeyPoint> &keypoints) {
  std::multimap<int, cv::KeyPoint> words;
  assert(wordIds.size() == keypoints.size());
  unsigned int i = 0;
  for (auto iter = wordIds.begin();
       iter != wordIds.end() && i < keypoints.size(); ++iter, ++i) {
    words.insert(std::pair<int, cv::KeyPoint>(*iter, keypoints[i]));
  }

  return words;
}
