#include "stage/WordSearchStage.h"
#include "data/Session.h"
#include "data/Signature.h"
#include "event/FeatureEvent.h"
#include "event/WordEvent.h"
#include "stage/SignatureSearchStage.h"
#include "util/Time.h"
#include <QCoreApplication>
#include <cassert>

WordSearchStage::WordSearchStage(std::unique_ptr<Words> &&words)
    : _signatureSearchStage(nullptr), _wordSearch(std::move(words)) {}

WordSearchStage::~WordSearchStage() { _signatureSearchStage = nullptr; }

void WordSearchStage::setSignatureSearchStage(
    SignatureSearchStage *signatureSearchStage) {
  _signatureSearchStage = signatureSearchStage;
}

bool WordSearchStage::event(QEvent *event) {
  if (event->type() == FeatureEvent::type()) {
    FeatureEvent *featureEvent = static_cast<FeatureEvent *>(event);
    std::unique_ptr<std::vector<cv::KeyPoint>> keyPoints =
        featureEvent->takeKeyPoints();
    std::unique_ptr<cv::Mat> descriptors = featureEvent->takeDescriptors();
    std::unique_ptr<CameraModel> camera = featureEvent->takeCameraModel();
    std::unique_ptr<Session> session = featureEvent->takeSession();
    std::unique_ptr<std::vector<int>> wordIds(new std::vector<int>());

    session->wordsStart = getTime();
    *wordIds = _wordSearch.searchWords(*descriptors);
    session->wordsEnd = getTime();

    QCoreApplication::postEvent(
        _signatureSearchStage,
        new WordEvent(std::move(wordIds), std::move(keyPoints),
                      std::move(camera), std::move(session)));
    return true;
  }
  return QObject::event(event);
}
