#include "stage/WordSearch.h"
#include "data/PerfData.h"
#include "data/Signature.h"
#include "event/FeatureEvent.h"
#include "event/WordEvent.h"
#include "stage/SignatureSearch.h"
#include "util/Time.h"
#include <QCoreApplication>
#include <cassert>

WordSearch::WordSearch() : _imageSearch(nullptr) {}

WordSearch::~WordSearch() { _imageSearch = nullptr; }

void WordSearch::putWords(std::unique_ptr<Words> &&words) {
  _words = std::move(words);
}

void WordSearch::setSignatureSearch(SignatureSearch *imageSearch) {
  _imageSearch = imageSearch;
}

bool WordSearch::event(QEvent *event) {
  if (event->type() == FeatureEvent::type()) {
    FeatureEvent *featureEvent = static_cast<FeatureEvent *>(event);
    std::unique_ptr<std::vector<cv::KeyPoint>> keyPoints =
        featureEvent->takeKeyPoints();
    std::unique_ptr<cv::Mat> descriptors = featureEvent->takeDescriptors();
    std::unique_ptr<CameraModel> camera = featureEvent->takeCameraModel();
    std::unique_ptr<PerfData> perfData = featureEvent->takePerfData();
    const void *session = featureEvent->getSession();
    std::unique_ptr<std::vector<int>> wordIds(new std::vector<int>());

    perfData->wordsStart = getTime();
    *wordIds = searchWords(*descriptors);
    perfData->wordsEnd = getTime();

    QCoreApplication::postEvent(
        _imageSearch,
        new WordEvent(std::move(wordIds), std::move(keyPoints),
                      std::move(camera), std::move(perfData), session));
    return true;
  }
  return QObject::event(event);
}

std::vector<int> WordSearch::searchWords(const cv::Mat &descriptors) const {
  assert(descriptors.rows > 0);
  return _words->findNNs(descriptors);
}
