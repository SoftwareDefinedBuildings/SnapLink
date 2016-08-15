#include "stage/WordSearch.h"
#include "data/PerfData.h"
#include "data/Signature.h"
#include "event/FeatureEvent.h"
#include "event/WordEvent.h"
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
    std::unique_ptr<SensorData> sensorData = featureEvent->takeSensorData();
    std::unique_ptr<PerfData> perfData = featureEvent->takePerfData();
    const void *session = featureEvent->getSession();
    std::unique_ptr<std::vector<int>> wordIds(new std::vector<int>());
    perfData->wordsStart = getTime();
    *wordIds = searchWords(*sensorData);
    perfData->wordsEnd = getTime();
    // a null pose notify that loc could not be computed
    QCoreApplication::postEvent(
        _imageSearch, new WordEvent(std::move(wordIds), std::move(sensorData),
                                    std::move(perfData), session));
    return true;
  }
  return QObject::event(event);
}

// TODO maybe only pass skeypoints, descriptors, and model
std::vector<int> WordSearch::searchWords(const SensorData &sensorData) const {
  assert(!sensorData.getImage().empty());

  cv::Mat descriptors = sensorData.descriptors();

  assert(descriptors.rows > 0);

  std::vector<int> wordIds = _words->findNNs(descriptors);

  return wordIds;
}
