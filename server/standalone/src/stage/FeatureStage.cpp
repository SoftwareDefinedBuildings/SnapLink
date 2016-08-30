#include "stage/FeatureStage.h"
#include "event/FeatureEvent.h"
#include "event/QueryEvent.h"
#include "stage/WordSearchStage.h"
#include "util/Time.h"
#include <QCoreApplication>

FeatureStage::FeatureStage() : _wordSearchStage(nullptr) {}

FeatureStage::~FeatureStage() { _wordSearchStage = nullptr; }

void FeatureStage::setWordSearchStage(WordSearchStage *wordSearchStage) {
  _wordSearchStage = wordSearchStage;
}

bool FeatureStage::event(QEvent *event) {
  if (event->type() == QueryEvent::type()) {
    QueryEvent *queryEvent = static_cast<QueryEvent *>(event);
    std::unique_ptr<cv::Mat> image = queryEvent->takeImage();
    std::unique_ptr<CameraModel> camera = queryEvent->takeCameraModel();
    std::unique_ptr<Session> session = queryEvent->takeSession();

    std::unique_ptr<std::vector<cv::KeyPoint>> keyPoints(
        new std::vector<cv::KeyPoint>());
    std::unique_ptr<cv::Mat> descriptors(new cv::Mat());
    session->featuresStart = getTime();
    _feature.extract(*image, *keyPoints, *descriptors);
    session->featuresEnd = getTime();

    QCoreApplication::postEvent(
        _wordSearchStage,
        new FeatureEvent(std::move(keyPoints), std::move(descriptors),
                         std::move(camera), std::move(session)));

    return true;
  }
  return QObject::event(event);
}
