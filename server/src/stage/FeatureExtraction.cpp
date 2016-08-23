#include "stage/FeatureExtraction.h"
#include "event/FeatureEvent.h"
#include "event/QueryEvent.h"
#include "stage/WordSearch.h"
#include "util/Time.h"
#include <QCoreApplication>

FeatureExtraction::FeatureExtraction() : _wordSearch(nullptr) {}

FeatureExtraction::~FeatureExtraction() { _wordSearch = nullptr; }

bool FeatureExtraction::init() {
  int minHessian = 400;
  _detector = cv::xfeatures2d::SURF::create(minHessian);

  return true;
}

void FeatureExtraction::setWordSearch(WordSearch *wordSearch) {
  _wordSearch = wordSearch;
}

bool FeatureExtraction::event(QEvent *event) {
  if (event->type() == QueryEvent::type()) {
    QueryEvent *queryEvent = static_cast<QueryEvent *>(event);
    std::unique_ptr<cv::Mat> image = queryEvent->takeImage();
    std::unique_ptr<CameraModel> camera = queryEvent->takeCameraModel();
    std::unique_ptr<PerfData> perfData = queryEvent->takePerfData();
    const void *session = queryEvent->getSession();

    std::unique_ptr<std::vector<cv::KeyPoint>> keyPoints(
        new std::vector<cv::KeyPoint>());
    std::unique_ptr<cv::Mat> descriptors(new cv::Mat());
    perfData->featuresStart = getTime();
    extractFeatures(*image, *keyPoints, *descriptors);
    perfData->featuresEnd = getTime();

    QCoreApplication::postEvent(
        _wordSearch,
        new FeatureEvent(std::move(keyPoints), std::move(descriptors),
                         std::move(camera), std::move(perfData), session));

    return true;
  }
  return QObject::event(event);
}

void FeatureExtraction::extractFeatures(const cv::Mat &image,
                                        std::vector<cv::KeyPoint> &keyPoints,
                                        cv::Mat &descriptors) const {
  _detector->detectAndCompute(image, cv::Mat(), keyPoints, descriptors);
}
