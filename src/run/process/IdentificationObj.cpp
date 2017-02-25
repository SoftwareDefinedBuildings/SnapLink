#include "run/process/IdentificationObj.h"
#include "lib/data/CameraModel.h"
#include "lib/data/Session.h"
#include "lib/data/Transform.h"
#include "run/event/DetectionEvent.h"
#include "run/event/FailureEvent.h"
#include "run/event/QueryEvent.h"
#include "run/front_end/bosswave/BWFrontEndObj.h"
#include "run/front_end/http/HTTPFrontEndObj.h"
#include "lib/util/Utility.h"
#include <QCoreApplication>

IdentificationObj::IdentificationObj(const std::shared_ptr<Words> &words,
                                     std::unique_ptr<Labels> &&labels,
                                     int sampleSize, int corrSize,
                                     double distRatio)
    : _httpFrontEndObj(nullptr), _bwFrontEndObj(nullptr), _feature(sampleSize),
      _wordSearch(words), _dbSearch(words),
      _perspective(words, corrSize, distRatio), _visibility(std::move(labels)) {
}

IdentificationObj::~IdentificationObj() {
  _httpFrontEndObj = nullptr;
  _bwFrontEndObj = nullptr;
}

void IdentificationObj::setBWFrontEndObj(
    std::shared_ptr<BWFrontEndObj> bwFrontEndObj) {
  _bwFrontEndObj = bwFrontEndObj;
}

void IdentificationObj::setHTTPFrontEndObj(
    std::shared_ptr<HTTPFrontEndObj> httpFrontEndObj) {
  _httpFrontEndObj = httpFrontEndObj;
}

bool IdentificationObj::event(QEvent *event) {
  if (event->type() == QueryEvent::type()) {
    QueryEvent *queryEvent = static_cast<QueryEvent *>(event);
    std::unique_ptr<cv::Mat> image = queryEvent->takeImage();
    std::unique_ptr<CameraModel> camera = queryEvent->takeCameraModel();
    std::unique_ptr<Session> session = queryEvent->takeSession();

    std::unique_ptr<std::vector<std::string>> names(
        new std::vector<std::string>);
    bool success = identify(*image, *camera, *names, *session);

    if (success) {
      if (session->type == BOSSWAVE) {
        QCoreApplication::postEvent(
            _bwFrontEndObj.get(),
            new DetectionEvent(std::move(names), std::move(session)));
      } else {
        QCoreApplication::postEvent(
            _httpFrontEndObj.get(),
            new DetectionEvent(std::move(names), std::move(session)));
      }
    } else {
      if (session->type == BOSSWAVE) {
        QCoreApplication::postEvent(_bwFrontEndObj.get(),
                                    new FailureEvent(std::move(session)));
      } else {
        QCoreApplication::postEvent(_httpFrontEndObj.get(),
                                    new FailureEvent(std::move(session)));
      }
    }
    return true;
  }
  return QObject::event(event);
}

bool IdentificationObj::identify(const cv::Mat &image,
                                 const CameraModel &camera,
                                 std::vector<std::string> &names,
                                 Session &session) {
  // feature extraction
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  session.featuresStart = Utility::getTime();
  _feature.extract(image, keyPoints, descriptors);
  session.featuresEnd = Utility::getTime();

  // word search
  session.wordSearchStart = Utility::getTime();
  std::vector<int> wordIds = _wordSearch.search(descriptors);
  session.wordSearchEnd = Utility::getTime();

  // db search
  session.dbSearchStart = Utility::getTime();
  int dbId = _dbSearch.search(wordIds);
  session.dbSearchEnd = Utility::getTime();

  // PnP
  Transform pose;
  session.perspectiveStart = Utility::getTime();
  _perspective.localize(wordIds, keyPoints, descriptors, camera, dbId, pose);
  session.perspectiveEnd = Utility::getTime();

  if (pose.isNull()) {
    session.visibilityStart = 1;
    session.visibilityEnd = 0;
    return false;
  }

  // visibility
  session.visibilityStart = Utility::getTime();
  names = _visibility.process(dbId, camera, pose);
  session.visibilityEnd = Utility::getTime();

  return true;
}
