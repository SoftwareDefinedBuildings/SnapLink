#include "process/IdentificationObj.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include "data/Transform.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "event/QueryEvent.h"
#include "front/BWServer.h"
#include "front/HTTPServer.h"
#include "util/Time.h"
#include <QCoreApplication>

IdentificationObj::IdentificationObj(const std::shared_ptr<Words> &words,
                                     std::unique_ptr<Labels> &&labels)
    : _httpServer(nullptr), _bwServer(nullptr), _wordSearch(words),
      _perspective(words), _visibility(std::move(labels)) {}

IdentificationObj::~IdentificationObj() {
  _httpServer = nullptr;
  _bwServer = nullptr;
}

void IdentificationObj::setBWServer(BWServer *bwServer) {
  _bwServer = bwServer;
}

void IdentificationObj::setHTTPServer(HTTPServer *httpServer) {
  _httpServer = httpServer;
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
            _bwServer,
            new DetectionEvent(std::move(names), std::move(session)));
      } else {
        QCoreApplication::postEvent(
            _httpServer,
            new DetectionEvent(std::move(names), std::move(session)));
      }
    } else {
      if (session->type == BOSSWAVE) {
        QCoreApplication::postEvent(_bwServer,
                                    new FailureEvent(std::move(session)));
      } else {
        QCoreApplication::postEvent(_httpServer,
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
  session.featuresStart = getTime();
  _feature.extract(image, keyPoints, descriptors);
  session.featuresEnd = getTime();

  std::vector<unsigned int> indices(keyPoints.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::random_shuffle(indices.begin(), indices.end());
  std::vector<cv::KeyPoint> subKeyPoints;
  cv::Mat subDescriptors;
  unsigned int sampleSize = SAMPLE_SIZE;
  for (int i = 0; i < sampleSize && i < indices.size(); i++) {
    subKeyPoints.emplace_back(keyPoints[i]);
    subDescriptors.push_back(descriptors.row(i));
  }

  // word search
  session.wordsStart = getTime();
  std::vector<int> subWordIds = _wordSearch.search(subDescriptors);
  session.wordsEnd = getTime();

  // PnP
  int dbId;
  Transform pose;
  session.perspectiveStart = getTime();
  _perspective.localize(subWordIds, subKeyPoints, subDescriptors, camera, dbId,
                        pose);
  session.perspectiveEnd = getTime();

  if (pose.isNull()) {
    return false;
  }

  // visibility
  names = _visibility.process(dbId, camera, pose);

  return true;
}
