#include "process/Identification.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include "data/Transform.h"
#include "event/DetectionEvent.h"
#include "event/QueryEvent.h"
#include "front/HTTPServer.h"
#include "util/Time.h"
#include <QCoreApplication>

Identification::Identification(const std::shared_ptr<Words> &words,
                               std::unique_ptr<Labels> &&labels)
    : _httpServer(nullptr), _wordSearch(words), _perspective(words),
      _visibility(std::move(labels)) {}

Identification::~Identification() { _httpServer = nullptr; }

void Identification::setHTTPServer(HTTPServer *httpServer) {
  _httpServer = httpServer;
}

bool Identification::event(QEvent *event) {
  if (event->type() == QueryEvent::type()) {
    QueryEvent *queryEvent = static_cast<QueryEvent *>(event);
    std::unique_ptr<cv::Mat> image = queryEvent->takeImage();
    std::unique_ptr<CameraModel> camera = queryEvent->takeCameraModel();
    std::unique_ptr<Session> session = queryEvent->takeSession();

    std::unique_ptr<std::vector<std::string>> names(
        new std::vector<std::string>);
    *names = identify(*image, *camera, *session);

    QCoreApplication::postEvent(
        _httpServer, new DetectionEvent(std::move(names), std::move(session)));

    return true;
  }
  return QObject::event(event);
}

std::vector<std::string> Identification::identify(const cv::Mat &image,
                                                  const CameraModel &camera,
                                                  Session &session) {
  // feature extraction
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  session.featuresStart = getTime();
  _feature.extract(image, keyPoints, descriptors);
  session.featuresEnd = getTime();

  // word search
  session.wordsStart = getTime();
  std::vector<int> wordIds = _wordSearch.search(descriptors);
  session.wordsEnd = getTime();

  // PnP
  int dbId;
  Transform pose;
  session.perspectiveStart = getTime();
  _perspective.localize(wordIds, keyPoints, camera, dbId, pose);
  session.perspectiveEnd = getTime();

  // visibility
  std::vector<std::string> names = _visibility.process(dbId, camera, pose);

  return names;
}
