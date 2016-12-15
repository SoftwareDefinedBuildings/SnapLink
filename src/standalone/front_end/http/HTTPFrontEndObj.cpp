#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "event/QueryEvent.h"
#include "lib/data/CameraModel.h"
#include "lib/front_end/http/HTTPFrontEndObj.h"
#include "lib/util/Time.h"
#include "process/Identification.h"
#include <QCoreApplication>
#include <cstdlib>
#include <cstring>
#include <strings.h>

HTTPFrontEndObj::HTTPFrontEndObj()
    : _httpFront(new HTTPFrontEnd()), _identification(nullptr), _gen(std::random_device()()) {}

HTTPFrontEndObj::~HTTPFrontEndObj() {
  stop();
  _identification = nullptr;
}

bool HTTPFrontEndObj::start(uint16_t port, unsigned int maxClients) {
  if (_httpFront != nullptr) {
    return false;
  }
  return _httpFront->start(port, maxClients);
}

void HTTPFrontEndObj::stop() {
  if (_httpFront != nullptr) {
    _httpFront->stop();
  }
}

void HTTPFrontEndObj::setIdentification(
    std::shared_ptr<Identification> identification) {
  _identification = identification;
}

bool HTTPFrontEndObj::event(QEvent *event) {
  if (event->type() == DetectionEvent::type()) {
    DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
    std::shared_ptr<Session> session = detectionEvent->takeSession();
    session->names = detectionEvent->takeNames();
    session->detected.release();
    session.reset();
    return true;
  } else if (event->type() == FailureEvent::type()) {
    FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
    std::shared_ptr<Session> session = failureEvent->getSession();
    session->detected.release();
    session.reset();
    return true;
  }
  return QObject::event(event);
}

std::vector<std::string>
HTTPFrontEndObj::onQuery(std::unique_ptr<cv::Mat> &&image,
                             std::unique_ptr<CameraModel> &&camera) {
  std::shared_ptr<SessionInfo> session(new Session());

  QCoreApplication::postEvent(httpServer->_identification,
                              new QueryEvent(std::move(image),
                                             std::move(camera),
                                             session));

  // TODO use condition variable?
  session->detected.acquire();

  _mutex.lock();
  std::vector<std::string> names = std::move(session->names);
  _mutex.unlock();

  return names;
}
