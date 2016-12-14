#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "event/QueryEvent.h"
#include "lib/data/CameraModel.h"
#include "lib/front_end/http/HTTPFrontEndWrapper.h"
#include "lib/util/Time.h"
#include "process/Identification.h"
#include <QCoreApplication>
#include <cstdlib>
#include <cstring>
#include <strings.h>

HTTPFrontEndWrapper::HTTPFrontEndWrapper()
    : _httpFront(new HTTPFrontEnd()), _identification(nullptr) {}

HTTPFrontEndWrapper::~HTTPFrontEndWrapper() {
  stop();
  _identification = nullptr;
}

bool HTTPFrontEndWrapper::start(uint16_t port, unsigned int maxClients) {
  if (_httpFront != nullptr) {
    return false;
  }
  return _httpFront->start(port, maxClients);
}

void HTTPFrontEndWrapper::stop() {
  if (_httpFront != nullptr) {
    _httpFront->stop();
  }
}

void HTTPFrontEndWrapper::setIdentification(
    std::shared_ptr<Identification> identification) {
  _identification = identification;
}

bool HTTPFrontEndWrapper::event(QEvent *event) {
  if (event->type() == DetectionEvent::type()) {
    DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
    std::unique_ptr<Session> session = detectionEvent->takeSession();
    // find() const is thread-safe
    const auto iter = _sessionInfo.find(session->id);
    ConnectionInfo *connInfo = iter->second;
    connInfo->names = detectionEvent->takeNames();
    connInfo->session = std::move(session);
    connInfo->detected.release();
    return true;
  } else if (event->type() == FailureEvent::type()) {
    FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
    std::unique_ptr<Session> session = failureEvent->takeSession();
    // find() const is thread-safe
    const auto iter = _sessionInfo.find(session->id);
    ConnectionInfo *connInfo = iter->second;
    connInfo->session = std::move(session);
    connInfo->detected.release();
    return true;
  }
  return QObject::event(event);
}

std::vector<std::string>
HTTPFrontEndWrapper::onQuery(std::unique_ptr<cv::Mat> &&image,
                             std::unique_ptr<CameraModel> &&camera,
                             std::unique_ptr<Session> &&session) {
  std::shared_ptr<SessionInfo> sessionInfo(new SessionInfo());

  httpServer->_mutex.lock();
  connInfo->session->id = httpServer->_dis(httpServer->_gen);
  httpServer->_connInfoMap.insert(
      std::make_pair(connInfo->session->id, connInfo));
  httpServer->_mutex.unlock();

  QCoreApplication::postEvent(httpServer->_identification,
                              new QueryEvent(std::move(image),
                                             std::move(camera),
                                             std::move(connInfo->session)));
}
