#include "front_end/http/HTTPFrontEndObj.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "event/QueryEvent.h"
#include "process/IdentificationObj.h"
#include "util/Time.h"
#include <QCoreApplication>
#include <QSemaphore>
#include <cstdlib>
#include <cstring>
#include <strings.h>

HTTPFrontEndObj::HTTPFrontEndObj()
    : _httpFront(new HTTPFrontEnd()), _identObj(nullptr),
      _gen(std::random_device()()) {}

HTTPFrontEndObj::~HTTPFrontEndObj() {
  stop();
  _identObj = nullptr;
}

bool HTTPFrontEndObj::init(uint16_t port, unsigned int maxClients) {
  if (_httpFront == nullptr) {
    return false;
  }
  bool success = _httpFront->start(port, maxClients);
  if (success) {
    _httpFront->registerOnQuery(std::bind(&HTTPFrontEndObj::onQuery, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
  }

  return success;
}

void HTTPFrontEndObj::stop() {
  if (_httpFront != nullptr) {
    _httpFront->stop();
  }
}

void HTTPFrontEndObj::setIdentificationObj(
    std::shared_ptr<IdentificationObj> identObj) {
  _identObj = identObj;
}

bool HTTPFrontEndObj::event(QEvent *event) {
  if (event->type() == DetectionEvent::type()) {
    DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
    std::unique_ptr<Session> session = detectionEvent->takeSession();
    session->names = detectionEvent->takeNames();
    QSemaphore &detected = session->detected;

    _mutex.lock();
    auto iter = _sessionMap.find(session->id);
    iter->second = std::move(session);
    _mutex.unlock();

    detected.release();
    return true;
  } else if (event->type() == FailureEvent::type()) {
    FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
    std::unique_ptr<Session> session = failureEvent->takeSession();
    QSemaphore &detected = session->detected;

    _mutex.lock();
    auto iter = _sessionMap.find(session->id);
    iter->second = std::move(session);
    _mutex.unlock();

    detected.release();
    return true;
  }
  return QObject::event(event);
}

std::vector<std::string>
HTTPFrontEndObj::onQuery(std::unique_ptr<cv::Mat> &&image,
                         std::unique_ptr<CameraModel> &&camera) {
  std::unique_ptr<Session> session(new Session);
  session->overallStart = getTime(); // log start of processing
  session->type = HTTP_POST;

  _mutex.lock();
  long id = _dis(_gen); // this is not thread safe
  _sessionMap.emplace(id, std::unique_ptr<Session>());
  _mutex.unlock();

  session->id = id;
  QSemaphore &detected = session->detected;

  QCoreApplication::postEvent(
      _identObj.get(),
      new QueryEvent(std::move(image), std::move(camera), std::move(session)));

  // TODO use condition variable?
  detected.acquire();

  _mutex.lock();
  auto iter = _sessionMap.find(id);
  session = std::move(iter->second);
  _sessionMap.erase(id);
  _mutex.unlock();

  assert(session != nullptr);

  // print time
  session->overallEnd = getTime(); // log processing end time
  std::cout << "Time overall: " << session->overallEnd - session->overallStart
            << " ms" << std::endl;
  std::cout << "Time features: "
            << session->featuresEnd - session->featuresStart << " ms"
            << std::endl;
  std::cout << "Time words: " << session->wordsEnd - session->wordsStart
            << " ms" << std::endl;
  std::cout << "Time perspective: "
            << session->perspectiveEnd - session->perspectiveStart << " ms"
            << std::endl;

  return *(session->names);
}
