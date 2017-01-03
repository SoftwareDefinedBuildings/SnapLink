#include "front_end/bosswave/BWFrontEndObj.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "event/QueryEvent.h"
#include "process/IdentificationObj.h"
#include "util/Utility.h"
#include <QCoreApplication>
#include <QSemaphore>
#include <cstdlib>
#include <cstring>
#include <strings.h>

BWFrontEndObj::BWFrontEndObj()
    : _bwFront(new BWFrontEnd()), _identObj(nullptr),
      _gen(std::random_device()()) {}

BWFrontEndObj::~BWFrontEndObj() {
  stop();
  _identObj = nullptr;
}

bool BWFrontEndObj::init(unsigned int maxClients) {
  if (_bwFront == nullptr) {
    return false;
  }
  bool success = _bwFront->start(maxClients);
  if (success) {
    _bwFront->registerOnQuery(std::bind(&BWFrontEndObj::onQuery, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
  }

  return success;
}

void BWFrontEndObj::stop() {
  if (_bwFront != nullptr) {
    _bwFront->stop();
  }
}

void BWFrontEndObj::setIdentificationObj(
    std::shared_ptr<IdentificationObj> identObj) {
  _identObj = identObj;
}

bool BWFrontEndObj::event(QEvent *event) {
  if (event->type() == DetectionEvent::type()) {
    DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
    std::unique_ptr<Session> session = detectionEvent->takeSession();

    _mutex.lock();
    auto iter = _sessionMap.find(session->id);
    std::unique_ptr<BWSessionData> &sessionData = iter->second;
    sessionData->session = std::move(session);
    sessionData->names = detectionEvent->takeNames();
    QSemaphore &detected = sessionData->detected;
    _mutex.unlock();

    detected.release();
    return true;
  } else if (event->type() == FailureEvent::type()) {
    FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
    std::unique_ptr<Session> session = failureEvent->takeSession();

    _mutex.lock();
    auto iter = _sessionMap.find(session->id);
    std::unique_ptr<BWSessionData> &sessionData = iter->second;
    sessionData->session = std::move(session);
    sessionData->names.reset(new std::vector<std::string>());
    QSemaphore &detected = sessionData->detected;
    _mutex.unlock();

    detected.release();
    return true;
  }
  return QObject::event(event);
}

std::vector<std::string>
BWFrontEndObj::onQuery(std::unique_ptr<cv::Mat> &&image,
                       std::unique_ptr<CameraModel> &&camera) {
  std::unique_ptr<Session> session(new Session);
  session->overallStart = Utility::getTime(); // log start of processing
  session->type = BOSSWAVE;

  std::unique_ptr<BWSessionData> sessionData(new BWSessionData);
  QSemaphore &detected = sessionData->detected;

  _mutex.lock();
  long id = _dis(_gen); // this is not thread safe
  _sessionMap.emplace(id, std::move(sessionData));
  _mutex.unlock();

  session->id = id;

  QCoreApplication::postEvent(
      _identObj.get(),
      new QueryEvent(std::move(image), std::move(camera), std::move(session)));

  // TODO use condition variable?
  detected.acquire();

  _mutex.lock();
  auto iter = _sessionMap.find(id);
  sessionData = std::move(iter->second);
  _sessionMap.erase(id);
  _mutex.unlock();

  assert(sessionData != nullptr);
  assert(sessionData->session != nullptr);

  session = std::move(sessionData->session);

  // print time
  session->overallEnd = Utility::getTime(); // log processing end time
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

  return *(sessionData->names);
}
