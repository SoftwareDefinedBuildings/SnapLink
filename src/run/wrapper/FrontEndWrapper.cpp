#include "run/wrapper/FrontEndWrapper.h"
#include "lib/util/Utility.h"
#include "run/event/DetectionEvent.h"
#include "run/event/FailureEvent.h"
#include "run/event/QueryEvent.h"
#include <QCoreApplication>
#include <QSemaphore>
#include <cstdlib>
#include <cstring>
#include <strings.h>

FrontEndWrapper::FrontEndWrapper(std::unique_ptr<FrontEnd> &&frontEnd)
    : _frontEnd(std::move(frontEnd)), _backEndWrapper(nullptr),
      _gen(std::random_device()()) {}

FrontEndWrapper::~FrontEndWrapper() {
  stop();
  _backEndWrapper.reset();
}

bool FrontEndWrapper::init() {
  if (_frontEnd == nullptr) {
    return false;
  }
  bool success = _frontEnd->start();
  if (success) {
    _frontEnd->registerOnQuery(std::bind(&FrontEndWrapper::onQuery, this,
                                         std::placeholders::_1,
                                         std::placeholders::_2));
  }

  return success;
}

void FrontEndWrapper::stop() {
  if (_frontEnd != nullptr) {
    _frontEnd->stop();
  }
}

void FrontEndWrapper::setBackEndWrapper(
    std::shared_ptr<BackEndWrapper> backEndWrapper) {
  _backEndWrapper = backEndWrapper;
}

bool FrontEndWrapper::event(QEvent *event) {
  if (event->type() == DetectionEvent::type()) {
    DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
    std::unique_ptr<Session> session = detectionEvent->takeSession();

    _mutex.lock();
    auto iter = _sessionMap.find(session->id);
    std::unique_ptr<SessionData> &sessionData = iter->second;
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
    std::unique_ptr<SessionData> &sessionData = iter->second;
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
FrontEndWrapper::onQuery(std::unique_ptr<cv::Mat> &&image,
                         std::unique_ptr<CameraModel> &&camera) {
  std::unique_ptr<Session> session(new Session);
  session->overallStart = Utility::getTime(); // log start of processing
  session->frontEnd.reset(this);

  std::unique_ptr<SessionData> sessionData(new SessionData);
  QSemaphore &detected = sessionData->detected;

  _mutex.lock();
  long id = _dis(_gen); // this is not thread safe
  _sessionMap.emplace(id, std::move(sessionData));
  _mutex.unlock();

  session->id = id;

  QCoreApplication::postEvent(
      _backEndWrapper.get(),
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
  std::cout << "Time wordSearch: "
            << session->wordSearchEnd - session->wordSearchStart << " ms"
            << std::endl;
  std::cout << "Time dbSearch: "
            << session->dbSearchEnd - session->dbSearchStart << " ms"
            << std::endl;
  std::cout << "Time perspective: "
            << session->perspectiveEnd - session->perspectiveStart << " ms"
            << std::endl;
  std::cout << "Time visibility: "
            << session->visibilityEnd - session->visibilityStart << " ms"
            << std::endl;

  return *(sessionData->names);
}
