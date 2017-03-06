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
  std::cerr << "FrontEndWrapper destructor" << std::endl;
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
    const std::shared_ptr<BackEndWrapper> &backEndWrapper) {
  _backEndWrapper = backEndWrapper;
}

bool FrontEndWrapper::event(QEvent *event) {
  if (event->type() == DetectionEvent::type()) {
    DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
    std::unique_ptr<Session> session = detectionEvent->takeSession();

    _mutex.lock();
    session->results = std::move(detectionEvent->takeResults());
    QSemaphore &detected = session->detected;
    _sessionMap[session->id] = std::move(session);
    _mutex.unlock();

    detected.release();
    return true;
  } else if (event->type() == FailureEvent::type()) {
    FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
    std::unique_ptr<Session> session = failureEvent->takeSession();

    _mutex.lock();
    QSemaphore &detected = session->detected;
    _sessionMap[session->id] = std::move(session);
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
  session->frontEndWrapper = shared_from_this();
  QSemaphore &detected = session->detected;

  _mutex.lock();
  long id = _dis(_gen); // _dis(_gen) is not thread safe
  _mutex.unlock();

  session->id = id;

  // handle multi-threads using message passing (as opposed to shared memory)
  QCoreApplication::postEvent(
      _backEndWrapper.get(),
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

  return *(session->results);
}
