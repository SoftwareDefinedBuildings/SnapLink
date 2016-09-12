#include "stage/VisibilityStage.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include "data/Transform.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "event/LocationEvent.h"
#include "front/HTTPServer.h"
#include "util/Utility.h"
#include <QCoreApplication>
#include <QDebug>
#include <QDirIterator>
#include <QTextStream>
#include <fstream>
#include <iostream>
#include <opencv/cv.h>
#include <pcl/point_types.h>

VisibilityStage::VisibilityStage(std::unique_ptr<Labels> &&labels)
    : _httpServer(nullptr), _visibility(std::move(labels)) {}

VisibilityStage::~VisibilityStage() { _httpServer = nullptr; }

void VisibilityStage::setHTTPServer(HTTPServer *httpServer) {
  _httpServer = httpServer;
}

bool VisibilityStage::event(QEvent *event) {
  if (event->type() == LocationEvent::type()) {
    LocationEvent *locEvent = static_cast<LocationEvent *>(event);
    std::unique_ptr<CameraModel> camera = locEvent->takeCameraModel();
    std::unique_ptr<Transform> pose = locEvent->takePose();
    std::unique_ptr<Session> session = locEvent->takeSession();
    std::unique_ptr<std::vector<std::string>> names(
        new std::vector<std::string>());

    *names = _visibility.process(locEvent->dbId(), *camera, *pose);

    if (!names->empty()) {
      QCoreApplication::postEvent(
          _httpServer,
          new DetectionEvent(std::move(names), std::move(session)));
    } else {
      QCoreApplication::postEvent(_httpServer,
                                  new FailureEvent(std::move(session)));
    }
    return true;
  }
  return QObject::event(event);
}
