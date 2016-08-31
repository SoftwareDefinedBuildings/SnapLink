#include "service/PerspectiveStage.h"
#include "data/Session.h"
#include "data/Signature.h"
#include "data/Transform.h"
#include "message/FailureEvent.h"
#include "message/LocationEvent.h"
#include "message/SignatureEvent.h"
#include "front/HTTPServer.h"
#include "service/VisibilityStage.h"
#include "util/Time.h"
#include "util/Utility.h"
#include <QCoreApplication>
#include <QDebug>
#include <cassert>
#include <pcl/common/transforms.h>

PerspectiveStage::PerspectiveStage(
    const std::shared_ptr<Signatures> &signatures)
    : _visibilityStage(nullptr), _httpServer(nullptr),
      _perspective(signatures) {}

PerspectiveStage::~PerspectiveStage() {
  _visibilityStage = nullptr;
  _httpServer = nullptr;
}

void PerspectiveStage::setVisibilityStage(VisibilityStage *visibilityStage) {
  _visibilityStage = visibilityStage;
}

void PerspectiveStage::setHTTPServer(HTTPServer *httpServer) {
  _httpServer = httpServer;
}

bool PerspectiveStage::event(QEvent *event) {
  if (event->type() == SignatureEvent::type()) {
    SignatureEvent *signatureEvent = static_cast<SignatureEvent *>(event);
    std::unique_ptr<std::vector<int>> wordIds = signatureEvent->takeWordIds();
    std::unique_ptr<std::vector<cv::KeyPoint>> keyPoints =
        signatureEvent->takeKeyPoints();
    std::unique_ptr<CameraModel> camera = signatureEvent->takeCameraModel();
    std::unique_ptr<std::vector<int>> signatureIds =
        signatureEvent->takeSignatureIds();
    std::unique_ptr<Session> session = signatureEvent->takeSession();
    int dbId;
    std::unique_ptr<Transform> pose(new Transform);

    session->perspectiveStart = getTime();
    _perspective.localize(*wordIds, *keyPoints, *camera, signatureIds->at(0),
                          dbId, *pose);
    session->perspectiveEnd = getTime();

    // a null pose notify that loc could not be computed
    if (pose->isNull() == false) {
      QCoreApplication::postEvent(_visibilityStage,
                                  new LocationEvent(dbId, std::move(camera),
                                                    std::move(pose),
                                                    std::move(session)));
    } else {
      QCoreApplication::postEvent(_httpServer,
                                  new FailureEvent(std::move(session)));
    }
    return true;
  }
  return QObject::event(event);
}
