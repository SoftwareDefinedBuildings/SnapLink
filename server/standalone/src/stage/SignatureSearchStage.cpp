#include "stage/SignatureSearchStage.h"
#include "data/Session.h"
#include "data/Signature.h"
#include "event/FailureEvent.h"
#include "event/SignatureEvent.h"
#include "event/WordEvent.h"
#include "stage/PerspectiveStage.h"
#include "util/Time.h"
#include "util/Utility.h"
#include <QCoreApplication>
#include <QtDebug>

SignatureSearchStage::SignatureSearchStage(
    const std::shared_ptr<Signatures> &signatures)
    : _perspectiveStage(nullptr), _signatureSearch(signatures) {}

SignatureSearchStage::~SignatureSearchStage() { _perspectiveStage = nullptr; }

void SignatureSearchStage::setPerspectiveStage(
    PerspectiveStage *perspectiveStage) {
  _perspectiveStage = perspectiveStage;
}

bool SignatureSearchStage::event(QEvent *event) {
  if (event->type() == WordEvent::type()) {
    WordEvent *wordEvent = static_cast<WordEvent *>(event);
    std::unique_ptr<std::vector<int>> wordIds = wordEvent->takeWordIds();
    std::unique_ptr<std::vector<cv::KeyPoint>> keyPoints =
        wordEvent->takeKeyPoints();
    std::unique_ptr<CameraModel> camera = wordEvent->takeCameraModel();
    std::unique_ptr<Session> session = wordEvent->takeSession();
    std::unique_ptr<std::vector<int>> signatureIds(new std::vector<int>());

    session->signaturesStart = getTime();
    *signatureIds = _signatureSearch.search(*wordIds);
    session->signaturesEnd = getTime();

    QCoreApplication::postEvent(
        _perspectiveStage,
        new SignatureEvent(std::move(wordIds), std::move(keyPoints),
                           std::move(camera), std::move(signatureIds),
                           std::move(session)));
    return true;
  }
  return QObject::event(event);
}
