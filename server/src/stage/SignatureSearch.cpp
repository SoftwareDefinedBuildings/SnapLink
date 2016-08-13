#include "stage/SignatureSearch.h"
#include "data/PerfData.h"
#include "data/Signature.h"
#include "event/FailureEvent.h"
#include "event/SignatureEvent.h"
#include "event/WordEvent.h"
#include "util/Time.h"
#include <QCoreApplication>
#include <QtDebug>

SignatureSearch::SignatureSearch() : _perspective(nullptr) {}

SignatureSearch::~SignatureSearch() { _perspective = nullptr; }

void SignatureSearch::putSignatures(std::unique_ptr<Signatures> &&signatures) {
  _signatures = std::move(signatures);
}

void SignatureSearch::setPerspective(Perspective *perspective) {
  _perspective = perspective;
}

bool SignatureSearch::event(QEvent *event) {
  if (event->type() == WordEvent::type()) {
    WordEvent *wordEvent = static_cast<WordEvent *>(event);
    std::unique_ptr<std::vector<int>> wordIds = wordEvent->takeWordIds();
    std::unique_ptr<SensorData> sensorData = wordEvent->takeSensorData();
    std::unique_ptr<PerfData> perfData = wordEvent->takePerfData();
    const void *session = wordEvent->getSession();
    perfData->signaturesStart = getTime();
    std::vector<std::unique_ptr<Signature>> signatures =
        searchSignatures(*wordIds);
    perfData->signaturesEnd = getTime();
    QCoreApplication::postEvent(
        _perspective, new SignatureEvent(
                          std::move(wordIds), std::move(sensorData),
                          std::move(signatures), std::move(perfData), session));
    return true;
  }
  return QObject::event(event);
}

std::vector<std::unique_ptr<Signature>>
SignatureSearch::searchSignatures(const std::vector<int> &wordIds) const {
  std::vector<int> topIds = _signatures->findKNN(wordIds, TOP_K);
  int topSigId = topIds[0];
  std::unique_ptr<Signature> topSig(
      new Signature(*_signatures->getSignatures().at(topSigId)));

  qDebug() << "topSigId: " << topSigId;

  std::vector<std::unique_ptr<Signature>> signatures;
  signatures.emplace_back(std::move(topSig));
  return signatures;
}
