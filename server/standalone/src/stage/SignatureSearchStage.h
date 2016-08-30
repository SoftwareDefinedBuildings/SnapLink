#pragma once

#include "algo/SignatureSearch.h"
#include "data/Signatures.h"
#include <QEvent>
#include <QObject>
#include <memory>

#define TOP_K 1

class PerspectiveStage;

class SignatureSearchStage : public QObject {
public:
  SignatureSearchStage(const std::shared_ptr<Signatures> &signatures);
  ~SignatureSearchStage();

  void setPerspectiveStage(PerspectiveStage *perspectiveStage);

protected:
  virtual bool event(QEvent *event);

private:
  PerspectiveStage *_perspectiveStage;
  SignatureSearch _signatureSearch;
};
