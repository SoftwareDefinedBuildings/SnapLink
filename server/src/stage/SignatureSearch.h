#pragma once

#include "data/SensorData.h"
#include "data/Signatures.h"
#include "data/Transform.h"
#include "stage/Perspective.h"
#include <QEvent>
#include <QObject>
#include <memory>

#define TOP_K 1

class Perspective;

class SignatureSearch : public QObject {
public:
  SignatureSearch();
  virtual ~SignatureSearch();

  void putSignatures(std::unique_ptr<Signatures> &&signatures);
  void setPerspective(Perspective *perspective);

protected:
  virtual bool event(QEvent *event);

private:
  std::vector<std::unique_ptr<Signature>>
  searchSignatures(const std::vector<int> &wordIds) const;

private:
  std::unique_ptr<Signatures> _signatures;
  Perspective *_perspective;
};
