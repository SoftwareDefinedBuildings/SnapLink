#pragma once

#include "data/Words.h"
#include "algo/WordSearch.h"
#include <QEvent>
#include <QObject>
#include <memory>

class SignatureSearchStage;

class WordSearchStage : public QObject {
public:
  WordSearchStage(std::unique_ptr<Words> &&words);
  ~WordSearchStage();

  void setSignatureSearchStage(SignatureSearchStage *signatureSearchStage);

protected:
  virtual bool event(QEvent *event);

private:
  SignatureSearchStage *_signatureSearchStage;
  WordSearch _wordSearch;
};
