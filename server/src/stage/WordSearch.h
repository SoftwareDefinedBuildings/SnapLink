#pragma once

#include "data/Words.h"
#include <QEvent>
#include <QObject>
#include <memory>

class SensorData;
class SignatureSearch;

class WordSearch : public QObject {
public:
  WordSearch();
  virtual ~WordSearch();

  void putWords(std::unique_ptr<Words> &&words);
  void setSignatureSearch(SignatureSearch *imageSearch);

protected:
  virtual bool event(QEvent *event);

private:
  std::vector<int> searchWords(const SensorData &sensorData) const;

private:
  std::unique_ptr<Words> _words;
  SignatureSearch *_imageSearch;
};
