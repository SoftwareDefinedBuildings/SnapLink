#pragma once

#include "adapter/RTABMapDBAdapter.h"
#include "data/SensorData.h"
#include "data/Transform.h"
#include "stage/SignatureSearch.h"
#include <QEvent>
#include <QObject>
#include <memory>

class SignatureSearch;
class HTTPServer;

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
