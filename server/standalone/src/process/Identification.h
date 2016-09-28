#pragma once

#include "algo/Feature.h"
#include "algo/Perspective.h"
#include "algo/SignatureSearch.h"
#include "algo/Visibility.h"
#include "algo/WordSearch.h"
#include <QEvent>
#include <QObject>

class HTTPServer;
class CameraModel;
class Session;

class Identification : public QObject {
public:
  Identification(std::unique_ptr<Words> &&words,
                 const std::shared_ptr<Signatures> &signatures,
                 std::unique_ptr<Labels> &&labels);
  ~Identification();

  void setHTTPServer(HTTPServer *httpServer);
  std::vector<std::string>
  identify(const cv::Mat &image, const CameraModel &camera, Session &session);

protected:
  virtual bool event(QEvent *event);

private:
  HTTPServer *_httpServer;

  Feature _feature;
  WordSearch _wordSearch;
  SignatureSearch _signatureSearch;
  Perspective _perspective;
  Visibility _visibility;
};
