#pragma once

#include "algo/Feature.h"
#include "algo/PerspectiveDirect.h"
#include "algo/Visibility.h"
#include "algo/WordSearch.h"
#include <QEvent>
#include <QObject>

class HTTPServer;
class CameraModel;
class Session;

class Identification : public QObject {
public:
  Identification(const std::shared_ptr<Words> &words,
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
  PerspectiveDirect _perspective;
  Visibility _visibility;
};
