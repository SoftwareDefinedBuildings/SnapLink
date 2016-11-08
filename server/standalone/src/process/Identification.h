#pragma once

#include "algo/Feature.h"
#include "algo/Perspective.h"
#include "algo/Visibility.h"
#include "algo/WordSearch.h"
#include <QEvent>
#include <QObject>

#define SAMPLE_SIZE 2

class HTTPServer;
class CameraModel;
class Session;

class Identification : public QObject {
public:
  Identification(const std::shared_ptr<Words> &words,
                 std::unique_ptr<Labels> &&labels);
  ~Identification();

  void setHTTPServer(HTTPServer *httpServer);
  bool identify(const cv::Mat &image, const CameraModel &camera,
                std::vector<std::string> &names, Session &session);

protected:
  virtual bool event(QEvent *event);

private:
  HTTPServer *_httpServer;

  Feature _feature;
  WordSearch _wordSearch;
  Perspective _perspective;
  Visibility _visibility;
};
