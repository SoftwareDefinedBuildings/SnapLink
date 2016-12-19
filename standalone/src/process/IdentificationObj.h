#pragma once

#include "algo/Feature.h"
#include "algo/Perspective.h"
#include "algo/Visibility.h"
#include "algo/WordSearch.h"
#include <QEvent>
#include <QObject>

#define SAMPLE_SIZE 200

class BWServer;
class HTTPServer;
class CameraModel;
class Session;

class IdentificationObj : public QObject {
public:
  IdentificationObj(const std::shared_ptr<Words> &words,
                 std::unique_ptr<Labels> &&labels);
  ~IdentificationObj();

  void setHTTPServer(HTTPServer *httpServer);
  void setBWServer(BWServer *bwServer);
  bool identify(const cv::Mat &image, const CameraModel &camera,
                std::vector<std::string> &names, Session &session);

protected:
  virtual bool event(QEvent *event);

private:
  HTTPServer *_httpServer;
  BWServer *_bwServer;
  Feature _feature;
  WordSearch _wordSearch;
  Perspective _perspective;
  Visibility _visibility;
};
