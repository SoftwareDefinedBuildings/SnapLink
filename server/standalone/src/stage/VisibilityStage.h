#pragma once

#include "data/Labels.h"
#include "algo/Visibility.h"
#include <QEvent>
#include <QObject>
#include <memory>

class Transform;
class CameraModel;
class HTTPServer;

class VisibilityStage : public QObject {
public:
  VisibilityStage(std::unique_ptr<Labels> &&labels);
  ~VisibilityStage();

  void setHTTPServer(HTTPServer *httpServer);

protected:
  virtual bool event(QEvent *event);

private:
  HTTPServer *_httpServer;
  Visibility _visibility;
};
