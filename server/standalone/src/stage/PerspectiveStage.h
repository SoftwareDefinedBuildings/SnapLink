#pragma once

#include "algo/Perspective.h"
#include "data/Signatures.h"
#include <QEvent>
#include <QObject>
#include <memory>
#include <opencv2/core/core.hpp>

class VisibilityStage;
class HTTPServer;

class PerspectiveStage : public QObject {
public:
  PerspectiveStage(const std::shared_ptr<Signatures> &signatures);
  ~PerspectiveStage();

  void setVisibilityStage(VisibilityStage *visibilityStage);
  void setHTTPServer(HTTPServer *httpServer);

protected:
  virtual bool event(QEvent *event);

private:
  VisibilityStage *_visibilityStage;
  HTTPServer *_httpServer;
  Perspective _perspective;
};
