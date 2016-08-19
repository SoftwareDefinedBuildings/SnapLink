#pragma once

#include "data/Signatures.h"
#include <QEvent>
#include <QObject>
#include <memory>
#include <opencv2/core/core.hpp>

class CameraModel;
class Signature;
class Transform;
class Visibility;
class HTTPServer;

class Perspective : public QObject {
public:
  Perspective();
  virtual ~Perspective();

  void setSignatures(const std::shared_ptr<Signatures> &signatures);
  void setVisibility(Visibility *vis);
  void setHTTPServer(HTTPServer *httpServer);

protected:
  virtual bool event(QEvent *event);

private:
  void localize(const std::multimap<int, cv::KeyPoint> &words,
                const CameraModel &camera, int oldSigId, int &dbId,
                Transform &transform) const;
  Transform estimateMotion3DTo2D(const std::map<int, cv::Point3f> &words3A,
                                 const std::map<int, cv::KeyPoint> &words2B,
                                 const CameraModel &camera,
                                 const Transform &guess,
                                 std::vector<int> *inliersOut,
                                 size_t minInliers) const;

private:
  std::shared_ptr<Signatures> _signatures;
  Visibility *_vis;
  HTTPServer *_httpServer;
};
