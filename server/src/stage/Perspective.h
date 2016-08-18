#pragma once

#include <QEvent>
#include <QObject>
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

  void setVisibility(Visibility *vis);
  void setHTTPServer(HTTPServer *httpServer);

protected:
  virtual bool event(QEvent *event);

private:
  // get pose from optimizedPoses if available, otherwise get from sig itself
  Transform localize(const std::multimap<int, cv::KeyPoint> &words,
                     const CameraModel &camera, const Signature &oldSig) const;
  Transform estimateMotion3DTo2D(const std::map<int, cv::Point3f> &words3A,
                                 const std::map<int, cv::KeyPoint> &words2B,
                                 const CameraModel &camera,
                                 const Transform &guess,
                                 std::vector<int> *inliersOut,
                                 size_t minInliers) const;

private:
  Visibility *_vis;
  HTTPServer *_httpServer;
};
