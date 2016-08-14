#pragma once

#include <QObject>
#include <QEvent>
#include "stage/Visibility.h"
#include "stage/HTTPServer.h"
#include "data/SensorData.h"
#include "data/Signature.h"
#include "data/Transform.h"

class Visibility;
class HTTPServer;

class Perspective :
    public QObject
{
public:
    Perspective();
    virtual ~Perspective();

    void setVisibility(Visibility *vis);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    // get pose from optimizedPoses if available, otherwise get from sig itself
    Transform localize(const std::vector<int> &wordIds, const SensorData &sensorData, const Signature &oldSig) const;
    Transform estimateMotion3DTo2D(
        const std::map<int, cv::Point3f> &words3A,
        const std::map<int, cv::KeyPoint> &words2B,
        const CameraModel &cameraModel,
        const Transform &guess,
        std::vector<int> *inliersOut,
        int minInliers) const;

private:
    Visibility *_vis;
    HTTPServer *_httpServer;
};
