#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Parameters.h>
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

    bool init(const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());

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
        int minInliers,
        int iterations,
        double reprojError,
        int flagsPnP,
        int refineIterations,
        const Transform &guess,
        const std::map<int, cv::Point3f> &words3B,
        double *varianceOut,
        std::vector<int> *matchesOut,
        std::vector<int> *inliersOut) const;

private:
    Visibility *_vis;
    HTTPServer *_httpServer;

    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
