#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <QObject>
#include <QEvent>
#include "stage/Visibility.h"
#include "stage/HTTPServer.h"
#include "data/SessionInfo.h"
#include "data/Signature.h"

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
    rtabmap::Transform localize(const std::vector<int> &wordIds, const rtabmap::SensorData &sensorData, const Signature &oldSig, void *context) const;

private:
    Visibility *_vis;
    HTTPServer *_httpServer;

    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
