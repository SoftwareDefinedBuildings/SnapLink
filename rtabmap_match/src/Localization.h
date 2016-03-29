#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <QObject>
#include <QEvent>
#include "MemoryLoc.h"
#include "Visibility.h"
#include "HTTPServer.h"

#define TOP_K 2

class Visibility;
class HTTPServer;

class Localization :
    public QObject
{
public:
    Localization();
    virtual ~Localization();

    bool init(const std::string &dbPath, const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());

    void setVisibility(Visibility *vis);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    rtabmap::Transform localize(rtabmap::SensorData *sensorData);
    // get pose from optimizedPoses if available, otherwise get from sig itself
    rtabmap::Transform getPose(const rtabmap::Signature *sig) const;
    static bool compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r);

private:
    int _topk;
    MemoryLoc *_memory;
    rtabmap::ParametersMap _memoryParams;
    Visibility *_vis;
    HTTPServer *_httpServer;
};
