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
#include "Time.h"

#define TOP_K 1

class Visibility;
class HTTPServer;

class Localization :
    public QObject
{
public:
    Localization();
    virtual ~Localization();

    void setMemory(MemoryLoc *memory);
    void setVisibility(Visibility *vis);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    rtabmap::Transform localize(rtabmap::SensorData *sensorData, void *context);
    // get pose from optimizedPoses if available, otherwise get from sig itself
    rtabmap::Transform getPose(const rtabmap::Signature *sig) const;
    static bool compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r);

private:
    MemoryLoc *_memory;
    Visibility *_vis;
    HTTPServer *_httpServer;
};
