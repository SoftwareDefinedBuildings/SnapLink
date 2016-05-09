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

#define TOP_K 1

class Visibility;
class HTTPServer;

class Localization :
    public QObject
{
public:
    Localization();
    virtual ~Localization();

    void setMemories(std::vector<MemoryLoc *> *memories);
    void setVisibility(Visibility *vis);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    bool localize(rtabmap::SensorData *sensorData, rtabmap::Transform *pose, int *dbId);
    // get pose from optimizedPoses if available, otherwise get from sig itself
    rtabmap::Transform getPose(const rtabmap::Signature *sig) const;
    static bool compareLikelihood(std::pair<std::pair<int, int>, float> const &l, std::pair<std::pair<int, int>, float> const &r);

private:
    std::vector<MemoryLoc *> *_memories;
    Visibility *_vis;
    HTTPServer *_httpServer;
};
