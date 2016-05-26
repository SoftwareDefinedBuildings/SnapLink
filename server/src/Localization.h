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

#define TOP_K 5

class Visibility;
class HTTPServer;

class Localization :
    public QObject
{
public:
    Localization();
    virtual ~Localization();

    bool init(const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());

    void setMemory(MemoryLoc *memory);
    void setVisibility(Visibility *vis);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    bool localize(rtabmap::SensorData *sensorData, rtabmap::Transform *pose, int *dbId, void *context);
    // get pose from optimizedPoses if available, otherwise get from sig itself
    std::vector< std::pair<int, int> > findKNearestSignatures(const rtabmap::Signature &signature, int k);
    //static float computeSimilarity(const rtabmap::Signature &s1, const rtabmap::Signature &s2);
    static bool compareSimilarity(std::pair<std::pair<int, int>, float> const &l, std::pair<std::pair<int, int>, float> const &r);
    rtabmap::Transform computeGlobalVisualTransform(const rtabmap::Signature *newSig, int oldDbId, int oldSigId) const;

private:
    MemoryLoc *_memory;
    Visibility *_vis;
    HTTPServer *_httpServer;

    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
