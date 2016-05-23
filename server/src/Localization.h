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

#define TOP_K 50

class Visibility;
class HTTPServer;

class Localization :
    public QObject
{
public:
    Localization();
    virtual ~Localization();

    bool init(const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());

    void setMemories(std::vector<MemoryLoc *> *memories);
    void setVisibility(Visibility *vis);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    bool localize(rtabmap::SensorData *sensorData, rtabmap::Transform *pose, int *dbId, void *context);
    // get pose from optimizedPoses if available, otherwise get from sig itself
    std::map<int, float> computeSimilarities(const rtabmap::Signature &signature, const std::list<int> &ids, const MemoryLoc *memory);
    static float computeSimilarity(const rtabmap::Signature &s1, const rtabmap::Signature &s2);
    static bool compareSimilarity(std::pair<std::pair<int, int>, float> const &l, std::pair<std::pair<int, int>, float> const &r);
    rtabmap::Transform computeGlobalVisualTransform(int oldId, int newId, const MemoryLoc *memory) const;
    rtabmap::Transform computeGlobalVisualTransform(const rtabmap::Signature *oldSig, const rtabmap::Signature *newSig, const MemoryLoc *memory) const;

private:
    std::vector<MemoryLoc *> *_memories;
    Visibility *_vis;
    HTTPServer *_httpServer;

    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
