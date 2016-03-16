#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <QObject>
#include <QEvent>
#include "MemoryLoc.h"
#include "Visibility.h"

#define TOP_K 2

class Visibility;

class Localization :
    public UEventsSender,
    public QObject
{
public:
    Localization(const std::string dbPath, const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    virtual ~Localization();

    Visibility *_vis;

protected:
    virtual bool event(QEvent *event);

private:
    virtual rtabmap::Transform localize(rtabmap::SensorData data); // TODO should I use const ref here?
    void optimizeGraph();  // optimize poses using TORO graph
    static bool compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r);

private:
    int _topk;
    std::string _dbPath;
    MemoryLoc *_memory;
    std::map<int, rtabmap::Transform> _optimizedPoses;
    rtabmap::ParametersMap _memoryParams;
    rtabmap::ParametersMap _memoryLocParams;
};
