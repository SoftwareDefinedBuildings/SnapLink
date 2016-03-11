#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>

#include "MemoryLoc.h"

#define TOP_K 2

class Localization
{
public:
    Localization(const std::string dbPath, const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    virtual ~Localization();
    virtual rtabmap::Transform localize(rtabmap::SensorData data); // TODO should I use const ref here?

private:
    void optimize();  // optimize poses using TORO graph
    static bool compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r);

private:
    int _topk;
    std::string _dbPath;
    MemoryLoc *_memory;
    std::map<int, rtabmap::Transform> _optimizedPoses;
    rtabmap::ParametersMap _memoryParams;
    rtabmap::ParametersMap _memoryLocParams;
};
