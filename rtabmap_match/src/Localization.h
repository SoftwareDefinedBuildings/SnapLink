#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>

#include "MemoryLoc.h"

#define TOP_K 2

class UTimer;

namespace rtabmap
{

class MemoryLoc;

class Localization
{
public:
    Localization(const std::string dbPath, const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    virtual ~Localization();
    virtual Transform localize(const SensorData &data);

private:
    void optimize();  // optimize poses using TORO graph
    static bool compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r);

private:
    int _topk;
    std::string _dbPath;
    MemoryLoc *_memory;
    std::map<int, Transform> _optimizedPoses;
    ParametersMap _memoryParams;
    ParametersMap _memoryLocParams;
};

} /* namespace rtabmap */
