#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>

#include "HTTPServer.h" // needed for ConnectionInfo
#include "MemoryLoc.h"
#include "Time.h"

class UTimer;

namespace rtabmap
{

class MemoryLoc;

class Localization
{
public:
    Localization(const std::string dbPath, const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());
    virtual ~Localization();
    virtual Transform localize(const SensorData &data, void *context);

private:
    static bool compareLikelihood(std::pair<const int, float> const &l, std::pair<const int, float> const &r);

private:
    MemoryLoc *_memory;
    std::string _dbPath;
    ParametersMap _memoryParams;
    ParametersMap _memoryLocParams;
};

} /* namespace rtabmap */
