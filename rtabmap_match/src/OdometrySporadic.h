#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>

#include "MemoryLoc.h"

class UTimer;

namespace rtabmap {

class OdometryInfo;
class MemoryLoc;

class OdometrySporadic : public Odometry
{
public:
    OdometrySporadic(const std::string dbPath, const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
    virtual ~OdometrySporadic();
    virtual void reset(const Transform & initialPose);

private:
    virtual Transform computeTransform(const SensorData & data, OdometryInfo *info = NULL);
    static bool compareLikelihood(std::pair<const int, float> const& l, std::pair<const int, float> const& r);

private:
    MemoryLoc *memory_;
    std::string dbPath_;
    ParametersMap memoryParams_;
    ParametersMap memoryLocParams_;
};

} /* namespace rtabmap */
