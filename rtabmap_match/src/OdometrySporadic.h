#pragma once

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>

class UTimer;

namespace rtabmap {

class Feature2D;
class OdometryInfo;
class ParticleFilter;

class Memory;

class RTABMAP_EXP OdometrySporadic : public Odometry
{
public:
    OdometrySporadic(const std::string dbPath, const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
    virtual ~OdometrySporadic();
    virtual void reset(const Transform & initialPose);

private:
    virtual Transform computeTransform(const SensorData & data, OdometryInfo *info = NULL);
    static bool compareLikelihood(std::pair<const int, float> const& l, std::pair<const int, float> const& r);

private:
    Memory *memory_;
    std::string dbPath_;
    ParametersMap memoryParams_;
    ParametersMap memoryLocParams_;
};

} /* namespace rtabmap */
