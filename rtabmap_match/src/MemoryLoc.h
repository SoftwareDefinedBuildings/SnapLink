#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Memory.h>

namespace rtabmap {

class Signature;

class MemoryLoc : public Memory
{
public:
    MemoryLoc(const ParametersMap & parameters = ParametersMap());

    Transform computeGlobalVisualTransform(const std::vector<int> & oldIds, int newId, std::string * rejectedMsg = 0, int * inliers = 0, double * variance = 0) const;
    Transform computeGlobalVisualTransform(const std::vector<Signature> & oldSs, const Signature & newS, std::string * rejectedMsg = 0, int * inliers = 0, double * variance = 0) const;

private:
    int _bowMinInliers;
    int _bowIterations;
    int _bowRefineIterations;
    double _bowPnPReprojError;
    int _bowPnPFlags;
};

} // namespace rtabmap
