#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include <pcl/common/common.h>
#include <pcl/io/vtk_lib_io.h>

class MemoryLoc :
    public rtabmap::Memory
{
public:
    MemoryLoc(const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());

    rtabmap::Transform computeGlobalVisualTransform(const std::vector<int> &oldIds, int newId) const;
    rtabmap::Transform computeGlobalVisualTransform(const std::vector<const rtabmap::Signature*> &oldSigs, const rtabmap::Signature *newSig) const;

private:
    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
