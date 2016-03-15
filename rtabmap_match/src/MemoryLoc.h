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

    rtabmap::Transform computeGlobalVisualTransform(const std::vector<int> &oldIds, int newId, const std::map<int, rtabmap::Transform> *optimizedPoses = NULL, std::string *rejectedMsg = NULL, int *inliers = NULL, double *variance = NULL) const;
    rtabmap::Transform computeGlobalVisualTransform(const std::vector<rtabmap::Signature> &oldSigs, const rtabmap::Signature &newSig, const std::map<int, rtabmap::Transform> *optimizedPoses = NULL, std::string *rejectedMsg = NULL, int *inliers = NULL, double *variance = NULL) const;

private:
    // get pose from optimizedPoses if available, otherwise get from sig itself
    const rtabmap::Transform getPose(const rtabmap::Signature &sig, const std::map<int, rtabmap::Transform> *optimizedPoses = NULL) const;

private:
    int _minInliers;
    int _iterations;
    int _pnpRefineIterations;
    double _pnpReprojError;
    int _pnpFlags;
};
