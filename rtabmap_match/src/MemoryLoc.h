#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Memory.h>
#include <pcl/common/common.h>

namespace rtabmap {

class Signature;

class MemoryLoc : public Memory
{
public:
    MemoryLoc(const ParametersMap & parameters = ParametersMap());

    void generateImages();
    Transform computeGlobalVisualTransform(const std::vector<int> & oldIds, int newId, std::string * rejectedMsg = 0, int * inliers = 0, double * variance = 0) const;
    Transform computeGlobalVisualTransform(const std::vector<Signature> & oldSs, const Signature & newS, std::string * rejectedMsg = 0, int * inliers = 0, double * variance = 0) const;

private:
    void getClouds(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &clouds, std::map<int, Transform> &poses);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembleClouds(const std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &clouds, 
                                                          const std::map<int, Transform> &poses);
    
private:
    int _bowMinInliers;
    int _bowIterations;
    int _bowRefineIterations;
    double _bowPnPReprojError;
    int _bowPnPFlags;

    double _voxelSize;
    int _normalK;
    double _gp3Radius;
    double _gp3Mu;
};

} // namespace rtabmap
